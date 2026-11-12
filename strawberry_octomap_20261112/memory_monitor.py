"""Sample Linux memory for one mapping session, including its save phase."""

from collections import defaultdict
import csv
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import select
import sys
import time


PROC = Path("/proc")
OWNER_ENV = "STRAWBERRY_OCTOMAP_OWNER_TOKEN"
PROCESS_METRICS = {
    "process_rss_kib": "rss_processes",
    "process_pss_kib": "pss_processes",
    "process_swap_kib": "swap_processes",
    "process_swap_pss_kib": "swap_pss_processes",
}
SYSTEM_FIELDS = (
    "system_total_kib",
    "system_available_kib",
    "system_used_kib",
    "system_swap_total_kib",
    "system_swap_free_kib",
    "system_swap_used_kib",
)
PEAK_FIELDS = (
    *PROCESS_METRICS,
    "system_used_kib",
    "system_used_delta_kib",
    "system_swap_used_kib",
)
CSV_FIELDS = (
    "utc",
    "elapsed_seconds",
    "phase",
    "sample_duration_ms",
    "owned_processes",
    "stable_processes",
    *PROCESS_METRICS.values(),
    *PROCESS_METRICS,
    *SYSTEM_FIELDS,
    "system_used_delta_kib",
)


def utc_now():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def read_kib_fields(path, keys):
    """Read selected counters from a procfs status file."""
    values = {}
    try:
        with path.open(encoding="ascii", errors="replace") as stream:
            for line in stream:
                name, separator, payload = line.partition(":")
                if not separator or name not in keys:
                    continue
                parts = payload.split()
                if len(parts) != 2 or parts[1] != "kB" or not parts[0].isdecimal():
                    continue
                values[name] = int(parts[0])
                if len(values) == len(keys):
                    break
    except OSError:
        return {}
    return values


def difference(total, free):
    return total - free if total is not None and free is not None else None


def system_memory():
    memory = read_kib_fields(PROC / "meminfo", ("MemTotal", "MemAvailable", "SwapTotal", "SwapFree"))
    total = memory.get("MemTotal")
    available = memory.get("MemAvailable")
    swap_total = memory.get("SwapTotal")
    swap_free = memory.get("SwapFree")
    return {
        "system_total_kib": total,
        "system_available_kib": available,
        "system_used_kib": difference(total, available),
        "system_swap_total_kib": swap_total,
        "system_swap_free_kib": swap_free,
        "system_swap_used_kib": difference(swap_total, swap_free),
    }


def process_identity(directory):
    """Read PPID and starttime. The comm field may contain spaces and parentheses."""
    try:
        stat = (directory / "stat").read_bytes()
        end_comm = stat.rfind(b")")
        if end_comm < 0:
            return None
        fields = stat[end_comm + 1:].split()
        if fields[0] in (b"Z", b"X", b"x"):
            return None
        return int(fields[1]), int(fields[19])
    except (OSError, ValueError, IndexError):
        return None


def mib(value):
    return "NA" if value is None else f"{value / 1024:.1f}"


class MemoryMonitor:
    """Write sampled memory counters without changing the monitored processes."""

    def __init__(self, output_dir: Path, owner_token: str, root_pid: int, sample_interval_seconds: float = 10.0):
        if root_pid <= 0 or not owner_token or "\0" in owner_token:
            raise ValueError("Memory monitoring needs a positive PID and a valid owner token")
        if not math.isfinite(sample_interval_seconds) or sample_interval_seconds <= 0:
            raise ValueError("Memory sampling interval must be positive and finite")
        root_identity = process_identity(PROC / str(root_pid))
        if root_identity is None:
            raise ProcessLookupError(f"Cannot read initial identity of PID {root_pid}")

        self.output_dir = Path(output_dir)
        self.root_pid = root_pid
        self.root_starttime = root_identity[1]
        self._uid = os.getuid()
        self._token_entry = f"{OWNER_ENV}={owner_token}".encode("utf-8")
        self._owned = {root_pid: self.root_starttime}
        self._started = time.monotonic()
        self._sample_interval_seconds = sample_interval_seconds
        self._first_sample_at = None
        self._last_sample_slot = -1
        self._baseline = {"utc": utc_now(), **system_memory()}
        self._stats = {"global": self._empty_stats(), "by_phase": {}}
        self._summary = None
        self._terminal_available = True
        self._csv = (self.output_dir / "memory.csv").open("x", encoding="utf-8", newline="")
        try:
            self._writer = csv.DictWriter(self._csv, fieldnames=CSV_FIELDS)
            self._writer.writeheader()
            self._csv.flush()
        except OSError:
            self._csv.close()
            raise

    def _has_owner_token(self, directory):
        try:
            if directory.stat().st_uid != self._uid:
                return False
            return self._token_entry in (directory / "environ").read_bytes().split(b"\0")
        except OSError:
            return False

    def _discover_owned(self):
        identities = {}
        children = defaultdict(list)
        for directory in PROC.iterdir():
            if not directory.name.isdecimal():
                continue
            identity = process_identity(directory)
            if identity is None:
                continue
            pid = int(directory.name)
            parent, starttime = identity
            identities[pid] = starttime
            children[parent].append(pid)

        owned = {pid for pid, starttime in self._owned.items() if identities.get(pid) == starttime}
        if identities.get(self.root_pid) == self.root_starttime:
            owned.add(self.root_pid)
        for pid in identities.keys() - owned:
            if self._has_owner_token(PROC / str(pid)):
                owned.add(pid)

        pending = list(owned)
        while pending:
            for child in children.get(pending.pop(), ()):
                if child in owned:
                    continue
                owned.add(child)
                pending.append(child)
        return {pid: identities[pid] for pid in owned}

    def _process_memory(self):
        owned = self._discover_owned()
        stable = {}
        totals = dict.fromkeys(PROCESS_METRICS, 0)
        coverage = dict.fromkeys(PROCESS_METRICS.values(), 0)
        for pid, starttime in owned.items():
            directory = PROC / str(pid)
            status = read_kib_fields(directory / "status", ("VmRSS", "VmSwap"))
            rollup = read_kib_fields(directory / "smaps_rollup", ("Pss", "SwapPss"))
            current = process_identity(directory)
            if current is None or current[1] != starttime:
                continue
            stable[pid] = starttime
            values = {
                "process_rss_kib": status.get("VmRSS"),
                "process_pss_kib": rollup.get("Pss"),
                "process_swap_kib": status.get("VmSwap"),
                "process_swap_pss_kib": rollup.get("SwapPss"),
            }
            for name, value in values.items():
                if value is not None:
                    totals[name] += value
                    coverage[PROCESS_METRICS[name]] += 1
        self._owned = stable
        return {
            "owned_processes": len(owned),
            "stable_processes": len(stable),
            **coverage,
            **{
                name: totals[name] if owned and coverage[count] == len(owned) else None
                for name, count in PROCESS_METRICS.items()
            },
        }

    def owned_identities(self):
        """Discover still-live session members without touching the CSV stream."""
        stable = {}
        for pid, starttime in self._discover_owned().items():
            current = process_identity(PROC / str(pid))
            if current is not None and current[1] == starttime:
                stable[pid] = starttime
        self._owned = stable
        return stable

    def owned_pids(self):
        return sorted(self.owned_identities())

    @staticmethod
    def _empty_stats():
        return {
            "samples": 0,
            "sampled_peaks": dict.fromkeys(PEAK_FIELDS),
            "coverage": {
                "owned_process_samples": 0,
                "stable_process_samples": 0,
                "readable_process_samples": dict.fromkeys(PROCESS_METRICS, 0),
                "complete_process_samples": dict.fromkeys(PROCESS_METRICS, 0),
                "system_used_samples": 0,
                "system_swap_samples": 0,
            },
        }

    @staticmethod
    def _accumulate(stats, row):
        stats["samples"] += 1
        peaks = stats["sampled_peaks"]
        for name in PEAK_FIELDS:
            value = row[name]
            if value is not None and (peaks[name] is None or value > peaks[name]):
                peaks[name] = value

        coverage = stats["coverage"]
        coverage["owned_process_samples"] += row["owned_processes"]
        coverage["stable_process_samples"] += row["stable_processes"]
        coverage["system_used_samples"] += row["system_used_kib"] is not None
        coverage["system_swap_samples"] += row["system_swap_used_kib"] is not None
        for metric, count in PROCESS_METRICS.items():
            coverage["readable_process_samples"][metric] += row[count]
            coverage["complete_process_samples"][metric] += row[metric] is not None

    def _print_sample(self, row):
        if not self._terminal_available:
            return
        line = (
            f"MEM phase={row['phase']} elapsed={row['elapsed_seconds']:.1f}s "
            f"PSS={mib(row['process_pss_kib'])} ({row['pss_processes']}/{row['owned_processes']}) "
            f"RSS={mib(row['process_rss_kib'])} ({row['rss_processes']}/{row['owned_processes']}) "
            f"available={mib(row['system_available_kib'])} used={mib(row['system_used_kib'])} "
            f"delta={mib(row['system_used_delta_kib'])} swap={mib(row['system_swap_used_kib'])} MiB\n"
        )
        try:
            output = sys.stdout.fileno()
            if select.select([], [output], [], 0)[1]:
                os.write(output, line.encode("utf-8"))
        except (AttributeError, OSError, ValueError):
            self._terminal_available = False

    def sample(self, phase: str) -> dict | None:
        """Collect one sample when due on the schedule anchored to the first call."""
        if self._csv.closed:
            raise RuntimeError("Memory monitor is closed")
        started = time.monotonic()
        if self._first_sample_at is None:
            self._first_sample_at = started
        slot = math.floor((started - self._first_sample_at) / self._sample_interval_seconds)
        if slot <= self._last_sample_slot:
            return None
        self._last_sample_slot = slot

        row = {
            "utc": utc_now(),
            "elapsed_seconds": round(started - self._started, 3),
            "phase": phase,
            **system_memory(),
            **self._process_memory(),
        }
        row["sample_duration_ms"] = round((time.monotonic() - started) * 1000, 1)
        row["system_used_delta_kib"] = difference(row["system_used_kib"], self._baseline["system_used_kib"])
        self._writer.writerow(row)
        self._csv.flush()

        phases = self._stats["by_phase"]
        if phase not in phases:
            phases[phase] = self._empty_stats()
        self._accumulate(phases[phase], row)
        self._accumulate(self._stats["global"], row)
        self._print_sample(row)
        return row

    def finish(self, extra: dict) -> dict:
        """Close the sample stream and publish the summary for this session."""
        if self._summary is not None:
            return self._summary
        self._csv.close()
        summary = {
            "units": "KiB",
            "sample_interval_seconds": self._sample_interval_seconds,
            "root_pid": self.root_pid,
            "root_starttime_ticks": self.root_starttime,
            "baseline": self._baseline,
            "finished_utc": utc_now(),
            "global": self._stats["global"],
            "by_phase": self._stats["by_phase"],
            "session": extra,
            "measurement_notes": [
                "PSS attributes a proportional share of each shared page to each process; RSS counts shared resident pages in each process that maps them.",
                "Process totals are recorded only when every discovered process has a valid reading for that counter.",
                "System used is MemTotal minus MemAvailable; its baseline delta includes cache and background activity.",
                "System swap is SwapTotal minus SwapFree; it measures logical swap usage, including on zram systems.",
                "Process swap uses VmSwap and proportional process swap uses SwapPss, when reported by the kernel.",
                "The first sample anchors the monotonic schedule. Missed intervals are skipped and phase boundaries use the same schedule.",
                "Each peak is the largest sampled total for that phase; brief allocations between samples may be missed.",
            ],
        }
        temporary = self.output_dir / ".memory_summary.json.tmp"
        with temporary.open("x", encoding="utf-8") as stream:
            json.dump(summary, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write("\n")
        temporary.replace(self.output_dir / "memory_summary.json")
        self._summary = summary
        return summary
