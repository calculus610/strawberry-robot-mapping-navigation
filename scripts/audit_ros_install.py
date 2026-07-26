#!/usr/bin/env python3
"""Audit editable ROS package resources in src/ against install/.

The script is intentionally read-only. It ignores generated ament/CMake files
and focuses on resources that are commonly edited by hand, such as launch,
config, RViz, behavior-tree, URDF, map, and script files.
"""

from __future__ import annotations

import hashlib
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


EDITABLE_DIRS = {
    "behavior_trees",
    "config",
    "launch",
    "maps",
    "meshes",
    "param",
    "params",
    "resource",
    "rviz",
    "scripts",
    "urdf",
    "worlds",
}
EDITABLE_SUFFIXES = {
    ".bt",
    ".cfg",
    ".json",
    ".lua",
    ".py",
    ".rviz",
    ".txt",
    ".urdf",
    ".xacro",
    ".xml",
    ".yaml",
    ".yml",
}
PRUNE_DIRS = {
    ".git",
    "__pycache__",
    "build",
    "devel",
    "install",
    "log",
}


def digest(path: Path) -> str:
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


def package_name(package_xml: Path) -> str | None:
    try:
        root = ET.parse(package_xml).getroot()
    except (ET.ParseError, OSError):
        return None
    element = root.find("name")
    if element is None or not element.text:
        return None
    return element.text.strip()


def discover_packages(src_root: Path) -> list[tuple[str, Path]]:
    packages: list[tuple[str, Path]] = []
    for current, directories, files in os.walk(src_root):
        directories[:] = sorted(name for name in directories if name not in PRUNE_DIRS)
        if "package.xml" not in files:
            continue
        package_dir = Path(current)
        name = package_name(package_dir / "package.xml")
        if name:
            packages.append((name, package_dir))
        directories[:] = []
    return sorted(packages)


def is_editable(relative: Path) -> bool:
    if relative == Path("package.xml"):
        return True
    if not relative.parts:
        return False
    if any(part == "__pycache__" or part.startswith("._") for part in relative.parts):
        return False
    first = relative.parts[0].lower()
    in_resource_directory = (
        first in EDITABLE_DIRS
        or first.startswith("launch")
        or first.startswith("config")
    )
    return in_resource_directory and relative.suffix.lower() in EDITABLE_SUFFIXES


def iter_files(root: Path):
    if not root.is_dir():
        return
    for current, directories, files in os.walk(root):
        directories[:] = sorted(name for name in directories if name not in PRUNE_DIRS)
        current_path = Path(current)
        for filename in sorted(files):
            path = current_path / filename
            if path.is_file() or path.is_symlink():
                yield path


def install_share_for(workspace: Path, package: str) -> Path | None:
    isolated = workspace / "install" / package / "share" / package
    if isolated.is_dir():
        return isolated
    merged = workspace / "install" / "share" / package
    if merged.is_dir():
        return merged
    return None


def link_description(path: Path) -> str:
    if not path.is_symlink():
        return "copy"
    try:
        return f"symlink->{path.resolve(strict=False)}"
    except OSError:
        return f"symlink->{os.readlink(path)}"


def main() -> int:
    workspace = Path(sys.argv[1] if len(sys.argv) > 1 else ".").expanduser().resolve()
    src_root = workspace / "src"
    install_root = workspace / "install"
    if not src_root.is_dir():
        print(f"ERROR\tmissing src directory\t{src_root}")
        return 2
    if not install_root.is_dir():
        print(f"ERROR\tmissing install directory\t{install_root}")
        return 2

    counts = {
        "packages": 0,
        "installed_packages": 0,
        "matched": 0,
        "different": 0,
        "install_only": 0,
        "source_only": 0,
        "broken": 0,
    }

    for package, source_dir in discover_packages(src_root):
        counts["packages"] += 1
        install_share = install_share_for(workspace, package)
        if install_share is None:
            print(f"PACKAGE_NOT_INSTALLED\t{package}\t{source_dir}")
            continue
        counts["installed_packages"] += 1

        installed_relatives: set[Path] = set()
        for installed in iter_files(install_share):
            relative = installed.relative_to(install_share)
            if not is_editable(relative):
                continue
            installed_relatives.add(relative)
            source = source_dir / relative
            if not source.is_file():
                counts["install_only"] += 1
                print(
                    f"INSTALL_ONLY\t{package}\t{relative}\t{link_description(installed)}"
                    f"\t{installed}"
                )
                continue
            try:
                same = digest(source) == digest(installed)
            except OSError as error:
                counts["broken"] += 1
                print(f"BROKEN\t{package}\t{relative}\t{error}")
                continue
            if same:
                counts["matched"] += 1
            else:
                counts["different"] += 1
                print(
                    f"DIFF\t{package}\t{relative}\t{link_description(installed)}"
                    f"\tSRC={source}\tINSTALL={installed}"
                )

        for source in iter_files(source_dir):
            relative = source.relative_to(source_dir)
            if not is_editable(relative) or relative in installed_relatives:
                continue
            if any(part in PRUNE_DIRS for part in relative.parts):
                continue
            counts["source_only"] += 1
            print(f"SOURCE_ONLY\t{package}\t{relative}\t{source}")

    summary = "\t".join(f"{key}={value}" for key, value in counts.items())
    print(f"SUMMARY\tworkspace={workspace}\t{summary}")
    return 1 if counts["broken"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
