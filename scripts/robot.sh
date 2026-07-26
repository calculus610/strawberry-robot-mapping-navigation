#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
CONFIG_FILE="${ROBOT_CONFIG_FILE:-${WORKSPACE_ROOT}/.robot.env}"

usage() {
  cat <<'EOF'
Usage: ./scripts/robot.sh <command> [arguments]

Connection:
  doctor                 Check local tools and configuration
  key                    Create a dedicated SSH key and install it on the robot
  check                  Verify SSH and show the robot host/user/workspace
  shell                  Open a clean ROS shell in the robot workspace

Robot services:
  restart-base           Restart robot-base.service and verify its driver

Inspection (read-only on the robot):
  context                Print an environment and hardware summary
  audit-install          Compare editable resources in robot src/ and install/
  exec <command...>      Run a command from the robot workspace
  get <relative> [dest]  Download one workspace file/directory to the Mac

Development:
  sync-dry               Preview the Mac -> robot incremental sync
  sync                   Sync this Mac workspace to the robot
  build                  Build with only /opt/ros/humble as the underlay
  sync-build             Sync, then build on the robot

Initial setup:
  cp .robot.env.example .robot.env
  Edit ROBOT_HOST in .robot.env, then run:
  ./scripts/robot.sh doctor
  ./scripts/robot.sh key
  ./scripts/robot.sh check
EOF
}

die() {
  printf 'error: %s\n' "$*" >&2
  exit 1
}

load_config() {
  if [[ ! -f "${CONFIG_FILE}" ]]; then
    die "missing ${CONFIG_FILE}; run: cp .robot.env.example .robot.env"
  fi

  # shellcheck source=/dev/null
  source "${CONFIG_FILE}"

  [[ -n "${ROBOT_HOST:-}" ]] || die "set ROBOT_HOST in .robot.env"
  : "${ROBOT_USER:=wheeltec}"
  : "${ROBOT_PORT:=22}"
  : "${ROBOT_WORKSPACE:=/home/wheeltec/zzj_ws}"
  : "${ROBOT_ROS_SETUP:=/opt/ros/humble/setup.bash}"
  : "${ROBOT_IDENTITY_FILE:=${HOME}/.ssh/wheeltec_robot_ed25519}"
  : "${ROBOT_BUILD_COMMAND:=colcon build --symlink-install}"
  : "${ROBOT_SYNC_DELETE:=0}"

  [[ "${ROBOT_USER}" =~ ^[A-Za-z_][A-Za-z0-9_-]*$ ]] || die "ROBOT_USER contains unsupported characters"
  [[ "${ROBOT_HOST}" =~ ^[A-Za-z0-9._-]+$ ]] || die "ROBOT_HOST must be an IPv4 address or hostname"
  [[ "${ROBOT_PORT}" =~ ^[0-9]+$ ]] || die "ROBOT_PORT must be a number"
  [[ "${ROBOT_WORKSPACE}" == /* ]] || die "ROBOT_WORKSPACE must be an absolute path"
  [[ "${ROBOT_WORKSPACE}" =~ ^/[A-Za-z0-9._/-]+$ ]] || die "ROBOT_WORKSPACE contains unsupported characters"
  [[ "${ROBOT_ROS_SETUP}" =~ ^/[A-Za-z0-9._/-]+$ ]] || die "ROBOT_ROS_SETUP must be a simple absolute path"
  case "${ROBOT_WORKSPACE}" in
    /|/home|"/home/${ROBOT_USER}")
      die "ROBOT_WORKSPACE is too broad and unsafe: ${ROBOT_WORKSPACE}"
      ;;
  esac
  [[ "${ROBOT_WORKSPACE}" != *[[:space:]]* ]] || die "ROBOT_WORKSPACE cannot contain spaces"

  SSH_TARGET="${ROBOT_USER}@${ROBOT_HOST}"
  SSH_ARGS=(
    -p "${ROBOT_PORT}"
    -o ConnectTimeout=8
    -o ServerAliveInterval=15
    -o ServerAliveCountMax=3
    -o StrictHostKeyChecking=accept-new
  )
  if [[ -f "${ROBOT_IDENTITY_FILE}" ]]; then
    SSH_ARGS+=( -i "${ROBOT_IDENTITY_FILE}" -o IdentitiesOnly=yes )
  fi
}

quote_remote() {
  printf '%q' "$1"
}

run_remote_bash() {
  local command="$1"
  ssh "${SSH_ARGS[@]}" "${SSH_TARGET}" "bash -lc $(quote_remote "${command}")"
}

rsync_transport() {
  local args=(
    ssh
    -p "${ROBOT_PORT}"
    -o ConnectTimeout=8
    -o ServerAliveInterval=15
    -o ServerAliveCountMax=3
    -o StrictHostKeyChecking=accept-new
  )
  if [[ -f "${ROBOT_IDENTITY_FILE}" ]]; then
    args+=( -i "${ROBOT_IDENTITY_FILE}" -o IdentitiesOnly=yes )
  fi
  printf '%q ' "${args[@]}"
}

ensure_remote_workspace() {
  run_remote_bash "mkdir -p $(quote_remote "${ROBOT_WORKSPACE}")"
}

sync_workspace() {
  local dry_run="${1:-0}"
  local transport backup_stamp
  local options=(
    -az
    --human-readable
    --itemize-changes
    --exclude=.git/
    --exclude=.robot.env
    --exclude=build/
    --exclude=install/
    --exclude=devel/
    --exclude=log/
    --exclude=.cache/
    --exclude=.colcon/
    --exclude=.robot-sync-backups/
    --exclude=maps/
    --exclude=__pycache__/
    --exclude='*.pyc'
    --exclude='*.pyo'
    --exclude=Log/
    --exclude=PCD/
    --exclude=.DS_Store
    --exclude='._*'
  )

  if [[ "${dry_run}" == "1" ]]; then
    options+=( --dry-run --checksum )
  elif [[ "${ROBOT_SYNC_DELETE}" == "1" ]]; then
    options+=( --delete )
    printf 'warning: ROBOT_SYNC_DELETE=1; remote files missing on the Mac may be deleted.\n' >&2
  fi

  if [[ "${dry_run}" != "1" ]]; then
    backup_stamp="$(date '+%Y%m%d-%H%M%S')"
    options+=( --backup "--backup-dir=.robot-sync-backups/${backup_stamp}" )
    printf 'remote overwrite backup: %s/.robot-sync-backups/%s\n' \
      "${ROBOT_WORKSPACE}" "${backup_stamp}"
  fi

  ensure_remote_workspace
  transport="$(rsync_transport)"
  rsync "${options[@]}" -e "${transport}" \
    "${WORKSPACE_ROOT}/" "${SSH_TARGET}:${ROBOT_WORKSPACE}/"
}

command_doctor() {
  local failed=0
  for tool in ssh ssh-keygen ssh-copy-id rsync git; do
    if command -v "${tool}" >/dev/null 2>&1; then
      printf 'ok: %-12s %s\n' "${tool}" "$(command -v "${tool}")"
    else
      printf 'missing: %s\n' "${tool}" >&2
      failed=1
    fi
  done

  if [[ -f "${CONFIG_FILE}" ]]; then
    printf 'ok: config       %s\n' "${CONFIG_FILE}"
  else
    printf 'missing: %s\n' "${CONFIG_FILE}" >&2
    printf 'create it with: cp .robot.env.example .robot.env\n' >&2
    failed=1
  fi

  if git -C "${WORKSPACE_ROOT}" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    printf 'ok: git repo     %s\n' "${WORKSPACE_ROOT}"
  else
    printf 'missing: Git repository at %s\n' "${WORKSPACE_ROOT}" >&2
    failed=1
  fi

  if [[ "${failed}" != "0" ]]; then
    return 1
  fi

  load_config
  printf 'ok: target       %s\n' "${SSH_TARGET}"
  printf 'ok: workspace    %s\n' "${ROBOT_WORKSPACE}"
  if [[ -f "${ROBOT_IDENTITY_FILE}" ]]; then
    printf 'ok: SSH key      %s\n' "${ROBOT_IDENTITY_FILE}"
  else
    printf 'next: run ./scripts/robot.sh key to create %s\n' "${ROBOT_IDENTITY_FILE}"
  fi
}

command_key() {
  load_config
  if [[ ! -f "${ROBOT_IDENTITY_FILE}" ]]; then
    mkdir -p "$(dirname "${ROBOT_IDENTITY_FILE}")"
    chmod 700 "$(dirname "${ROBOT_IDENTITY_FILE}")"
    ssh-keygen -t ed25519 -f "${ROBOT_IDENTITY_FILE}" -N '' \
      -C "wheeltec-robot-from-$(hostname -s)"
  fi

  printf 'Installing the public key. Enter the robot password if prompted.\n'
  ssh-copy-id -i "${ROBOT_IDENTITY_FILE}.pub" -p "${ROBOT_PORT}" "${SSH_TARGET}"
}

command_check() {
  local workspace_q
  load_config
  workspace_q="$(quote_remote "${ROBOT_WORKSPACE}")"
  run_remote_bash \
    "printf 'SSH connected\\n'; printf 'host: '; hostname; printf 'user: '; whoami; printf 'workspace: %s\\n' ${workspace_q}; test -d ${workspace_q} && echo 'workspace status: exists' || echo 'workspace status: not created yet'"
}

command_restart_base() {
  local restart_command restart_command_q
  load_config
  restart_command='set -e; unit=robot-base.service; if systemctl is-active --quiet "${unit}"; then printf "base service is running; restarting its complete process group\n"; else printf "base service is not running; starting it\n"; fi; sudo systemctl restart "${unit}"; state=unknown; for attempt in $(seq 1 40); do state=$(systemctl is-active "${unit}" 2>/dev/null || true); if [ "${state}" = active ]; then break; fi; sleep 1; done; if [ "${state}" != active ]; then printf "error: %s did not become active (state=%s)\n" "${unit}" "${state}" >&2; systemctl --no-pager --full status "${unit}" || true; exit 1; fi; driver_pid=""; for attempt in $(seq 1 20); do driver_pid=$(pgrep -f "/turn_on_wheeltec_robot/wheeltec_robot_node([[:space:]]|$)" | head -n 1 || true); if [ -n "${driver_pid}" ]; then break; fi; sleep 1; done; if [ -z "${driver_pid}" ]; then printf "error: service is active but wheeltec_robot_node is missing\n" >&2; journalctl -u "${unit}" -n 30 --no-pager >&2 || true; exit 1; fi; printf "base service: active\nwheeltec_robot_node pid: %s\n" "${driver_pid}"'
  restart_command_q="$(quote_remote "${restart_command}")"

  # Allocate a TTY so sudo can request the robot password when its cached
  # credential has expired. systemctl restart cleanly stops the whole service
  # cgroup before starting it again, avoiding unrelated process-name kills.
  ssh -t "${SSH_ARGS[@]}" "${SSH_TARGET}" \
    "bash -lc ${restart_command_q}"
}

command_context() {
  local ros_setup_q context_command
  load_config
  ros_setup_q="$(quote_remote "${ROBOT_ROS_SETUP}")"
  context_command="if test -f ${ros_setup_q}; then source ${ros_setup_q}; fi; "
  context_command+='printf "== hostname ==\n"; hostname; printf "== user ==\n"; whoami; printf "== kernel ==\n"; uname -a; printf "== os ==\n"; cat /etc/os-release; printf "== hardware model ==\n"; tr -d "\\000" </proc/device-tree/model 2>/dev/null || true; printf "\n"; dpkg-query -W -f="nvidia-jetpack=\${Version}\n" nvidia-jetpack 2>/dev/null || true; printf "== architecture ==\n"; uname -m; printf "== ROS ==\n"; printf "ROS_DISTRO=%s\n" "${ROS_DISTRO:-unset}"; command -v ros2 || true; command -v colcon || true; printf "== tool versions ==\n"; python3 --version 2>&1 || true; cmake --version 2>&1 | head -n 1 || true; gcc --version 2>&1 | head -n 1 || true; printf "== network ==\n"; hostname -I 2>/dev/null || true; printf "== serial devices ==\n"; ls -l /dev/wheeltec_controller /dev/ttyCH343USB* /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true; printf "== disks ==\n"; df -h / 2>/dev/null || true; printf "== memory ==\n"; free -h 2>/dev/null || true'
  run_remote_bash "${context_command}"
}

command_audit_install() {
  load_config
  [[ -f "${SCRIPT_DIR}/audit_ros_install.py" ]] \
    || die "missing ${SCRIPT_DIR}/audit_ros_install.py"
  ssh "${SSH_ARGS[@]}" "${SSH_TARGET}" \
    "python3 - $(quote_remote "${ROBOT_WORKSPACE}")" \
    < "${SCRIPT_DIR}/audit_ros_install.py"
}

command_build() {
  local workspace_q ros_setup_q clean_command clean_command_q
  load_config
  workspace_q="$(quote_remote "${ROBOT_WORKSPACE}")"
  ros_setup_q="$(quote_remote "${ROBOT_ROS_SETUP}")"
  clean_command="test -f ${ros_setup_q} || { printf 'missing ROS setup: %s\\n' ${ros_setup_q} >&2; exit 1; }; source ${ros_setup_q}; cd ${workspace_q}; ${ROBOT_BUILD_COMMAND}"
  clean_command_q="$(quote_remote "${clean_command}")"
  run_remote_bash \
    "env -i HOME=$(quote_remote "/home/${ROBOT_USER}") USER=$(quote_remote "${ROBOT_USER}") LOGNAME=$(quote_remote "${ROBOT_USER}") SHELL=/bin/bash PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin LANG=C.UTF-8 LC_ALL=C.UTF-8 bash --noprofile --norc -c ${clean_command_q}"
}

command_shell() {
  local workspace_q ros_setup_q shell_command shell_command_q remote_term
  load_config
  workspace_q="$(quote_remote "${ROBOT_WORKSPACE}")"
  ros_setup_q="$(quote_remote "${ROBOT_ROS_SETUP}")"
  remote_term="${TERM:-xterm-256color}"
  shell_command="test -f ${ros_setup_q} || { printf 'missing ROS setup: %s\\n' ${ros_setup_q} >&2; exit 1; }; source ${ros_setup_q}; cd ${workspace_q}; test -f install/setup.bash || { printf 'missing install/setup.bash; run ./scripts/robot.sh build first\\n' >&2; exit 1; }; source install/setup.bash; exec bash --noprofile --norc -i"
  shell_command_q="$(quote_remote "${shell_command}")"
  ssh -t "${SSH_ARGS[@]}" "${SSH_TARGET}" \
    "env -i HOME=$(quote_remote "/home/${ROBOT_USER}") USER=$(quote_remote "${ROBOT_USER}") LOGNAME=$(quote_remote "${ROBOT_USER}") SHELL=/bin/bash TERM=$(quote_remote "${remote_term}") PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin LANG=C.UTF-8 LC_ALL=C.UTF-8 bash --noprofile --norc -c ${shell_command_q}"
}

command_exec() {
  local workspace_q command
  load_config
  [[ "$#" -gt 0 ]] || die "exec requires a command"
  workspace_q="$(quote_remote "${ROBOT_WORKSPACE}")"
  printf -v command '%q ' "$@"
  run_remote_bash "cd ${workspace_q}; ${command}"
}

command_get() {
  local relative_path="${1:-}"
  local destination="${2:-${WORKSPACE_ROOT}/robot-downloads/}"
  local transport
  load_config
  [[ -n "${relative_path}" ]] || die "get requires a path relative to the robot workspace"
  [[ "${relative_path}" != /* ]] || die "get path must be relative to ROBOT_WORKSPACE"
  [[ "${relative_path}" != '..' && "${relative_path}" != ../* && "${relative_path}" != */../* ]] \
    || die "get path cannot contain .."
  mkdir -p "${destination}"
  transport="$(rsync_transport)"
  rsync -az --human-readable --itemize-changes -e "${transport}" \
    "${SSH_TARGET}:${ROBOT_WORKSPACE}/${relative_path}" "${destination}"
}

main() {
  local command="${1:-help}"
  if [[ "$#" -gt 0 ]]; then
    shift
  fi

  case "${command}" in
    help|-h|--help) usage ;;
    doctor) command_doctor ;;
    key) command_key ;;
    check) command_check ;;
    restart-base) command_restart_base ;;
    context) command_context ;;
    audit-install) command_audit_install ;;
    shell) command_shell ;;
    exec) command_exec "$@" ;;
    get) command_get "$@" ;;
    sync-dry)
      load_config
      sync_workspace 1
      ;;
    sync)
      load_config
      sync_workspace 0
      ;;
    build) command_build ;;
    sync-build)
      load_config
      sync_workspace 0
      command_build
      ;;
    *)
      usage >&2
      die "unknown command: ${command}"
      ;;
  esac
}

main "$@"
