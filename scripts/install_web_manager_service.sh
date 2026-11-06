#!/usr/bin/env bash

set -euo pipefail

readonly SERVICE_NAME="web-manager.service"
readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly SERVICE_SOURCE="${WORKSPACE_ROOT}/systemd/${SERVICE_NAME}"
readonly SERVICE_TARGET="/etc/systemd/system/${SERVICE_NAME}"
readonly WEB_MANAGER_START="/home/wheeltec/web_manager_ws/start_web_manager.sh"

if [[ "${EUID}" -ne 0 ]]; then
  echo "请使用 sudo 运行：sudo ${BASH_SOURCE[0]}" >&2
  exit 1
fi

if [[ ! -x "${WEB_MANAGER_START}" ]]; then
  echo "Web 管理器启动脚本不存在或不可执行：${WEB_MANAGER_START}" >&2
  exit 1
fi

install -m 0644 "${SERVICE_SOURCE}" "${SERVICE_TARGET}"
systemctl daemon-reload
systemctl enable --now "${SERVICE_NAME}"

echo "${SERVICE_NAME} 已安装并启用。"
systemctl --no-pager --full status "${SERVICE_NAME}"
