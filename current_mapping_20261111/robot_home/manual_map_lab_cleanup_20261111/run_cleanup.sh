#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
export PYTHONDONTWRITEBYTECODE=1
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
if [[ $# -gt 0 && "$1" != --* ]]; then
    SOURCE_SESSION="$1"
    shift
    set -- --source-session "$SOURCE_SESSION" "$@"
fi
exec "${PYTHON_BIN:-/usr/bin/python3}" -B "$ROOT_DIR/offline_cleanup.py" "$@"
