#!/usr/bin/env bash
set -euo pipefail
exec env NATIVE_OPTIMIZATION_STAGE=0 "$(dirname -- "${BASH_SOURCE[0]}")/start_manual_native180.sh" "$@"
