#!/usr/bin/env bash
set -euo pipefail
YOPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec bash "${YOPO_DIR}/script/run_yopo_ros2.sh" "$@"
