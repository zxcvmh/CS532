#!/usr/bin/env bash
# Runner for Standalone Cartographer 2D SLAM
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

VENV_PY="$PROJECT_ROOT/.venv/bin/python"
if [ ! -f "$VENV_PY" ]; then
    VENV_PY="python3"
fi

exec "$VENV_PY" "$SCRIPT_DIR/run_cartographer_standalone.py" "$@"
