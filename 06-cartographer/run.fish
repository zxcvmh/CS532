#!/usr/bin/env fish
# Runner for Standalone Cartographer 2D SLAM in fish shell
set SCRIPT_DIR (status dirname)
set PROJECT_ROOT (builtin cd "$SCRIPT_DIR/.."; pwd)

set VENV_PY "$PROJECT_ROOT/.venv/bin/python"
if not test -f "$VENV_PY"
    set VENV_PY "python3"
end

exec "$VENV_PY" "$SCRIPT_DIR/run_cartographer_standalone.py" $argv
