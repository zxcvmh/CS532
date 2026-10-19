"""
Logging configuration for the JetBot Dashboard backend.

Phase 8 of the fix plan ("make the next failure provable"): the operator could not confirm
whether ``server.log`` existed, which is what made the Auto-mode session unprovable. The
backend now logs to a rotating file *by default* — no flag, no environment variable needed.

Environment overrides:
    JETBOT_LOG_DIR   directory for the rotating log (default: <repo>/logs)
    JETBOT_LOG_LEVEL level name (default: INFO)
    JETBOT_LOG_FILE  explicit file name (default: backend.log)
"""

import logging
import os
import sys
from logging.handlers import RotatingFileHandler

_CONFIGURED = False

_LOG_FORMAT = "%(asctime)s %(levelname)-7s %(name)-28s %(message)s"
_DATE_FORMAT = "%Y-%m-%d %H:%M:%S"


def _default_log_dir() -> str:
    """<repo>/logs, i.e. 03-MinhHieu/logs next to the frontend and the bridge."""
    backend_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    repo_dir = os.path.dirname(backend_dir)
    return os.path.join(repo_dir, "logs")


def configure_logging() -> str:
    """Installs a stdout handler plus a rotating file handler. Returns the log file path."""
    global _CONFIGURED
    log_dir = os.environ.get("JETBOT_LOG_DIR") or _default_log_dir()
    log_file = os.path.join(log_dir, os.environ.get("JETBOT_LOG_FILE", "backend.log"))
    level = getattr(
        logging, os.environ.get("JETBOT_LOG_LEVEL", "INFO").upper(), logging.INFO
    )

    if _CONFIGURED:
        return log_file

    formatter = logging.Formatter(_LOG_FORMAT, datefmt=_DATE_FORMAT)
    root = logging.getLogger()
    root.setLevel(level)

    try:
        os.makedirs(log_dir, exist_ok=True)
        file_handler = RotatingFileHandler(
            log_file, maxBytes=5 * 1024 * 1024, backupCount=3, encoding="utf-8"
        )
        file_handler.setFormatter(formatter)
        root.addHandler(file_handler)
    except Exception as exc:  # read-only FS, permissions, ...
        print(f"[WARN] Could not open rotating log {log_file}: {exc}", flush=True)
        log_file = ""

    # Stdout last so the console keeps its plain-text look.
    stream_handler = logging.StreamHandler(sys.stdout)
    stream_handler.setFormatter(formatter)
    root.addHandler(stream_handler)

    # uvicorn installs its own handlers; route them to the same destination.
    for name in ("uvicorn", "uvicorn.error", "uvicorn.access"):
        lg = logging.getLogger(name)
        lg.handlers = []
        lg.propagate = True

    _CONFIGURED = True
    logging.getLogger("jetbot.main").info(
        "Logging configured (file=%s, level=%s)", log_file or "<stdout only>", level
    )
    return log_file
