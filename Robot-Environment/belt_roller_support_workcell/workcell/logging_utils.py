from __future__ import annotations

import logging
import sys


def setup_logging(level: int = logging.INFO) -> None:
    """Simple console logging."""
    root = logging.getLogger()
    if root.handlers:
        return  # avoid duplicate handlers in Isaac re-runs
    root.setLevel(level)
    h = logging.StreamHandler(sys.stdout)
    fmt = logging.Formatter("[%(levelname)s] %(name)s: %(message)s")
    h.setFormatter(fmt)
    root.addHandler(h)
