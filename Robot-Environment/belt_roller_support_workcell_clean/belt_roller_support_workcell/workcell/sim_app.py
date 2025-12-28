from __future__ import annotations

import logging
from contextlib import AbstractContextManager
from typing import Any, Dict, Optional

log = logging.getLogger(__name__)


class IsaacSimApp(AbstractContextManager):
    """
    Wrapper around Isaac Sim's SimulationApp.

    Compatible import strategy:
      - Isaac Sim 2022/2023: from omni.isaac.kit import SimulationApp
      - Some newer builds: from isaacsim import SimulationApp
    """

    def __init__(self, headless: bool = False, extra_config: Optional[Dict[str, Any]] = None):
        self._headless = headless
        self._extra = extra_config or {}
        self._app = None

    def __enter__(self):
        cfg = {"headless": self._headless}
        cfg.update(self._extra)
        SimulationApp = None

        try:
            from omni.isaac.kit import SimulationApp as _SimApp  # type: ignore
            SimulationApp = _SimApp
        except Exception:
            try:
                from isaacsim import SimulationApp as _SimApp  # type: ignore
                SimulationApp = _SimApp
            except Exception as e:
                raise ImportError(
                    "SimulationApp not found. Run this using Isaac Sim's python.bat / python.sh."
                ) from e

        log.info("Starting Isaac Sim (headless=%s)", self._headless)
        self._app = SimulationApp(cfg)
        return self

    def __exit__(self, exc_type, exc, tb):
        if self._app is not None:
            log.info("Closing Isaac Sim")
            self._app.close()
            self._app = None
        return False  # don't suppress exceptions

    @property
    def app(self):
        return self._app
