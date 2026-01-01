from __future__ import annotations

import os
from pathlib import Path
from typing import Any, Optional


def _inside_isaac_gui() -> bool:
    """
    Returns True when this code is running inside an already-running Isaac Sim GUI / Kit app
    (e.g., Script Editor). In that case, we MUST NOT create a new SimulationApp.
    """
    try:
        import omni.kit.app  # type: ignore
        app = omni.kit.app.get_app()
        return app is not None
    except Exception:
        return False


class IsaacSimApp:
    """
    Small wrapper used by main.py.

    - When running from Isaac Sim GUI (Script Editor): no-op (uses existing app).
    - When running from python.bat: creates SimulationApp and closes it on exit.

    This also guards against KeyError: 'EXP_PATH' by setting a default when needed.
    """

    def __init__(self, headless: bool = False, app_settings: Optional[dict[str, Any]] = None) -> None:
        self.headless = bool(headless)
        self.app_settings = dict(app_settings or {})
        self._created = False
        self._app = None

    def __enter__(self) -> "IsaacSimApp":
        # If Isaac Sim is already running (GUI/Kit), do NOT create a second SimulationApp.
        if _inside_isaac_gui():
            self._created = False
            self._app = None
            return self

        # Ensure EXP_PATH exists (some Isaac Sim builds read it with os.environ["EXP_PATH"])
        os.environ.setdefault("EXP_PATH", str(Path(__file__).resolve().parents[1]))

        settings = {"headless": self.headless}
        settings.update(self.app_settings)

        # Import SimulationApp (newer vs older locations)
        try:
            from isaacsim.simulation_app import SimulationApp  # type: ignore
        except Exception:
            try:
                from omni.isaac.kit import SimulationApp  # type: ignore
            except Exception as e:
                raise ImportError(
                    "Could not import SimulationApp. Run using Isaac Sim's python.bat "
                    "or run inside Isaac Sim GUI (Script Editor)."
                ) from e

        self._app = SimulationApp(settings)
        self._created = True
        return self

    def update(self) -> None:
        """Advance one frame if we created a SimulationApp."""
        if self._created and self._app is not None:
            try:
                self._app.update()
            except Exception:
                pass

    def close(self) -> None:
        """Close SimulationApp if we created it."""
        if self._created and self._app is not None:
            try:
                self._app.close()
            except Exception:
                pass
        self._created = False
        self._app = None

    def __exit__(self, exc_type, exc, tb) -> bool:
        self.close()
        # do not suppress exceptions
        return False
