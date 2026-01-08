# run_in_isaac.py
# Run inside Isaac Sim GUI: Window -> Script Editor -> open this file -> Run (Ctrl+Enter)
#
# Isaac Sim Script Editor executes scripts from a TEMP copy, so normal imports like
#   from main import run
# can fail because your project folder is not on sys.path. This loader fixes that.

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path


# ======= 1) PROJECT PATH =======
# Point to your PROJECT FOLDER (the folder that contains main.py).
# You may also point directly to main.py; both are supported.
#
# IMPORTANT (Windows): prefer forward slashes (/) to avoid "\U" unicode-escape issues.
PROJECT_ROOT = "C:/Users/ardiw/OneDrive/Documents/Fusion/Robot-Environment/belt_roller_support_workcell"

# ======= 2) OPTIONAL OVERRIDES =======
# Leave these as None to let main.py auto-detect task.usd and your asset folder.
ENV_USD = "C:/Users/ardiw/OneDrive/Documents/Fusion/Robot-Environment/belt_roller_support_workcell/task.usd"
ASSET_ROOT = "C:/Users/ardiw/OneDrive/Documents/Fusion/Robot-Environment/Belt Roller Support"

# Example overrides:
# ENV_USD = "C:/.../belt_roller_support_workcell/task.usd"
# ASSET_ROOT = "C:/.../Robot-Environment/Belt Roller Support"


def load_main_module(project_root: str):
    # Accept either a folder containing main.py OR a direct path to main.py
    p = Path(project_root).expanduser().resolve()
    if p.is_file() and p.suffix.lower() == ".py":
        main_path = p
        project_dir = p.parent
    else:
        project_dir = p
        main_path = project_dir / "main.py"

    if not main_path.exists():
        raise FileNotFoundError(f"main.py not found at: {main_path}")

    # Ensure `import workcell...` works when Script Editor runs from TEMP
    if str(project_dir) not in sys.path:
        sys.path.insert(0, str(project_dir))

    # Load main.py by file path (works even when Script Editor runs from Temp)
    spec = importlib.util.spec_from_file_location("workcell_main", str(main_path))
    if spec is None or spec.loader is None:
        raise RuntimeError("Failed to create import spec for main.py")

    mod = importlib.util.module_from_spec(spec)
    sys.modules["workcell_main"] = mod
    spec.loader.exec_module(mod)
    return mod


# Load your project's main.py safely
main_mod = load_main_module(PROJECT_ROOT)

# Call run() from main.py
main_mod.run(
    env_usd=ENV_USD,
    asset_root=ASSET_ROOT,
    units=10,
    defect_rate=0.15,
    headless=False,
)




