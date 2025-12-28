"""Run the workcell from Isaac Sim GUI.

How to use:
  1) Open Isaac Sim GUI.
  2) Window -> Script Editor.
  3) Paste or open this file, then press Run.

This uses auto-detection: it looks for task.usd and for a folder containing your USDZ assets.
If auto-detection fails, edit ENV_USD and ASSET_ROOT below.
"""

from pathlib import Path

# Optional overrides (leave as None to auto-detect)
ENV_USD: str | None = None
ASSET_ROOT: str | None = None

# Example:
# ENV_USD = r"C:\Users\ardi\OneDrive\Documents\Fusion\Robot-Environment\task.usd"
# ASSET_ROOT = r"C:\Users\ardi\OneDrive\Documents\Fusion\Robot-Environment\Belt Roller Support"

from main import run

run(env_usd=ENV_USD, asset_root=ASSET_ROOT, units=10, defect_rate=0.15, headless=False)
