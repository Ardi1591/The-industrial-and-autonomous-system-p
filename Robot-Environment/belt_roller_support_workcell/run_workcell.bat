@echo off
REM Convenience launcher for Windows.
REM Run this from Command Prompt / PowerShell, NOT from inside Isaac Sim's Content Browser.

setlocal

REM Edit this to your Isaac Sim install folder if needed:
set ISAAC_PY=C:\Isaac-sim\python.bat

if not exist "%ISAAC_PY%" (
  echo ERROR: Could not find Isaac Sim python.bat at: %ISAAC_PY%
  echo Edit ISAAC_PY in run_workcell.bat to match your installation.
  exit /b 1
)

REM Auto-detect task.usd and assets near the project.
"%ISAAC_PY%" main.py %*
endlocal
