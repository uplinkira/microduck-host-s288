@echo off
rem Build name/title defaults are in app_config.py. Pass --name to override exe name.
rem Calibration and pose JSON files are created next to the exe at runtime.
rem (ASCII only: cmd.exe reads .bat in the OEM code page, Chinese text breaks parsing.)
cd /d "%~dp0"
python -m pip install -r requirements.txt
if errorlevel 1 exit /b 1
python build.py %*
if errorlevel 1 (
  echo BUILD FAILED
  exit /b 1
)
exit /b 0
