@echo off
rem Launcher for the host UI (Windows, PC connected directly to the Unitree serial board)
rem   ui.bat              pose/zero UI, choose COM port in the main window
rem   ui.bat COM5         pick serial port
rem   ui.bat COM5 limit   old limit-tuning UI (rdk_linux\servo_ui.py, tkinter)
rem   ui.bat sim          offline simulation
rem (ASCII only: cmd.exe reads .bat in the OEM code page, Chinese text breaks parsing.)
cd /d "%~dp0"
python -c "import serial, PyQt6, ruckig; assert ruckig.__version__ == '0.19.4'" 2>nul || python -m pip install -r requirements.txt
if errorlevel 1 exit /b 1
if "%1"=="sim" (
  python pose_ui.py --sim
  goto :eof
)
set ARGS=
if not "%1"=="" set ARGS=--port %1
if "%2"=="limit" (
  python ..\rdk_linux\servo_ui.py %ARGS%
) else (
  python pose_ui.py %ARGS%
)
