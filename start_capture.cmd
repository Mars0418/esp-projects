@echo off
setlocal
cd /d "%~dp0"
echo Starting ESP32 telemetry capture on COM5...
"C:\Espressif\tools\python\v5.5.5\venv\Scripts\python.exe" "%~dp0capture_telemetry.py" --port COM5
echo.
echo Capture program finished. Press any key to close this window.
pause >nul
