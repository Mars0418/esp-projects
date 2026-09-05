@echo off
setlocal
call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 exit /b 1
cd /d "%~dp0.."
if not exist build-host mkdir build-host
cl /nologo /LD /O2 /std:c11 /Imain /Fobuild-host\ /Febuild-host\digit.dll main\digit_model.c main\digit_vision.c main\digit_gate.c /link /EXPORT:digit_model_predict /EXPORT:digit_model_selftest /EXPORT:digit_vision_prepare /EXPORT:digit_gate_reset /EXPORT:digit_gate_update
exit /b %errorlevel%
