@echo off
rem Frees a HackRF held by a stuck hackrf_transfer: ends the process, then
rem restarts the HackRF's USB device (cancels its pending transfers).
rem Needs administrator rights: right-click > Run as administrator, or run
rem from an elevated prompt.
net session >nul 2>&1 || (echo Run this as administrator. & exit /b 1)
taskkill /F /IM hackrf_transfer.exe 2>nul
for /f "tokens=*" %%i in ('powershell -NoProfile -Command "(Get-CimInstance Win32_PnPEntity | Where-Object { $_.DeviceID -match 'VID_1D50&PID_60' } | Select-Object -First 1).DeviceID"') do set HACKRF=%%i
if "%HACKRF%"=="" (echo No HackRF found. & exit /b 2)
echo Restarting %HACKRF%
pnputil /restart-device "%HACKRF%"
timeout /t 3 >nul
"%~dp0bin\hackrf_info.exe" | findstr /C:"Found HackRF" /C:"failed"
