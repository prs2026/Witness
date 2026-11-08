param(
    [string]$Port = 'rfc2217://prs2026hitl:7001',
    [string]$Python = 'C:\Espressif\tools\python\v6.1\venv\Scripts\python.exe'
)
$ErrorActionPreference = 'Stop'
Push-Location $PSScriptRoot
try {
    & $Python -m esptool --port $Port --baud 460800 --before usb-reset --after hard-reset --chip esp32c3 write-flash --flash-mode dio --flash-freq 80m --flash-size 4MB 0x0 build/bootloader/bootloader.bin 0x8000 build/partition_table/partition-table.bin 0x10000 build/softwarev1.bin
    if ($LASTEXITCODE -ne 0) { throw "Iris flashing failed: exit $LASTEXITCODE" }
} finally {
    Pop-Location
}
