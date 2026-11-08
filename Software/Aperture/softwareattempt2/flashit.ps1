param(
    [string]$Port = 'rfc2217://prs2026hitl:7001',
    [int]$Baud = 460800
)
$ErrorActionPreference = 'Stop'
$python = 'C:\Espressif\tools\python\v6.1\venv\Scripts\python.exe'
$esptool = 'C:\esp\v6.1\esp-idf\components\esptool_py\esptool\esptool.py'
Push-Location $PSScriptRoot
try {
    & $python $esptool -p $Port -b $Baud --before usb-reset --after hard-reset --chip esp32s3 write_flash --flash_mode dio --flash_freq 80m --flash_size 2MB 0x0 build/bootloader/bootloader.bin 0x8000 build/partition_table/partition-table.bin 0x10000 build/softwareattempt2.bin
    if ($LASTEXITCODE -ne 0) { throw "Aperture flash failed ($LASTEXITCODE)" }
} finally {
    Pop-Location
}
