# Flash Witness through the bench Pi's RFC2217 port, but only after checking
# that the board there is a Witness: an ESP32-S3 whose installed application
# is this project ("softwarev1"; Aperture is "softwareattempt2" and Iris is an
# ESP32-C3). ser2net ports follow /dev/ttyACM enumeration, which changes when
# boards are replugged or the hub is power-cycled, so a port number alone can
# reach a different board; MACs change when a board is swapped.
#
#   .\flash_verified.ps1                                   # port 7000
#   .\flash_verified.ps1 -Port rfc2217://prs2026hitl:7002
#   .\flash_verified.ps1 -Force   # blank or unreadable app (first flash)
param(
    [string]$Port = 'rfc2217://prs2026hitl:7000',
    [string]$Project = 'softwarev1',
    [string]$Chip = 'ESP32-S3',
    [switch]$Force,
    [switch]$CheckOnly,   # identify and stop; never writes
    [string]$Python = 'C:\Espressif\tools\python\v6.1\venv\Scripts\python.exe',
    [string]$Esptool = 'C:\esp\v6.1\esp-idf\components\esptool_py\esptool\esptool.py'
)
# Native tools write warnings to stderr; judge them by exit code and output.
$ErrorActionPreference = 'Continue'
Set-Location $PSScriptRoot
$desc = Join-Path $env:TEMP 'witness_app_desc.bin'
Remove-Item $desc -ErrorAction SilentlyContinue
# The application descriptor (esp_app_desc_t) is 0x20 bytes into the app.
$out = & $Python $Esptool -p $Port --before usb-reset --after hard-reset read-flash 0x10000 0x100 $desc 2>&1 | Out-String
$chipFound = ([regex]::Match($out, 'Chip type:\s*(ESP32-\w+)')).Groups[1].Value
$mac = ([regex]::Match($out, 'MAC:\s*([0-9a-fA-F:]{17})')).Groups[1].Value
if ($chipFound -ne $Chip) {
    Write-Error "refusing to flash: $Port is a '$chipFound' (MAC $mac), expected $Chip`n$out"; exit 3
}
# PowerShell names are case-insensitive: $installedApp must not be called
# $project, or it would overwrite the -Project parameter.
$installedApp = ''
if (Test-Path $desc) {
    $bytes = [IO.File]::ReadAllBytes($desc)
    $magic = [BitConverter]::ToUInt32($bytes, 0x20)
    if ($magic -eq [uint32]'0xABCD5432') {
        $installedApp = [Text.Encoding]::ASCII.GetString($bytes, 0x20 + 48, 32).Split([char]0)[0]
    }
}
if ($installedApp -ne $Project -and -not $Force) {
    Write-Error "refusing to flash: $Port runs '$installedApp' (MAC $mac), expected '$Project'. Use -Force only for a blank Witness."; exit 4
}
if ($CheckOnly) { Write-Host "$Port is a $chipFound running '$installedApp' (MAC $mac): OK to flash"; exit 0 }
Write-Host "$Port is a $chipFound running '$installedApp' (MAC $mac); flashing Witness"
& $Python $Esptool -p $Port -b 460800 --before usb-reset --after hard-reset --chip esp32s3 write_flash `
    --flash_mode dio --flash_freq 80m --flash_size 2MB `
    0x0 build/bootloader/bootloader.bin 0x8000 build/partition_table/partition-table.bin `
    0x10000 build/softwarev1.bin
if ($LASTEXITCODE -ne 0) { Write-Error "esptool failed ($LASTEXITCODE)"; exit $LASTEXITCODE }
