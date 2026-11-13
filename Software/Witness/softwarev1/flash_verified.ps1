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
#
# The flash layout comes from build/flash_args (bootloader, partition table,
# ota_data_initial, app at ota_0), so it follows partitions.csv and the flash
# size in sdkconfig. The identity check reads the installed app descriptor
# from ota_0, then ota_1 (after a Wi-Fi OTA the newest image may be in either),
# then 0x10000 (the old 2 MB single-app layout, before the one-time USB
# migration flash).
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
# The application descriptor (esp_app_desc_t) is 0x20 bytes into the app.
# Candidate app offsets: ota_0, ota_1 (partitions.csv), old single-app layout.
$candidates = @(0x20000, 0x320000, 0x10000)
$chipFound = ''
$mac = ''
$installedApp = ''
$installedAt = ''
function Read-AppDescriptor([int]$offset) {
    $desc = Join-Path $env:TEMP 'witness_app_desc.bin'
    Remove-Item $desc -ErrorAction SilentlyContinue
    $out = & $Python $Esptool -p $Port --before usb-reset --after no-reset read-flash $offset 0x100 $desc 2>&1 | Out-String
    $name = ''
    if (Test-Path $desc) {
        $bytes = [IO.File]::ReadAllBytes($desc)
        if ($bytes.Length -ge 0x20 + 48 + 32 -and [BitConverter]::ToUInt32($bytes, 0x20) -eq [uint32]'0xABCD5432') {
            $name = [Text.Encoding]::ASCII.GetString($bytes, 0x20 + 48, 32).Split([char]0)[0]
        }
    }
    return @{ Output = $out; Name = $name }
}
function Reset-Board {
    & $Python $Esptool -p $Port --before no-reset --after hard-reset chip-id 2>&1 | Out-Null
}
foreach ($offset in $candidates) {
    $r = Read-AppDescriptor $offset
    if (-not $chipFound) {
        $chipFound = ([regex]::Match($r.Output, 'Chip type:\s*(ESP32-\w+)')).Groups[1].Value
        $mac = ([regex]::Match($r.Output, 'MAC:\s*([0-9a-fA-F:]{17})')).Groups[1].Value
        if ($chipFound -ne $Chip) {
            Write-Error "refusing to flash: $Port is a '$chipFound' (MAC $mac), expected $Chip`n$($r.Output)"; Reset-Board; exit 3
        }
    }
    if ($r.Name) { $installedApp = $r.Name; $installedAt = ('0x{0:X}' -f $offset); break }
}
# PowerShell names are case-insensitive: $installedApp must not be called
# $project, or it would overwrite the -Project parameter.
if ($installedApp -ne $Project -and -not $Force) {
    Write-Error "refusing to flash: $Port runs '$installedApp' (MAC $mac), expected '$Project'. Use -Force only for a blank Witness."; Reset-Board; exit 4
}
if ($CheckOnly) { Reset-Board; Write-Host "$Port is a $chipFound running '$installedApp' at $installedAt (MAC $mac): OK to flash"; exit 0 }
Write-Host "$Port is a $chipFound running '$installedApp' at $installedAt (MAC $mac); flashing Witness"
# build/flash_args: the first line holds the esptool options (flash mode,
# frequency, size); each further line is "<offset> <file relative to build/>".
$flashArgs = Get-Content (Join-Path $PSScriptRoot 'build/flash_args')
$options = @($flashArgs[0] -split '\s+' | Where-Object { $_ })
$files = @()
foreach ($line in ($flashArgs | Select-Object -Skip 1)) {
    $parts = @($line.Trim() -split '\s+' | Where-Object { $_ })
    if ($parts.Count -eq 2) { $files += $parts[0]; $files += ('build/' + $parts[1]) }
}
if ($files.Count -lt 8) { Write-Error "build/flash_args looks incomplete; run idf.py build first"; Reset-Board; exit 5 }
& $Python $Esptool -p $Port -b 460800 --before usb-reset --after hard-reset --chip esp32s3 write_flash @options @files
if ($LASTEXITCODE -ne 0) { Write-Error "esptool failed ($LASTEXITCODE)"; exit $LASTEXITCODE }
