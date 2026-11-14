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
# TWO-IMAGE LAYOUT (partitions.csv): flight_a 0x20000, flight_b 0x220000,
# maint_a 0x420000, maint_b 0x5A0000. The flash step writes images/flash_args
# (made by build_images.ps1): bootloader, partition table, otadata, the FLIGHT
# image at flight_a and the MAINTENANCE image at maint_a. This is the one-time
# USB migration flash; afterwards both images update over Wi-Fi OTA. Falls back
# to build/flash_args (flight only) if images\ does not exist.
# The identity check reads the installed app descriptor from any app slot
# (flight_a, flight_b, maint_a, maint_b, or the old layouts' 0x320000 and
# 0x10000) and accepts either image kind (both are project "softwarev1").
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
# Candidate app offsets: the four slots above, then the old layouts.
$candidates = @(0x20000, 0x220000, 0x420000, 0x5A0000, 0x320000, 0x10000)
$chipFound = ''
$mac = ''
$installedApp = ''
$installedAt = ''
$installedKind = ''
function Read-AppDescriptor([int]$offset) {
    $desc = Join-Path $env:TEMP 'witness_app_desc.bin'
    Remove-Item $desc -ErrorAction SilentlyContinue
    $out = & $Python $Esptool -p $Port --before usb-reset --after no-reset read-flash $offset 0x140 $desc 2>&1 | Out-String
    $name = ''
    $kind = ''
    if (Test-Path $desc) {
        $bytes = [IO.File]::ReadAllBytes($desc)
        if ($bytes.Length -ge 0x20 + 48 + 32 -and [BitConverter]::ToUInt32($bytes, 0x20) -eq [uint32]'0xABCD5432') {
            $name = [Text.Encoding]::ASCII.GetString($bytes, 0x20 + 48, 32).Split([char]0)[0]
            # Image-info record at 0x120: magic "WIMG", kind byte at +5.
            if ($bytes.Length -ge 0x126 -and [BitConverter]::ToUInt32($bytes, 0x120) -eq [uint32]'0x474D4957') {
                $kind = @{ 1 = 'flight'; 2 = 'maintenance' }[[int]$bytes[0x125]]
            }
        }
    }
    return @{ Output = $out; Name = $name; Kind = $kind }
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
    if ($r.Name) { $installedApp = $r.Name; $installedKind = $r.Kind; $installedAt = ('0x{0:X}' -f $offset); break }
}
# PowerShell names are case-insensitive: $installedApp must not be called
# $project, or it would overwrite the -Project parameter.
if ($installedApp -ne $Project -and -not $Force) {
    Write-Error "refusing to flash: $Port runs '$installedApp' (MAC $mac), expected '$Project'. Use -Force only for a blank Witness."; Reset-Board; exit 4
}
if ($CheckOnly) { Reset-Board; Write-Host "$Port is a $chipFound running '$installedApp' ($installedKind image) at $installedAt (MAC $mac): OK to flash"; exit 0 }
Write-Host "$Port is a $chipFound running '$installedApp' ($installedKind image) at $installedAt (MAC $mac); flashing Witness"
# images/flash_args (build_images.ps1) or build/flash_args: the first line holds
# the esptool options (flash mode, frequency, size); each further line is
# "<offset> <file relative to that directory>".
$argsDir = if (Test-Path (Join-Path $PSScriptRoot 'images/flash_args')) { 'images' } else { 'build' }
$flashArgs = Get-Content (Join-Path $PSScriptRoot "$argsDir/flash_args")
$options = @($flashArgs[0] -split '\s+' | Where-Object { $_ })
$files = @()
foreach ($line in ($flashArgs | Select-Object -Skip 1)) {
    $parts = @($line.Trim() -split '\s+' | Where-Object { $_ })
    if ($parts.Count -eq 2) { $files += $parts[0]; $files += ("$argsDir/" + $parts[1]) }
}
if ($files.Count -lt 8) { Write-Error "$argsDir/flash_args looks incomplete; run build_images.ps1 first"; Reset-Board; exit 5 }
if ($argsDir -eq 'build') { Write-Warning 'images/flash_args not found: flashing the FLIGHT image only (no maintenance image); run build_images.ps1' }
& $Python $Esptool -p $Port -b 460800 --before usb-reset --after hard-reset --chip esp32s3 write_flash @options @files
if ($LASTEXITCODE -ne 0) { Write-Error "esptool failed ($LASTEXITCODE)"; exit $LASTEXITCODE }
