# Builds both Witness images and collects them in .\images:
#   flight.bin   FLIGHT image       (build\softwarev1.bin; flash at flight_a 0x20000)
#   maint.bin    MAINTENANCE image  (build_maint\softwarev1.bin; maint_a 0x420000)
#   bootloader.bin, partition-table.bin, ota_data_initial.bin
#   flash_args   combined esptool arguments for the ONE-TIME USB migration flash
# After that, both images are updated over Wi-Fi OTA (witness_ota.py).
#
#   .\build_images.ps1            # build both (incremental) and collect
#   .\build_images.ps1 -Clean     # full clean of both build dirs first
#
# The image is chosen by the WITNESS_IMAGE environment variable (unset =
# flight) and the sdkconfig overlay: sdkconfig.defaults + sdkconfig.flight /
# sdkconfig.maint, each with its own generated sdkconfig.
param([switch]$Clean)
$ErrorActionPreference = 'Stop'
. 'C:\Espressif\tools\Microsoft.v6.1.PowerShell_profile.ps1' *> $null
Set-Location $PSScriptRoot

function Invoke-Build([string]$Name, [string]$Dir, [string]$Overlay, [string]$Image) {
    Write-Host "=== $Name image ($Dir) ==="
    if ($Image) { $env:WITNESS_IMAGE = $Image } else { Remove-Item Env:WITNESS_IMAGE -ErrorAction SilentlyContinue }
    try {
        if ($Clean -and (Test-Path $Dir)) { Remove-Item -Recurse -Force $Dir }
        $defaults = "sdkconfig.defaults;$Overlay"
        $out = cmd /c "idf.py -B $Dir -D SDKCONFIG=$Dir/sdkconfig -D `"SDKCONFIG_DEFAULTS=$defaults`" build 2>&1"
        $out | Select-String 'error:|warning: |Project build complete|FAILED' | ForEach-Object { $_.Line }
        if ($LASTEXITCODE -ne 0) { throw "$Name build failed" }
    } finally { Remove-Item Env:WITNESS_IMAGE -ErrorAction SilentlyContinue }
}

Invoke-Build 'flight' 'build' 'sdkconfig.flight' ''
Invoke-Build 'maintenance' 'build_maint' 'sdkconfig.maint' 'maintenance'

New-Item -ItemType Directory -Force images | Out-Null
Copy-Item build\softwarev1.bin images\flight.bin -Force
Copy-Item build_maint\softwarev1.bin images\maint.bin -Force
Copy-Item build\bootloader\bootloader.bin images\bootloader.bin -Force
Copy-Item build\partition_table\partition-table.bin images\partition-table.bin -Force
Copy-Item build\ota_data_initial.bin images\ota_data_initial.bin -Force

# Offsets from partitions.csv: flight_a 0x20000, maint_a 0x420000.
$first = (Get-Content build\flash_args)[0]
@(
    $first,
    '0x0 bootloader.bin',
    '0x8000 partition-table.bin',
    '0x10000 ota_data_initial.bin',
    '0x20000 flight.bin',
    '0x420000 maint.bin'
) | Set-Content images\flash_args -Encoding ascii

foreach ($f in 'flight.bin', 'maint.bin') {
    $size = (Get-Item "images\$f").Length
    Write-Host ("{0,-12} {1,9} bytes" -f $f, $size)
}
Write-Host 'images\flash_args is ready for the one-time USB migration flash.'
