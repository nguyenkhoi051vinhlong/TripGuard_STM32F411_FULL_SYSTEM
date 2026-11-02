param(
    [string] $BuildRoot = (Join-Path ([System.IO.Path]::GetTempPath()) 'tripguard_esp32_rear_pod')
)

$ErrorActionPreference = 'Stop'

if ([string]::IsNullOrWhiteSpace($env:IDF_PATH)) {
    throw 'Activate the ESP-IDF PowerShell environment first; IDF_PATH is not set.'
}

$idfScript = Join-Path $env:IDF_PATH 'tools\idf.py'
if (-not (Test-Path -LiteralPath $idfScript)) {
    throw "idf.py was not found under IDF_PATH: $env:IDF_PATH"
}

New-Item -ItemType Directory -Force -Path $BuildRoot | Out-Null
foreach ($item in @('CMakeLists.txt', 'sdkconfig.defaults', 'main')) {
    Copy-Item -LiteralPath (Join-Path $PSScriptRoot $item) -Destination $BuildRoot -Recurse -Force
}

& python $idfScript -C $BuildRoot build
if ($LASTEXITCODE -ne 0) {
    throw 'Firmware build failed.'
}

$artifacts = Join-Path $PSScriptRoot 'firmware'
New-Item -ItemType Directory -Force -Path (Join-Path $artifacts 'bootloader'), (Join-Path $artifacts 'partition_table') | Out-Null
Copy-Item -LiteralPath (Join-Path $BuildRoot 'build\tripguard_rear_pod.bin'), (Join-Path $BuildRoot 'build\flasher_args.json') -Destination $artifacts -Force
Copy-Item -LiteralPath (Join-Path $BuildRoot 'build\bootloader\bootloader.bin') -Destination (Join-Path $artifacts 'bootloader') -Force
Copy-Item -LiteralPath (Join-Path $BuildRoot 'build\partition_table\partition-table.bin') -Destination (Join-Path $artifacts 'partition_table') -Force
Copy-Item -LiteralPath (Join-Path $BuildRoot 'sdkconfig') -Destination (Join-Path $artifacts 'sdkconfig.build') -Force
Write-Output "Build complete: $BuildRoot\build"
