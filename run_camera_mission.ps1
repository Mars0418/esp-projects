[CmdletBinding()]
param(
    [string]$Port = "",
    [string]$IdfPath = "",
    [switch]$NoFlash
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"
$env:PYTHONUTF8 = "1"

function Test-IdfPath {
    param([string]$Path)
    return $Path -and (Test-Path -LiteralPath (Join-Path $Path "tools\idf.py"))
}

function Resolve-IdfPath {
    param([string]$RequestedPath)
    if ($RequestedPath) {
        if (-not (Test-IdfPath $RequestedPath)) {
            throw "ESP-IDF was not found at '$RequestedPath'."
        }
        return (Resolve-Path -LiteralPath $RequestedPath).Path
    }
    if (Test-IdfPath $env:IDF_PATH) {
        return (Resolve-Path -LiteralPath $env:IDF_PATH).Path
    }
    $preferred = "C:\esp\v6.1-beta1\esp-idf"
    if (Test-IdfPath $preferred) {
        return $preferred
    }
    $candidates = @(Get-ChildItem -Path "C:\esp\v*\esp-idf" -Directory `
        -ErrorAction SilentlyContinue | Sort-Object LastWriteTime -Descending)
    if ($candidates.Count -eq 0) {
        throw "No ESP-IDF installation was found. Pass -IdfPath <path>."
    }
    return $candidates[0].FullName
}

function Get-SerialPorts {
    foreach ($entity in @(Get-CimInstance Win32_PnPEntity -ErrorAction Stop |
            Where-Object { $_.Name -match "\(COM\d+\)" })) {
        if ([string]$entity.Name -match "\((COM\d+)\)") {
            [pscustomobject]@{
                Device = $Matches[1].ToUpperInvariant()
                Name = [string]$entity.Name
                HardwareId = [string]$entity.PNPDeviceID
            }
        }
    }
}

$ports = @(Get-SerialPorts)
if ($Port) {
    $Port = $Port.ToUpperInvariant()
    if ($Port -notin @($ports.Device)) {
        throw "$Port is not currently available."
    }
} else {
    $preferredPorts = @($ports | Where-Object {
        $_.HardwareId -match "VID_10C4" -or
        $_.HardwareId -match "VID_303A" -or
        $_.Name -match "CP210|USB.*UART|USB.*Serial"
    })
    if ($preferredPorts.Count -eq 1) {
        $Port = $preferredPorts[0].Device
    } elseif ($ports.Count -eq 1) {
        $Port = $ports[0].Device
    } else {
        $available = if ($ports.Count) { $ports.Device -join ", " } else { "none" }
        throw "Unable to select one ESP32 serial port (available: $available). Pass -Port COMx."
    }
}

Write-Host "Using ESP32 serial port $Port"
if (-not $NoFlash) {
    $resolvedIdfPath = Resolve-IdfPath $IdfPath
    $exportScript = Join-Path $resolvedIdfPath "export.ps1"
    Write-Host "Using ESP-IDF at $resolvedIdfPath"
    . $exportScript

    $idfPy = Join-Path $resolvedIdfPath "tools\idf.py"
    $idfEnvironmentPython = if ($env:IDF_PYTHON_ENV_PATH) {
        Join-Path $env:IDF_PYTHON_ENV_PATH "Scripts\python.exe"
    } else {
        ""
    }
    if ($idfEnvironmentPython -and
        (Test-Path -LiteralPath $idfEnvironmentPython)) {
        $python = $idfEnvironmentPython
    } else {
        $python = (Get-Command python -CommandType Application -ErrorAction Stop |
            Select-Object -First 1).Source
    }

    $versionOutput = @(& $python $idfPy --version 2>&1)
    $versionExitCode = $LASTEXITCODE
    $versionLine = @($versionOutput | ForEach-Object { [string]$_ } |
        Where-Object { $_ -match "^ESP-IDF\s+v?" } |
        Select-Object -First 1)
    if ($versionExitCode -eq 0 -and $versionLine.Count -eq 1) {
        $version = (($versionLine[0]).Trim() -replace "^ESP-IDF\s+v?", "") `
            -replace "[^A-Za-z0-9._-]", "-"
        $buildDirectory = "build-local-$version"
    } else {
        $buildDirectory = "build-local"
        Write-Warning "ESP-IDF version text was unavailable; using $buildDirectory."
    }
    $project = Join-Path $PSScriptRoot "camera-line-follow-test"
    Push-Location $project
    try {
        & $python $idfPy -B $buildDirectory -p $Port flash
        if ($LASTEXITCODE -ne 0) {
            throw "Firmware flash failed with exit code $LASTEXITCODE."
        }
    } finally {
        Pop-Location
    }
}

Write-Host "Opening integrated image and telemetry debugger"
& (Join-Path $PSScriptRoot "start_push_debug_viewer.ps1") `
    -Port $Port -AutoConnect
