[CmdletBinding()]
param(
    [ValidateSet("build", "flash", "monitor", "run", "ports")]
    [string]$Action = "run",
    [string]$Port = "",
    [string]$IdfPath = ""
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$projectDir = Join-Path $PSScriptRoot "tb6612-motor-a-test"
$preferredIdfVersion = "5.5.5"
$env:PYTHONUTF8 = "1"

function Test-IdfPath {
    param([string]$Path)

    return $Path -and (Test-Path -LiteralPath (Join-Path $Path "tools\idf.py"))
}

function Resolve-IdfPath {
    param([string]$RequestedPath)

    if ($RequestedPath) {
        if (-not (Test-IdfPath $RequestedPath)) {
            throw "ESP-IDF was not found at '$RequestedPath'. Expected tools\idf.py below that directory."
        }
        return (Resolve-Path -LiteralPath $RequestedPath).Path
    }

    if (Test-IdfPath $env:IDF_PATH) {
        return (Resolve-Path -LiteralPath $env:IDF_PATH).Path
    }

    $searchPatterns = @(
        "C:\Espressif\frameworks\esp-idf-v*",
        "C:\Espressif\v*\esp-idf",
        "C:\esp\v*\esp-idf",
        (Join-Path $env:USERPROFILE ".espressif\frameworks\esp-idf-v*"),
        (Join-Path $env:USERPROFILE "esp\v*\esp-idf")
    )

    $validPaths = @(@(
        foreach ($pattern in $searchPatterns) {
            foreach ($candidate in @(Get-Item -Path $pattern -ErrorAction SilentlyContinue)) {
                if (Test-IdfPath $candidate.FullName) {
                    $candidate.FullName
                }
            }
        }
    ) | Select-Object -Unique)

    if ($validPaths.Count -eq 0) {
        throw "No ESP-IDF installation was found. Activate ESP-IDF first or pass -IdfPath <path>."
    }
    if ($validPaths.Count -eq 1) {
        return $validPaths[0]
    }

    $escapedPreferredVersion = [regex]::Escape($preferredIdfVersion)
    $preferredPaths = @($validPaths | Where-Object {
        $_ -match "[\\/]v?$escapedPreferredVersion([\\/]|$)" -or
        $_ -match "esp-idf-v$escapedPreferredVersion$"
    })
    if ($preferredPaths.Count -eq 1) {
        return $preferredPaths[0]
    }

    $formattedPaths = $validPaths -join "`n  "
    throw "Multiple ESP-IDF installations were found. Select one with -IdfPath:`n  $formattedPaths"
}

function Get-SerialPorts {
    $portEntities = @(Get-CimInstance -ClassName Win32_PnPEntity -ErrorAction Stop |
        Where-Object { $_.Name -match "\(COM\d+\)" })

    foreach ($entity in $portEntities) {
        if ([string]$entity.Name -notmatch "\((COM\d+)\)") {
            continue
        }
        $device = $Matches[1].ToUpperInvariant()
        $hardwareId = [string]$entity.PNPDeviceID
        $vid = $null
        $usbPid = $null

        if ($hardwareId -match "VID_([0-9A-Fa-f]{4})") {
            $vid = [Convert]::ToInt32($Matches[1], 16)
        }
        if ($hardwareId -match "PID_([0-9A-Fa-f]{4})") {
            $usbPid = [Convert]::ToInt32($Matches[1], 16)
        }

        [pscustomobject]@{
            device      = $device
            description = [string]$entity.Name
            vid         = $vid
            pid         = $usbPid
            hwid        = $hardwareId
        }
    }
}

$serialPorts = @(Get-SerialPorts | Sort-Object {
    [int]($_.device -replace "\D", "")
})
$activePortNames = @($serialPorts | ForEach-Object { $_.device })

if ($Action -eq "ports") {
    if ($serialPorts.Count -eq 0) {
        Write-Host "No active serial ports. Connect the ESP32-S3 with a USB data cable."
        exit 0
    }
    $serialPorts | Select-Object device, description, vid, pid, hwid | Format-Table -AutoSize
    exit 0
}

if ($Action -ne "build") {
    if ($Port) {
        $Port = $Port.ToUpperInvariant()
        if ($Port -notin $activePortNames) {
            throw "$Port is not active. Reconnect the ESP32-S3 and run '.\car.ps1 ports'."
        }
    } else {
        $espPorts = @($serialPorts | Where-Object { $_.vid -eq 0x303A })
        if ($espPorts.Count -eq 1) {
            $Port = $espPorts[0].device
        } elseif ($espPorts.Count -gt 1) {
            $portList = $espPorts.device -join ", "
            throw "Multiple Espressif serial ports were found ($portList). Select one with -Port COMx."
        } else {
            throw "No Espressif serial port was found. Connect the ESP32-S3 and run '.\car.ps1 ports'."
        }
    }
    Write-Host "Using ESP32-S3 port $Port"
}

$resolvedIdfPath = Resolve-IdfPath $IdfPath
$exportScript = Join-Path $resolvedIdfPath "export.ps1"
if (-not (Test-Path -LiteralPath $exportScript)) {
    throw "ESP-IDF activation script was not found at '$exportScript'."
}

Write-Host "Using ESP-IDF at $resolvedIdfPath"
. $exportScript
$env:IDF_CCACHE_ENABLE = "0"

$idfPy = Join-Path $resolvedIdfPath "tools\idf.py"
$pythonCommand = Get-Command python -CommandType Application -ErrorAction Stop | Select-Object -First 1
$idfVersionLine = (& $pythonCommand.Source $idfPy --version | Select-Object -First 1).Trim()
if ($LASTEXITCODE -ne 0 -or -not $idfVersionLine) {
    throw "Unable to determine the ESP-IDF version."
}
$idfVersion = $idfVersionLine -replace "^ESP-IDF\s+v?", ""
$buildVersion = $idfVersion -replace "[^A-Za-z0-9._-]", "-"
$buildDir = "build-local-$buildVersion"
$sharedSdkconfig = Join-Path $projectDir "sdkconfig"
$localSdkconfig = Join-Path $projectDir "sdkconfig.local-$buildVersion"

if (-not (Test-Path -LiteralPath $localSdkconfig)) {
    Copy-Item -LiteralPath $sharedSdkconfig -Destination $localSdkconfig
}

Write-Host "Using $idfVersionLine with build directory $buildDir"
Write-Host "Using local project configuration $localSdkconfig"

Push-Location $projectDir
try {
    $idfArgs = @("-D", "SDKCONFIG=$localSdkconfig", "-B", $buildDir)
    switch ($Action) {
        "build"   { $idfArgs += "build" }
        "flash"   { $idfArgs += @("-p", $Port, "flash") }
        "monitor" { $idfArgs += @("-p", $Port, "monitor") }
        "run"     { $idfArgs += @("-p", $Port, "flash", "monitor") }
    }
    & $pythonCommand.Source $idfPy @idfArgs
    if ($LASTEXITCODE -ne 0) {
        throw "idf.py failed with exit code $LASTEXITCODE."
    }
} finally {
    Pop-Location
}
