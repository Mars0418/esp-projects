[CmdletBinding()]
param(
    [ValidateSet("build", "flash", "monitor", "run", "ports")]
    [string]$Action = "run",
    [string]$Port = ""
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$projectDir = Join-Path $PSScriptRoot "tb6612-motor-a-test"
$idfPath = "C:\Espressif\v5.5.5\esp-idf"
$idfPython = "C:\Espressif\tools\python\v5.5.5\venv\Scripts\python.exe"
$activationScript = "C:\Espressif\tools\Microsoft.v5.5.5.PowerShell_profile.ps1"
$buildDir = "build-local-v5.5.5"
$expectedPort = "COM5"
$env:PYTHONUTF8 = "1"

if (-not (Test-Path -LiteralPath (Join-Path $idfPath "tools\idf.py"))) {
    throw "ESP-IDF 5.5.5 was not found at $idfPath."
}
if (-not (Test-Path -LiteralPath $idfPython)) {
    throw "The ESP-IDF Python environment was not found at $idfPython."
}
if (-not (Test-Path -LiteralPath $activationScript)) {
    throw "The ESP-IDF activation profile was not found at $activationScript."
}

function Get-SerialPorts {
    $pythonCode = @'
import json
from serial.tools import list_ports

items = []
for port in list_ports.comports():
    # Use keyword arguments so Windows PowerShell cannot strip Python's
    # dictionary-key quotes while forwarding the native -c argument.
    items.append(dict(
        device=port.device,
        description=port.description,
        hwid=port.hwid,
        vid=port.vid,
        pid=port.pid,
    ))
print(json.dumps(items, ensure_ascii=False))
'@
    $json = & $idfPython -c $pythonCode
    if ($LASTEXITCODE -ne 0) {
        throw "Unable to enumerate serial ports."
    }
    if (-not $json -or $json.Trim() -eq "[]") {
        return
    }
    # Windows PowerShell 5.1 can preserve a JSON array as one nested object.
    # Emit each port explicitly so callers always receive flat port records.
    $parsedPorts = ConvertFrom-Json -InputObject $json
    foreach ($parsedPort in $parsedPorts) {
        Write-Output $parsedPort
    }
}

$serialPorts = @(Get-SerialPorts)
$activePortNames = @($serialPorts | ForEach-Object { $_.device })

if ($Action -eq "ports") {
    if ($serialPorts.Count -eq 0) {
        Write-Host "No active serial ports. The ESP32-S3 previously used $expectedPort, but it is disconnected."
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
        $espPorts = @($serialPorts | Where-Object {
            $_.vid -eq 0x303A -and $_.pid -eq 0x1001
        })
        if ($espPorts.Count -eq 1) {
            $Port = $espPorts[0].device
        } elseif ($expectedPort -in $activePortNames) {
            $Port = $expectedPort
        } else {
            throw "The ESP32-S3 serial port is not active. Reconnect its data USB cable; its assigned port is $expectedPort."
        }
    }
    Write-Host "Using ESP32-S3 port $Port"
}

. $activationScript
$env:IDF_CCACHE_ENABLE = "0"

Push-Location $projectDir
try {
    $idfPy = Join-Path $idfPath "tools\idf.py"
    $idfArgs = @("-B", $buildDir)
    switch ($Action) {
        "build"   { $idfArgs += "build" }
        "flash"   { $idfArgs += @("-p", $Port, "flash") }
        "monitor" { $idfArgs += @("-p", $Port, "monitor") }
        "run"     { $idfArgs += @("-p", $Port, "flash", "monitor") }
    }
    & $idfPython $idfPy @idfArgs
    if ($LASTEXITCODE -ne 0) {
        throw "idf.py failed with exit code $LASTEXITCODE."
    }
} finally {
    Pop-Location
}
