[CmdletBinding()]
param(
    [string]$Port = ""
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$candidates = @()
if ($env:IDF_PYTHON_ENV_PATH) {
    $candidates += Join-Path $env:IDF_PYTHON_ENV_PATH "Scripts\python.exe"
}

$idfPythonRoot = Join-Path $env:USERPROFILE ".espressif\python_env"
if (Test-Path -LiteralPath $idfPythonRoot) {
    $candidates += Get-ChildItem -LiteralPath $idfPythonRoot -Directory |
        Sort-Object LastWriteTime -Descending |
        ForEach-Object { Join-Path $_.FullName "Scripts\python.exe" }
}

$pythonCommand = Get-Command python -CommandType Application -ErrorAction SilentlyContinue |
    Select-Object -First 1
if ($pythonCommand) {
    $candidates += $pythonCommand.Source
}

$selectedPython = $null
foreach ($candidate in @($candidates | Select-Object -Unique)) {
    if (-not (Test-Path -LiteralPath $candidate)) {
        continue
    }
    & $candidate -c "import serial, tkinter; assert hasattr(serial, 'Serial')" 2>$null
    if ($LASTEXITCODE -eq 0) {
        $selectedPython = $candidate
        break
    }
}

if (-not $selectedPython) {
    throw "No Python environment with pyserial and tkinter was found. Activate ESP-IDF and retry."
}

$toolPath = Join-Path $PSScriptRoot "camera_threshold_tuner.py"
$toolArguments = @($toolPath)
if ($Port) {
    $toolArguments += @("--port", $Port.ToUpperInvariant())
}

Write-Host "Using Python at $selectedPython"
& $selectedPython @toolArguments
exit $LASTEXITCODE
