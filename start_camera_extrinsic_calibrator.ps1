[CmdletBinding()]
param(
    [string]$Port = "",
    [switch]$SelfTest
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$candidates = @()
$pythonCommand = Get-Command python -CommandType Application -ErrorAction SilentlyContinue |
    Select-Object -First 1
if ($pythonCommand) {
    $candidates += $pythonCommand.Source
}

if ($env:IDF_PYTHON_ENV_PATH) {
    $candidates += Join-Path $env:IDF_PYTHON_ENV_PATH "Scripts\python.exe"
}

$idfPythonRoot = Join-Path $env:USERPROFILE ".espressif\python_env"
if (Test-Path -LiteralPath $idfPythonRoot) {
    $candidates += Get-ChildItem -LiteralPath $idfPythonRoot -Directory |
        Sort-Object LastWriteTime -Descending |
        ForEach-Object { Join-Path $_.FullName "Scripts\python.exe" }
}

$selectedPython = $null
foreach ($candidate in @($candidates | Select-Object -Unique)) {
    if (-not (Test-Path -LiteralPath $candidate)) {
        continue
    }
    & $candidate -c "import cv2, numpy, PIL, serial, tkinter; assert hasattr(serial, 'Serial')" 2>$null
    if ($LASTEXITCODE -eq 0) {
        $selectedPython = $candidate
        break
    }
}

if (-not $selectedPython) {
    throw "No Python environment with OpenCV, NumPy, Pillow, pyserial and tkinter was found."
}

$toolPath = Join-Path $PSScriptRoot "camera_extrinsic_calibrator.py"
$toolArguments = @($toolPath)
if ($Port) {
    $toolArguments += @("--port", $Port.ToUpperInvariant())
}
if ($SelfTest) {
    $toolArguments += "--self-test"
}

Write-Host "Using Python at $selectedPython"
& $selectedPython @toolArguments
exit $LASTEXITCODE
