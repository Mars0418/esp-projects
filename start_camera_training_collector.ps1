[CmdletBinding()]
param(
    [string]$Port = "",
    [string]$Output = "camera-datasets",
    [ValidateRange(0, 2147483647)]
    [int]$Count = 0
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$candidates = @()
$pythonCommands = Get-Command python -CommandType Application -All `
    -ErrorAction SilentlyContinue
if ($pythonCommands) {
    $candidates += $pythonCommands | ForEach-Object { $_.Source }
}
if ($env:CONDA_PREFIX) {
    $candidates += Join-Path $env:CONDA_PREFIX "python.exe"
}
if ($env:CONDA_EXE) {
    $condaRoot = Split-Path (Split-Path $env:CONDA_EXE -Parent) -Parent
    $candidates += Join-Path $condaRoot "python.exe"
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
    $savedErrorActionPreference = $ErrorActionPreference
    try {
        $ErrorActionPreference = "Continue"
        $dependencyOutput = & $candidate -c `
            "import serial; assert hasattr(serial, 'Serial')" 2>&1
        $dependencyExitCode = $LASTEXITCODE
    } catch {
        $dependencyExitCode = 1
    } finally {
        $ErrorActionPreference = $savedErrorActionPreference
    }
    if ($dependencyExitCode -eq 0) {
        $selectedPython = $candidate
        break
    }
}

if (-not $selectedPython) {
    throw "No Python environment with pyserial was found."
}

$toolPath = Join-Path $PSScriptRoot "camera_training_collector.py"
$arguments = @($toolPath, "--output", $Output, "--count", $Count)
if ($Port) {
    $arguments += @("--port", $Port.ToUpperInvariant())
}

Write-Host "Using Python at $selectedPython"
& $selectedPython @arguments
exit $LASTEXITCODE
