[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$Dataset,
    [string]$Output = "",
    [ValidateSet("pooled_mlp", "tiny_cnn")]
    [string]$Architecture = "tiny_cnn",
    [ValidateRange(1, 100000)]
    [int]$Epochs = 600
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

$selectedPython = $null
foreach ($candidate in @($candidates | Select-Object -Unique)) {
    if (-not (Test-Path -LiteralPath $candidate)) {
        continue
    }
    $savedErrorActionPreference = $ErrorActionPreference
    try {
        $ErrorActionPreference = "Continue"
        $dependencyOutput = & $candidate -c "import numpy, torch" 2>&1
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
    throw "No Python environment with NumPy and PyTorch was found."
}

$arguments = @(
    (Join-Path $PSScriptRoot "camera_model_trainer.py"),
    "--dataset", (Resolve-Path -LiteralPath $Dataset).Path,
    "--epochs", $Epochs,
    "--architecture", $Architecture
)
if ($Output) {
    $arguments += @("--output", $Output)
}

Write-Host "Using Python at $selectedPython"
& $selectedPython @arguments
exit $LASTEXITCODE
