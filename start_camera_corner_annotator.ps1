[CmdletBinding()]
param(
    [string]$Session = "",
    [string]$Calibration = "",
    [switch]$SelfTest
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$candidates = @()
$pythonCommands = Get-Command pythonw -CommandType Application -All `
    -ErrorAction SilentlyContinue
if ($pythonCommands) {
    $candidates += $pythonCommands | ForEach-Object { $_.Source }
}

if ($env:CONDA_PREFIX) {
    $candidates += Join-Path $env:CONDA_PREFIX "pythonw.exe"
}
if ($env:CONDA_EXE) {
    $condaRoot = Split-Path (Split-Path $env:CONDA_EXE -Parent) -Parent
    $candidates += Join-Path $condaRoot "pythonw.exe"
}

if ($env:IDF_PYTHON_ENV_PATH) {
    $candidates += Join-Path $env:IDF_PYTHON_ENV_PATH "Scripts\pythonw.exe"
}

$idfPythonRoot = Join-Path $env:USERPROFILE ".espressif\python_env"
if (Test-Path -LiteralPath $idfPythonRoot) {
    $candidates += Get-ChildItem -LiteralPath $idfPythonRoot -Directory |
        Sort-Object LastWriteTime -Descending |
        ForEach-Object { Join-Path $_.FullName "Scripts\pythonw.exe" }
}

$pythonw = $null
foreach ($candidate in @($candidates | Select-Object -Unique)) {
    if (-not (Test-Path -LiteralPath $candidate)) {
        continue
    }
    $python = Join-Path (Split-Path $candidate -Parent) "python.exe"
    $savedErrorActionPreference = $ErrorActionPreference
    try {
        $ErrorActionPreference = "Continue"
        $dependencyOutput = & $python -c `
            "import numpy, PIL, tkinter" 2>&1
        $dependencyExitCode = $LASTEXITCODE
    } catch {
        $dependencyExitCode = 1
    } finally {
        $ErrorActionPreference = $savedErrorActionPreference
    }
    if ($dependencyExitCode -eq 0) {
        $pythonw = $candidate
        break
    }
}

if (-not $pythonw) {
    throw "No Python environment with tkinter was found."
}

$toolPath = Join-Path $PSScriptRoot "camera_corner_annotator.py"
$arguments = @($toolPath)
if ($Session) {
    $arguments += @("--session", (Resolve-Path -LiteralPath $Session).Path)
}
if ($Calibration) {
    $arguments += @(
        "--calibration",
        (Resolve-Path -LiteralPath $Calibration).Path
    )
}
if ($SelfTest) {
    $arguments += "--self-test"
}

if ($SelfTest) {
    $python = Join-Path (Split-Path $pythonw -Parent) "python.exe"
    & $python @arguments
    exit $LASTEXITCODE
}

Start-Process -FilePath $pythonw -ArgumentList $arguments
