[CmdletBinding()]
param(
    [ValidateSet('build','flash','monitor')][string]$Action='build',
    [string]$Port='',
    [string]$IdfPath='C:\Espressif\v5.5.5\esp-idf',
    [string]$ToolsPath='C:\Espressif\tools',
    [string]$BuildDir=(Join-Path ([IO.Path]::GetTempPath()) 'esp32-smart-tour-v1')
)
$ErrorActionPreference='Stop'
$python=Join-Path $ToolsPath 'python\v5.5.5\venv\Scripts\python.exe'
$argsForIdf=@((Join-Path $PSScriptRoot 'tools\run_idf.py'),'--idf',$IdfPath,'--tools',$ToolsPath,'--build',$BuildDir,'--action',$Action)
if($Port) {$argsForIdf+=@('--port',$Port)}
& $python @argsForIdf
if($LASTEXITCODE-ne0) {throw "ESP-IDF failed: $LASTEXITCODE"}

