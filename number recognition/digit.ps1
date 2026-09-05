[CmdletBinding()]
param(
    [ValidateSet('build','flash','monitor')][string]$Action='build',
    [string]$Port='',
    [string]$IdfPath='C:\Espressif\v5.5.5\esp-idf',
    [string]$ToolsPath='C:\Espressif\tools',
    [string]$BuildDir=(Join-Path ([IO.Path]::GetTempPath()) 'esp32-digit-recognition-5.5.5')
)
$ErrorActionPreference='Stop'
$env:PYTHONUTF8='1'
$env:IDF_PATH=$IdfPath
$env:IDF_TOOLS_PATH=$ToolsPath
$env:IDF_PYTHON_ENV_PATH=Join-Path $ToolsPath 'python\v5.5.5\venv'
$env:IDF_CCACHE_ENABLE='0'
$romDir=Get-Item (Join-Path $ToolsPath 'esp-rom-elfs\*') | Where-Object PSIsContainer | Sort-Object FullName -Descending | Select-Object -First 1
if($romDir) {$env:ESP_ROM_ELF_DIR=$romDir.FullName}
$gitConfigCount=0
if($env:GIT_CONFIG_COUNT) { $gitConfigCount=[int]$env:GIT_CONFIG_COUNT }
Set-Item "Env:GIT_CONFIG_KEY_$gitConfigCount" 'safe.directory'
Set-Item "Env:GIT_CONFIG_VALUE_$gitConfigCount" $IdfPath
$env:GIT_CONFIG_COUNT=[string]($gitConfigCount+1)
$env:IDF_COMPONENT_CACHE_PATH=Join-Path $PSScriptRoot '..\..\tmp\idf-component-cache'
$env:IDF_COMPONENT_LOCAL_STORAGE_URL='file://'+$ToolsPath
$toolDirs=@(
    'cmake\*\bin','ninja\*','xtensa-esp-elf\*\xtensa-esp-elf\bin',
    'esp32ulp-elf\*\esp32ulp-elf\bin','riscv32-esp-elf\*\riscv32-esp-elf\bin'
)
foreach($pattern in $toolDirs) {
    $found=Get-Item (Join-Path $ToolsPath $pattern) | Sort-Object FullName -Descending | Select-Object -First 1
    if($found) { $env:PATH=$found.FullName+';'+$env:PATH }
}
$env:PATH=(Join-Path $env:IDF_PYTHON_ENV_PATH 'Scripts')+';'+$env:PATH
$python=Join-Path $env:IDF_PYTHON_ENV_PATH 'Scripts\python.exe'
if($Action-ne'build' -and !$Port) {throw 'Specify the connected car port, e.g. -Port COM6.'}
Push-Location $PSScriptRoot
try {
    $ninja=(Get-Command ninja -CommandType Application).Source
    # Espressif's objdump launcher cannot read archive names under a Chinese path.
    # Keep build intermediates in an ASCII directory; source files stay here.
    if($BuildDir -match '[^\x00-\x7F]') {throw 'Use an ASCII-only -BuildDir for this Windows toolchain.'}
    $idfArgs=@('-B',$BuildDir,'-D',"CMAKE_MAKE_PROGRAM=$ninja")
    if($Action-eq'flash') {
        # Application only: preserve the car's bootloader, partition table and NVS.
        $idfArgs+=@('-p',$Port,'-b','460800','app-flash')
    } elseif($Action-eq'monitor') {$idfArgs+=@('-p',$Port,'monitor')}
    else {$idfArgs+='build'}
    & $python (Join-Path $IdfPath 'tools\idf.py') @idfArgs
    if($LASTEXITCODE-ne0) {throw "idf.py failed: $LASTEXITCODE"}
    if($Action -eq 'build') {
        New-Item -ItemType Directory -Force firmware | Out-Null
        foreach($name in @('digit-recognition.bin','digit-recognition.elf','digit-recognition.map')) {
            Copy-Item -LiteralPath (Join-Path $BuildDir $name) -Destination (Join-Path $PSScriptRoot 'firmware')
        }
    }
} finally {Pop-Location}
