param(
    [ValidateSet('build', 'flash', 'monitor', 'flash-monitor')]
    [string]$Action = 'build',
    [string]$Port = 'COM9'
)
$ErrorActionPreference = 'Stop'
. 'D:\esp\tools\Microsoft.v5.4.4.PowerShell_profile.ps1'
$env:GIT_CONFIG_COUNT = '1'
$env:GIT_CONFIG_KEY_0 = 'safe.directory'
$env:GIT_CONFIG_VALUE_0 = 'D:/esp/v5.4.4/esp-idf'
$idfArgs = @(
    '-B', 'build-wifi-debug',
    '-DSDKCONFIG=sdkconfig.local-wifi-debug',
    '-DSDKCONFIG_DEFAULTS=sdkconfig.defaults;sdkconfig.wifi-debug.defaults',
    '-DCMAKE_MAKE_PROGRAM=D:/esp/tools/ninja/1.12.1/ninja.exe',
    '-DCMAKE_PROGRAM_PATH=D:/esp/tools/xtensa-esp-elf/esp-14.2.0_20260121/xtensa-esp-elf/bin'
)
if ($Action -eq 'build') { $idfArgs += 'build' }
else {
    $idfArgs += @('-p', $Port)
    if ($Action -eq 'flash-monitor') { $idfArgs += @('flash', 'monitor') }
    else { $idfArgs += $Action }
}
Push-Location $PSScriptRoot
try {
    & 'D:\esp\tools\python\v5.4.4\venv\Scripts\python.exe' 'D:\esp\v5.4.4\esp-idf\tools\idf.py' @idfArgs
    $buildExitCode = $LASTEXITCODE
} finally { Pop-Location }
exit $buildExitCode
