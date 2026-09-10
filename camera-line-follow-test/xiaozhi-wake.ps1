param(
    [ValidateSet('build', 'flash', 'monitor', 'flash-monitor')]
    [string]$Action = 'build',
    [string]$Port = 'COM9'
)
$ErrorActionPreference = 'Stop'
. 'D:\esp\tools\Microsoft.v5.4.4.PowerShell_profile.ps1'
$env:PATH = 'D:\esp\tools\python\v5.4.4\venv\Scripts;D:\esp\tools\ninja\1.12.1;D:\esp\tools\xtensa-esp-elf\esp-14.2.0_20260121\xtensa-esp-elf\bin;D:\esp\tools\cmake\3.30.2\cmake-3.30.2-windows-x86_64\bin;' + $env:PATH
$env:GIT_CONFIG_COUNT = '1'
$env:GIT_CONFIG_KEY_0 = 'safe.directory'
$env:GIT_CONFIG_VALUE_0 = 'D:/esp/v5.4.4/esp-idf'
$idfArgs = @(
    '-B', 'build-xiaozhi-wake',
    '-DSDKCONFIG=sdkconfig.local-xiaozhi-wake',
    '-DSDKCONFIG_DEFAULTS=sdkconfig.defaults;sdkconfig.wifi-debug.defaults;sdkconfig.xiaozhi-wake.defaults',
    '-DCMAKE_MAKE_PROGRAM=D:/esp/tools/ninja/1.12.1/ninja.exe',
    '-DCMAKE_PROGRAM_PATH=D:/esp/tools/xtensa-esp-elf/esp-14.2.0_20260121/xtensa-esp-elf/bin'
)
Push-Location $PSScriptRoot
try {
    if (!(Test-Path 'build-xiaozhi-wake/build.ninja')) {
        & 'D:\esp\tools\python\v5.4.4\venv\Scripts\python.exe' 'D:\esp\v5.4.4\esp-idf\tools\idf.py' @idfArgs reconfigure
        if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    }
    # Invoke Ninja directly so nested model/bootloader tools inherit this PATH.
    if ($Action -ne 'monitor') {
        & 'D:\esp\tools\ninja\1.12.1\ninja.exe' -C build-xiaozhi-wake all
        if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    }
    $buildExitCode = 0
    if ($Action -ne 'build') {
        $idfArgs += @('-p', $Port)
        if ($Action -eq 'flash-monitor') { $idfArgs += @('flash', 'monitor') }
        else { $idfArgs += $Action }
        & 'D:\esp\tools\python\v5.4.4\venv\Scripts\python.exe' 'D:\esp\v5.4.4\esp-idf\tools\idf.py' @idfArgs
        $buildExitCode = $LASTEXITCODE
    }
} finally { Pop-Location }
exit $buildExitCode
