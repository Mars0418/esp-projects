$ErrorActionPreference='Stop'
Push-Location (Join-Path $PSScriptRoot '..')
try {
    $flags=@('-std=c11','-I','tests/tour_stubs','-I','tests/host_stubs','-I','main','-Dheap_caps_free=free','-DESP_FAIL=-1')
    & gcc @flags tests/tour_corner_test.c -o tests/tour_corner_test.exe
    if($LASTEXITCODE) {throw 'corner test compilation failed'}
    & ./tests/tour_corner_test.exe
    if($LASTEXITCODE) {throw 'corner test failed'}
    & gcc @flags tests/tour_vision_test.c main/line_vision.c -lm -o tests/tour_vision_test.exe
    if($LASTEXITCODE) {throw 'vision test compilation failed'}
    & ./tests/tour_vision_test.exe
    if($LASTEXITCODE) {throw 'vision test failed'}
    & gcc @flags tests/tour_digit_test.c main/tour_guide.c main/digit_gate.c -o tests/tour_digit_test.exe
    if($LASTEXITCODE) {throw 'digit test compilation failed'}
    foreach($digit in 1,2) {
        & ./tests/tour_digit_test.exe $digit
        if($LASTEXITCODE) {throw 'digit test failed'}
    }
    & gcc @flags tests/tour_model_test.c main/digit_model.c -lm -o tests/tour_model_test.exe
    if($LASTEXITCODE) {throw 'model test compilation failed'}
    & ./tests/tour_model_test.exe
    if($LASTEXITCODE) {throw 'model test failed'}
} finally {Pop-Location}

