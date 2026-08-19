$ErrorActionPreference = 'Stop'

$testRoot = Split-Path -Parent $PSCommandPath
$userRoot = Split-Path -Parent $testRoot
$output = Join-Path $env:TEMP 'map_check_ifgetvisionloc_test.exe'

$gccArgs = @(
    '-std=c11', '-Wall', '-Wextra', '-fno-omit-frame-pointer',
    '-ffunction-sections', '-fdata-sections', '-Wl,--gc-sections',
    '-I', (Join-Path $testRoot 'mocks'),
    '-I', (Join-Path $userRoot 'algorithm_inc'),
    '-I', (Join-Path $userRoot 'inc'),
    (Join-Path $testRoot 'map_check_ifgetvisionloc_test.c'),
    (Join-Path $userRoot 'algorithm_src\sokoban_engine.c'),
    '-o', $output
)

$asan = & gcc -print-file-name=libasan.a
$ubsan = & gcc -print-file-name=libubsan.a
if ((Test-Path $asan) -and (Test-Path $ubsan))
{
    $gccArgs += '-fsanitize=address,undefined'
}
else
{
    Write-Warning 'AddressSanitizer/UndefinedBehaviorSanitizer runtime unavailable; running boundary assertions without sanitizers.'
}

& gcc @gccArgs
if ($LASTEXITCODE -ne 0)
{
    exit $LASTEXITCODE
}

& $output
exit $LASTEXITCODE
