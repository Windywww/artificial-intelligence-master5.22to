$ErrorActionPreference = 'Stop'

$testRoot = Split-Path -Parent $PSCommandPath
$repoRoot = Split-Path -Parent $testRoot
$userRoot = Join-Path $repoRoot 'artificial-intelligence-vision-master\SeekFree_RT1064_Opensource_Library\project\user'
$output = Join-Path $env:TEMP 'sokoban_engine_host_test.exe'

$gccArgs = @(
    '-std=c11', '-Wall', '-Wextra', '-Werror', '-fno-omit-frame-pointer',
    '-ffunction-sections', '-fdata-sections', '-Wl,--gc-sections',
    '-DSOKOBAN_ENGINE_TEST',
    '-I', (Join-Path $testRoot 'mocks'),
    '-I', (Join-Path $userRoot 'algorithm_inc'),
    '-I', (Join-Path $userRoot 'inc'),
    (Join-Path $testRoot 'sokoban_engine_test.c'),
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
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& $output
exit $LASTEXITCODE
