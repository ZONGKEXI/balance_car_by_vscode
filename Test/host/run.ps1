param([switch]$RegressionOnly)
$ErrorActionPreference = 'Stop'
$testProjectRoot = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$testBuildDirectory = Join-Path $testProjectRoot 'build/host-tests'
New-Item -ItemType Directory -Path $testBuildDirectory -Force | Out-Null
$testCompiler = (Get-Command gcc -ErrorAction Stop).Source
$testExecutable = Join-Path $testBuildDirectory 'verify_tests.exe'
$testCompilerArguments = @(
    '-std=c11', '-Wall', '-Wextra', '-Werror',
    '-I', (Join-Path $PSScriptRoot 'include'),
    '-I', (Join-Path $testProjectRoot 'Core/Inc'),
    '-I', (Join-Path $testProjectRoot 'Test'),
    (Join-Path $PSScriptRoot 'verify_tests.c'),
    (Join-Path $testProjectRoot 'Core/Src/app_pwm.c'),
    (Join-Path $testProjectRoot 'Core/Src/app_encoder.c')
)
if ($RegressionOnly) {
    $testCompilerArguments += '-DDRIVER_REGRESSION_ONLY'
} else {
    $testCompilerArguments += @(
        (Join-Path $testProjectRoot 'Test/test_voltage.c'),
        (Join-Path $testProjectRoot 'Test/test_encoder.c'),
        (Join-Path $testProjectRoot 'Test/test_pwm.c')
    )
}
$testCompilerArguments += @('-o', $testExecutable, '-lm')
& $testCompiler @testCompilerArguments
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& $testExecutable
exit $LASTEXITCODE
