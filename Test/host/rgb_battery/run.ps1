$ErrorActionPreference = 'Stop'
$testProjectRoot = (Resolve-Path (Join-Path $PSScriptRoot '../../..')).Path
$testBuildDirectory = Join-Path $testProjectRoot 'build/host-tests/rgb-battery'
New-Item -ItemType Directory -Path $testBuildDirectory -Force | Out-Null
$testCompiler = (Get-Command gcc -ErrorAction Stop).Source
$testExecutable = Join-Path $testBuildDirectory 'verify_battery.exe'
$testCompilerArguments = @(
    '-std=c11', '-Wall', '-Wextra', '-Werror',
    '-I', (Join-Path $PSScriptRoot 'include'),
    '-I', (Join-Path $testProjectRoot 'Core/Inc'),
    (Join-Path $PSScriptRoot 'verify_battery.c'),
    (Join-Path $testProjectRoot 'Core/Src/app_bat.c'),
    (Join-Path $testProjectRoot 'Core/Src/adc.c'),
    '-o', $testExecutable, '-lm'
)
& $testCompiler @testCompilerArguments
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& $testExecutable
exit $LASTEXITCODE
