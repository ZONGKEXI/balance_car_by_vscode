$ErrorActionPreference = 'Stop'
$rcProjectRoot = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$rcBuildDirectory = Join-Path $rcProjectRoot 'build/host-tests'
New-Item -ItemType Directory -Path $rcBuildDirectory -Force | Out-Null
$rcExecutable = Join-Path $rcBuildDirectory 'verify_rc.exe'
$rcArguments = @(
    '-std=c11', '-Wall', '-Wextra', '-Werror',
    '-ffunction-sections', '-fdata-sections', '-Wl,--gc-sections',
    '-I', (Join-Path $PSScriptRoot 'include'),
    '-I', (Join-Path $rcProjectRoot 'Core/Inc'),
    (Join-Path $PSScriptRoot 'verify_rc.c'),
    (Join-Path $rcProjectRoot 'Core/Src/pid.c'),
    (Join-Path $rcProjectRoot 'Core/Src/qmath.c'),
    '-o', $rcExecutable, '-lm'
)
& gcc @rcArguments
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& $rcExecutable
exit $LASTEXITCODE
