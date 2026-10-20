param(
    [string]$SourcePath = '',
    [switch]$InitOnly
)
$ErrorActionPreference = 'Stop'
$rgbProjectRoot = (Resolve-Path (Join-Path $PSScriptRoot '../../..')).Path
if ($SourcePath -eq '') {
    $SourcePath = Join-Path $rgbProjectRoot 'Core/Src/app_rgb.c'
}
$rgbBuildDirectory = Join-Path $rgbProjectRoot 'build/host-tests/rgb-dma'
New-Item -ItemType Directory -Path $rgbBuildDirectory -Force | Out-Null
$rgbCompiler = (Get-Command gcc -ErrorAction Stop).Source
$rgbExecutable = Join-Path $rgbBuildDirectory 'verify_rgb_dma.exe'
$rgbCompilerArguments = @(
    '-std=c11', '-Wall', '-Wextra', '-Werror', '-O2',
    '-I', (Join-Path $PSScriptRoot 'include'),
    '-I', (Join-Path $rgbProjectRoot 'Core/Inc'),
    (Join-Path $PSScriptRoot 'verify_rgb_dma.c'),
    $SourcePath, '-o', $rgbExecutable
)
& $rgbCompiler @rgbCompilerArguments
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
if ($InitOnly) { & $rgbExecutable '--init-only' } else { & $rgbExecutable }
exit $LASTEXITCODE
