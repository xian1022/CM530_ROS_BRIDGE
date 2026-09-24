param(
    [string]$Python = 'python',
    [string]$Compiler = 'gcc'
)
$ErrorActionPreference = 'Stop'
$projectDir = Split-Path $PSScriptRoot -Parent
Push-Location -LiteralPath $projectDir
try {
    & $Python manual_position_terminal.py --self-test
    if ($LASTEXITCODE -ne 0) { throw 'Python tests failed' }
    & $Compiler -std=gnu89 -Wall -Wextra -IAPP/inc APP/src/bridge.c APP/src/dynamixel.c tests/test_bridge.c -o tests/test_bridge.exe
    if ($LASTEXITCODE -ne 0) { throw 'C host build failed' }
    & './tests/test_bridge.exe'
    if ($LASTEXITCODE -ne 0) { throw 'C tests failed' }
} finally {
    Pop-Location
}
