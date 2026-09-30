param(
    [string]$Python = 'python',
    [string]$Compiler = 'gcc',
    [string[]]$CompilerArgs = @()
)
$ErrorActionPreference = 'Stop'
$projectDir = Split-Path $PSScriptRoot -Parent
Push-Location -LiteralPath $projectDir
try {
    & $Python manual_position_terminal.py --self-test
    if ($LASTEXITCODE -ne 0) { throw 'Python tests failed' }
    & $Compiler @CompilerArgs -std=gnu89 -Wall -Wextra -Werror -IAPP/inc APP/src/bridge.c APP/src/ax12.c APP/src/dynamixel.c tests/test_bridge.c -o tests/test_bridge.exe
    if ($LASTEXITCODE -ne 0) { throw 'C host build failed' }
    & './tests/test_bridge.exe'
    if ($LASTEXITCODE -ne 0) { throw 'C tests failed' }
    & $Compiler @CompilerArgs -std=gnu89 -Wall -Wextra -Werror -Itests/fakes -IAPP/inc APP/src/arm_led.c tests/test_arm_led.c -o tests/test_arm_led.exe
    if ($LASTEXITCODE -ne 0) { throw 'LED GPIO host build failed' }
    & './tests/test_arm_led.exe'
    if ($LASTEXITCODE -ne 0) { throw 'LED GPIO tests failed' }
} finally {
    Pop-Location
}
