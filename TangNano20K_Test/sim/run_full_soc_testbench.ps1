$ErrorActionPreference = 'Stop'

$projectRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$iverilog = 'C:\msys64\mingw64\bin\iverilog.exe'
$vvp = 'C:\msys64\mingw64\bin\vvp.exe'

if (!(Test-Path -LiteralPath $iverilog) -or !(Test-Path -LiteralPath $vvp)) {
    throw 'Icarus Verilog was not found in C:\msys64\mingw64\bin.'
}

# Icarus/MSYS cannot reliably open the Unicode project path, so copy the exact
# simulation inputs to a temporary ASCII-only working directory.
$tempBase = Join-Path $env:LOCALAPPDATA 'Temp'
$work = Join-Path $tempBase ('picorv32_ld2450_soc_' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $work | Out-Null

$sources = @(
    'sim\picorv32_ld2450_soc_tb.v',
    'src\countdown_timer.v',
    'src\edge_finder.v',
    'src\i2c.v',
    'src\i2c_mmio.v',
    'src\ld2450_uart.v',
    'src\leds.v',
    'src\picorv32.v',
    'src\reset.v',
    'src\sd_spi_helper.v',
    'src\simpleuart.v',
    'src\sram.v',
    'src\sram8bit.v',
    'src\top.v',
    'src\uart_wrap.v',
    'src\ws2812b.v',
    'src\ws2812b_tgt.v',
    'src\global_defs.v',
    'src\sys_parameters.v',
    'src\mem_init0.ini',
    'src\mem_init1.ini',
    'src\mem_init2.ini',
    'src\mem_init3.ini'
)

try {
    foreach ($relative in $sources) {
        $source = Join-Path $projectRoot $relative
        Copy-Item -LiteralPath $source -Destination (Join-Path $work (Split-Path $relative -Leaf))
    }

    Push-Location $work
    try {
        $rtl = @(
            'picorv32_ld2450_soc_tb.v',
            'countdown_timer.v', 'edge_finder.v', 'i2c.v', 'i2c_mmio.v',
            'ld2450_uart.v', 'leds.v', 'picorv32.v', 'reset.v',
            'sd_spi_helper.v', 'simpleuart.v', 'sram.v', 'sram8bit.v',
            'top.v', 'uart_wrap.v', 'ws2812b.v', 'ws2812b_tgt.v'
        )

        & $iverilog -g2012 -Wall -I. -s picorv32_ld2450_soc_tb -o soc_tb.vvp @rtl
        if ($LASTEXITCODE -ne 0) {
            throw "Icarus compilation failed with exit code $LASTEXITCODE."
        }

        & $vvp .\soc_tb.vvp
        if ($LASTEXITCODE -ne 0) {
            throw "SoC simulation failed with exit code $LASTEXITCODE."
        }
    } finally {
        Pop-Location
    }
} finally {
    if (Test-Path -LiteralPath $work) {
        Remove-Item -LiteralPath $work -Recurse -Force
    }
}
