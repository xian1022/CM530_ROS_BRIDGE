param(
    [string]$Port = "COM4",
    [int]$Baud = 57600
)

$ErrorActionPreference = "Stop"

function Read-Cm530Serial {
    param(
        [System.IO.Ports.SerialPort]$SerialPort,
        [double]$Seconds = 3.0
    )

    $deadline = (Get-Date).AddSeconds($Seconds)
    $builder = New-Object System.Text.StringBuilder

    while ((Get-Date) -lt $deadline) {
        $count = $SerialPort.BytesToRead
        if ($count -gt 0) {
            $bytes = New-Object byte[] $count
            [void]$SerialPort.Read($bytes, 0, $count)
            [void]$builder.Append([System.Text.Encoding]::ASCII.GetString($bytes))
        } else {
            Start-Sleep -Milliseconds 20
        }
    }

    $text = $builder.ToString()
    if ($text.Length -eq 0) {
        Write-Host "RX <- (none)"
        return
    }

    $lines = ($text -replace "`r", "`n") -split "`n"
    foreach ($line in $lines) {
        if ($line.Length -gt 0) {
            Write-Host "RX <- $line"
        }
    }
}

function Send-AsciiBytes {
    param(
        [System.IO.Ports.SerialPort]$SerialPort,
        [byte[]]$Bytes,
        [double]$CharDelay = 0.05
    )

    Write-Host ("TX raw -> " + [System.BitConverter]::ToString($Bytes))
    foreach ($b in $Bytes) {
        $one = [byte[]]@($b)
        $SerialPort.BaseStream.Write($one, 0, 1)
        $SerialPort.BaseStream.Flush()
        if ($CharDelay -gt 0) {
            Start-Sleep -Milliseconds ([int]($CharDelay * 1000))
        }
    }
}

Write-Host "CM-530 .NET SerialPort TX Probe"
Write-Host "Port       : $Port"
Write-Host "Baud       : $Baud"
Write-Host "Target FW  : CM530_sdk_serial_probe.hex"
Write-Host ""
Write-Host "Close RoboPlus / Python serial windows before continuing."
Write-Host "Flash CM530_sdk_serial_probe.hex first, then press Enter here."
[void](Read-Host)

$sp = [System.IO.Ports.SerialPort]::new($Port, $Baud, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
$sp.Handshake = [System.IO.Ports.Handshake]::None
$sp.ReadTimeout = 100
$sp.WriteTimeout = 2000
$sp.DtrEnable = $false
$sp.RtsEnable = $false

try {
    $sp.Open()
    $sp.DiscardInBuffer()
    $sp.DiscardOutBuffer()
    Write-Host "PASS: open serial - $Port @ $Baud"
    Write-Host ""

    Write-Host "Press CM-530 RESET / power-cycle now, then press Enter..."
    [void](Read-Host)
    Write-Host "Listening for READY,SDK_SERIAL_PROBE:"
    Read-Cm530Serial -SerialPort $sp -Seconds 8.0
    Write-Host ""

    $states = @(
        @{ Name = "dtr-off rts-off"; Dtr = $false; Rts = $false },
        @{ Name = "dtr-on rts-off";  Dtr = $true;  Rts = $false },
        @{ Name = "dtr-off rts-on";  Dtr = $false; Rts = $true  },
        @{ Name = "dtr-on rts-on";   Dtr = $true;  Rts = $true  }
    )

    foreach ($state in $states) {
        Write-Host "--- line state: $($state.Name) ---"
        $sp.DtrEnable = $state.Dtr
        $sp.RtsEnable = $state.Rts
        Start-Sleep -Milliseconds 250

        Send-AsciiBytes -SerialPort $sp -Bytes ([System.Text.Encoding]::ASCII.GetBytes("PING`n")) -CharDelay 0.05
        Read-Cm530Serial -SerialPort $sp -Seconds 3.0

        Send-AsciiBytes -SerialPort $sp -Bytes ([byte[]]@(0x50)) -CharDelay 0.05
        Read-Cm530Serial -SerialPort $sp -Seconds 3.0
        Write-Host ""
    }
}
catch {
    Write-Host "FAIL: $($_.Exception.Message)"
}
finally {
    if ($sp.IsOpen) {
        $sp.Close()
    }
}

Write-Host "Done. Press Enter to exit."
[void](Read-Host)
