param(
    [string]$Port = "COM4",
    [int]$Baud = 57600
)

$ErrorActionPreference = "Stop"

function Read-PortText {
    param(
        [System.IO.Ports.SerialPort]$SerialPort,
        [double]$Seconds = 1.5
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
        return ""
    }

    $lines = ($text -replace "`r", "`n") -split "`n"
    foreach ($line in $lines) {
        if ($line.Length -gt 0) {
            Write-Host "RX <- $line"
        }
    }

    return $text
}

function New-TestPort {
    param(
        [hashtable]$Mode
    )

    $sp = [System.IO.Ports.SerialPort]::new(
        $Port,
        $Baud,
        [System.IO.Ports.Parity]::None,
        8,
        [System.IO.Ports.StopBits]::One
    )
    $sp.ReadTimeout = 100
    $sp.WriteTimeout = 2000
    $sp.Handshake = $Mode.Handshake

    if ($null -ne $Mode.Dtr) {
        $sp.DtrEnable = [bool]$Mode.Dtr
    }
    if ($Mode.Handshake -eq [System.IO.Ports.Handshake]::None -and $null -ne $Mode.Rts) {
        $sp.RtsEnable = [bool]$Mode.Rts
    }

    $sp.Open()
    return $sp
}

function Send-Variants {
    param(
        [System.IO.Ports.SerialPort]$SerialPort
    )

    $variants = @(
        @{ Name = "Write string P";       Bytes = [System.Text.Encoding]::ASCII.GetBytes("P");      Kind = "string" },
        @{ Name = "Write bytes P";        Bytes = [byte[]]@(0x50);                                  Kind = "bytes" },
        @{ Name = "BaseStream P";         Bytes = [byte[]]@(0x50);                                  Kind = "base" },
        @{ Name = "Write string PING LF"; Bytes = [System.Text.Encoding]::ASCII.GetBytes("PING`n"); Kind = "string" },
        @{ Name = "WriteLine P";          Bytes = [System.Text.Encoding]::ASCII.GetBytes("P");      Kind = "writeline" }
    )

    foreach ($variant in $variants) {
        Write-Host ("TX -> " + $variant.Name)
        try {
            if ($variant.Kind -eq "string") {
                $SerialPort.Write([System.Text.Encoding]::ASCII.GetString($variant.Bytes))
            } elseif ($variant.Kind -eq "writeline") {
                $SerialPort.NewLine = "`n"
                $SerialPort.WriteLine([System.Text.Encoding]::ASCII.GetString($variant.Bytes))
            } elseif ($variant.Kind -eq "base") {
                $SerialPort.BaseStream.Write($variant.Bytes, 0, $variant.Bytes.Length)
                $SerialPort.BaseStream.Flush()
            } else {
                $SerialPort.Write($variant.Bytes, 0, $variant.Bytes.Length)
            }
        }
        catch {
            Write-Host "WRITE FAIL: $($_.Exception.Message)"
        }

        Start-Sleep -Milliseconds 100
        $rx = Read-PortText -SerialPort $SerialPort -Seconds 1.5
        if ($rx -match "PONG" -or $rx -match "\[RXDBG\]") {
            Write-Host "PASS: CM-530 received TX in this mode."
            return $true
        }
    }

    return $false
}

Write-Host "CM-530 Serial Reopen Matrix Probe"
Write-Host "Port       : $Port"
Write-Host "Baud       : $Baud"
Write-Host "Target FW  : CM530_sdk_serial_probe.hex"
Write-Host ""
Write-Host "This test reopens COM4 for each DTR/RTS/Handshake mode."
Write-Host "Close RoboPlus / Python / other COM4 windows first."
Write-Host "Flash CM530_sdk_serial_probe.hex, then press Enter."
[void](Read-Host)

$modes = @(
    @{ Name = "none dtr-off rts-off";       Handshake = [System.IO.Ports.Handshake]::None;                 Dtr = $false; Rts = $false },
    @{ Name = "none dtr-on rts-off";        Handshake = [System.IO.Ports.Handshake]::None;                 Dtr = $true;  Rts = $false },
    @{ Name = "none dtr-off rts-on";        Handshake = [System.IO.Ports.Handshake]::None;                 Dtr = $false; Rts = $true  },
    @{ Name = "none dtr-on rts-on";         Handshake = [System.IO.Ports.Handshake]::None;                 Dtr = $true;  Rts = $true  },
    @{ Name = "xonxoff dtr-off rts-off";    Handshake = [System.IO.Ports.Handshake]::XOnXOff;              Dtr = $false; Rts = $false },
    @{ Name = "xonxoff dtr-on rts-on";      Handshake = [System.IO.Ports.Handshake]::XOnXOff;              Dtr = $true;  Rts = $true  },
    @{ Name = "rtscts dtr-off";             Handshake = [System.IO.Ports.Handshake]::RequestToSend;        Dtr = $false; Rts = $null  },
    @{ Name = "rtscts dtr-on";              Handshake = [System.IO.Ports.Handshake]::RequestToSend;        Dtr = $true;  Rts = $null  },
    @{ Name = "rtscts+xonxoff dtr-off";     Handshake = [System.IO.Ports.Handshake]::RequestToSendXOnXOff; Dtr = $false; Rts = $null  },
    @{ Name = "rtscts+xonxoff dtr-on";      Handshake = [System.IO.Ports.Handshake]::RequestToSendXOnXOff; Dtr = $true;  Rts = $null  }
)

foreach ($mode in $modes) {
    Write-Host ""
    Write-Host ("=== open mode: " + $mode.Name + " ===")
    $sp = $null
    try {
        $sp = New-TestPort -Mode $mode
        Write-Host "Opened."
        Write-Host "DTR/RTS      : $($sp.DtrEnable)/$($sp.RtsEnable)"
        Write-Host "CTS/DSR/CD/RI: $($sp.CtsHolding)/$($sp.DsrHolding)/$($sp.CDHolding)/$($sp.RingHolding)"
        Write-Host "Initial listen:"
        [void](Read-PortText -SerialPort $sp -Seconds 1.0)
        $ok = Send-Variants -SerialPort $sp
        if ($ok) {
            Write-Host ""
            Write-Host ("SELECTED MODE: " + $mode.Name)
            break
        }
    }
    catch {
        Write-Host "OPEN/TEST FAIL: $($_.Exception.Message)"
    }
    finally {
        if ($null -ne $sp -and $sp.IsOpen) {
            $sp.Close()
        }
        Start-Sleep -Milliseconds 500
    }
}

Write-Host ""
Write-Host "Matrix test finished. Press Enter to exit."
[void](Read-Host)
