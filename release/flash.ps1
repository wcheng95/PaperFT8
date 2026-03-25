# Flash Mini-FT8 merged firmware image (ESP32-S3, 8MB)
# Requires: pip install esptool
# Usage: .\flash.ps1

$ErrorActionPreference = "Stop"

function Require-NonEmpty([string]$Prompt) {
    while ($true) {
        $v = Read-Host $Prompt
        if (-not [string]::IsNullOrWhiteSpace($v)) { return $v }
        Write-Host "Value required." -ForegroundColor Red
    }
}

function Resolve-ExistingPath([string]$Path) {
    if (-not (Test-Path -LiteralPath $Path)) {
        throw "File not found: $Path"
    }
    return (Resolve-Path -LiteralPath $Path).Path
}

$Chip = "esp32s3"
$Baud = 460800

Write-Host "Mini-FT8 flasher (merged .bin)" -ForegroundColor Cyan
Write-Host ""

$PortIn = Require-NonEmpty "Enter serial port (e.g., COM11)"
$BinIn  = Require-NonEmpty "Enter merged firmware .bin path (e.g., MiniFT8_V1.3.2.bin)"

$Port = $PortIn.Trim()
$Bin  = Resolve-ExistingPath $BinIn.Trim()

$Args = @(
    "--chip", $Chip,
    "-p", $Port,
    "-b", $Baud,
    "--before", "default_reset",
    "--after", "hard_reset",
    "write_flash",
    "--flash_mode", "dio",
    "--flash_size", "8MB",
    "--flash_freq", "80m",
    "0x0", $Bin
)

Write-Host ""
Write-Host "Flashing $Chip on $Port @ $Baud..." -ForegroundColor Green
Write-Host "  File: $Bin"
esptool @Args
