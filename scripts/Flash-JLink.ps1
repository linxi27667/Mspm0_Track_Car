param(
    [Parameter(Mandatory=$true)][switch]$ConfirmMotorSafe,
    [string]$ProjectRoot = (Split-Path -Parent $PSScriptRoot),
    [string]$JLinkExe = 'C:\Program Files\SEGGER\JLink_V958\JLink.exe',
    [string]$HexPath = 'keil\Objects\gpio_toggle_output_LP_MSPM0G3507_nortos_keil.hex',
    [string]$Device = 'MSPM0G3507',
    [string]$Interface = 'SWD',
    [int]$Speed = 1000,
    [string]$ProbeSerial = '69514110',
    [switch]$ConnectUnderReset,
    [string]$LogPath = 'logs\closed_loop\jlink-flash.log'
)

$ErrorActionPreference = 'Stop'
$hex = [IO.Path]::GetFullPath((Join-Path $ProjectRoot $HexPath))
$log = [IO.Path]::GetFullPath((Join-Path $ProjectRoot $LogPath))
if (-not $ConfirmMotorSafe) { throw 'Refusing flash: ConfirmMotorSafe is required.' }
if (-not (Test-Path -LiteralPath $JLinkExe)) { throw "J-Link CLI not found: $JLinkExe" }
if (-not (Test-Path -LiteralPath $hex)) { throw "HEX not found: $hex" }
New-Item -ItemType Directory -Force -Path (Split-Path -Parent $log) | Out-Null

$commandFile = Join-Path ([IO.Path]::GetTempPath()) ("track-car-flash-{0}.jlink" -f [guid]::NewGuid())
try {
    $commands = if ($ConnectUnderReset) {
        @('ClrRESET', 'sleep 100', 'connect', 'SetRESET', 'sleep 100',
          'h', "loadfile $hex", 'r', 'g', 'q')
    } else {
        @('r', 'h', "loadfile $hex", 'r', 'g', 'q')
    }
    $commands | Set-Content -LiteralPath $commandFile -Encoding ascii
    Write-Host "[FLASH] device=$Device probe=$ProbeSerial interface=$Interface speed=${Speed}kHz"
    & $JLinkExe -USB $ProbeSerial -Device $Device -If $Interface -Speed $Speed `
        -AutoConnect 1 -CommandFile $commandFile -Log $log
    $exitCode = $LASTEXITCODE
    $text = Get-Content -Raw -LiteralPath $log
    $fatalPattern = 'Failed to initialize DAP|No PWR-AP detected|Could not connect to the target device|Error occurred:'
    $targetConnected = $text -match 'DAP initialized successfully\.'
    $downloadStarted = $text -match 'JLINK_DownloadFile\(sFile = '
    $downloadCompleted = $text -match 'JLINK_DownloadFile[\s\S]*?End of comparing flash[\s\S]*?returns 0x00'
    if ($exitCode -ne 0 -or $text -match $fatalPattern -or
        -not $targetConnected -or -not $downloadStarted -or
        -not $downloadCompleted) {
        throw "J-Link flash/verify failed (exit=$exitCode). See $log"
    }
    Write-Host "[FLASH] PASS: programmed and verified; log=$log"
}
finally {
    Remove-Item -LiteralPath $commandFile -Force -ErrorAction SilentlyContinue
}
