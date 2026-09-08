param(
    [Parameter(Mandatory=$true)][string]$OutputPath,
    [int]$DurationSeconds = 30,
    [string]$JLinkRttLogger = 'C:\Program Files\SEGGER\JLink_V958\JLinkRTTLogger.exe',
    [string]$Device = 'MSPM0G3507',
    [string]$Interface = 'SWD',
    [int]$Speed = 1000,
    [string]$ProbeSerial = '69514110',
    [int]$Channel = 0
)

$ErrorActionPreference = 'Stop'
if (-not (Test-Path -LiteralPath $JLinkRttLogger)) { throw "RTT logger not found: $JLinkRttLogger" }
if ($DurationSeconds -lt 1) { throw 'DurationSeconds must be positive.' }
$output = [IO.Path]::GetFullPath($OutputPath)
New-Item -ItemType Directory -Force -Path (Split-Path -Parent $output) | Out-Null
Remove-Item -LiteralPath $output -Force -ErrorAction SilentlyContinue

$args = @('-Device', $Device, '-If', $Interface, '-Speed', $Speed,
          '-USB', $ProbeSerial, '-RTTChannel', $Channel, $output)
$proc = Start-Process -FilePath $JLinkRttLogger -ArgumentList $args -PassThru -NoNewWindow
try {
    Write-Host "[RTT] capturing channel $Channel for ${DurationSeconds}s -> $output"
    $deadline = [DateTime]::UtcNow.AddSeconds($DurationSeconds)
    while (-not $proc.HasExited -and [DateTime]::UtcNow -lt $deadline) {
        Start-Sleep -Milliseconds 200
        $proc.Refresh()
    }
}
finally {
    if (-not $proc.HasExited) {
        Stop-Process -Id $proc.Id -Force
        $proc.WaitForExit()
    }
}
if (-not (Test-Path -LiteralPath $output)) { throw 'RTT capture produced no output file.' }
Write-Host "[RTT] complete bytes=$((Get-Item -LiteralPath $output).Length)"

