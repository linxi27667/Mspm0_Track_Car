param(
    [Parameter(Mandatory=$true)][switch]$Lifted,
    [int]$Runs = 5,
    [switch]$Build,
    [switch]$Flash,
    [string]$ProjectRoot = (Split-Path -Parent $PSScriptRoot),
    [string]$ProbeSerial = '69514110',
    [string]$Device = 'MSPM0G3507',
    [int]$Speed = 1000,
    [string]$JLinkRttLogger = 'C:\Program Files\SEGGER\JLink_V958\JLinkRTTLogger.exe',
    [int]$RunTimeoutSeconds = 20
)

$ErrorActionPreference = 'Stop'
if (-not $Lifted) { throw 'Refusing motor qualification: -Lifted is mandatory.' }
if ($Runs -lt 2) { throw 'At least two runs are required to measure repeatability.' }

$stamp = Get-Date -Format 'yyyyMMdd-HHmmss'
$evidenceDir = Join-Path $ProjectRoot "logs\closed_loop\motor-$stamp"
New-Item -ItemType Directory -Force -Path $evidenceDir | Out-Null
$rttLog = Join-Path $evidenceDir 'rtt.log'
$hostOut = Join-Path $evidenceDir 'host.out.log'
$hostErr = Join-Path $evidenceDir 'host.err.log'
$hostProcess = $null
$rttProcess = $null

function Test-ListeningPort([int]$Port) {
    return [bool](Get-NetTCPConnection -State Listen -LocalPort $Port -ErrorAction SilentlyContinue)
}

function Get-TerminalCount([string]$Path) {
    if (-not (Test-Path -LiteralPath $Path)) { return 0 }
    return @(Select-String -LiteralPath $Path -Pattern 'MOTORTEST.*end result=').Count
}

try {
    if ($Build) {
        & (Join-Path $PSScriptRoot 'Build-KeilFirmware.ps1') -ProjectRoot $ProjectRoot `
            -LogPath (Join-Path $evidenceDir 'keil-build.log')
    }
    if ($Flash) {
        & (Join-Path $PSScriptRoot 'Flash-JLink.ps1') -ConfirmMotorSafe `
            -ProjectRoot $ProjectRoot -ProbeSerial $ProbeSerial -Device $Device -Speed $Speed `
            -LogPath (Join-Path $evidenceDir 'jlink-flash.log')
        Start-Sleep -Seconds 2
    }

    if (-not (Test-ListeningPort 5000) -or -not (Test-ListeningPort 8888)) {
        Write-Host '[HOST] starting local host without browser'
        $hostProcess = Start-Process -FilePath 'python' `
            -ArgumentList @('host\app.py', '--no-browser') -WorkingDirectory $ProjectRoot `
            -RedirectStandardOutput $hostOut -RedirectStandardError $hostErr `
            -WindowStyle Hidden -PassThru
        $deadline = [DateTime]::UtcNow.AddSeconds(12)
        while ((-not (Test-ListeningPort 5000) -or -not (Test-ListeningPort 8888)) -and
               [DateTime]::UtcNow -lt $deadline) {
            Start-Sleep -Milliseconds 250
        }
        if (-not (Test-ListeningPort 5000) -or -not (Test-ListeningPort 8888)) {
            throw "Host failed to listen. See $hostErr"
        }
    }

    if (-not (Test-Path -LiteralPath $JLinkRttLogger)) {
        throw "RTT logger not found: $JLinkRttLogger"
    }
    $rttArgs = @('-Device', $Device, '-If', 'SWD', '-Speed', $Speed,
                 '-USB', $ProbeSerial, '-RTTChannel', 0, $rttLog)
    $rttProcess = Start-Process -FilePath $JLinkRttLogger -ArgumentList $rttArgs `
        -NoNewWindow -PassThru
    Start-Sleep -Seconds 2

    for ($run = 1; $run -le $Runs; $run++) {
        $before = Get-TerminalCount $rttLog
        Write-Host "[MOTOR] run $run/$Runs request"
        $requestDeadline = [DateTime]::UtcNow.AddSeconds(30)
        do {
            & python (Join-Path $PSScriptRoot 'send_motor_test.py') --timeout 2
            $requestExit = $LASTEXITCODE
            if ($requestExit -ne 0) { Start-Sleep -Seconds 2 }
        } while ($requestExit -ne 0 -and [DateTime]::UtcNow -lt $requestDeadline)
        if ($requestExit -ne 0) { throw "Run $run request was rejected for 30 seconds." }

        $deadline = [DateTime]::UtcNow.AddSeconds($RunTimeoutSeconds)
        do {
            Start-Sleep -Milliseconds 250
            $after = Get-TerminalCount $rttLog
        } while ($after -le $before -and [DateTime]::UtcNow -lt $deadline)
        if ($after -le $before) { throw "Run $run timed out; evidence=$rttLog" }
        $terminal = Select-String -LiteralPath $rttLog -Pattern 'MOTORTEST.*end result=' | Select-Object -Last 1
        Write-Host "[MOTOR] run $run terminal: $($terminal.Line.Trim())"
        Start-Sleep -Milliseconds 1200
    }
}
finally {
    if ($null -ne $rttProcess -and -not $rttProcess.HasExited) {
        Stop-Process -Id $rttProcess.Id -Force
        $rttProcess.WaitForExit()
    }
    if ($null -ne $hostProcess -and -not $hostProcess.HasExited) {
        Stop-Process -Id $hostProcess.Id -Force
        $hostProcess.WaitForExit()
    }
}

$jsonReport = Join-Path $evidenceDir 'qualification.json'
$mdReport = Join-Path $evidenceDir 'qualification.md'
& python (Join-Path $PSScriptRoot 'analyze_motor_qualification.py') $rttLog `
    --runs $Runs --max-cv 0.05 --json $jsonReport --markdown $mdReport
$analysisExit = $LASTEXITCODE
Write-Host "[EVIDENCE] directory=$evidenceDir"
if ($analysisExit -ne 0) { throw "Motor qualification FAILED. See $mdReport" }
Write-Host '[MOTOR] BENCH PASS. Eligible only for supervised low-speed ground straight test.'
