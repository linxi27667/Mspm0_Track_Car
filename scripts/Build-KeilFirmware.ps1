param(
    [string]$ProjectRoot = (Split-Path -Parent $PSScriptRoot),
    [string]$KeilExe = 'C:\Keil_v5\UV4\UV4.exe',
    [string]$Uvprojx = 'keil\gpio_toggle_output_LP_MSPM0G3507_nortos_keil.uvprojx',
    [string]$LogPath = 'logs\closed_loop\keil-build.log'
)

$ErrorActionPreference = 'Stop'
$project = [IO.Path]::GetFullPath((Join-Path $ProjectRoot $Uvprojx))
$log = [IO.Path]::GetFullPath((Join-Path $ProjectRoot $LogPath))
if (-not (Test-Path -LiteralPath $KeilExe)) { throw "Keil not found: $KeilExe" }
if (-not (Test-Path -LiteralPath $project)) { throw "Keil project not found: $project" }
New-Item -ItemType Directory -Force -Path (Split-Path -Parent $log) | Out-Null

Write-Host "[BUILD] project=$project"
& $KeilExe -b $project -j0 -o $log
$exitCode = $LASTEXITCODE
$text = Get-Content -Raw -LiteralPath $log
if ($text -notmatch '0 Error\(s\), 0 Warning\(s\)') {
    throw "Keil build failed (exit=$exitCode). See $log"
}
if ($null -ne $exitCode -and $exitCode -ne 0) {
    Write-Warning "UV4 returned $exitCode although its build log reports zero errors/warnings."
}
Write-Host "[BUILD] PASS: 0 errors, 0 warnings; log=$log"
