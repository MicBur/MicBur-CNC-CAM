# Baut MicBur-CNC-CAM und erstellt
#   installer_build\MicBur-CNC-CAM_<Version>_Setup.exe     (Installation ohne Administratorrechte)
#   installer_build\MicBur-CNC-CAM_<Version>_Portable.zip  (entpacken und vom USB-Stick starten)
#
# Aufruf:  powershell -ExecutionPolicy Bypass -File packaging\build_release.ps1 [-SkipTests]

param(
    [switch]$SkipTests
)

$ErrorActionPreference = 'Stop'
$Version  = '1.1'
$Root     = Split-Path -Parent $PSScriptRoot
$Build    = Join-Path $Root 'build'
$Out      = Join-Path $Root 'installer_build'
$Stage    = Join-Path $Out 'MicBur-CNC-CAM'
$Portable = Join-Path $Out 'MicBur-CNC-CAM_Portable'
$QtBin    = 'G:\Qt\6.12.0\mingw_64\bin'
$Iscc     = Join-Path $env:LOCALAPPDATA 'Programs\Inno Setup 6\ISCC.exe'

$env:PATH = "$QtBin;G:\Qt\Tools\mingw1310_64\bin;G:\Qt\Tools\Ninja;$env:PATH"

function Step($text) { Write-Host "`n=== $text ===" -ForegroundColor Cyan }

Step 'Programm bauen'
if (Get-Process -Name 'GeminiCNC', 'MicBur-CNC-CAM' -ErrorAction SilentlyContinue) {
    throw 'MicBur-CNC-CAM läuft noch – bitte zuerst schließen.'
}
& ninja -C $Build
if ($LASTEXITCODE -ne 0) { throw 'Build fehlgeschlagen' }

if (-not $SkipTests) {
    Step 'Tests'
    $env:QT_QPA_PLATFORM = 'offscreen'
    foreach ($test in 'TestGeometry', 'TestCamEngine', 'TestHardware', 'TestWinMax') {
        & (Join-Path $Build "tests\$test.exe") | Select-Object -Last 1
        if ($LASTEXITCODE -ne 0) { throw "$test fehlgeschlagen" }
    }
    Remove-Item Env:\QT_QPA_PLATFORM
}

Step 'Programmdateien zusammenstellen'
if (Test-Path $Stage) { Remove-Item $Stage -Recurse -Force }
New-Item -ItemType Directory -Force $Stage | Out-Null
Copy-Item (Join-Path $Build 'GeminiCNC.exe') (Join-Path $Stage 'MicBur-CNC-CAM.exe')
& (Join-Path $QtBin 'windeployqt.exe') --release --no-translations --compiler-runtime (Join-Path $Stage 'MicBur-CNC-CAM.exe')
if ($LASTEXITCODE -ne 0) { throw 'windeployqt fehlgeschlagen' }
# Laufzeit von MinGW sicherstellen (falls windeployqt sie nicht gefunden hat)
foreach ($dll in 'libgcc_s_seh-1.dll', 'libstdc++-6.dll', 'libwinpthread-1.dll') {
    if (-not (Test-Path (Join-Path $Stage $dll))) { Copy-Item (Join-Path $QtBin $dll) $Stage }
}
Copy-Item (Join-Path $Root 'samples') (Join-Path $Stage 'Beispiele') -Recurse
Copy-Item (Join-Path $Root 'resources\icons\app_icon.ico') $Stage

Step 'Installer erstellen'
& python (Join-Path $PSScriptRoot 'make_wizard_images.py') $Out
if ($LASTEXITCODE -ne 0) { throw 'Assistentenbilder fehlgeschlagen' }
& $Iscc /Q (Join-Path $Root 'installer.iss')
if ($LASTEXITCODE -ne 0) { throw 'Inno Setup fehlgeschlagen' }

Step 'Portable ZIP erstellen'
if (Test-Path $Portable) { Remove-Item $Portable -Recurse -Force }
Copy-Item $Stage $Portable -Recurse
@"
MicBur-CNC-CAM – portable Version

Diese Datei schaltet den portablen Modus ein: Werkzeugbibliothek und Einstellungen
werden im Ordner "daten" neben der Programmdatei gespeichert (z. B. auf dem USB-Stick).
Datei löschen = Daten wie bei der installierten Version im Benutzerprofil.
"@ | Set-Content -Path (Join-Path $Portable 'portable.txt') -Encoding UTF8
@"
MicBur-CNC-CAM $Version – portable Version
============================================

1. ZIP-Datei komplett entpacken (z. B. auf den USB-Stick).
2. MicBur-CNC-CAM.exe starten – keine Installation und keine Administratorrechte nötig.

Werkzeuge und Einstellungen liegen im Unterordner "daten".
"@ | Set-Content -Path (Join-Path $Portable 'LIESMICH.txt') -Encoding UTF8

$Zip = Join-Path $Out "MicBur-CNC-CAM_${Version}_Portable.zip"
if (Test-Path $Zip) { Remove-Item $Zip -Force }
Compress-Archive -Path $Portable -DestinationPath $Zip -CompressionLevel Optimal

Step 'Fertig'
Get-Item (Join-Path $Out "MicBur-CNC-CAM_${Version}_Setup.exe"), $Zip |
    Select-Object Name, @{ Name = 'MB'; Expression = { [math]::Round($_.Length / 1MB, 1) } } | Format-Table -AutoSize
