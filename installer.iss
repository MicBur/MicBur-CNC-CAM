; MicBur-CNC-CAM Installer (Inno Setup 6)
; Installation für den angemeldeten Benutzer – keine Administratorrechte nötig.
; Erstellen: packaging\build_release.ps1 (stellt die Programmdateien in installer_build\MicBur-CNC-CAM bereit)

#define AppName "MicBur-CNC-CAM"
#define AppVersion "1.1"
#define StageDir "g:\cnc\installer_build\MicBur-CNC-CAM"

[Setup]
AppName={#AppName}
AppVersion={#AppVersion}
AppVerName={#AppName} {#AppVersion}
AppPublisher=Michael Burzlaff
VersionInfoVersion=1.1.0.0
VersionInfoDescription={#AppName} Setup
PrivilegesRequired=lowest
UsedUserAreasWarning=no
DefaultDirName={localappdata}\Programs\{#AppName}
DefaultGroupName={#AppName}
DisableProgramGroupPage=yes
ArchitecturesAllowed=x64compatible
ArchitecturesInstallIn64BitMode=x64compatible
OutputDir=g:\cnc\installer_build
OutputBaseFilename=MicBur-CNC-CAM_{#AppVersion}_Setup
Compression=lzma2/ultra64
SolidCompression=yes
SetupIconFile=g:\cnc\resources\icons\app_icon.ico
WizardImageFile=g:\cnc\installer_build\wizard_large.bmp
WizardSmallImageFile=g:\cnc\installer_build\wizard_small.bmp
UninstallDisplayIcon={app}\MicBur-CNC-CAM.exe
UninstallDisplayName={#AppName}
WizardStyle=modern
CloseApplications=yes

[Languages]
Name: "de"; MessagesFile: "compiler:Languages\German.isl"

[Tasks]
Name: "desktopicon"; Description: "{cm:CreateDesktopIcon}"; GroupDescription: "{cm:AdditionalIcons}"

[Files]
Source: "{#StageDir}\*"; DestDir: "{app}"; Flags: ignoreversion recursesubdirs createallsubdirs

[Icons]
Name: "{group}\{#AppName}"; Filename: "{app}\MicBur-CNC-CAM.exe"
Name: "{group}\{#AppName} deinstallieren"; Filename: "{uninstallexe}"
Name: "{autodesktop}\{#AppName}"; Filename: "{app}\MicBur-CNC-CAM.exe"; Tasks: desktopicon

[Run]
Filename: "{app}\MicBur-CNC-CAM.exe"; Description: "{cm:LaunchProgram,{#AppName}}"; Flags: nowait postinstall skipifsilent
