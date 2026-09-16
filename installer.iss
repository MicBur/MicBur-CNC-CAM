[Setup]
AppName=MicBur-CNC-CAM
AppVersion=1.0
AppPublisher=Michael Burzlaff
PrivilegesRequired=lowest
DefaultDirName={localappdata}\MicBur-CNC-CAM
DefaultGroupName=MicBur-CNC-CAM
OutputDir=g:\cnc\installer_build
OutputBaseFilename=MicBur_CNC_CAM_Setup
Compression=lzma
SolidCompression=yes
SetupIconFile=g:\cnc\resources\icons\app_icon.ico
UninstallDisplayIcon={app}\MicBur-CNC-CAM.exe
WizardStyle=modern

[Tasks]
Name: "desktopicon"; Description: "{cm:CreateDesktopIcon}"; GroupDescription: "{cm:AdditionalIcons}"; Flags: unchecked

[Files]
Source: "g:\cnc\dist\*"; DestDir: "{app}"; Flags: ignoreversion recursesubdirs createallsubdirs
Source: "g:\cnc\resources\*"; DestDir: "{app}\resources"; Flags: ignoreversion recursesubdirs createallsubdirs

[Icons]
Name: "{group}\MicBur-CNC-CAM"; Filename: "{app}\MicBur-CNC-CAM.exe"
Name: "{autodesktop}\MicBur-CNC-CAM"; Filename: "{app}\MicBur-CNC-CAM.exe"; Tasks: desktopicon

[Run]
Filename: "{app}\MicBur-CNC-CAM.exe"; Description: "{cm:LaunchProgram,MicBur-CNC-CAM}"; Flags: nowait postinstall skipifsilent