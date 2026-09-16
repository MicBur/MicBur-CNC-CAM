# MicBur-CNC-CAM – Projektzusammenfassung für KI-Assistenten

Dialog-CAM im Stil der **Hurco WinMax**-Steuerung mit 3D-Frässimulation, G-Code-Postprozessoren und
Klipper/Moonraker-Anbindung. Früherer Name: *GeminiCNC* (Klassen-Namespace `GeminiCNC::`, Build-Ziel `GeminiCNC`).
Entwickler: Michael Burzlaff (MicBur). Sprache der Oberfläche, Kommentare und Commits: **Deutsch**.
Vorbild für Bedienung und Datensätze ist immer die echte WinMax – bei Unklarheit wie WinMax lösen.

## Build & Test

Toolchain (alles auf Laufwerk G:): Qt 6.12.0 MinGW 64-Bit, MinGW 13.1, CMake, Ninja, C++20, OpenGL 3.3 Core.

```powershell
$env:PATH = "G:\Qt\6.12.0\mingw_64\bin;G:\Qt\Tools\mingw1310_64\bin;G:\Qt\Tools\Ninja;$env:PATH"
ninja -C G:\cnc\build                                   # Programm + Tests bauen
$env:QT_QPA_PLATFORM = "offscreen"
G:\cnc\build\tests\TestCamEngine.exe                     # ebenso TestGeometry, TestHardware, TestWinMax
G:\cnc\build\tests\RenderPreview.exe <Ordner>            # rendert Beispielteil als PNG (Sichtprüfung Darstellung)
powershell -ExecutionPolicy Bypass -File G:\cnc\packaging\build_release.ps1   # Setup.exe + Portable.zip
```

Fallstricke:
- **Läuft `GeminiCNC.exe` noch, scheitert das Linken** („Permission denied“). Den Benutzer bitten, das Programm zu schließen – nicht selbst beenden.
- Release-Build mit `-DNDEBUG`: `assert` ist wirkungslos. In Tests `require()` (TestCamEngine) bzw. `check()` (TestWinMax) verwenden.
- Viele Quelldateien haben **CRLF**-Zeilenenden und teils kaputt kodierte Umlaute in alten Kommentaren. Beim Bearbeiten Zeilenenden erhalten.
- Shader stehen inline in `ShaderProgram.cpp` (nicht in `resources/shaders`). Prüfbar mit `G:\VulkanSDK\1.4.357.0\Bin\glslangValidator.exe`.

## Architektur (`src/`)

| Modul | Inhalt |
|---|---|
| `core/` | `Vector3D` (X,Y,Z,A), `BoundingBox`, `ToolDefinition` (Werkzeugbibliothek als JSON, portabler Modus über `portable.txt` neben der .exe), `MachineConfig` (Nullpunkt G54–G59), `MaterialDatabase` + `TechnologyCalculator` |
| `geometry/` | `Mesh`, STL/OBJ/DXF-Lader, `Contour` (+ `ContourSegment` mit Bogen-Mittelpunkt, `zStart`, Z je Segment, `applyStartDepth`), `ContourSolver` (WinMax-Maßberechnung Linie/Bogen), `PolygonOffset` |
| `cam/` | `Toolpath`/`PathSegment`, `ToolpathGenerator` (Planen, Kontur mit An-/Abfahrt, Rampe, Stegen, Tasche Zickzack/Spirale/konturparallel mit Inseln, 3D-STL), `ConversationalBlock`, `ConversationalProgram`, `CollisionDetector`, Postprozessoren (ISO, Hurco, Heidenhain, Saeilo, Klipper) |
| `simulation/` | `StockModel` (Dexel-Rohteil), `SimulationEngine` (Timer-Interpolation, Werkzeugwechsel) |
| `hardware/` | Moonraker-Client (mit Offline-Simulator), Jog, Antasten |
| `ui/` | `MainWindow`, `SimulationWindow`, `viewport/` (`Viewport3D`, `GPUStockModel`, `ShaderProgram`), `dialogs/` (u. a. `ConversationalEditorDialog` = Arbeitsplan, `ContourSegmentEditorDialog` = WinMax-Datensatz-Editor), `winmax/` (Kopfzeile, Softkeys F1–F8) |

## Fachliche Kernkonzepte

**Arbeitsplan / Blöcke** (`ConversationalBlock`, gespeichert als `.gprog`-JSON, jedes Feld wird gespeichert):
- Blocktypen: Planen, Kontur, Tasche, Langloch, Helix/Gewinde, **Bohrungen**, **Bohrpositionen**, 3D-STL, NC, **Muster Start / Muster Ende**.
- `ConversationalProgram::generateFullToolpath` erzeugt den Gesamtweg mit Werkzeugwechseln, Rückzügen und Verbindungswegen.
  `resolvedBlock(i)` ergänzt einen Block um Daten seiner Folgeblöcke (siehe unten).
- **Muster**: Blöcke zwischen Muster Start und Muster Ende werden je Musterposition wiederholt (linear, Raster, Kreis, Spiegeln; verschachtelbar, `PatternTransform`).
- **Bohren wie Hurco**: Der Bohrungen-Block (`BlockType::Drill`) hat Z START / Z UNTEN und eine geordnete Liste `drillOps`
  (`DrillOperation`: Bohrer, Zentrieren, Flachsenken, NC-Anbohren, Kegelsenken, Tieflochbohrer, Benutzerdef., Gewindebohrer,
  Synchron-Gewinde, Ausdrehen, Reiben – jeweils mit eigenem Werkzeug, Drehzahl, Eintauchvorschub, Bohr-Typ, Stufentiefe usw.).
  Die Lage der Bohrungen steht in den **direkt folgenden `DrillPositions`-Blöcken** (Einzeln, Teilkreis, Raster, Reihe, Bogen, Rahmen, Positionsliste).
  Leere `drillOps` = älterer Einzelzyklus (`drillCycle`, eigenes Bohrbild); `loadFromFile` wandelt alte Dateien automatisch um.
- **Kontur-Startsegment**: Konturart `ContourRole` Kontur / Tasche / Insel. Inseln sind die direkt auf eine Kontur-Tasche folgenden Kontur-Blöcke mit Rolle Insel.
  `contourZForAll`: Z UNTEN von Segment 0 gilt für alle Segmente (neue Blöcke: ja; ältere Dateien: Z je Segment).
- Schnittwerte kommen aus dem Block (Technologie-Reiter), nicht aus den Werkzeug-Standardwerten.
- **Rückgängig/Wiederholen** im Arbeitsplan: `QUndoStack` mit vollständigen Programmständen; alle Änderungen einer Aktion in einer `UndoGroup`.

**G-Code**: Bögen werden aus Geradenzügen zurückgewonnen (G2/G3), Bohrungen als `G98 G8x … G80` je Bohrvorgang,
Nullpunkt G54–G59, `G43 H`. Klipper: keine Bögen/Zyklen, M3/M5-Makros, PAUSE beim Werkzeugwechsel.

**Rohteil-Simulation** (`StockModel`):
- Raster mit bis zu 4 Materialschichten je Punkt (`layerLo/layerHi`) → gekippte Teile, Überhänge, Durchbrüche. `initFromMesh` für STL-Rohteile.
- `carveSegment` trägt als senkrechter Zylinder ab und merkt sich je Punkt die **Fräserspur** (`marks`: Richtung, Bahnkoordinaten, Vorschub/U)
  und die **genaue Kantenlage** (`edgeHints`), damit Radien und Schrägen glatt statt treppenförmig dargestellt werden.
- `buildSurface()` wird gemeinsam für GPU und STL-Export genutzt (steile Zellen mit eigenen Eckpunkten, verschobene Randpunkte, Umgebungsverdeckung je Punkt).

**Darstellung** (`Viewport3D` + Rohteil-Shader): Qualitätsschalter Schnell (Blinn-Phong) / Realistisch (PBR GGX, Werkstatt-Umgebung,
Fräserspuren im Shader, ACES) / Realistisch + Schatten (Shadow-Map). Materialvorgaben Alu, Messing, Stahl, Holz, POM, Edelstahl.
Der GPU-Upload passiert in `paintGL`; danach VBO/IBO wieder lösen (andere Zeichnungen nutzen Client-Arrays).

## Auslieferung

- `packaging/build_release.ps1`: baut, testet, stellt mit `windeployqt` zusammen, erstellt
  `installer_build/MicBur-CNC-CAM_<Version>_Setup.exe` (Inno Setup, `installer.iss`, **ohne Administratorrechte**, Ziel `%LOCALAPPDATA%\Programs`)
  und `installer_build/MicBur-CNC-CAM_<Version>_Portable.zip` (mit `portable.txt` → Daten im Unterordner `daten`, für USB-Stick).
- Programmsymbol: `resources/icons/make_icon.py` erzeugt `app_icon.ico/.png`; eingebunden über `resources/app.rc` (exe) und `resources/app.qrc` (Fenster).
- Die Programmdatei wird beim Paketieren von `GeminiCNC.exe` in `MicBur-CNC-CAM.exe` umbenannt. Organisations-/Programmname in `main.cpp`
  nicht ändern – sonst findet die installierte Version ihre gespeicherte Werkzeugbibliothek im AppData-Ordner nicht mehr.

## Arbeitsweise

- Git: Arbeitszweig `feature/professional-cam-2026-09` (noch nicht nach `master` gemergt, nicht gepusht). Kleine, thematische Commits.
- Neue Funktionen immer mit Test in `tests/` absichern; bestehende Tests müssen grün bleiben.
- Der Benutzer testet über Screenshots aus der laufenden App und vergleicht mit WinMax-Screenshots.

## Offene Punkte

- Stufe 3 Darstellung: Knopf „Fertig rendern“ (hochauflösend neu berechnen, PNG).
- Autosave, zuletzt geöffnete Dateien, Einstellungen merken (QSettings); Einrichteblatt drucken.
- Restmaterial-Heatmap mit echtem Soll-Z; Kollisionsprüfung Halter/Schaft gegen das Rohteil.
- Tasche „Startkante“ ohne Wirkung; G41/G42-Ausgabe nicht überarbeitet; Heidenhain ohne Zyklen/Nullpunkt; Zoll fehlt;
  Laufzeitschätzung rechnet Eilgang mit 1000 mm/min; 3D-STL-Block speichert nur den Dateipfad.
- Simulation rechnet alle Werkzeuge als flachen Zylinder (Kugel- und Fasenfräser nur als Spur, nicht in der Geometrie).
