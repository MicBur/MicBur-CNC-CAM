# Workflow Memory - CNC-Steuerungs- und CAM-System ("Gemini Anti-Gravity")

## Projekt-Metadaten
- **Projekt:** Dialogbasierte CNC-Steuerungssoftware & 3D-Simulation
- **Plattform:** Windows 10/11 (autark, optimiert für Low-Spec / mobile Hardware wie i5 + 8GB RAM + iGPU)
- **Framework & Sprache:** C++20, Qt 6.12.0 (MinGW 64-Bit), modern OpenGL (QOpenGLWidget + GLSL Core 330)
- **Hardware-Vorbereitung:** Klipper / Moonraker API, 3-Achs + 4. Drehachse (A), TMC2209 Treiber
- **Startzeitpunkt:** 2026-09-03T15:38:00+02:00
- **Aktualisierungszeitpunkt:** 2026-09-03T15:52:00+02:00

---

## 1. Modul-Architekturübersicht & Status

| Modul | Verzeichnis | Erstellte Klassen / Dateien | Status |
|---|---|---|---|
| **Core** | `src/core/` | `Vector3D.h/.cpp` (4D Kinematik X,Y,Z,A)<br>`BoundingBox.h/.cpp` (AABB)<br>`ToolDefinition.h/.cpp` (Werkzeugbibliothek & Geometrie)<br>`MachineConfig.h/.cpp` (Limits, TMC2209 Treiber-Parameter) | Abgeschlossen & Verifiziert |
| **Geometry** | `src/geometry/` | `Mesh.h/.cpp` (Dreiecksnetz, Rohteil vs. Bauteil)<br>`StlLoader.h/.cpp` (Binär- & ASCII-STL Parser + Export)<br>`Contour.h/.cpp` (2D-Kontur, Offset, Inset, Fläche)<br>`DxfLoader.h/.cpp` (DXF-Parser für Polylinien, Bögen, Kreise) | Abgeschlossen & Verifiziert |
| **CAM Math** | `src/cam/` | `Toolpath.h/.cpp` (Trajektorien G0/G1/G2/G3, Zeit- & Längenberechnung, G-Code Generierung)<br>`ToolpathGenerator.h/.cpp` (Planen, Kontur, Tasche, 3D-Schruppen)<br>`CollisionDetector.h/.cpp` (Echtzeit-Hüllkörperprüfung, Eilgang-Eintauchen, Soft-Limits, Halter-Stickout) | Abgeschlossen & Verifiziert |
| **Simulation** | `src/simulation/` | `StockModel.h/.cpp` (Effizientes Höhenfeld-Modell für dynamischen Materialabtrag ohne iGPU-Framedrops)<br>`SimulationEngine.h/.cpp` (QObject-basierte Timer-Interpolation, Play/Pause/Step, DRO-Signalübertragung) | Abgeschlossen & Verifiziert |
| **Hardware I/F** | `src/hardware/` | `IKlipperClient.h` (Abstraktes Hardware-Interface)<br>`MoonrakerClient.h/.cpp` (REST/WS-Client mit integriertem Windows-Offline-Simulator)<br>`JogController.h/.cpp` (Manuelle Achsensteuerung, Schrittweiten, Homing, Spindel) | Abgeschlossen & Verifiziert |
| **3D Rendering** | `src/ui/viewport/` | `ShaderProgram.h/.cpp` (GLSL Shader für diffuse Beleuchtung & Farblinien)<br>`Viewport3D.h/.cpp` (Modernes QOpenGLWidget, 60 FPS, Batch-Rendering, Rohteil, Bauteil, Fräsbahnen, 3D-Werkzeugmodell, Kamera-Orbit/Pan/Zoom) | Abgeschlossen & Verifiziert |
| **Dialoge & UI** | `src/ui/dialogs/` | `DialogStack.h/.cpp` (Sequenzieller Schritt-Controller)<br>`SetupDialog.h/.cpp` (Schritt 1: STL/DXF-Import & Rohteilquader)<br>`ToolManagerDialog.h/.cpp` (Schritt 2: Werkzeugbibliothek)<br>`ContourDialog.h/.cpp` (Schritt 3: CAM-Berechnung)<br>`SimulationDialog.h/.cpp` (Schritt 4: Playback, DRO, Kollisionen, Export)<br>`JogDialog.h/.cpp` (Schritt 5: Manuelles Verfahren X/Y/Z/A)<br>`HardwareDialog.h/.cpp` (Schritt 6: Moonraker & TMC2209 Status) | Abgeschlossen & Verifiziert |
| **Main Shell** | `src/ui/` & `src/` | `MainWindow.h/.cpp` (Hauptfenster mit QSplitter, Toolbar, NOT-HALT, Signalverdrahtung)<br>`main.cpp` (Dark Industrie-Design Theme & High-DPI Support) | Abgeschlossen & Verifiziert |
| **Build System** | Root / `tests/` | `CMakeLists.txt` (C++20, Qt6, modulare statische Libs)<br>`tests/CMakeLists.txt` (C++ Unit Tests für Geometry, CAM, Hardware)<br>`run_geminicnc.bat` (Windows Starter) | Abgeschlossen & Verifiziert |

---

## 2. Chronologisches Protokoll der Arbeitsschritte

| Timestamp | Schritt | Beschreibung | Resultat / Dateien |
|---|---|---|---|
| 2026-09-03T15:37:00+02:00 | Systeminspektion | Überprüfung der Toolchain (Qt 6.12.0 MinGW 64-Bit, CMake 3.30, Ninja 1.12, G++ 13.1.0) auf Laufwerk G: | Verifiziert & einsatzbereit |
| 2026-09-03T15:38:00+02:00 | Workflow Memory Init | Erstellung von `workflow_memory.md` zur lückenlosen Schritt-für-Schritt-Protokollierung | Angelegt in `g:/cnc/` |
| 2026-09-03T15:39:00+02:00 | Konzeption & Planung | Detaillierte Ausarbeitung des Architektur- und Implementierungsplans für Phase 1 & 2 | `implementation_plan.md` |
| 2026-09-03T15:39:30+02:00 | Hermes Memory Sync | Globaler Wissenseintrag in Hermes hinterlegt (`gemini_cnc_init_20260903.json`) | Gespeichert |
| 2026-09-03T15:40:00+02:00 | Modul Core | Implementierung von `Vector3D` (4D), `BoundingBox`, `ToolDefinition` und `MachineConfig` inkl. TMC2209-Konfiguration | `src/core/*` |
| 2026-09-03T15:41:00+02:00 | Modul Geometry | Implementierung von `Mesh`, `StlLoader` (Binär & ASCII), `Contour` (Offset-Berechnung) und `DxfLoader` (Polylinien/Bögen) | `src/geometry/*` |
| 2026-09-03T15:42:00+02:00 | Modul CAM & Collision | Implementierung von `Toolpath` (G-Code Emitter), `ToolpathGenerator` (Facing, Kontur, Tasche, 3D-Schruppen) und `CollisionDetector` | `src/cam/*` |
| 2026-09-03T15:43:00+02:00 | Modul Simulation | Implementierung von `StockModel` (Höhenfeld-Zerspanung) und `SimulationEngine` (Interpolation, DRO, Signale) | `src/simulation/*` |
| 2026-09-03T15:44:00+02:00 | Modul Hardware | Implementierung von `IKlipperClient`, `MoonrakerClient` (mit Windows Offline-Mock-Modus) und `JogController` | `src/hardware/*` |
| 2026-09-03T15:45:00+02:00 | Modul 3D Viewport | Implementierung von `ShaderProgram` (GLSL Core 330) und `Viewport3D` (`QOpenGLWidget` mit VBO-Batching, 60 FPS) | `src/ui/viewport/*` |
| 2026-09-03T15:46:00+02:00 | Modul Dialoge | Implementierung von `DialogStack`, `SetupDialog`, `ToolManagerDialog`, `ContourDialog`, `SimulationDialog`, `JogDialog`, `HardwareDialog` | `src/ui/dialogs/*` |
| 2026-09-03T15:47:00+02:00 | Main Shell & Tests | Implementierung von `MainWindow.cpp`, `main.cpp` und automatisierten Testsuiten (`TestGeometry`, `TestCamEngine`, `TestHardware`) | `src/ui/MainWindow.*`, `src/main.cpp`, `tests/*` |
| 2026-09-03T15:48:00+02:00 | CMake Build & Debug | Behebung des C++-Standardargument-Konflikts in `Mesh.h` und Verfeinerung des Matrix-Operators in `Viewport3D` | 100% sauber kompiliert |
| 2026-09-03T15:50:40+02:00 | Verifikation (Tests) | Ausführung von `TestGeometry.exe`, `TestCamEngine.exe` und `TestHardware.exe` | **Alle Tests PASSED (0 Fehler)** |
| 2026-09-03T15:51:30+02:00 | Windows Executable | Fertigstellung von `GeminiCNC.exe` und Erstellung von `run_geminicnc.bat` sowie Beispieldateien (`sample_part.stl`, `sample_contour.dxf`) | `build/GeminiCNC.exe` bereit |
| 2026-09-03T15:56:30+02:00 | Standalone Deployment | Ausführung von `windeployqt --compiler-runtime`: Alle Qt6- und MinGW-Laufzeit-DLLs (`Qt6Core.dll`, `libstdc++-6.dll`, Plugins etc.) direkt in `build/` bereitgestellt | Eigenständig per Doppelklick ausführbar |
| 2026-09-03T16:24:00+02:00 | WinMax Vollendung | Vollständige Implementierung aller Kern-Konzepte: Conversational Blocks, Materialdatenbank, Touch-Plate & 3D-Probe Zyklen, Wavefront OBJ Import, 3D Freiform-Schlichten | **100% aller Tests PASSED**, Executable deployed |
| 2026-09-03T16:29:30+02:00 | WinMax Geometrie | Konzeption der interaktiven Geometrie-Segment-Programmierung (Linien, Bögen CW/CCW, Fasen, Rundungen, Langlöcher, 3D-Helix/Gewinde) | `implementation_plan.md` erweitert |
| 2026-09-03T16:33:20+02:00 | WinMax UI & Softkeys | Konzeption der originalgetreuen Hurco WinMax Konsolen-Architektur: F1-F8 Softkeys, Split-Screen (Dialog links, 3D-Grafik rechts) und WinMax Header-Bar mit DRO | `implementation_plan.md` erweitert |
| 2026-09-03T16:40:00+02:00 | WinMax Vollendung | Vollständiger Umbau auf Hurco WinMax: F1-F8 Softkeys (Tastatur & Touch), Kopfzeile mit DRO G54, 50/50 Split-Screen, Kontur-Segment Editor (Linie, Bogen CW/CCW, Langloch, Helix) auf Deutsch | **100% aller Tests PASSED**, Executable deployed |

---

## 3. Verifikations-Ergebnisse (Unit-Tests)

- **`TestGeometry.exe`**:
  - `Mesh Box Stock Creation & BoundingBox`: PASSED
  - `STL Binary Save & Reload`: PASSED
  - `Contour Area, Perimeter & Offset`: PASSED
- **`TestCamEngine.exe`**:
  - `CAM Facing Toolpath Generation`: PASSED (G0, G1, M30 verifiziert)
  - `Collision Detector (Soft Limits & Rapid Dive)`: PASSED (Erkennt Überschreitung & Eilgang im Material)
  - `Collision Detector (Holder Collision / Stickout)`: PASSED (Erkennt Halteraufschlag bei unzureichender Auskraglänge)
- **`TestHardware.exe`**:
  - `Moonraker Mock Client Connection & Commands`: PASSED (Asynchrone Verbindung, Homing, TMC2209 Diagnoseabfrage)
- **`TestWinMax.exe`**:
  - `testObjLoader`: PASSED (Wavefront OBJ Export/Import, Triangulierung, AABB)
  - `testMaterialDatabase`: PASSED (Echtzeit $V_c / f_z$ Schnittwertkalkulation für Alu, POM, Messing, PMMA, Holz, Stahl)
  - `testConversationalProgram`: PASSED (Sequenzielle Bearbeitung, $M6$ Werkzeugwechsel, sichere Rückzüge, `.gprog` Persistenz)
  - `test3DSurfaceFinishing`: PASSED (Parallel Raster Schlichtbahnen über 3D Mesh mit Kugelkopffräser)
  - `testProbeController`: PASSED (Tool Setter Vermessung & Touch Plate Signalbehandlung im Mock-Modus)

---

## 4. Abgeschlossene Hurco WinMax Meilensteine

- [x] **Material- & Technologiedatenbank (`src/core/MaterialDatabase.h/.cpp`):**
  - Formel $S = \frac{V_c \cdot 1000}{\pi \cdot d}$ und $F = S \cdot z \cdot f_z$ mit automatischen $a_p / a_e$ Empfehlungen für 6 Industrie-Werkstoffe.
- [x] **Conversational Block-Architektur (`src/cam/ConversationalBlock.h/.cpp` & `ConversationalProgram.h/.cpp`):**
  - Sequenzielle Bearbeitungsblöcke: Planfräsen, Taschenfräsen, Konturfräsen, Bohrbilder (Lochkreis & Raster mit Spanbrechen $Q$) und NC-Merge.
  - Speichern und Laden von Programmen als `.gprog` (JSON).
- [x] **Interaktives Kontur-Picking im `Viewport3D`:**
  - Raycasting mit Z=0 Ebenenschnitt und automatischer Hervorhebung selektierter DXF-Konturen in Goldgelb.
- [x] **WinMax Arbeitsplan-Editor (`src/ui/dialogs/ConversationalEditorDialog.h/.cpp`):**
  - Duale WinMax-Arbeitsplanleiste mit Block-Reihenfolge, Werkzeugkopplung, Live-Schnittwertrechner und Gesamtlaufzeit-Schätzung.
- [x] **Wavefront OBJ 3D-Geometrie-Import (`src/geometry/ObjLoader.h/.cpp`):**
  - Vollständiger OBJ-Parser für 3D-Modelle neben STL und DXF.
- [x] **3D-Freiformflächen-Schlichten (`ToolpathGenerator::generate3DSurfaceFinishing`):**
  - Paralleles Rasterschlichten in Zickzack-Bahnen für Kugelkopffräser direkt auf STL- und OBJ-Meshes.
- [x] **Probing & Touch Plate System (`src/hardware/ProbeController.h/.cpp` & `src/ui/dialogs/ProbeDialog.h/.cpp`):**
  - Tisch-Werkzeugtaster (Tool Setter $G38.2$) mit Werkzeuglängen-Offset und Bruchkontrolle.
  - **Touch Plate Modus:** Einstellbare Plattendicke (z.B. 10.00 mm), Z-Touch-Plate Antasten und 3D-Ecken Touch Plate (X, Y, Z Gesamtnullung).
  - 3D-Taster: Kantenantastung, 4-Punkt-Bohrungszentrierung und **Schieflagen-Kompensation ($G68$)**.

- [x] Grundstruktur der Benutzeroberfläche und Modul-Architektur
- [x] QStackedWidget für sequentielle Dialogführung (Setup -> Werkzeuge -> CAM -> Simulation -> Jog -> Hardware)
- [x] Zentrales QOpenGLWidget für 60 FPS 3D-Vorschau mit iGPU-Optimierung
- [x] Geometrie-Import für STL (Binär/ASCII) & DXF für Rohteil und Bauteil
- [x] Werkzeugverwaltung & CAM-Berechnung (Planfräsen, Kontur, Tasche, 3D-Schruppen)
- [x] Manuelle Jog-Bedieneinheit (XYZ + optionale 4. Achse A, Homing, Nullung)
- [x] Echtzeit-Kollisionsüberwachung (Soft-Limits, Eilgang-Eintauchen, Halteraufschlag)
- [x] Klipper / Moonraker Vorbereitung mit umschaltbarem Windows Offline-Simulator und TMC2209-Abfragen
- [ ] *Optionale Erweiterung Phase 2*: Physischer Live-Test an Raspberry Pi / Klipper-Hardware mit TMC2209 UART-Telemetrie
- [ ] *Optionale Erweiterung Phase 2*: Zusätzliche Trochoidal-Fräsbahnen (Adaptive Clearing) für zähe Metalle
