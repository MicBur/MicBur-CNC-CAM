# Demo: STL 3D-Fräs-Simulation

Automatischer Test der MicBur-CNC-CAM Fräs-Pipeline.

## Dateien

| Datei | Beschreibung | Größe |
|-------|-------------|-------|
| `aschenbecher.stl` | Original-STL (Zielgeometrie) | 34 KB |
| `aschenbecher_gefraest.stl` | Simulationsergebnis nach 4 Frässtrategien | 3.1 MB |

## Aschenbecher — Geometrie

- Ø70 mm Zylinder, 15 mm Höhe
- Zentrale Mulde Ø56 mm, 10 mm tief
- 4 Zigarettenkerben (8 mm breit, 3 mm tief)
- Rohteil: 80 × 80 × 15 mm Aluminium

## Verwendete Bearbeitungsstrategien

| # | Strategie | Werkzeug | Segmente |
|---|-----------|----------|----------|
| 1 | Schruppen Z-Ebenen (Waterline) | T1 Ø6 mm Schaftfräser | 30.362 |
| 2 | Schlichten Raster X | T4 Ø4 mm Kugelfräser | 42.506 |
| 3 | Schlichten Raster Y (Kreuzschliff) | T4 Ø4 mm Kugelfräser | 42.506 |
| 4 | Schlichten Waterline (steile Wände) | T4 Ø4 mm Kugelfräser | 85.010 |

**Gesamt: 200.384 Werkzeugbahn-Segmente**

## Anzeigen

Beide STL-Dateien können in jedem 3D-Viewer geöffnet werden:
- Windows 3D-Viewer
- MeshLab
- Blender
- Online: [ViewSTL](https://www.viewstl.com/)
