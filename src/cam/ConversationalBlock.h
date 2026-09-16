#ifndef GEMINI_CNC_CONVERSATIONALBLOCK_H
#define GEMINI_CNC_CONVERSATIONALBLOCK_H

#include <QString>
#include <QJsonObject>
#include <vector>
#include "Toolpath.h"
#include "ToolpathGenerator.h"
#include "core/ToolDefinition.h"
#include "core/BoundingBox.h"
#include "geometry/Contour.h"

namespace GeminiCNC::CAM {

enum class BlockType {
    Facing,      // Planfräsen
    Contour,     // 2D-Konturfräsen
    Pocket,      // Taschenfräsen (Rechteck / Kreis / Freiform)
    Slot,        // Langloch / Nut
    HelixThread, // 3D-Helix & Gewindefräsen
    Drill,       // Bohrbild (Einzeln, Lochkreis, Raster)
    Stl3D,       // 3D-STL Freiformflächen-Fräsen
    RawNC,       // Direkter G-Code / Klipper Makros
    PatternStart,// Muster Start: wiederholt alle folgenden Blöcke bis Muster Ende
    PatternEnd,  // Muster Ende: schließt das zuletzt geöffnete Muster
    DrillPositions // Bohrpositionen: Lage der Bohrungen für den vorangehenden Bohrungen-Block (Hurco)
};

enum class PatternType {
    Linear = 0,  // Lineare Reihe (Anzahl, Abstand, Richtung)
    Rectangular, // Rechteckraster (Spalten × Zeilen)
    Circular,    // Kreismuster / Drehen um Mittelpunkt
    Mirror       // Spiegeln an senkrechter und/oder waagrechter Achse
};

QString blockTypeToString(BlockType type);
BlockType stringToBlockType(const QString& str);

enum class StlMillingStrategy {
    RoughAndFinishX = 0, // Komplett: Z-Ebenen Schruppen + Freiform-Schlichten X (Standard)
    RoughAndFinishY,     // Komplett: Z-Ebenen Schruppen + Freiform-Schlichten Y
    RoughOnly,           // Nur Z-Ebenen Schruppen (Waterline)
    RasterX,             // Nur Freiform-Schlichten (Parallel X)
    RasterY,             // Nur Freiform-Schlichten (Parallel Y)
    RasterXY,            // Kreuzraster-Schlichten (X dann Y) für perfekte Oberflächen
    WaterlineFinish      // Z-Ebenen Schlichten (Konstantes Z) für steile Wände
};

QString stlStrategyToString(StlMillingStrategy strat);
StlMillingStrategy stringToStlStrategy(const QString& str);

enum class DrillPattern {
    Single,      // Einzelbohrung
    BoltCircle,  // Lochkreis / Teilkreis
    Grid,        // Lochmatrix / Raster
    Line,        // Lochreihe (auf einer Linie mit Winkel)
    Arc,         // Bogenreihe (Kreisbogen mit Start-/Endwinkel)
    Frame,       // Rahmen-Muster (Umfang eines Rechtecks)
    Manual       // Manuelle Positionen (freie XY-Eingabe)
};

/**
 * @brief Bohrvorgang im Bohrungen-Block (Hurco WinMax: Bohren (Zyklen), Gewindebohren, Ausdrehen und Reiben).
 */
enum class DrillOperationType {
    Drill = 0,       // Bohrer
    CenterDrill,     // Zentrieren (Zentrierbohrer 60°)
    SpotFace,        // Flachsenken
    NcSpotDrill,     // NC-Anbohren (90°)
    Countersink,     // Kegelsenken (90°)
    PeckDrill,       // Tieflochbohrer
    CustomDrill,     // Benutzerdefiniertes Bohren
    Tap,             // Gewindebohrer (Ausgleichsfutter)
    RigidTap,        // Synchron-Gewindebohren
    Bore,            // Ausdrehen
    Ream             // Reiben
};

QString drillOperationName(DrillOperationType type);

// Bohr-Typ der Bohrzyklen (Hurco: STANDARD / VERWEILZEIT / SPANBRUCH / TIEFLOCH)
enum class DrillCycleType {
    Standard = 0,    // G81
    Dwell,           // G82 mit Verweilzeit
    ChipBreak,       // G73 Spanbruch (kurzer Rückzug)
    DeepHole         // G83 Tiefloch (Rückzug zur R-Ebene)
};

struct DrillOperation {
    DrillOperationType type{DrillOperationType::Drill};
    int toolId{1};
    double spindleRpm{1000.0};
    double plungeFeed{100.0};                   // Eintauchvorschub (mm/min)
    DrillCycleType cycleType{DrillCycleType::Standard};
    double peckDepth{0.0};                       // Stufentiefe (mm, 0 = ohne Stufen)
    double retractDistance{0.5};                 // Rückzugsabstand beim Spanbruch (mm)
    double dwellSec{0.0};                        // Verweilzeit am Grund (s)
    double diameter{0.0};                        // Senk-/Anbohrdurchmesser (Zentrieren, NC-Anbohren, Kegelsenken)
    double tipAngleDeg{90.0};                    // Spitzenwinkel des Werkzeugs
    bool ownDepth{false};                        // eigene Tiefe statt Z UNTEN des Blocks
    double depthZ{-1.0};                         // eigene Z UNTEN (Flachsenken oder ownDepth)
    double threadPitch{1.0};                     // Gewindesteigung (mm)
    bool boreSpindleStop{false};                 // Ausdrehen: Spindel halt + Eilgang zurück (G86) statt Vorschub (G85)

    [[nodiscard]] bool usesDrillCycleType() const;
    [[nodiscard]] bool usesDiameter() const;     // Tiefe ergibt sich aus Durchmesser und Spitzenwinkel
    [[nodiscard]] bool isTapping() const { return type == DrillOperationType::Tap || type == DrillOperationType::RigidTap; }
    // Endtiefe dieses Bohrvorgangs bei gegebener Z START / Z UNTEN des Blocks
    [[nodiscard]] double bottomZ(double blockStartZ, double blockBottomZ) const;
    [[nodiscard]] int cycleCode() const;         // PathSegment::drillCycle (0 G81/G82, 1 G83, 2 G73, 3 G84, 4 G85, 5 G86)
    [[nodiscard]] double effectiveFeed() const;  // Gewinde: Drehzahl × Steigung

    [[nodiscard]] QJsonObject toJson() const;
    [[nodiscard]] static DrillOperation fromJson(const QJsonObject& json);
    [[nodiscard]] static DrillOperation createDefault(DrillOperationType type, int toolId);
};

// Bedeutung einer gezeichneten Kontur (Hurco: Kontur, Tasche, Insel)
enum class ContourRole {
    Profile = 0,  // Fräsbahn entlang der Kontur (auf / innen / außen)
    Pocket = 1,   // Taschengrenze: Innenraum ausräumen
    Island = 2    // Insel der vorangehenden Tasche (bleibt stehen)
};

enum class MillingType {
    OnContour = 0, // Auf Kontur (Keine Radiuskorrektur)
    Inside = 1,    // Innen
    Outside = 2,   // Außen
    Pocket = 3     // Tasche (Voll ausräumen)
};

enum class FrameStartSide {
    Bottom = 0,
    Top = 1,
    Left = 2,
    Right = 3
};

enum class PocketShape {
    Rectangle,   // Parametrisches Rechteck (Hurco Frame)
    Circle,      // Parametrischer Kreis (Hurco Circle)
    CustomContour// Freie DXF-Kontur
};

/**
 * @brief Einzelner Bearbeitungsblock im Hurco WinMax-Stil.
 */
class ConversationalBlock {
public:
    int id{1};
    BlockType type{BlockType::Facing};
    QString name{"1: Planfräsen"};
    bool enabled{true};
    bool visible{true}; // Werkzeugweg im Viewport anzeigen
    MillingType millingType{MillingType::Pocket}; // Hurco: ON / INSIDE / OUTSIDE / POCKET
    FrameStartSide startSide{FrameStartSide::Bottom};

    // Werkzeug & Schnittwerte
    int toolId{1};
    int finishToolId{-1}; // Werkzeug für Schlichtgänge (-1 = Hauptwerkzeug nutzen)
    int materialId{1};
    double spindleRpm{18000.0};
    double feedRate{1500.0};
    double plungeFeedRate{500.0};

    // ═══ Globale Z-Ebenen ═══
    double startZ{0.0};          // Z-Startebene (Oberkante Werkstück)
    double targetZ{-2.0};        // Z-Zieltiefe
    double stepDown{1.0};        // Zustellung pro Durchgang (ap)
    double stepOver{3.0};        // Seitliche Zustellung (ae)
    double clearanceZ{5.0};      // Sicherheitsebene über Werkstück

    // ═══ Globale XY-Position (für alle Typen) ═══
    double posX{0.0};            // X-Position (Startecke oder Mittelpunkt)
    double posY{0.0};            // Y-Position (Startecke oder Mittelpunkt)

    // ═══ Globale Bearbeitungsoptionen ═══
    int millingDirection{0};     // 0 = Gleichlauf, 1 = Gegenlauf
    bool coolantOn{true};        // Kühlmittel ein/aus
    bool finishPass{false};      // Schlichtgang ja/nein
    double finishStepDown{0.2};  // Schlicht-Zustellung (mm)
    int approachType{0};         // 0 = Direkt, 1 = Tangential, 2 = Rampe

    // ═══ Planfräsen (Facing) ═══
    double areaWidth{100.0};     // Bearbeitungsbreite in X
    double areaDepth{80.0};      // Bearbeitungstiefe in Y
    bool useStockDimensions{true}; // Gesamtes Rohteil planen

    // ═══ Kontur ═══
    ContourSide contourSide{ContourSide::Outside};
    ContourRole contourRole{ContourRole::Profile};
    bool contourZForAll{false};  // Z UNTEN von Segment 0 gilt für alle Segmente (neue Blöcke: ja; ältere Dateien: Z je Segment)
    double finishAllowance{0.2}; // Schlichtaufmaß (mm)
    double leadRadius{2.0};      // An-/Abfahrt-Radius (mm)
    int leadType{1};             // 0 = Direkt, 1 = Tangentialbogen, 2 = Senkrecht
    bool useTabs{false};         // Haltestege verwenden
    double tabWidth{5.0};        // Haltesteg-Breite (mm)
    double tabHeight{1.0};       // Haltesteg-Höhe (mm)
    int tabCount{4};             // Anzahl Haltestege
    Geometry::Contour contour;
    std::vector<Geometry::ContourSegment> segments;

    // ═══ Tasche (Pocket) ═══
    PocketShape pocketShape{PocketShape::Rectangle};
    double pocketWidthX{40.0};   // Breite X
    double pocketDepthY{30.0};   // Tiefe Y
    double pocketRadius{15.0};   // Radius (Kreis oder Eckenradius)
    double pocketCornerR{3.0};   // Eckenradius bei Rechteck
    int pocketStrategy{0};       // 0 = Zickzack, 1 = Spiral, 2 = Konturparallel

    // -- Schrupp-/Schlicht-Optionen --
    bool enableFinishing{false};
    double finishAllowanceXY{0.0}; // Schlichtaufmaß Seite
    double finishAllowanceZ{0.0};  // Schlichtaufmaß Tiefe
    double finishFeedRate{1500.0};
    double finishSpindleRpm{18000.0};
    // finishToolId is already defined above

    // -- Insel-Management --
    std::vector<std::vector<Geometry::ContourSegment>> pocketIslands; // Lokale Insel-Konturen

    // ═══ Langloch (Slot) ═══
    double slotLength{50.0};     // Nutlänge
    double slotWidth{12.0};      // Nutbreite
    double slotAngleDeg{0.0};    // Drehwinkel
    double slotCornerR{0.0};     // Eckenradius (0 = volle Rundung)
    int slotCount{1};            // Anzahl Wiederholungen
    double slotSpacing{25.0};    // Abstand zwischen Wiederholungen

    // ═══ Helix & Gewindefräsen ═══
    double helixDiameter{20.0};  // Durchmesser
    double helixPitch{2.0};      // Steigung (mm/Umdrehung)
    bool helixInternal{true};    // True = Innengewinde, False = Außengewinde
    bool helixCW{true};          // True = Rechtsgewinde (CW), False = Links (CCW)
    int helixStarts{1};          // Mehrgängigkeit (1 = eingängig)

    // ═══ Bohren ═══
    DrillPattern drillPattern{DrillPattern::BoltCircle};
    // Bohrvorgänge in Reihenfolge (Hurco-Datensatz). Leer = älterer Einzelzyklus aus drillCycle/peckDepth/dwellTimeSec
    std::vector<DrillOperation> drillOps;
    // Bohrpositionen aus den folgenden Bohrpositionen-Blöcken (vom Programm vor der Berechnung gesetzt, nicht gespeichert)
    std::vector<std::pair<double, double>> resolvedDrillPositions;
    int drillCycle{0};           // älterer Einzelzyklus: 0 = G81, 1 = Tiefloch (G83), 2 = Spanbruch (G73), 3 = Gewinde (G84), 4 = Ausbohren (G85), 5 = Ausspindeln (G86)
    double peckDepth{2.0};       // Q-Tiefe für Spanbruch (mm)
    double dwellTimeSec{0.5};    // P-Verweilzeit am Grund (s)
    double boltCircleRadius{25.0};
    int boltCircleHoleCount{6};
    double boltCircleStartAngle{0.0};
    int gridCols{3};
    int gridRows{2};
    double gridPitchX{20.0};
    double gridPitchY{20.0};

    // Lochreihe (Line)
    int lineHoleCount{5};        // Anzahl Bohrungen in der Reihe
    double lineSpacing{15.0};    // Abstand zwischen Bohrungen (mm)
    double lineAngleDeg{0.0};    // Winkel der Reihe (°, 0 = X-Richtung)

    // Bogenreihe (Arc)
    double arcRadius{30.0};      // Bogenradius
    int arcHoleCount{5};         // Anzahl Bohrungen
    double arcStartAngle{0.0};   // Startwinkel (°)
    double arcEndAngle{180.0};   // Endwinkel (°)

    // Rahmen (Frame)
    double frameWidth{60.0};     // Rahmenbreite X (mm)
    double frameHeight{40.0};    // Rahmenhöhe Y (mm)
    int frameCountX{3};          // Bohrungen pro X-Seite (inkl. Ecken)
    int frameCountY{2};          // Bohrungen pro Y-Seite (ohne Ecken)

    // Manuelle Positionen
    std::vector<std::pair<double, double>> manualPositions;  // {X, Y} pro Bohrung

    // ═══ 3D-STL Freiformflächen-Fräsen ═══
    QString stlFilePath;
    StlMillingStrategy stlStrategy{StlMillingStrategy::RoughAndFinishX};
    double stlStepOver{1.0};     // Zeilenabstand ae (mm)
    double stlStepDown{2.0};     // Z-Zustellung ap (mm)
    double stlAllowance{0.0};    // Schlichtaufmaß (mm)
    double stlSampleStep{0.8};   // Abtastschrittweite entlang der Zeile (mm)
    bool stlUseStockDims{false}; // False = Nur Bauteilbereich (+ Fräserradius), True = Gesamtes Rohteil
    Geometry::Mesh directStlMesh; // Direkt im Block geladenes STL

    // Raw NC Code
    QString rawGCode;

    // ═══ Muster (Muster Start) ═══
    PatternType patternType{PatternType::Linear};
    int patternCountX{3};             // Anzahl (Linear/Kreis) bzw. Spalten (Raster)
    int patternCountY{2};             // Zeilen (Raster)
    double patternSpacingX{20.0};     // Abstand (Linear) bzw. Spaltenabstand (Raster)
    double patternSpacingY{20.0};     // Zeilenabstand (Raster)
    double patternAngleDeg{0.0};      // Richtung (Linear/Raster) bzw. Startwinkel (Kreis)
    double patternStepAngleDeg{0.0};  // Winkelschritt (Kreis, 0 = gleichmäßig auf 360°)
    double patternCenterX{0.0};       // Dreh-/Spiegelzentrum
    double patternCenterY{0.0};
    bool patternMirrorX{true};        // Spiegeln an senkrechter Achse (X → -X)
    bool patternMirrorY{false};       // Spiegeln an waagrechter Achse (Y → -Y)

    // Bohrungen-Block aus einem älteren Einzelzyklus in Hurco-Bohrvorgänge umwandeln
    void convertLegacyDrillCycle();

    // Bohrpositionen dieses Bohrpositionen-Blocks (bzw. eines älteren Bohrblocks mit eigenem Muster)
    [[nodiscard]] std::vector<Core::Vector3D> calculateDrillPositions() const;

    [[nodiscard]] bool isPatternBlock() const {
        return type == BlockType::PatternStart || type == BlockType::PatternEnd;
    }

    ConversationalBlock() = default;
    explicit ConversationalBlock(int blockId, BlockType blockType, const QString& blockName);

    /**
     * @brief Generiert die Werkzeugwege für diesen spezifischen Block.
     */
    [[nodiscard]] Toolpath generateToolpath(const Core::ToolDefinition& tool,
                                            const Core::ToolDefinition& finishTool,
                                            const Core::BoundingBox& stockBounds,
                                            const Geometry::Mesh& partMesh = Geometry::Mesh(),
                                            const QList<Core::ToolDefinition>& toolLibrary = {}) const;

    [[nodiscard]] QJsonObject toJson() const;
    [[nodiscard]] static ConversationalBlock fromJson(const QJsonObject& json);

};

} // namespace GeminiCNC::CAM

#endif // GEMINI_CNC_CONVERSATIONALBLOCK_H
