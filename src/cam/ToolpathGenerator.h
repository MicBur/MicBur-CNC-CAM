#ifndef GEMINI_CNC_TOOLPATHGENERATOR_H
#define GEMINI_CNC_TOOLPATHGENERATOR_H

#include "Toolpath.h"
#include "core/ToolDefinition.h"
#include "core/BoundingBox.h"
#include "geometry/Mesh.h"
#include "geometry/Contour.h"

namespace GeminiCNC::CAM {

enum class ContourSide {
    Outside, // Werkzeug fährt außen an der Kontur
    Inside,  // Werkzeug fährt innen an der Kontur
    OnLine,  // Werkzeugmitte fährt direkt auf der Konturlinie
    Left,    // links der programmierten Richtung (Hurco: Gleichlauf) – wird vor der Bahnberechnung aufgelöst
    Right    // rechts der programmierten Richtung (Hurco: Gegenlauf)
};

struct FacingParams {
    double startZ{0.0};      // Rohteil-Oberkante
    double targetZ{-1.0};    // Fertigteil-Höhe nach Planfräsen
    double stepDown{0.5};    // Axiale Zustellung ap je Durchgang
    double stepOver{20.0};   // Radiale Überlappung (ae)
    double clearanceZ{5.0};  // Sicherheitshöhe
    double extension{5.0};   // Überlauf über Kante hinaus
};

struct ContourParams {
    double startZ{0.0};
    double targetZ{-5.0};
    double stepDown{1.5};
    ContourSide side{ContourSide::Outside};
    double finishAllowance{0.2}; // Schlichtaufmaß (mm)
    double clearanceZ{5.0};
    bool useRampEntry{false};

    // Fräsrichtung (Spindel M3): Gleichlauf = außen im, innen gegen den Uhrzeigersinn
    bool climbMilling{true};
    // An-/Abfahrt auf der materialabgewandten Seite: 0 = Direkt, 1 = Tangentialbogen, 2 = Senkrecht
    int leadType{0};
    double leadRadius{2.0};
    // Eintauchen: 0 = Senkrecht, 1/2 = Rampe entlang der Bahn
    int entryType{0};
    double rampAngleDeg{5.0};
    // Haltestege (nur geschlossene Konturen)
    bool useTabs{false};
    int tabCount{4};
    double tabWidth{5.0};   // Stegbreite am Werkstück (mm)
    double tabHeight{1.0};  // Steghöhe über Endtiefe (mm)

    // Tiefenprofil je Konturpunkt (Z ENDE der Segmente); leer = überall targetZ.
    // Jede Zustellebene fräst höchstens bis zu diesem Profil.
    std::vector<double> vertexZ;
};

struct PocketParams {
    double startZ{0.0};
    double targetZ{-5.0};
    double stepDown{1.5};
    double stepOverRatio{0.5}; // 50% Fräserdurchmesser
    double finishAllowance{0.2};
    double clearanceZ{5.0};

    // 0 = Zickzack, 1 = Spiral (innen → außen), 2 = Konturparallel (außen → innen)
    int strategy{1};
    bool climbMilling{true};
    // Eintauchen: 0 = Senkrecht, 1 = Helix, 2 = Rampe
    int entryType{0};
    double rampAngleDeg{5.0};
};

struct StockRoughingParams {
    double stepDown{2.0};
    double stepOverRatio{0.6};
    double finishAllowance{0.5};
    double clearanceZ{5.0};
};

struct SurfaceFinishingParams {
    double stepOver{0.5};     // Zeilenabstand (mm)
    double sampleStep{0.5};   // Abtastschrittweite entlang der Zeile (mm)
    double clearanceZ{5.0};   // Sicherheitshöhe
    bool rasterAlongX{true};  // True = X-Zeilen, False = Y-Zeilen
};

enum class StlMillingMode {
    RoughAndFinish, // 1. Schruppen mit Aufmaß + 2. Schlichten auf Endmaß
    RoughOnly,      // Nur Schruppen (Material schnell ausräumen)
    FinishOnly,     // Nur Schlichten (Freiformfläche glätten)
    FinishRasterXY, // Kreuzraster-Schlichten in X und Y
    WaterlineFinish // Z-Ebenen-Schlichten (konstantes Z) für steile Wände
};

struct StlMillingParams {
    StlMillingMode mode{StlMillingMode::RoughAndFinish};
    int finishDirection{0};         // 0 = X-Richtung, 1 = Y-Richtung
    double roughStepDown{2.0};      // ap beim Schruppen (mm)
    double roughStepOverRatio{0.6}; // ae beim Schruppen (60% Werkzeug-Ø)
    double finishStepOver{0.8};     // ae beim Schlichten (mm)
    double finishAllowance{0.4};    // Schlichtaufmaß beim Schruppen (mm)
    double sampleStep{0.8};         // Punktabtastung entlang der Zeile (mm)
    double clearanceZ{5.0};         // Sicherheitshöhe über Werkstück (mm)
    bool useTrochoidal{false};      // Trochoidales Schruppen (geringer ae, hoher Vorschub)
    double trochoidalEngagement{0.0}; // ae/d (0 = auto aus Material)
    double trochoidalFeedFactor{0.0}; // Vorschub-Multiplikator (0 = auto)
};

/**
 * @brief CAM-Rechenkern zur Erzeugung von Werkzeugwegen aus 2D/3D-Geometrien.
 */
class ToolpathGenerator {
public:
    /**
     * @brief Erzeugt Planfräsbahnen (Facing) über einem Rohteilbereich.
     */
    [[nodiscard]] static Toolpath generateFacing(const Core::BoundingBox& stockBounds,
                                                const Core::ToolDefinition& tool,
                                                const FacingParams& params);

    /**
     * @brief Erzeugt 2D-Kontur-Fräsbahnen mit Z-Ebenen-Zustellung.
     */
    [[nodiscard]] static Toolpath generateContourMilling(const Geometry::Contour& contour,
                                                        const Core::ToolDefinition& tool,
                                                        const ContourParams& params);

    /**
     * @brief Erzeugt Taschen-Fräsbahnen (Pocketing) OHNE Inseln.
     */
    [[nodiscard]] static Toolpath generatePocketMilling(const Geometry::Contour& boundary,
                                                       const Core::ToolDefinition& tool,
                                                       const PocketParams& params);

    /**
     * @brief Erzeugt Taschen-Fräsbahnen (Pocketing) MIT Inseln.
     */
    [[nodiscard]] static Toolpath generatePocketMilling(const Geometry::Contour& boundary,
                                                       const std::vector<Geometry::Contour>& islands,
                                                       const Core::ToolDefinition& tool,
                                                       const PocketParams& params);

    /**
     * @brief Erzeugt 3D-Schruppbahnen (Rohteil zu Fertigteil Z-Level Slicing).
     */
    [[nodiscard]] static Toolpath generateStockRoughing(const Geometry::Mesh& stockMesh,
                                                       const Geometry::Mesh& targetPartMesh,
                                                       const Core::ToolDefinition& tool,
                                                       const StockRoughingParams& params);

    /**
     * @brief Erzeugt 3D-Freiformflächen-Schlichtbahnen (Parallel Finishing mit Kugelkopffräser).
     */
    [[nodiscard]] static Toolpath generate3DSurfaceFinishing(const Geometry::Mesh& targetPartMesh,
                                                            const Core::ToolDefinition& tool,
                                                            const SurfaceFinishingParams& params);

    /**
     * @brief Erzeugt 3D-STL Fräsbahnen (Raster-Schlichten in X/Y oder Z-Level Schruppen) über einem Dreiecksnetz.
     */
    [[nodiscard]] static Toolpath generateStlMilling(const Geometry::Mesh& mesh,
                                                    const Core::BoundingBox& stockBounds,
                                                    const Core::ToolDefinition& tool,
                                                    const StlMillingParams& params);

    // ═══ Trochoidales Fräsen ═══

    /**
     * @brief Erzeugt trochoidale Nutbahnen entlang einer Linie.
     * Kreisförmige Schnittbögen mit geringem ae, hohem Vorschub, voller Tiefe.
     */
    [[nodiscard]] static Toolpath generateTrochoidalSlot(
        double startX, double startY,
        double endX, double endY,
        double slotWidth,
        const Core::ToolDefinition& tool,
        double startZ, double targetZ, double clearanceZ,
        double engagement, double feedFactor,
        bool climbMilling = true);

    /**
     * @brief Erzeugt trochoidale Taschenbahnen (Spirale + Konturreinigung).
     */
    [[nodiscard]] static Toolpath generateTrochoidalPocket(
        const Geometry::Contour& boundary,
        const Core::ToolDefinition& tool,
        double startZ, double targetZ, double clearanceZ,
        double engagement, double feedFactor,
        bool climbMilling = true);
};

} // namespace GeminiCNC::CAM

#endif // GEMINI_CNC_TOOLPATHGENERATOR_H
