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
    OnLine   // Werkzeugmitte fährt direkt auf der Konturlinie
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
};

struct PocketParams {
    double startZ{0.0};
    double targetZ{-5.0};
    double stepDown{1.5};
    double stepOverRatio{0.5}; // 50% Fräserdurchmesser
    double finishAllowance{0.2};
    double clearanceZ{5.0};
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
};

} // namespace GeminiCNC::CAM

#endif // GEMINI_CNC_TOOLPATHGENERATOR_H
