#ifndef GEMINI_CNC_TOOLPATH_H
#define GEMINI_CNC_TOOLPATH_H

#include "core/Vector3D.h"
#include "core/BoundingBox.h"
#include "core/MachineConfig.h"
#include "core/ToolDefinition.h"
#include <vector>
#include <QString>
#include <QStringList>
#include <QList>

namespace GeminiCNC::CAM {

enum class MotionType {
    Rapid,      // G00 Eilgang
    LinearFeed, // G01 Linearer Arbeitsvorschub
    ArcCW,      // G02 Kreisbogen im Uhrzeigersinn
    ArcCCW      // G03 Kreisbogen gegen den Uhrzeigersinn
};

QString motionTypeToString(MotionType type);

struct PathSegment {
    MotionType motion{MotionType::LinearFeed};
    Core::Vector3D startPos;
    Core::Vector3D endPos;
    Core::Vector3D arcCenter; // Nur für G02/G03

    double feedRate{1000.0};  // mm/min
    double spindleRpm{18000.0};
    bool coolantOn{false};

    int toolId{-1};
    double toolDiameter{0.0};  // Werkzeugdurchmesser für Simulation (mm), 0 = aktives Werkzeug
    int lineNumber{0};

    bool hasCollision{false};
    QString collisionWarning;
    bool visible{true}; // Sichtbarkeit im 3D Viewport
    int blockId{-1}; // Zugehöriger ConversationalBlock ID

    // Bohrzyklus (nur Bohrblöcke): 0..5 = G81/G83/G73/G84/G85/G86, -1 = kein Zyklus
    int drillCycle{-1};
    int drillOperation{-1};     // laufender Bohrvorgang innerhalb des Blocks
    int drillHole{-1};          // laufende Bohrungsnummer innerhalb des Blocks
    double drillDepthZ{0.0};    // Endtiefe Z
    double drillRPlaneZ{0.0};   // R-Ebene
    double drillPeck{0.0};      // Zustelltiefe Q
    double drillDwellSec{0.0};  // Verweilzeit P (s)

    [[nodiscard]] double length() const;
    [[nodiscard]] double estimatedSeconds() const;
};

/**
 * @brief Gesamte Fräsbahn bestehend aus einzelnen Bewegungssegmenten.
 */
class Toolpath {
public:
    std::vector<PathSegment> segments;
    QString operationName{"Konturfräsen"};
    bool visible{true};   // Sichtbarkeit im Viewport (pro Block)
    int blockIndex{-1};   // Zugehöriger Block-Index (-1 = unzugeordnet)

    Toolpath() = default;
    explicit Toolpath(const QString& name);

    void clear();
    void addSegment(const PathSegment& segment);

    [[nodiscard]] size_t size() const { return segments.size(); }
    [[nodiscard]] bool empty() const { return segments.empty(); }

    [[nodiscard]] double totalLength() const;
    [[nodiscard]] double totalRapidLength() const;
    [[nodiscard]] double totalFeedLength() const;
    [[nodiscard]] double estimatedTotalTimeSeconds() const;

    [[nodiscard]] Core::BoundingBox boundingBox() const;
    [[nodiscard]] bool hasAnyCollision() const;

    /**
     * @brief Exportiert G-Code über die PostProcessor-Pipeline.
     * @param controllerType Steuerungstyp (0=ISO, 1=Hurco, 2=Heidenhain, 3=Klipper, 4=Saeilo)
     * @param machineConfig Maschinenkonfiguration
     * @param tools Werkzeugbibliothek
     * @return Formatierter G-Code als QString
     */
    [[nodiscard]] QString exportWithPostProcessor(
        int controllerType = 0,
        const Core::MachineConfig& machineConfig = Core::MachineConfig(),
        const QList<Core::ToolDefinition>& tools = {}) const;

    /**
     * @brief Speichert G-Code über PostProcessor-Pipeline in eine Datei.
     * @return true bei Erfolg
     */
    [[nodiscard]] bool exportToFileWithPostProcessor(
        const QString& filePath,
        int controllerType = 0,
        const Core::MachineConfig& machineConfig = Core::MachineConfig(),
        const QList<Core::ToolDefinition>& tools = {}) const;

    /**
     * @brief [LEGACY] Generiert ISO G-Code direkt ohne PostProcessor.
     * @deprecated Nutze exportWithPostProcessor() stattdessen.
     */
    [[deprecated("Nutze exportWithPostProcessor()")]]
    [[nodiscard]] QString generateGCode(bool klipperExtended = false) const;

    /**
     * @brief [LEGACY] Speichert G-Code in Datei ohne PostProcessor.
     * @deprecated Nutze exportToFileWithPostProcessor() stattdessen.
     */
    [[deprecated("Nutze exportToFileWithPostProcessor()")]]
    [[nodiscard]] bool exportToFile(const QString& filePath, bool klipperExtended = false) const;

    /**
     * @brief Parst Standard G-Code (G0, G1, G2, G3, F, S, T, M3, M5, etc.).
     */
    [[nodiscard]] static Toolpath parseGCode(const QString& gcodeText, const QString& opName = "Importierter G-Code");
};

} // namespace GeminiCNC::CAM

#endif // GEMINI_CNC_TOOLPATH_H
