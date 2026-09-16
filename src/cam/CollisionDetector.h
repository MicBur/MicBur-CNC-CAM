#ifndef GEMINI_CNC_COLLISIONDETECTOR_H
#define GEMINI_CNC_COLLISIONDETECTOR_H

#include "Toolpath.h"
#include "core/MachineConfig.h"
#include "core/ToolDefinition.h"
#include "core/BoundingBox.h"
#include "geometry/Mesh.h"
#include <vector>
#include <QString>

namespace GeminiCNC::CAM {

enum class CollisionSeverity {
    Warning, // z.B. Hoher Vorschub oder geringer Sicherheitsabstand
    Critical // z.B. Halterkollision, Eilgang-Eintauchen in Material, Achsüberlauf
};

struct CollisionViolation {
    int segmentIndex{0};
    int lineNumber{0};
    Core::Vector3D position;
    CollisionSeverity severity{CollisionSeverity::Critical};
    QString description;

    [[nodiscard]] QString toString() const;
};

struct CollisionReport {
    bool hasErrors{false};
    int totalViolations{0};
    std::vector<CollisionViolation> violations;

    void addViolation(const CollisionViolation& v);
    void clear();
};

/**
 * @brief Echtzeit-Kollisionsüberwachung für Werkzeugbahnen.
 */
class CollisionDetector {
public:
    /**
     * @brief Überprüft eine Werkzeugbahn auf Maschinengrenzen, Halterkollisionen und Eilgangfehler.
     */
    [[nodiscard]] static CollisionReport verifyToolpath(
        Toolpath& toolpath,
        const Core::MachineConfig& machine,
        const Core::ToolDefinition& tool,
        const Core::BoundingBox& stockBounds,
        const std::vector<Core::BoundingBox>& fixtures = {});

    /**
     * @brief Wie oben, aber prüft jedes Segment mit seinem eigenen Werkzeug (seg.toolId → Bibliothek).
     *        Unbekannte Werkzeug-IDs werden mit fallbackTool geprüft.
     */
    [[nodiscard]] static CollisionReport verifyToolpath(
        Toolpath& toolpath,
        const Core::MachineConfig& machine,
        const QList<Core::ToolDefinition>& tools,
        const Core::ToolDefinition& fallbackTool,
        const Core::BoundingBox& stockBounds,
        const std::vector<Core::BoundingBox>& fixtures = {});
};

} // namespace GeminiCNC::CAM

#endif // GEMINI_CNC_COLLISIONDETECTOR_H
