#include "CollisionDetector.h"

namespace GeminiCNC::CAM {

QString CollisionViolation::toString() const {
    const QString sevStr = (severity == CollisionSeverity::Critical) ? QStringLiteral("KRITISCH") : QStringLiteral("WARNUNG");
    return QStringLiteral("[%1] Segment #%2 (Zeile %3) bei %4: %5")
        .arg(sevStr)
        .arg(segmentIndex)
        .arg(lineNumber)
        .arg(position.toString(2))
        .arg(description);
}

void CollisionReport::addViolation(const CollisionViolation& v) {
    if (v.severity == CollisionSeverity::Critical) {
        hasErrors = true;
    }
    violations.push_back(v);
    totalViolations = static_cast<int>(violations.size());
}

void CollisionReport::clear() {
    hasErrors = false;
    totalViolations = 0;
    violations.clear();
}

CollisionReport CollisionDetector::verifyToolpath(
    Toolpath& toolpath,
    const Core::MachineConfig& machine,
    const Core::ToolDefinition& tool,
    const Core::BoundingBox& stockBounds,
    const std::vector<Core::BoundingBox>& fixtures) {
    return verifyToolpath(toolpath, machine, QList<Core::ToolDefinition>{}, tool, stockBounds, fixtures);
}

CollisionReport CollisionDetector::verifyToolpath(
    Toolpath& toolpath,
    const Core::MachineConfig& machine,
    const QList<Core::ToolDefinition>& tools,
    const Core::ToolDefinition& fallbackTool,
    const Core::BoundingBox& stockBounds,
    const std::vector<Core::BoundingBox>& fixtures) {

    CollisionReport report;

    auto resolveTool = [&](int toolId) -> const Core::ToolDefinition& {
        for (const auto& t : tools) {
            if (t.id == toolId) return t;
        }
        return fallbackTool;
    };

    for (size_t i = 0; i < toolpath.segments.size(); ++i) {
        auto& seg = toolpath.segments[i];
        const Core::ToolDefinition& tool = resolveTool(seg.toolId);
        seg.hasCollision = false;
        seg.collisionWarning.clear();

        // 1. Maschinengrenzen (Soft-Limits)
        if (!machine.isWithinLimits(seg.endPos)) {
            CollisionViolation v;
            v.segmentIndex = static_cast<int>(i);
            v.lineNumber = seg.lineNumber;
            v.position = seg.endPos;
            v.severity = CollisionSeverity::Critical;

            QString axisDetail;
            if (seg.endPos.x < machine.minLimits.x) axisDetail = QString("X-Min (%1 < %2)").arg(seg.endPos.x, 0, 'f', 1).arg(machine.minLimits.x, 0, 'f', 1);
            else if (seg.endPos.x > machine.maxLimits.x) axisDetail = QString("X-Max (%1 > %2)").arg(seg.endPos.x, 0, 'f', 1).arg(machine.maxLimits.x, 0, 'f', 1);
            else if (seg.endPos.y < machine.minLimits.y) axisDetail = QString("Y-Min (%1 < %2)").arg(seg.endPos.y, 0, 'f', 1).arg(machine.minLimits.y, 0, 'f', 1);
            else if (seg.endPos.y > machine.maxLimits.y) axisDetail = QString("Y-Max (%1 > %2)").arg(seg.endPos.y, 0, 'f', 1).arg(machine.maxLimits.y, 0, 'f', 1);
            else if (seg.endPos.z < machine.minLimits.z) axisDetail = QString("Z-Min (%1 < %2)").arg(seg.endPos.z, 0, 'f', 1).arg(machine.minLimits.z, 0, 'f', 1);
            else if (seg.endPos.z > machine.maxLimits.z) axisDetail = QString("Z-Max (%1 > %2)").arg(seg.endPos.z, 0, 'f', 1).arg(machine.maxLimits.z, 0, 'f', 1);

            v.description = QString("Achs-Endschalter überschritten! [%1]").arg(axisDetail);
            report.addViolation(v);

            seg.hasCollision = true;
            seg.collisionWarning = v.description;
            continue;
        }

        // 2. Eilgang-Kollision im Material (G0 unterhalb Rohteiloberkante)
        if (seg.motion == MotionType::Rapid && stockBounds.isValid()) {
            if (stockBounds.contains(seg.endPos) || seg.endPos.z < stockBounds.maxPoint.z - 0.05) {
                // Wenn Eilgang im horizontalen Bereich des Rohteils unterhalb der Oberfläche stattfindet
                if (seg.endPos.x >= stockBounds.minPoint.x && seg.endPos.x <= stockBounds.maxPoint.x &&
                    seg.endPos.y >= stockBounds.minPoint.y && seg.endPos.y <= stockBounds.maxPoint.y) {
                    CollisionViolation v;
                    v.segmentIndex = static_cast<int>(i);
                    v.lineNumber = seg.lineNumber;
                    v.position = seg.endPos;
                    v.severity = CollisionSeverity::Critical;
                    v.description = QStringLiteral("G00 Eilgang-Kollision im Rohteilmaterial!");
                    report.addViolation(v);

                    seg.hasCollision = true;
                    seg.collisionWarning = v.description;
                    continue;
                }
            }
        }

        // 3. Spindel ausgeschaltet während Arbeitsvorschub
        if (seg.motion != MotionType::Rapid && seg.spindleRpm < 100.0) {
            CollisionViolation v;
            v.segmentIndex = static_cast<int>(i);
            v.lineNumber = seg.lineNumber;
            v.position = seg.endPos;
            v.severity = CollisionSeverity::Critical;
            v.description = QStringLiteral("Arbeitsvorschub G01/G02 bei stehender Spindel (Werkzeugbruchgefahr)!");
            report.addViolation(v);

            seg.hasCollision = true;
            seg.collisionWarning = v.description;
            continue;
        }

        // 4. Halterkollision / Maximale Schnitttiefe (Stickout-Check)
        if (stockBounds.isValid()) {
            double cutDepthFromTop = stockBounds.maxPoint.z - seg.endPos.z;
            if (cutDepthFromTop > tool.stickOutLength) {
                CollisionViolation v;
                v.segmentIndex = static_cast<int>(i);
                v.lineNumber = seg.lineNumber;
                v.position = seg.endPos;
                v.severity = CollisionSeverity::Critical;
                v.description = QStringLiteral("Werkzeughalter schlägt auf Rohteil auf! Eintauchtiefe (%1 mm) übersteigt Auskraglänge (%2 mm).")
                    .arg(cutDepthFromTop, 0, 'f', 1)
                    .arg(tool.stickOutLength, 0, 'f', 1);
                report.addViolation(v);

                seg.hasCollision = true;
                seg.collisionWarning = v.description;
                continue;
            }

            if (cutDepthFromTop > tool.fluteLength + 2.0 && seg.motion != MotionType::Rapid) {
                CollisionViolation v;
                v.segmentIndex = static_cast<int>(i);
                v.lineNumber = seg.lineNumber;
                v.position = seg.endPos;
                v.severity = CollisionSeverity::Warning;
                v.description = QStringLiteral("Werkzeugschaft reibt im Schnittkanal (Tiefe %1 mm > Schneidenlänge %2 mm).")
                    .arg(cutDepthFromTop, 0, 'f', 1)
                    .arg(tool.fluteLength, 0, 'f', 1);
                report.addViolation(v);
            }
        }

        // 5. Kollision mit Spannmitteln / Klemmen
        for (const auto& fixtureBox : fixtures) {
            if (fixtureBox.contains(seg.endPos)) {
                CollisionViolation v;
                v.segmentIndex = static_cast<int>(i);
                v.lineNumber = seg.lineNumber;
                v.position = seg.endPos;
                v.severity = CollisionSeverity::Critical;
                v.description = QStringLiteral("Kollision mit Spannmittel / Schraubstock!");
                report.addViolation(v);

                seg.hasCollision = true;
                seg.collisionWarning = v.description;
                break;
            }
        }
    }

    return report;
}

} // namespace GeminiCNC::CAM
