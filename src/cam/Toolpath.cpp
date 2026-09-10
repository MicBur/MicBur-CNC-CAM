#include "Toolpath.h"
#include "PostProcessor.h"
#include <QFile>
#include <QTextStream>
#include <cmath>

namespace GeminiCNC::CAM {

QString motionTypeToString(MotionType type) {
    switch (type) {
        case MotionType::Rapid: return QStringLiteral("G00 Eilgang");
        case MotionType::LinearFeed: return QStringLiteral("G01 Vorschub");
        case MotionType::ArcCW: return QStringLiteral("G02 Kreis CW");
        case MotionType::ArcCCW: return QStringLiteral("G03 Kreis CCW");
    }
    return QStringLiteral("Unbekannt");
}

double PathSegment::length() const {
    if (motion == MotionType::LinearFeed || motion == MotionType::Rapid) {
        return startPos.distanceTo3D(endPos);
    }
    // Kreisbogen: Radius aus startPos zu arcCenter
    double r1 = startPos.distanceTo3D(arcCenter);
    double r2 = endPos.distanceTo3D(arcCenter);
    double r = (r1 + r2) * 0.5;
    if (r < 1e-6) return startPos.distanceTo3D(endPos);

    double chord = startPos.distanceTo3D(endPos);
    double sinHalfAngle = std::clamp(chord / (2.0 * r), -1.0, 1.0);
    double angle = 2.0 * std::asin(sinHalfAngle);
    return r * angle;
}

double PathSegment::estimatedSeconds() const {
    double f = feedRate;
    if (f < 1.0) f = 1000.0;
    return (length() / f) * 60.0;
}

Toolpath::Toolpath(const QString& name) : operationName(name) {}

void Toolpath::clear() {
    segments.clear();
}

void Toolpath::addSegment(const PathSegment& segment) {
    segments.push_back(segment);
}

double Toolpath::totalLength() const {
    double sum = 0.0;
    for (const auto& s : segments) sum += s.length();
    return sum;
}

double Toolpath::totalRapidLength() const {
    double sum = 0.0;
    for (const auto& s : segments) {
        if (s.motion == MotionType::Rapid) sum += s.length();
    }
    return sum;
}

double Toolpath::totalFeedLength() const {
    double sum = 0.0;
    for (const auto& s : segments) {
        if (s.motion != MotionType::Rapid) sum += s.length();
    }
    return sum;
}

double Toolpath::estimatedTotalTimeSeconds() const {
    double sum = 0.0;
    for (const auto& s : segments) sum += s.estimatedSeconds();
    return sum;
}

bool Toolpath::hasAnyCollision() const {
    for (const auto& s : segments) {
        if (s.hasCollision) return true;
    }
    return false;
}

Core::BoundingBox Toolpath::boundingBox() const {
    Core::BoundingBox box;
    for (const auto& s : segments) {
        box.expand(s.startPos);
        box.expand(s.endPos);
    }
    return box;
}

QString Toolpath::exportWithPostProcessor(
    int controllerType,
    const Core::MachineConfig& machineConfig,
    const QList<Core::ToolDefinition>& tools) const
{
    auto pp = PostProcessorFactory::create(static_cast<ControllerType>(controllerType));

    PostProcessorContext ctx;
    ctx.crcMode = static_cast<CrcOutputMode>(machineConfig.crcOutputMode);
    if (!tools.isEmpty()) {
        ctx.toolNumber = tools.first().id;
        ctx.toolRadius = tools.first().diameter * 0.5;
    }

    return pp->process(*this, machineConfig, tools, ctx);
}

bool Toolpath::exportToFileWithPostProcessor(
    const QString& filePath,
    int controllerType,
    const Core::MachineConfig& machineConfig,
    const QList<Core::ToolDefinition>& tools) const
{
    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        return false;
    }
    file.write(exportWithPostProcessor(controllerType, machineConfig, tools).toUtf8());
    file.close();
    return true;
}

// ── Legacy API (deprecated) ─────────────────────────────────────────────────

QString Toolpath::generateGCode(bool klipperExtended) const {
    QString out;
    QTextStream ts(&out);

    ts << "; ==========================================\n";
    ts << "; Erstellt mit Gemini CNC Control Software\n";
    ts << "; Operation: " << operationName << "\n";
    ts << "; Segmente: " << static_cast<qint64>(segments.size()) << "\n";
    ts << "; Gesamtweg: " << QString::number(totalLength(), 'f', 2) << " mm\n";
    ts << "; Geschaetzte Zeit: " << QString::number(estimatedTotalTimeSeconds() / 60.0, 'f', 1) << " min\n";
    ts << "; ==========================================\n\n";

    if (klipperExtended) {
        ts << "; --- Klipper Setup ---\n";
        ts << "G21 ; Millimeter Modus\n";
        ts << "G90 ; Absolute Positionierung\n";
        ts << "SET_GCODE_OFFSET Z=0\n";
        ts << "BED_MESH_PROFILE LOAD=default\n";
    } else {
        ts << "G21 ; mm\n";
        ts << "G90 ; Absolut\n";
        ts << "G17 ; XY Ebene\n";
        ts << "G94 ; mm/min Vorschub\n";
    }

    int currentTool = -1;
    double currentFeed = -1.0;
    double currentSpindle = -1.0;

    int lineNum = 10;
    for (const auto& s : segments) {
        if (s.toolId != currentTool && s.toolId > 0) {
            currentTool = s.toolId;
            ts << "M05 ; Spindel aus\n";
            ts << "T" << currentTool << " M06 ; Werkzeugwechsel T" << currentTool << "\n";
        }

        if (std::abs(s.spindleRpm - currentSpindle) > 1.0 && s.spindleRpm > 0.0) {
            currentSpindle = s.spindleRpm;
            ts << "S" << static_cast<int>(currentSpindle) << " M03 ; Spindel Start CW\n";
        }

        QString gCmd;
        switch (s.motion) {
            case MotionType::Rapid: gCmd = "G0"; break;
            case MotionType::LinearFeed: gCmd = "G1"; break;
            case MotionType::ArcCW: gCmd = "G2"; break;
            case MotionType::ArcCCW: gCmd = "G3"; break;
        }

        ts << "N" << lineNum << " " << gCmd
           << " X" << QString::number(s.endPos.x, 'f', 3)
           << " Y" << QString::number(s.endPos.y, 'f', 3)
           << " Z" << QString::number(s.endPos.z, 'f', 3);

        if (std::abs(s.endPos.a) > 1e-4) {
            ts << " A" << QString::number(s.endPos.a, 'f', 3);
        }

        if (s.motion == MotionType::ArcCW || s.motion == MotionType::ArcCCW) {
            double i = s.arcCenter.x - s.startPos.x;
            double j = s.arcCenter.y - s.startPos.y;
            ts << " I" << QString::number(i, 'f', 3)
               << " J" << QString::number(j, 'f', 3);
        }

        if (s.motion != MotionType::Rapid && std::abs(s.feedRate - currentFeed) > 0.1) {
            currentFeed = s.feedRate;
            ts << " F" << QString::number(currentFeed, 'f', 1);
        }

        ts << "\n";
        lineNum += 5;
    }

    ts << "\n; --- Programmende ---\n";
    ts << "M05 ; Spindel Stopp\n";
    ts << "G0 Z20.000 ; Auf sichere Rueckzugshoehe fahren\n";
    ts << "M30 ; Programmende / Rueckspulen\n";

    return out;
}

bool Toolpath::exportToFile(const QString& filePath, bool klipperExtended) const {
    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        return false;
    }
    QTextStream out(&file);
    out << generateGCode(klipperExtended);
    file.close();
    return true;
}

Toolpath Toolpath::parseGCode(const QString& gcodeText, const QString& opName) {
    Toolpath tp(opName);
    QTextStream stream(const_cast<QString*>(&gcodeText));

    Core::Vector3D currentPos{0.0, 0.0, 10.0, 0.0};
    double currentFeed = 1000.0;
    double currentSpindle = 18000.0;
    int currentTool = 1;
    int lineCounter = 1;

    while (!stream.atEnd()) {
        QString line = stream.readLine().trimmed();
        int commentIdx = line.indexOf(';');
        if (commentIdx >= 0) line = line.left(commentIdx).trimmed();
        int parenIdx = line.indexOf('(');
        if (parenIdx >= 0) line = line.left(parenIdx).trimmed();

        if (line.isEmpty()) continue;

        QStringList tokens = line.split(QLatin1Char(' '), Qt::SkipEmptyParts);
        MotionType motion = MotionType::LinearFeed;
        bool hasMove = false;
        Core::Vector3D targetPos = currentPos;
        double iOffset = 0.0, jOffset = 0.0;

        for (const auto& tok : tokens) {
            if (tok.isEmpty()) continue;
            QChar cmd = tok[0].toUpper();
            QString valStr = tok.mid(1);
            bool ok = false;
            double val = valStr.toDouble(&ok);

            if (cmd == 'G') {
                int gVal = static_cast<int>(val);
                if (gVal == 0) { motion = MotionType::Rapid; hasMove = true; }
                else if (gVal == 1) { motion = MotionType::LinearFeed; hasMove = true; }
                else if (gVal == 2) { motion = MotionType::ArcCW; hasMove = true; }
                else if (gVal == 3) { motion = MotionType::ArcCCW; hasMove = true; }
            } else if (cmd == 'X' && ok) { targetPos.x = val; hasMove = true; }
            else if (cmd == 'Y' && ok) { targetPos.y = val; hasMove = true; }
            else if (cmd == 'Z' && ok) { targetPos.z = val; hasMove = true; }
            else if (cmd == 'A' && ok) { targetPos.a = val; hasMove = true; }
            else if (cmd == 'I' && ok) { iOffset = val; }
            else if (cmd == 'J' && ok) { jOffset = val; }
            else if (cmd == 'F' && ok) { currentFeed = val; }
            else if (cmd == 'S' && ok) { currentSpindle = val; }
            else if (cmd == 'T' && ok) { currentTool = static_cast<int>(val); }
        }

        if (hasMove) {
            PathSegment seg;
            seg.motion = motion;
            seg.startPos = currentPos;
            seg.endPos = targetPos;
            if (motion == MotionType::ArcCW || motion == MotionType::ArcCCW) {
                seg.arcCenter = Core::Vector3D(currentPos.x + iOffset, currentPos.y + jOffset, currentPos.z);
            }
            seg.feedRate = currentFeed;
            seg.spindleRpm = currentSpindle;
            seg.toolId = currentTool;
            seg.lineNumber = lineCounter++;

            tp.addSegment(seg);
            currentPos = targetPos;
        }
    }

    return tp;
}

} // namespace GeminiCNC::CAM
