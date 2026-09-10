#include "PostProcessor.h"
#include <QFile>

namespace GeminiCNC::CAM {

// ═══════════════════════════════════════════════════════════
// Modality-Tracking
// ═══════════════════════════════════════════════════════════

void PostProcessor::resetModality() {
    m_lastFeed = -1.0;
    m_lastSpindleRpm = -1.0;
    m_lastMotion = static_cast<MotionType>(-1); // Force emit first G0/G1
    m_lastToolId = -1;
    m_lineNumber = 10;
}

bool PostProcessor::feedChanged(double newFeed) {
    if (std::abs(newFeed - m_lastFeed) > 0.1) {
        m_lastFeed = newFeed;
        return true;
    }
    return false;
}

bool PostProcessor::spindleChanged(double newRpm) {
    if (std::abs(newRpm - m_lastSpindleRpm) > 1.0) {
        m_lastSpindleRpm = newRpm;
        return true;
    }
    return false;
}

bool PostProcessor::motionChanged(MotionType newMotion) {
    if (newMotion != m_lastMotion) {
        m_lastMotion = newMotion;
        return true;
    }
    return false;
}

QString PostProcessor::fmtCoord(double val, int decimals) const {
    return QString::number(val, 'f', decimals);
}

Core::ToolDefinition PostProcessor::findTool(const QList<Core::ToolDefinition>& tools, int toolId) {
    for (const auto& t : tools) {
        if (t.id == toolId) return t;
    }
    // Fallback
    return Core::ToolDefinition(toolId, QStringLiteral("T%1").arg(toolId), Core::ToolType::EndMill, 6.0);
}

QString PostProcessor::formatLineNumber() {
    QString ln = QStringLiteral("N%1").arg(m_lineNumber);
    m_lineNumber += m_lineIncrement;
    return ln;
}

QString PostProcessor::formatComment(const QString& text) const {
    return QStringLiteral("; %1").arg(text);
}

// ═══════════════════════════════════════════════════════════
// Basis-Implementierungen der virtuellen Hooks
// ═══════════════════════════════════════════════════════════

void PostProcessor::emitHeader(QTextStream& out, const Toolpath& tp,
                               const Core::MachineConfig& machine,
                               const PostProcessorContext& ctx) {
    Q_UNUSED(ctx)
    out << formatComment(QStringLiteral("==========================================")) << "\n";
    out << formatComment(QStringLiteral("Erstellt mit GeminiCNC – PostProcessor: %1").arg(name())) << "\n";
    out << formatComment(QStringLiteral("Maschine: %1").arg(machine.machineName)) << "\n";
    out << formatComment(QStringLiteral("Operation: %1").arg(tp.operationName)) << "\n";
    out << formatComment(QStringLiteral("Segmente: %1").arg(tp.size())) << "\n";
    out << formatComment(QStringLiteral("Gesamtweg: %1 mm").arg(QString::number(tp.totalLength(), 'f', 2))) << "\n";
    out << formatComment(QStringLiteral("Zeit: %1 min").arg(QString::number(tp.estimatedTotalTimeSeconds() / 60.0, 'f', 1))) << "\n";
    out << formatComment(QStringLiteral("==========================================")) << "\n\n";

    out << "G21" << " " << formatComment("Millimeter") << "\n";
    out << "G90" << " " << formatComment("Absolut") << "\n";
    out << "G17" << " " << formatComment("XY-Ebene") << "\n";
    out << "G94" << " " << formatComment("Vorschub mm/min") << "\n";
    out << "G40" << " " << formatComment("Radiuskorrektur AUS") << "\n\n";
}

void PostProcessor::emitFooter(QTextStream& out, const Toolpath& tp,
                               const Core::MachineConfig& machine) {
    Q_UNUSED(tp)
    out << "\n" << formatComment("--- Programmende ---") << "\n";
    out << "M05" << " " << formatComment("Spindel Stopp") << "\n";
    out << "G0 Z" << fmtCoord(machine.safeRetractZ + 15.0) << " " << formatComment("Rückzug") << "\n";
    out << "M30" << " " << formatComment("Programmende") << "\n";
}

void PostProcessor::emitToolChange(QTextStream& out, int toolId,
                                   const Core::ToolDefinition& tool) {
    out << "\n" << formatComment(QStringLiteral("--- Werkzeugwechsel: T%1 %2 (Ø%3mm) ---")
           .arg(toolId).arg(tool.name).arg(tool.diameter)) << "\n";
    out << "M05" << " " << formatComment("Spindel Stopp") << "\n";
    out << "T" << toolId << " M06" << " " << formatComment(QStringLiteral("Werkzeug T%1").arg(toolId)) << "\n";
}

void PostProcessor::emitSpindleStart(QTextStream& out, double rpm, bool cw) {
    out << "S" << static_cast<int>(rpm) << " " << (cw ? "M03" : "M04")
        << " " << formatComment("Spindel Start") << "\n";
}

void PostProcessor::emitSpindelStop(QTextStream& out) {
    out << "M05" << " " << formatComment("Spindel Stopp") << "\n";
}

void PostProcessor::emitCrcOn(QTextStream& out, ContourSide side, int toolNumber) {
    if (side == ContourSide::Inside) {
        out << formatLineNumber() << " G41 D" << toolNumber
            << " " << formatComment("Radiuskorrektur LINKS") << "\n";
    } else if (side == ContourSide::Outside) {
        out << formatLineNumber() << " G42 D" << toolNumber
            << " " << formatComment("Radiuskorrektur RECHTS") << "\n";
    }
}

void PostProcessor::emitCrcOff(QTextStream& out) {
    out << formatLineNumber() << " G40"
        << " " << formatComment("Radiuskorrektur AUS") << "\n";
}

void PostProcessor::emitSegment(QTextStream& out, const PathSegment& seg,
                                const PostProcessorContext& ctx) {
    Q_UNUSED(ctx)
    QString gCmd;
    bool emitG = motionChanged(seg.motion);
    switch (seg.motion) {
        case MotionType::Rapid:      gCmd = QStringLiteral("G0"); break;
        case MotionType::LinearFeed: gCmd = QStringLiteral("G1"); break;
        case MotionType::ArcCW:      gCmd = QStringLiteral("G2"); break;
        case MotionType::ArcCCW:     gCmd = QStringLiteral("G3"); break;
    }

    out << formatLineNumber() << " ";
    if (emitG) out << gCmd << " ";

    out << "X" << fmtCoord(seg.endPos.x) << " "
        << "Y" << fmtCoord(seg.endPos.y) << " "
        << "Z" << fmtCoord(seg.endPos.z);

    if (std::abs(seg.endPos.a) > 1e-4) {
        out << " A" << fmtCoord(seg.endPos.a);
    }

    if (seg.motion == MotionType::ArcCW || seg.motion == MotionType::ArcCCW) {
        double i = seg.arcCenter.x - seg.startPos.x;
        double j = seg.arcCenter.y - seg.startPos.y;
        out << " I" << fmtCoord(i) << " J" << fmtCoord(j);
    }

    if (seg.motion != MotionType::Rapid && feedChanged(seg.feedRate)) {
        out << " F" << fmtCoord(seg.feedRate, 1);
    }

    out << "\n";
}

bool PostProcessor::emitNativeCycle(QTextStream& out, const Toolpath& tp,
                                    const PostProcessorContext& ctx) {
    Q_UNUSED(out)
    Q_UNUSED(tp)
    Q_UNUSED(ctx)
    return false; // Basisklasse hat keine nativen Zyklen
}

// ═══════════════════════════════════════════════════════════
// Haupt-Verarbeitungspipeline
// ═══════════════════════════════════════════════════════════

QString PostProcessor::process(const Toolpath& toolpath,
                               const Core::MachineConfig& machine,
                               const QList<Core::ToolDefinition>& tools,
                               const PostProcessorContext& ctx) {
    QString output;
    QTextStream out(&output);
    resetModality();

    // 1. Header
    emitHeader(out, toolpath, machine, ctx);

    // 2. Spezialzyklus-Versuch (Gewinde, Bohren)
    if (ctx.isHelixOperation || ctx.isDrillingOperation) {
        if (emitNativeCycle(out, toolpath, ctx)) {
            emitFooter(out, toolpath, machine);
            return output;
        }
    }

    // 3. CRC aktivieren (wenn Controller-Modus und Seite != OnLine)
    bool crcActive = (ctx.crcMode == CrcOutputMode::ControllerCRC)
                     && (ctx.contourSide != ContourSide::OnLine);
    if (crcActive) {
        emitCrcOn(out, ctx.contourSide, ctx.toolNumber);
    }

    // 4. Segmente iterieren
    for (const auto& seg : toolpath.segments) {
        // Werkzeugwechsel erkennen
        if (seg.toolId > 0 && seg.toolId != m_lastToolId) {
            if (crcActive) {
                emitCrcOff(out);
            }
            auto tool = findTool(tools, seg.toolId);
            emitToolChange(out, seg.toolId, tool);
            m_lastToolId = seg.toolId;

            if (spindleChanged(seg.spindleRpm)) {
                emitSpindleStart(out, seg.spindleRpm);
            }
            if (crcActive) {
                emitCrcOn(out, ctx.contourSide, seg.toolId);
            }
        } else if (seg.spindleRpm > 0.0 && spindleChanged(seg.spindleRpm)) {
            emitSpindleStart(out, seg.spindleRpm);
        }

        // Bewegungssegment ausgeben
        emitSegment(out, seg, ctx);
    }

    // 5. CRC abschalten
    if (crcActive) {
        emitCrcOff(out);
    }

    // 6. Footer
    emitFooter(out, toolpath, machine);

    return output;
}

// ═══════════════════════════════════════════════════════════
// Controller-Typ String-Konvertierung
// ═══════════════════════════════════════════════════════════

QString controllerTypeToString(ControllerType type) {
    switch (type) {
        case ControllerType::ISO_Standard:  return QStringLiteral("ISO 6983 Standard");
        case ControllerType::Hurco_WinMax:  return QStringLiteral("Hurco WinMax");
        case ControllerType::Heidenhain:    return QStringLiteral("Heidenhain TNC");
        case ControllerType::Klipper:       return QStringLiteral("Klipper Firmware");
        case ControllerType::Saeilo:        return QStringLiteral("Saeilo / Mach3");
    }
    return QStringLiteral("Unbekannt");
}

ControllerType stringToControllerType(const QString& str) {
    if (str.contains(QStringLiteral("Hurco"), Qt::CaseInsensitive)) return ControllerType::Hurco_WinMax;
    if (str.contains(QStringLiteral("Heidenhain"), Qt::CaseInsensitive)) return ControllerType::Heidenhain;
    if (str.contains(QStringLiteral("Klipper"), Qt::CaseInsensitive)) return ControllerType::Klipper;
    if (str.contains(QStringLiteral("Saeilo"), Qt::CaseInsensitive) || str.contains(QStringLiteral("Mach"), Qt::CaseInsensitive)) return ControllerType::Saeilo;
    return ControllerType::ISO_Standard;
}

QStringList PostProcessorFactory::availableProfiles() {
    return {
        controllerTypeToString(ControllerType::ISO_Standard),
        controllerTypeToString(ControllerType::Hurco_WinMax),
        controllerTypeToString(ControllerType::Heidenhain),
        controllerTypeToString(ControllerType::Klipper),
        controllerTypeToString(ControllerType::Saeilo)
    };
}

} // namespace GeminiCNC::CAM
