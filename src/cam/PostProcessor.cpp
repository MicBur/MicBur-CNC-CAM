#include "PostProcessor.h"
#include <QFile>
#include <algorithm>
#include <cmath>

namespace GeminiCNC::CAM {

namespace {

constexpr double kPi = 3.14159265358979323846;

// Erkennt ab Index i einen lückenlosen Geradenzug (gleicher Vorschub, Werkzeug, Kühlmittel) auf einem
// Kreisbogen: bis 180°, einheitliche Drehrichtung, Z konstant oder linear über dem Winkel (Helix).
// Rückgabe: Anzahl zusammengefasster Segmente (0 = kein Bogen); arc = Ersatz-Bogensegment.
size_t fitArc(const std::vector<PathSegment>& segs, size_t i, PathSegment& arc) {
    constexpr double tol = 0.01;       // zulässige Radiusabweichung der Stützpunkte (mm)
    constexpr size_t minSegments = 3;
    constexpr size_t maxSegments = 360;

    const PathSegment& first = segs[i];
    if (first.motion != MotionType::LinearFeed || first.drillCycle >= 0) return 0;

    std::vector<const PathSegment*> run{&first};
    for (size_t j = i + 1; j < segs.size() && run.size() < maxSegments; ++j) {
        const PathSegment& s = segs[j];
        const bool sameKind = s.motion == MotionType::LinearFeed && s.drillCycle < 0
            && std::abs(s.feedRate - first.feedRate) < 1e-6 && s.toolId == first.toolId
            && s.coolantOn == first.coolantOn && std::abs(s.spindleRpm - first.spindleRpm) < 1e-6
            && s.startPos.distanceTo3D(run.back()->endPos) < 1e-6;
        if (!sameKind) break;
        run.push_back(&s);
    }
    if (run.size() < minSegments) return 0;

    auto fits = [&run](size_t n, double& cx, double& cy, bool& cw) {
        const Core::Vector3D& a = run[0]->startPos;
        const Core::Vector3D& m = run[n / 2]->startPos;
        const Core::Vector3D& b = run[n - 1]->endPos;

        // Umkreismittelpunkt aus Anfang, Mitte und Ende (XY)
        const double d = 2.0 * (a.x * (m.y - b.y) + m.x * (b.y - a.y) + b.x * (a.y - m.y));
        if (std::abs(d) < 1e-9) return false;
        const double a2 = a.x * a.x + a.y * a.y;
        const double m2 = m.x * m.x + m.y * m.y;
        const double b2 = b.x * b.x + b.y * b.y;
        cx = (a2 * (m.y - b.y) + m2 * (b.y - a.y) + b2 * (a.y - m.y)) / d;
        cy = (a2 * (b.x - m.x) + m2 * (a.x - b.x) + b2 * (m.x - a.x)) / d;
        const double r = std::hypot(a.x - cx, a.y - cy);
        if (r < 0.05 || r > 5000.0) return false;

        std::vector<double> sweepAt{0.0};
        double sweep = 0.0;
        double prevAngle = std::atan2(a.y - cy, a.x - cx);
        int dir = 0;
        for (size_t k = 0; k < n; ++k) {
            const Core::Vector3D& p = run[k]->endPos;
            if (std::abs(std::hypot(p.x - cx, p.y - cy) - r) > tol) return false;
            const double angle = std::atan2(p.y - cy, p.x - cx);
            double step = angle - prevAngle;
            while (step > kPi) step -= 2.0 * kPi;
            while (step < -kPi) step += 2.0 * kPi;
            if (std::abs(step) < 1e-9 || std::abs(step) > kPi / 4.0) return false; // max. 45° je Teilstück
            const int stepDir = step > 0.0 ? 1 : -1;
            if (dir != 0 && stepDir != dir) return false;
            dir = stepDir;
            sweep += step;
            sweepAt.push_back(sweep);
            prevAngle = angle;
        }
        if (std::abs(sweep) > kPi + 1e-9) return false; // max. 180° je Bogen

        // Z konstant oder linear über dem Winkel (Helix)
        for (size_t k = 1; k <= n; ++k) {
            const double expected = a.z + (b.z - a.z) * (sweepAt[k] / sweep);
            if (std::abs(run[k - 1]->endPos.z - expected) > tol) return false;
        }
        cw = dir < 0;
        return true;
    };

    size_t best = 0;
    double bestCx = 0.0;
    double bestCy = 0.0;
    bool bestCw = false;
    for (size_t n = minSegments; n <= run.size(); ++n) {
        double cx = 0.0;
        double cy = 0.0;
        bool cw = false;
        if (!fits(n, cx, cy, cw)) break;
        best = n;
        bestCx = cx;
        bestCy = cy;
        bestCw = cw;
    }
    if (best < minSegments) return 0;

    arc = first;
    arc.motion = bestCw ? MotionType::ArcCW : MotionType::ArcCCW;
    arc.endPos = run[best - 1]->endPos;
    arc.arcCenter = Core::Vector3D(bestCx, bestCy, first.startPos.z);
    return best;
}

} // namespace

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

void PostProcessor::forgetMotionModality() {
    m_lastMotion = static_cast<MotionType>(-1);
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

QString PostProcessor::workOffsetCode(int index) {
    return QStringLiteral("G%1").arg(54 + std::clamp(index, 0, 5));
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
    out << "G40" << " " << formatComment("Radiuskorrektur AUS") << "\n";
    out << "G80" << " " << formatComment("Zyklen AUS") << "\n";
    out << workOffsetCode(machine.workOffset) << " " << formatComment("Werkstueck-Nullpunkt") << "\n\n";
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
    out << "G43 H" << toolId << " " << formatComment(QStringLiteral("Werkzeuglaengenkorrektur")) << "\n";
}

void PostProcessor::emitSpindleStart(QTextStream& out, double rpm, bool cw) {
    out << "S" << static_cast<int>(rpm) << " " << (cw ? "M03" : "M04")
        << " " << formatComment("Spindel Start") << "\n";
}

void PostProcessor::emitSpindelStop(QTextStream& out) {
    out << "M05" << " " << formatComment("Spindel Stopp") << "\n";
}

void PostProcessor::emitCoolant(QTextStream& out, bool on) {
    out << (on ? "M08 " : "M09 ")
        << formatComment(on ? QStringLiteral("Kuehlmittel ein") : QStringLiteral("Kuehlmittel aus")) << "\n";
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

void PostProcessor::emitCannedDrillCycle(QTextStream& out, const std::vector<const PathSegment*>& holes, double feed) {
    if (holes.empty()) return;
    const PathSegment& first = *holes.front();

    QString code;
    switch (first.drillCycle) {
        case 1: code = QStringLiteral("G83"); break; // Spanbruch mit Rückzug zur R-Ebene
        case 2: code = QStringLiteral("G73"); break; // Spanbruch, kurzer Rückzug
        case 3: code = QStringLiteral("G84"); break; // Gewindebohren
        case 4: code = QStringLiteral("G85"); break; // Ausbohren
        case 5: code = QStringLiteral("G86"); break; // Ausspindeln
        default: code = (first.drillDwellSec > 1e-6) ? QStringLiteral("G82") : QStringLiteral("G81"); break;
    }

    // G98: zwischen den Bohrungen zurück auf die Ausgangshöhe (Sicherheitsebene)
    out << formatLineNumber() << " G98 " << code
        << " X" << fmtCoord(first.endPos.x) << " Y" << fmtCoord(first.endPos.y)
        << " Z" << fmtCoord(first.drillDepthZ) << " R" << fmtCoord(first.drillRPlaneZ);
    if (first.drillCycle == 1 || first.drillCycle == 2) {
        out << " Q" << fmtCoord(first.drillPeck);
    }
    if (code == QStringLiteral("G82")) {
        out << " P" << static_cast<int>(std::lround(first.drillDwellSec * 1000.0));
    }
    if (feed > 0.0) {
        out << " F" << fmtCoord(feed, 1);
        (void)feedChanged(feed);
    }
    out << " " << formatComment(QStringLiteral("Bohrzyklus")) << "\n";

    for (size_t k = 1; k < holes.size(); ++k) {
        out << formatLineNumber() << " X" << fmtCoord(holes[k]->endPos.x) << " Y" << fmtCoord(holes[k]->endPos.y) << "\n";
    }
    out << formatLineNumber() << " G80 " << formatComment(QStringLiteral("Zyklus aus")) << "\n";
    forgetMotionModality();
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
    bool coolantActive = false;
    const auto& segs = toolpath.segments;
    for (size_t i = 0; i < segs.size(); ++i) {
        const auto& seg = segs[i];

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

        // Kühlmittel bei Zustandswechsel schalten
        if (seg.coolantOn != coolantActive) {
            emitCoolant(out, seg.coolantOn);
            coolantActive = seg.coolantOn;
        }

        // Bohrzyklus: alle Bohrungen des Blocks als G8x-Zyklus statt Einzelbewegungen
        if (seg.drillCycle >= 0 && supportsCannedCycles()) {
            std::vector<const PathSegment*> holes;
            double drillFeed = 0.0;
            int lastHole = -1;
            size_t j = i;
            while (j < segs.size() && segs[j].drillCycle == seg.drillCycle
                   && segs[j].blockId == seg.blockId && segs[j].toolId == seg.toolId) {
                if (segs[j].drillHole != lastHole) {
                    holes.push_back(&segs[j]);
                    lastHole = segs[j].drillHole;
                }
                if (drillFeed <= 0.0 && segs[j].motion != MotionType::Rapid) drillFeed = segs[j].feedRate;
                ++j;
            }
            emitSegment(out, seg, ctx); // Anfahrt über die erste Bohrung auf Sicherheitshöhe
            emitCannedDrillCycle(out, holes, drillFeed);
            i = j - 1;
            continue;
        }

        // Kreisbogen-Erkennung: Geradenzüge auf einem Kreis als G2/G3
        if (supportsArcs() && seg.motion == MotionType::LinearFeed) {
            PathSegment arc;
            const size_t used = fitArc(segs, i, arc);
            if (used > 0) {
                emitSegment(out, arc, ctx);
                i += used - 1;
                continue;
            }
        }

        // Bewegungssegment ausgeben
        emitSegment(out, seg, ctx);
    }

    if (coolantActive) {
        emitCoolant(out, false);
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
