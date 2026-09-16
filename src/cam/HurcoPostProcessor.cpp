#include "HurcoPostProcessor.h"
#include <cmath>

namespace GeminiCNC::CAM {

// ═══════════════════════════════════════════════════════════
// Identifikation
// ═══════════════════════════════════════════════════════════

QString HurcoPostProcessor::name() const {
    return QStringLiteral("Hurco VMX 30 – WinMax NC");
}

QString HurcoPostProcessor::fileExtension() const {
    return QStringLiteral(".nc");
}

QString HurcoPostProcessor::formatComment(const QString& text) const {
    return QStringLiteral("(%1)").arg(text);
}

// ═══════════════════════════════════════════════════════════
// Header: Hurco-spezifischer Sicherheitsblock
// ═══════════════════════════════════════════════════════════

void HurcoPostProcessor::emitHeader(QTextStream& out, const Toolpath& tp,
                                     const Core::MachineConfig& machine,
                                     const PostProcessorContext& ctx) {
    Q_UNUSED(ctx)
    out << "%" << "\n";
    out << formatComment(QStringLiteral("GEMINI CNC – HURCO VMX 30 POSTPROZESSOR")) << "\n";
    out << formatComment(QStringLiteral("PROGRAMM: %1").arg(tp.operationName)) << "\n";
    out << formatComment(QStringLiteral("MASCHINE: %1").arg(machine.machineName)) << "\n";
    out << formatComment(QStringLiteral("SEGMENTE: %1  LAENGE: %2 mm  ZEIT: %3 min")
           .arg(tp.size())
           .arg(QString::number(tp.totalLength(), 'f', 1))
           .arg(QString::number(tp.estimatedTotalTimeSeconds() / 60.0, 'f', 1))) << "\n\n";

    // Hurco Sicherheitsblock (WinMax Standard-Einleitung)
    out << formatLineNumber() << " G21 " << formatComment("METRISCH") << "\n";
    out << formatLineNumber() << " G17 " << formatComment("XY-EBENE") << "\n";
    out << formatLineNumber() << " G90 " << formatComment("ABSOLUT") << "\n";
    out << formatLineNumber() << " G94 " << formatComment("VORSCHUB MM/MIN") << "\n";
    out << formatLineNumber() << " " << workOffsetCode(machine.workOffset) << " " << formatComment("WERKSTUECK-NULLPUNKT AKTIVIEREN") << "\n";
    out << formatLineNumber() << " G40 " << formatComment("RADIUSKORREKTUR AUS") << "\n";
    out << formatLineNumber() << " G49 " << formatComment("LAENGENKORREKTUR AUS") << "\n";
    out << formatLineNumber() << " G80 " << formatComment("FESTZYKLEN AUS") << "\n\n";
}

// ═══════════════════════════════════════════════════════════
// Footer: Hurco-spezifisches Programmende
// ═══════════════════════════════════════════════════════════

void HurcoPostProcessor::emitFooter(QTextStream& out, const Toolpath& tp,
                                     const Core::MachineConfig& machine) {
    Q_UNUSED(tp)
    out << "\n" << formatComment("--- PROGRAMMENDE ---") << "\n";
    out << formatLineNumber() << " M05 " << formatComment("SPINDEL STOPP") << "\n";
    out << formatLineNumber() << " M09 " << formatComment("KUEHLMITTEL AUS") << "\n";
    out << formatLineNumber() << " G91 G28 Z0 " << formatComment("Z-REFERENZ") << "\n";
    out << formatLineNumber() << " G91 G28 X0 Y0 " << formatComment("XY-REFERENZ") << "\n";
    out << formatLineNumber() << " G90 " << formatComment("ZURUECK AUF ABSOLUT") << "\n";
    out << formatLineNumber() << " M30 " << formatComment("PROGRAMMENDE") << "\n";
    out << "%" << "\n";
}

// ═══════════════════════════════════════════════════════════
// Werkzeugwechsel: G28 Z → T M06 → G43 H (Hurco-Sequenz)
// ═══════════════════════════════════════════════════════════

void HurcoPostProcessor::emitToolChange(QTextStream& out, int toolId,
                                         const Core::ToolDefinition& tool) {
    out << "\n" << formatComment(QStringLiteral("--- WZW: T%1 %2 D%3mm ---")
           .arg(toolId).arg(tool.name).arg(tool.diameter)) << "\n";
    out << formatLineNumber() << " M05 " << formatComment("SPINDEL STOPP") << "\n";
    out << formatLineNumber() << " M09 " << formatComment("KUEHLMITTEL AUS") << "\n";
    out << formatLineNumber() << " G91 G28 Z0 " << formatComment("Z-REF FUER WZW") << "\n";
    out << formatLineNumber() << " G90" << "\n";
    out << formatLineNumber() << " T" << toolId << " M06 "
        << formatComment(QStringLiteral("T%1 EINWECHSELN").arg(toolId)) << "\n";
    out << formatLineNumber() << " G43 H" << toolId << " "
        << formatComment("LAENGENKORREKTUR AKTIVIEREN") << "\n";
}

// ═══════════════════════════════════════════════════════════
// Spindel: M03/M04 mit Drehzahl
// ═══════════════════════════════════════════════════════════

void HurcoPostProcessor::emitSpindleStart(QTextStream& out, double rpm, bool cw) {
    out << formatLineNumber() << " S" << static_cast<int>(rpm) << " "
        << (cw ? "M03" : "M04") << " "
        << formatComment("SPINDEL START") << "\n";
}

// ═══════════════════════════════════════════════════════════
// Segment: G0/G1/G2/G3 mit Modality-Tracking
// ═══════════════════════════════════════════════════════════

void HurcoPostProcessor::emitSegment(QTextStream& out, const PathSegment& seg,
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

    out << "X" << fmtCoord(seg.endPos.x)
        << " Y" << fmtCoord(seg.endPos.y)
        << " Z" << fmtCoord(seg.endPos.z);

    if (std::abs(seg.endPos.a) > 1e-4) {
        out << " A" << fmtCoord(seg.endPos.a);
    }

    if (seg.motion == MotionType::ArcCW || seg.motion == MotionType::ArcCCW) {
        double i = seg.arcCenter.x - seg.startPos.x;
        double j = seg.arcCenter.y - seg.startPos.y;
        out << " I" << fmtCoord(i) << " J" << fmtCoord(j);
    }

    if (seg.motion != MotionType::Rapid && feedChanged(seg.feedRate)) {
        out << " F" << fmtCoord(seg.feedRate, 0);
    }

    out << "\n";
}

// ═══════════════════════════════════════════════════════════
// Radiuskorrektur: G41/G42 mit D-Nummer (Hurco)
// ═══════════════════════════════════════════════════════════

void HurcoPostProcessor::emitCrcOn(QTextStream& out, ContourSide side, int toolNumber) {
    if (side == ContourSide::Inside) {
        out << formatLineNumber() << " G41 D" << toolNumber << " "
            << formatComment("RADIUSKORR. LINKS") << "\n";
    } else if (side == ContourSide::Outside) {
        out << formatLineNumber() << " G42 D" << toolNumber << " "
            << formatComment("RADIUSKORR. RECHTS") << "\n";
    }
}

void HurcoPostProcessor::emitCrcOff(QTextStream& out) {
    out << formatLineNumber() << " G40 "
        << formatComment("RADIUSKORR. AUS") << "\n";
}

// ═══════════════════════════════════════════════════════════
// Native Zyklen: Hurco G3.1 Helical + Bohrzyklen
// ═══════════════════════════════════════════════════════════

bool HurcoPostProcessor::emitNativeCycle(QTextStream& out, const Toolpath& tp,
                                          const PostProcessorContext& ctx) {
    if (ctx.isHelixOperation && ctx.helixPitch > 0.0) {
        emitHelicalThread(out, ctx);
        return true;
    }
    if (ctx.isDrillingOperation) {
        emitDrillingCycle(out, tp, ctx);
        return true;
    }
    return false;
}

void HurcoPostProcessor::emitHelicalThread(QTextStream& out,
                                            const PostProcessorContext& ctx) {
    // Hurco G3.1: Helical Thread Milling
    // G3.1 = CCW helix, G2.1 = CW helix (Hurco-spezifisch)
    // Format: G3.1 X_ Y_ Z_ I_ J_ K_ F_
    //   X,Y = Kreismittelpunkt (Gewindezentrum)
    //   Z   = Zieltiefe (berechnet aus Toolpath-Kontext)
    //   I,J = Kreisradius-Offset (relativ zu Startposition)
    //   K   = Steigung pro Umdrehung (= helixPitch)
    //   F   = Vorschub

    double toolRadius = ctx.toolRadius;
    double pathRadius = ctx.helixInternal
        ? std::max(0.5, (ctx.helixDiameter - toolRadius * 2.0) * 0.5)
        : std::max(0.5, (ctx.helixDiameter + toolRadius * 2.0) * 0.5);

    // Rechtsgewinde = G3.1 (CCW Helix von oben betrachtet für Innengewinde)
    // Linksgewinde  = G2.1 (CW Helix)
    QString helixG = ctx.helixCW ? QStringLiteral("G3.1") : QStringLiteral("G2.1");

    out << "\n" << formatComment(QStringLiteral("=== GEWINDEFRAESEN M%1x%2 %3 ===")
           .arg(QString::number(ctx.helixDiameter, 'f', 1))
           .arg(QString::number(ctx.helixPitch, 'f', 2))
           .arg(ctx.helixInternal ? "INNEN" : "AUSSEN")) << "\n";
    out << formatComment(QStringLiteral("WERKZEUGRADIUS: %1mm  BAHNRADIUS: %2mm")
           .arg(QString::number(toolRadius, 'f', 2))
           .arg(QString::number(pathRadius, 'f', 3))) << "\n\n";

    // Anfahrt über Zentrum auf Sicherheitshöhe
    out << formatLineNumber() << " G0 X0.000 Y0.000 Z5.000 "
        << formatComment("POSITIONIERUNG UEBER ZENTRUM") << "\n";

    // Eintauchen auf Starttiefe (Oberkante Gewinde)
    out << formatLineNumber() << " G0 Z0.500 "
        << formatComment("ANNAEHERUNG") << "\n";

    // Seitlich auf Bahnradius positionieren
    out << formatLineNumber() << " G1 X" << fmtCoord(pathRadius) << " Y0.000 Z0.000 F200 "
        << formatComment("EINFAEDELN") << "\n";

    // Hurco G3.1 Helix-Interpolation
    out << formatLineNumber() << " " << helixG
        << " X" << fmtCoord(pathRadius)
        << " Y0.000"
        << " Z" << fmtCoord(-ctx.helixPitch)  // Eine volle Steigung abwärts
        << " I" << fmtCoord(-pathRadius)        // Kreismittelpunkt X-Offset
        << " J0.000"                             // Kreismittelpunkt Y-Offset
        << " K" << fmtCoord(ctx.helixPitch)     // Steigung
        << " F" << fmtCoord(100.0, 0)           // Gewindevorschub
        << " " << formatComment("HELIX-GEWINDEGANG") << "\n";

    // Rückzug: erst seitlich wegfahren, dann Z hoch
    out << formatLineNumber() << " G1 X0.000 Y0.000 F200 "
        << formatComment("AUSFAEDELN ZUM ZENTRUM") << "\n";
    out << formatLineNumber() << " G0 Z5.000 "
        << formatComment("RUECKZUG") << "\n";
}

void HurcoPostProcessor::emitDrillingCycle(QTextStream& out, const Toolpath& tp,
                                            const PostProcessorContext& ctx) {
    // Hurco Bohrzyklen: G81/G73/G83/G84/G85/G86
    // Format: G8x Z_ R_ [Q_] [P_] F_
    //   Z = Endtiefe, R = Rückzugsebene, Q = Spanbruchtiefe, P = Verweilzeit

    QString cycleG;
    switch (ctx.drillCycle) {
        case 0: cycleG = QStringLiteral("G81"); break; // Einfach
        case 1: cycleG = QStringLiteral("G83"); break; // Spanbruch (Tieflochbohren)
        case 2: cycleG = QStringLiteral("G73"); break; // Spänebrechen (Chip Break)
        case 3: cycleG = QStringLiteral("G84"); break; // Gewindebohren
        case 4: cycleG = QStringLiteral("G85"); break; // Ausbohren
        case 5: cycleG = QStringLiteral("G86"); break; // Ausspindeln
        default: cycleG = QStringLiteral("G81"); break;
    }

    out << "\n" << formatComment(QStringLiteral("=== BOHRZYKLUS %1 ===").arg(cycleG)) << "\n";

    // Bohrpositionen aus dem Toolpath extrahieren
    // Wir suchen die tiefsten Z-Positionen an jeder XY-Position
    double retractZ = 5.0;
    double targetZ = 0.0;
    if (!tp.segments.empty()) {
        targetZ = tp.segments.back().endPos.z;
        for (const auto& seg : tp.segments) {
            targetZ = std::min(targetZ, seg.endPos.z);
        }
    }

    // Zyklus definieren
    out << formatLineNumber() << " G99 " << formatComment("RUECKZUG AUF R-EBENE") << "\n";

    // Erste Bohrposition mit Zyklus-Definition
    bool firstHole = true;
    Core::Vector3D lastXY{-99999, -99999, 0};

    for (const auto& seg : tp.segments) {
        // Nur Abwärtsbewegungen = Bohrpositionen
        if (seg.endPos.z < seg.startPos.z && seg.motion != MotionType::Rapid) {
            double holeX = seg.endPos.x;
            double holeY = seg.endPos.y;

            // Doppelte Positionen überspringen
            if (std::abs(holeX - lastXY.x) < 0.01 && std::abs(holeY - lastXY.y) < 0.01) {
                continue;
            }
            lastXY = {holeX, holeY, 0};

            if (firstHole) {
                out << formatLineNumber() << " " << cycleG
                    << " X" << fmtCoord(holeX)
                    << " Y" << fmtCoord(holeY)
                    << " Z" << fmtCoord(targetZ)
                    << " R" << fmtCoord(retractZ);

                if (ctx.drillCycle == 1 || ctx.drillCycle == 2) {
                    out << " Q" << fmtCoord(ctx.peckDepth);
                }
                if (ctx.drillCycle == 4 || ctx.drillCycle == 5) {
                    out << " P" << static_cast<int>(ctx.dwellTime * 1000);
                }
                out << " F" << fmtCoord(seg.feedRate, 0) << "\n";
                firstHole = false;
            } else {
                // Folgende Löcher: nur XY
                out << formatLineNumber()
                    << " X" << fmtCoord(holeX)
                    << " Y" << fmtCoord(holeY) << "\n";
            }
        }
    }

    // Zyklus aufheben
    out << formatLineNumber() << " G80 " << formatComment("FESTZYKLUS AUS") << "\n";
}

} // namespace GeminiCNC::CAM
