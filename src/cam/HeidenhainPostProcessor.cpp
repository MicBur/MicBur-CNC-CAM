#include "HeidenhainPostProcessor.h"

namespace GeminiCNC::CAM {

QString HeidenhainPostProcessor::name() const {
    return QStringLiteral("Heidenhain TNC 640 – ISO");
}

QString HeidenhainPostProcessor::fileExtension() const {
    return QStringLiteral(".h");
}

QString HeidenhainPostProcessor::formatComment(const QString& text) const {
    // Heidenhain: Semikolon-Kommentare
    return QStringLiteral("; %1").arg(text);
}

void HeidenhainPostProcessor::emitHeader(QTextStream& out, const Toolpath& tp,
                                          const Core::MachineConfig& machine,
                                          const PostProcessorContext& ctx) {
    Q_UNUSED(ctx)
    out << formatComment(QStringLiteral("==========================================")) << "\n";
    out << formatComment(QStringLiteral("GEMINI CNC – HEIDENHAIN TNC POSTPROZESSOR")) << "\n";
    out << formatComment(QStringLiteral("PROGRAMM: %1").arg(tp.operationName)) << "\n";
    out << formatComment(QStringLiteral("MASCHINE: %1").arg(machine.machineName)) << "\n";
    out << formatComment(QStringLiteral("SEGMENTE: %1").arg(tp.size())) << "\n";
    out << formatComment(QStringLiteral("==========================================")) << "\n\n";

    // Heidenhain ISO-Header
    out << formatLineNumber() << " G71 " << formatComment("METRISCH") << "\n";
    out << formatLineNumber() << " G17 " << formatComment("BEARBEITUNGSEBENE XY") << "\n";
    out << formatLineNumber() << " G90 " << formatComment("ABSOLUT") << "\n";
    out << formatLineNumber() << " G94 " << formatComment("VORSCHUB MM/MIN") << "\n";
    out << formatLineNumber() << " G40 " << formatComment("RADIUSKORREKTUR AUS") << "\n\n";
}

void HeidenhainPostProcessor::emitFooter(QTextStream& out, const Toolpath& tp,
                                          const Core::MachineConfig& machine) {
    Q_UNUSED(tp)
    out << "\n" << formatComment("--- PROGRAMMENDE ---") << "\n";
    out << formatLineNumber() << " M05 " << formatComment("SPINDEL STOPP") << "\n";
    out << formatLineNumber() << " G0 Z" << fmtCoord(machine.safeRetractZ + 20.0)
        << " " << formatComment("RUECKZUG") << "\n";
    out << formatLineNumber() << " M30 " << formatComment("PROGRAMMENDE") << "\n";
}

void HeidenhainPostProcessor::emitToolChange(QTextStream& out, int toolId,
                                              const Core::ToolDefinition& tool) {
    // Heidenhain: TOOL CALL in ISO-Modus oder Klartext
    // Stub: ISO-kompatibler Werkzeugwechsel
    out << "\n" << formatComment(QStringLiteral("--- WZW: T%1 %2 D%3mm ---")
           .arg(toolId).arg(tool.name).arg(tool.diameter)) << "\n";
    out << formatLineNumber() << " M05" << "\n";
    out << formatLineNumber() << " T" << toolId << " M06 "
        << formatComment(QStringLiteral("TOOL CALL %1").arg(toolId)) << "\n";
    out << formatLineNumber() << " G43 H" << toolId << " "
        << formatComment("LAENGENKORREKTUR") << "\n";
}

} // namespace GeminiCNC::CAM
