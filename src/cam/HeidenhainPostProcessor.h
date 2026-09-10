#ifndef GEMINI_CNC_HEIDENHAINPOSTPROCESSOR_H
#define GEMINI_CNC_HEIDENHAINPOSTPROCESSOR_H

#include "PostProcessor.h"

namespace GeminiCNC::CAM {

/**
 * @brief Postprozessor-Stub für Heidenhain TNC / iTNC Steuerungen.
 *
 * Unterstützt perspektivisch:
 *  - Heidenhain Klartext-Programmierung (L X+.. Y+.. Z+.. F.. M..)
 *  - ISO-Modus (G-Code mit Heidenhain-Erweiterungen)
 *  - Zyklen: CYCL DEF 200 (Bohren), CYCL DEF 205 (Tieflochbohren),
 *            CYCL DEF 207 (Gewindebohren), CYCL DEF 262/263 (Gewindefräsen)
 *  - Bahnkorrektur: RL/RR (Rechts/Links der Kontur) oder G41/G42
 *  - Werkzeugruf: TOOL CALL .. Z S..
 *
 * Aktuell: Stub — gibt ISO-kompatiblen G-Code mit Heidenhain-Kommentaren aus.
 */
class HeidenhainPostProcessor : public PostProcessor {
public:
    [[nodiscard]] QString name() const override;
    [[nodiscard]] QString fileExtension() const override;

protected:
    void emitHeader(QTextStream& out, const Toolpath& tp,
                    const Core::MachineConfig& machine,
                    const PostProcessorContext& ctx) override;

    void emitFooter(QTextStream& out, const Toolpath& tp,
                    const Core::MachineConfig& machine) override;

    void emitToolChange(QTextStream& out, int toolId,
                        const Core::ToolDefinition& tool) override;

    [[nodiscard]] QString formatComment(const QString& text) const override;
};

} // namespace GeminiCNC::CAM

#endif // GEMINI_CNC_HEIDENHAINPOSTPROCESSOR_H
