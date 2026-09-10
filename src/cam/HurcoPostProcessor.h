#ifndef GEMINI_CNC_HURCOPOSTPROCESSOR_H
#define GEMINI_CNC_HURCOPOSTPROCESSOR_H

#include "PostProcessor.h"

namespace GeminiCNC::CAM {

/**
 * @brief Postprozessor für Hurco VMX 30 / WinMax NC-Format.
 *
 * Besonderheiten:
 *  - Native G3.1-Unterstützung (Helical Interpolation / Gewindefräsen)
 *  - Hurco-Zyklen: G81-G89 Bohrzyklen mit L-Wiederholung
 *  - Kommentarformat: (Kommentar) statt ; Kommentar
 *  - Satznummern: N10, N20, ... im 10er-Inkrement
 *  - Werkzeugwechsel: G28 Z0 → T.. M06 → G43 H..
 *  - Optionaler D-Korrekturwert für G41/G42 aus Werkzeugtabelle
 */
class HurcoPostProcessor : public PostProcessor {
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

    void emitSpindleStart(QTextStream& out, double rpm, bool cw = true) override;

    void emitSegment(QTextStream& out, const PathSegment& seg,
                     const PostProcessorContext& ctx) override;

    void emitCrcOn(QTextStream& out, ContourSide side, int toolNumber) override;
    void emitCrcOff(QTextStream& out) override;

    bool emitNativeCycle(QTextStream& out, const Toolpath& tp,
                         const PostProcessorContext& ctx) override;

    [[nodiscard]] QString formatComment(const QString& text) const override;

private:
    /**
     * @brief Hurco G3.1 Gewindefräs-Zyklus nativ ausgeben.
     * G3.1 X.. Y.. Z.. I.. J.. K.. F.. (Helical Interpolation)
     */
    void emitHelicalThread(QTextStream& out, const PostProcessorContext& ctx);

    /**
     * @brief Hurco Bohrzyklen (G81, G83 mit Q, G73 mit Q, G84, G85, G86).
     */
    void emitDrillingCycle(QTextStream& out, const Toolpath& tp,
                           const PostProcessorContext& ctx);
};

} // namespace GeminiCNC::CAM

#endif // GEMINI_CNC_HURCOPOSTPROCESSOR_H
