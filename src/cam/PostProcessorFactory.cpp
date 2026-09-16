#include "PostProcessor.h"
#include "HurcoPostProcessor.h"
#include "HeidenhainPostProcessor.h"

namespace GeminiCNC::CAM {

namespace {

// Generischer ISO 6983 G-Code: Basisklasse mit Default-Hooks
class IsoPostProcessor final : public PostProcessor {
public:
    [[nodiscard]] QString name() const override { return QStringLiteral("ISO 6983 Standard"); }
    [[nodiscard]] QString fileExtension() const override { return QStringLiteral(".nc"); }
};

// Saeilo / Mach3/4: ISO-nah, Kommentare in Klammern (ohne verschachtelte Klammern)
class SaeiloPostProcessor final : public PostProcessor {
public:
    [[nodiscard]] QString name() const override { return QStringLiteral("Saeilo / Mach3"); }
    [[nodiscard]] QString fileExtension() const override { return QStringLiteral(".nc"); }

protected:
    [[nodiscard]] QString formatComment(const QString& text) const override {
        QString clean = text;
        clean.replace(QLatin1Char('('), QLatin1Char('[')).replace(QLatin1Char(')'), QLatin1Char(']'));
        return QStringLiteral("(%1)").arg(clean);
    }
};

// Klipper Firmware: nur Befehle, die Klipper kennt. Keine Bohrzyklen, keine G2/G3
// (nur mit [gcode_arcs]), keine Satznummern; Spindel über M3/M5-Makros, Werkzeugwechsel per PAUSE.
class KlipperPostProcessor final : public PostProcessor {
public:
    [[nodiscard]] QString name() const override { return QStringLiteral("Klipper Firmware"); }
    [[nodiscard]] QString fileExtension() const override { return QStringLiteral(".gcode"); }

protected:
    [[nodiscard]] bool supportsArcs() const override { return false; }
    [[nodiscard]] bool supportsCannedCycles() const override { return false; }
    [[nodiscard]] QString formatLineNumber() override { return QString(); }

    void emitHeader(QTextStream& out, const Toolpath& tp, const Core::MachineConfig& machine,
                    const PostProcessorContext& ctx) override {
        Q_UNUSED(ctx)
        m_firstTool = true;
        out << formatComment(QStringLiteral("Erstellt mit GeminiCNC – Klipper")) << "\n";
        out << formatComment(QStringLiteral("Maschine: %1").arg(machine.machineName)) << "\n";
        out << formatComment(QStringLiteral("Operation: %1").arg(tp.operationName)) << "\n";
        out << "G90 " << formatComment(QStringLiteral("Absolut")) << "\n\n";
    }

    void emitFooter(QTextStream& out, const Toolpath& tp, const Core::MachineConfig& machine) override {
        Q_UNUSED(tp)
        out << "\n" << "M5 " << formatComment(QStringLiteral("Spindel Stopp (Makro)")) << "\n";
        out << "G0 Z" << fmtCoord(machine.safeRetractZ + 15.0) << " " << formatComment(QStringLiteral("Rueckzug")) << "\n";
    }

    void emitToolChange(QTextStream& out, int toolId, const Core::ToolDefinition& tool) override {
        out << "\n" << formatComment(QStringLiteral("Werkzeug T%1 %2 (D%3mm)").arg(toolId).arg(tool.name).arg(tool.diameter)) << "\n";
        if (!m_firstTool) {
            out << "M5 " << formatComment(QStringLiteral("Spindel Stopp (Makro)")) << "\n";
            out << "PAUSE " << formatComment(QStringLiteral("Werkzeug wechseln, dann RESUME")) << "\n";
        }
        m_firstTool = false;
    }

    void emitSpindleStart(QTextStream& out, double rpm, bool cw) override {
        out << (cw ? "M3" : "M4") << " S" << static_cast<int>(rpm) << " " << formatComment(QStringLiteral("Spindel (Makro)")) << "\n";
    }

    void emitSpindelStop(QTextStream& out) override {
        out << "M5\n";
    }

    void emitCoolant(QTextStream& out, bool on) override {
        out << formatComment(on ? QStringLiteral("Kuehlmittel ein") : QStringLiteral("Kuehlmittel aus")) << "\n";
    }

private:
    bool m_firstTool{true};
};

} // namespace

std::unique_ptr<PostProcessor> PostProcessorFactory::create(ControllerType type) {
    switch (type) {
        case ControllerType::Hurco_WinMax:
            return std::make_unique<HurcoPostProcessor>();
        case ControllerType::Heidenhain:
            return std::make_unique<HeidenhainPostProcessor>();
        case ControllerType::Klipper:
            return std::make_unique<KlipperPostProcessor>();
        case ControllerType::Saeilo:
            return std::make_unique<SaeiloPostProcessor>();
        case ControllerType::ISO_Standard:
        default:
            return std::make_unique<IsoPostProcessor>();
    }
}

} // namespace GeminiCNC::CAM
