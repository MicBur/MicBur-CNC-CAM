#ifndef GEMINI_CNC_POSTPROCESSOR_H
#define GEMINI_CNC_POSTPROCESSOR_H

#include "cam/Toolpath.h"
#include "cam/ToolpathGenerator.h"
#include "core/ToolDefinition.h"
#include "core/MachineConfig.h"
#include <QString>
#include <QTextStream>
#include <QList>
#include <memory>
#include <cmath>

namespace GeminiCNC::CAM {

/**
 * @brief Modus für die Schneidradiuskorrektur-Ausgabe.
 *
 * CenterLine: Software berechnet alles vor — Postprozessor gibt pure XYZ-Daten aus.
 * ControllerCRC: Postprozessor gibt G41/G42 + Originalkontur aus, die Steuerung korrigiert.
 */
enum class CrcOutputMode {
    CenterLine,    // Software-Kompensation: reine Mittelpunktsbahn
    ControllerCRC  // Steuerungs-Kompensation: G41/G42 + Originalgeometrie
};

/**
 * @brief Kontext-Informationen die vom CAM-Kern an den Postprozessor übergeben werden.
 *
 * Enthält alles was ein Postprozessor braucht um maschinenspezifische Entscheidungen
 * zu treffen (z.B. ob ein Helix-Zyklus nativ unterstützt wird).
 */
struct PostProcessorContext {
    CrcOutputMode crcMode{CrcOutputMode::CenterLine};
    ContourSide contourSide{ContourSide::OnLine};       // Links/Rechts/Auf Kontur
    double toolRadius{3.0};                              // Effektiver Werkzeugradius (mm)
    int toolNumber{1};                                   // Werkzeugnummer für T-Befehl
    double helixDiameter{0.0};                           // Gewinde-Nenndurchmesser (für native Zyklen)
    double helixPitch{0.0};                              // Gewindesteigung (für native Zyklen)
    bool helixInternal{true};                            // Innen-/Außengewinde
    bool helixCW{true};                                  // Rechts-/Linksgewinde
    bool isHelixOperation{false};                        // Markiert HelixThread-Blöcke
    bool isDrillingOperation{false};                     // Markiert Bohrzyklen
    int drillCycle{0};                                   // G81/G83/G73/G84/G85/G86
    double peckDepth{2.0};                               // Spanbruchtiefe Q
    double dwellTime{0.5};                               // Verweilzeit P (Sekunden)
};

/**
 * @brief Abstrakte Basisklasse für alle G-Code-Postprozessoren.
 *
 * Implementiert Modality-Tracking (G-, F-, S-Werte werden nur bei Änderung ausgegeben)
 * und definiert die virtuelle Schnittstelle für maschinenspezifische Ableitungen.
 */
class PostProcessor {
public:
    virtual ~PostProcessor() = default;

    /**
     * @brief Erzeugt den vollständigen G-Code-String für eine Toolpath.
     */
    [[nodiscard]] QString process(const Toolpath& toolpath,
                                  const Core::MachineConfig& machine,
                                  const QList<Core::ToolDefinition>& tools,
                                  const PostProcessorContext& ctx = {});

    /**
     * @brief Menschenlesbarer Name des Postprozessors (z.B. "Hurco VMX 30 – WinMax").
     */
    [[nodiscard]] virtual QString name() const = 0;

    /**
     * @brief Dateiendung für den Export (z.B. ".nc", ".h", ".cnc").
     */
    [[nodiscard]] virtual QString fileExtension() const = 0;

protected:
    // ── Virtuelle Hooks die Subklassen überschreiben ──

    /** Programmkopf (Sicherheitsblock, G-Code-Defaults, Kommentare). */
    virtual void emitHeader(QTextStream& out, const Toolpath& tp,
                            const Core::MachineConfig& machine,
                            const PostProcessorContext& ctx);

    /** Programmfuß (M30/M02, Rückzug). */
    virtual void emitFooter(QTextStream& out, const Toolpath& tp,
                            const Core::MachineConfig& machine);

    /** Werkzeugwechsel-Sequenz. */
    virtual void emitToolChange(QTextStream& out, int toolId,
                                const Core::ToolDefinition& tool);

    /** Spindelstart (M03/M04 + S). */
    virtual void emitSpindleStart(QTextStream& out, double rpm, bool cw = true);

    /** Spindelstopp. */
    virtual void emitSpindelStop(QTextStream& out);

    /** Einzelnes Bewegungssegment in G-Code umwandeln. */
    virtual void emitSegment(QTextStream& out, const PathSegment& seg,
                             const PostProcessorContext& ctx);

    /** Schneidradiuskorrektur aktivieren (G41/G42). */
    virtual void emitCrcOn(QTextStream& out, ContourSide side, int toolNumber);

    /** Schneidradiuskorrektur deaktivieren (G40). */
    virtual void emitCrcOff(QTextStream& out);

    /**
     * @brief Hook für Spezialzyklen (Gewinde, Bohren).
     * @return true wenn der PP den Zyklus nativ ausgegeben hat (Toolpath-Segmente werden übersprungen).
     *         false wenn der PP keinen nativen Zyklus hat und die Segmente normal ausgegeben werden sollen.
     */
    virtual bool emitNativeCycle(QTextStream& out, const Toolpath& tp,
                                 const PostProcessorContext& ctx);

    /** Zeilenkommentar erzeugen (steuerungsspezifisches Format). */
    [[nodiscard]] virtual QString formatComment(const QString& text) const;

    /** Satznummer formatieren ("N10", "N20", ...). */
    [[nodiscard]] virtual QString formatLineNumber();

    // ── Modality-Tracking (in Basisklasse) ──

    void resetModality();
    [[nodiscard]] bool feedChanged(double newFeed);
    [[nodiscard]] bool spindleChanged(double newRpm);
    [[nodiscard]] bool motionChanged(MotionType newMotion);

    // ── Helper ──

    /** Findet ToolDefinition anhand toolId in der Library. */
    [[nodiscard]] static Core::ToolDefinition findTool(const QList<Core::ToolDefinition>& tools, int toolId);

    /** Formatiert eine Koordinate mit konfigurierbarer Genauigkeit. */
    [[nodiscard]] QString fmtCoord(double val, int decimals = 3) const;

    int m_lineNumber{10};
    int m_lineIncrement{10};

private:
    // Modality-State
    double m_lastFeed{-1.0};
    double m_lastSpindleRpm{-1.0};
    MotionType m_lastMotion{MotionType::Rapid};
    int m_lastToolId{-1};
};

// ═══════════════════════════════════════════════════════════
// PostProcessor Factory
// ═══════════════════════════════════════════════════════════

/**
 * @brief Maschinensteuerungs-Typ für die Factory.
 */
enum class ControllerType {
    ISO_Standard,  // Generischer ISO 6983 G-Code
    Hurco_WinMax,  // Hurco VMX / WinMax NC (G3.1, Zyklen)
    Heidenhain,    // Heidenhain TNC / iTNC (Klartext / ISO)
    Klipper,       // Klipper Firmware (3D-Drucker mit Frässpindel)
    Saeilo         // Saeilo / Mach3/4 kompatibel
};

QString controllerTypeToString(ControllerType type);
ControllerType stringToControllerType(const QString& str);

/**
 * @brief Factory für die Erzeugung maschinenspezifischer Postprozessoren.
 */
class PostProcessorFactory {
public:
    /**
     * @brief Erzeugt den passenden PostProcessor für den gewünschten Controller.
     */
    [[nodiscard]] static std::unique_ptr<PostProcessor> create(ControllerType type);

    /**
     * @brief Liste aller verfügbaren Controller-Profile (für UI-Dropdown).
     */
    [[nodiscard]] static QStringList availableProfiles();
};

} // namespace GeminiCNC::CAM

#endif // GEMINI_CNC_POSTPROCESSOR_H
