#ifndef GEMINI_CNC_TOOLDEFINITION_H
#define GEMINI_CNC_TOOLDEFINITION_H

#include <QString>
#include <QJsonObject>
#include <QList>

namespace GeminiCNC::Core {

enum class ToolType {
    EndMill,     // Schaftfräser
    BallMill,    // Kugelkopffräser
    ChamferMill, // Fasenfräser
    FaceMill,    // Planfräser
    Drill        // Spiralbohrer
};

QString toolTypeToString(ToolType type);
ToolType stringToToolType(const QString& str);

/**
 * @brief Definition eines Zerspanungswerkzeugs für CAM und Kollisionsüberwachung.
 */
class ToolDefinition {
public:
    int id{1};
    QString name{"6mm Schaftfräser"};
    ToolType type{ToolType::EndMill};

    double diameter{6.0};           // Schneidendurchmesser (mm)
    double fluteLength{18.0};       // Schneidenlänge (mm)
    double shaftDiameter{6.0};      // Schaftdurchmesser (mm)
    double overallLength{50.0};     // Gesamtlänge (mm)
    double stickOutLength{30.0};    // Auskraglänge aus Halter (mm)
    double holderDiameter{32.0};    // Durchmesser des Werkzeughalters / Spannzange (mm)
    int flutes{2};                  // Zähnezahl

    double defaultFeedRate{1200.0}; // Vorschub F (mm/min)
    double plungeFeedRate{400.0};   // Eintauchvorschub (mm/min)
    double spindleSpeed{18000.0};   // Spindeldrehzahl S (U/min)
    double maxStepDown{2.0};        // Maximale axiale Schnitttiefe ap (mm)
    double stepOverPercentage{50.0};// Radiale Zustellung ae in % (z.B. 50% = 3mm)
    QString color{"#FFE614"};       // Frässpur-Farbe in Simulation (Standard: Hurco Signalgelb)

    ToolDefinition() = default;
    ToolDefinition(int toolId, const QString& toolName, ToolType toolType, double diam);

    [[nodiscard]] double effectiveStepOver() const;
    [[nodiscard]] QJsonObject toJson() const;
    [[nodiscard]] static ToolDefinition fromJson(const QJsonObject& json);

    // Standard-Bibliothek generieren
    [[nodiscard]] static QList<ToolDefinition> createDefaultLibrary();

    // Werkzeugbibliothek dauerhaft speichern/laden (JSON)
    [[nodiscard]] static QString defaultLibraryPath();
    [[nodiscard]] static bool saveLibrary(const QString& filePath, const QList<ToolDefinition>& tools, int activeToolId);
    [[nodiscard]] static bool loadLibrary(const QString& filePath, QList<ToolDefinition>& tools, int& activeToolId);
};

} // namespace GeminiCNC::Core

#endif // GEMINI_CNC_TOOLDEFINITION_H
