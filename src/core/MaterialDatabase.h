#ifndef GEMINI_CNC_MATERIALDATABASE_H
#define GEMINI_CNC_MATERIALDATABASE_H

#include <QString>
#include <QList>
#include <QJsonObject>
#include "ToolDefinition.h"

namespace GeminiCNC::Core {

struct Material {
    int id{1};
    QString name{"Aluminium (EN AW-6060 / 6082)"};
    QString category{"Nichteisenmetall"};
    double vc{250.0};              // Schnittgeschwindigkeit Vc in m/min (für VHM Fräser)
    double fzBase{0.04};           // Basis-Vorschub pro Zahn fz für Ø 6mm (mm/Zahn)
    double maxStepDownRatio{0.5};  // Empfohlene max. axiale Zustellung ap relativ zu Ø (z.B. 0.5 * d)
    double maxStepOverRatio{0.5};  // Empfohlene radiale Zustellung ae relativ zu Ø
    double plungeRatio{0.4};       // Eintauchvorschub-Faktor (z.B. 40% des Arbeitsvorschubs)

    [[nodiscard]] QJsonObject toJson() const;
    [[nodiscard]] static Material fromJson(const QJsonObject& json);
};

struct CuttingParameters {
    double spindleRpm{18000.0};       // Spindeldrehzahl S (U/min)
    double feedRate{1500.0};          // Arbeitsvorschub F (mm/min)
    double plungeFeedRate{500.0};     // Eintauchvorschub Fz (mm/min)
    double recommendedStepDown{3.0};  // ap (mm)
    double recommendedStepOver{3.0};  // ae (mm)
};

/**
 * @brief Berechnet physikalische Schnittdaten basierend auf Werkstoff, Werkzeuggeometrie und Zähnezahl.
 */
class TechnologyCalculator {
public:
    [[nodiscard]] static CuttingParameters calculate(
        const Material& material,
        const ToolDefinition& tool,
        double maxSpindleRpm = 24000.0,
        double minSpindleRpm = 3000.0);
};

/**
 * @brief Werkstoffdatenbank mit vorkonfigurierten Industrie-Standardmaterialien.
 */
class MaterialDatabase {
public:
    MaterialDatabase();

    [[nodiscard]] const QList<Material>& materials() const { return m_materials; }
    [[nodiscard]] Material findById(int id) const;
    [[nodiscard]] Material findByName(const QString& name) const;

    void addMaterial(const Material& mat);
    void removeMaterial(int id);
    void initializeDefaults();

    [[nodiscard]] static MaterialDatabase createDefault();

private:
    QList<Material> m_materials;
};

} // namespace GeminiCNC::Core

#endif // GEMINI_CNC_MATERIALDATABASE_H
