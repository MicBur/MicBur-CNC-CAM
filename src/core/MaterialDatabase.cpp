#include "MaterialDatabase.h"
#include <cmath>
#include <algorithm>

namespace GeminiCNC::Core {

QJsonObject Material::toJson() const {
    QJsonObject obj;
    obj[QStringLiteral("id")] = id;
    obj[QStringLiteral("name")] = name;
    obj[QStringLiteral("category")] = category;
    obj[QStringLiteral("vc")] = vc;
    obj[QStringLiteral("fzBase")] = fzBase;
    obj[QStringLiteral("maxStepDownRatio")] = maxStepDownRatio;
    obj[QStringLiteral("maxStepOverRatio")] = maxStepOverRatio;
    obj[QStringLiteral("plungeRatio")] = plungeRatio;
    obj[QStringLiteral("trochoidalEngagement")] = trochoidalEngagement;
    obj[QStringLiteral("trochoidalFeedFactor")] = trochoidalFeedFactor;
    return obj;
}

Material Material::fromJson(const QJsonObject& json) {
    Material m;
    if (json.contains(QStringLiteral("id"))) m.id = json[QStringLiteral("id")].toInt();
    if (json.contains(QStringLiteral("name"))) m.name = json[QStringLiteral("name")].toString();
    if (json.contains(QStringLiteral("category"))) m.category = json[QStringLiteral("category")].toString();
    if (json.contains(QStringLiteral("vc"))) m.vc = json[QStringLiteral("vc")].toDouble();
    if (json.contains(QStringLiteral("fzBase"))) m.fzBase = json[QStringLiteral("fzBase")].toDouble();
    if (json.contains(QStringLiteral("maxStepDownRatio"))) m.maxStepDownRatio = json[QStringLiteral("maxStepDownRatio")].toDouble();
    if (json.contains(QStringLiteral("maxStepOverRatio"))) m.maxStepOverRatio = json[QStringLiteral("maxStepOverRatio")].toDouble();
    if (json.contains(QStringLiteral("plungeRatio"))) m.plungeRatio = json[QStringLiteral("plungeRatio")].toDouble();
    if (json.contains(QStringLiteral("trochoidalEngagement"))) m.trochoidalEngagement = json[QStringLiteral("trochoidalEngagement")].toDouble();
    if (json.contains(QStringLiteral("trochoidalFeedFactor"))) m.trochoidalFeedFactor = json[QStringLiteral("trochoidalFeedFactor")].toDouble();
    return m;
}

CuttingParameters TechnologyCalculator::calculate(
    const Material& material,
    const ToolDefinition& tool,
    double maxSpindleRpm,
    double minSpindleRpm) {

    CuttingParameters params;
    const double d = std::max(0.5, tool.diameter);
    const int z = std::max(1, tool.flutes);

    // 1. Spindeldrehzahl: n = (Vc * 1000) / (pi * d)
    double rawRpm = (material.vc * 1000.0) / (M_PI * d);
    params.spindleRpm = std::clamp(std::round(rawRpm / 100.0) * 100.0, minSpindleRpm, maxSpindleRpm);

    // 2. Zahnvorschub fz skalieren basierend auf Fräserdurchmesser
    // Für kleinere Fräser (z.B. 2mm) sinkt der Zahnvorschub quadratisch ab
    double sizeFactor = std::sqrt(d / 6.0);
    double fz = material.fzBase * sizeFactor;

    // 3. Vorschub vf = n * z * fz
    double rawFeed = params.spindleRpm * z * fz;
    params.feedRate = std::round(rawFeed / 10.0) * 10.0;

    // 4. Eintauchvorschub
    params.plungeFeedRate = std::round(params.feedRate * material.plungeRatio / 10.0) * 10.0;

    // 5. Zustellungen ap und ae
    params.recommendedStepDown = std::max(0.2, d * material.maxStepDownRatio);
    params.recommendedStepOver = std::max(0.2, d * material.maxStepOverRatio);

    return params;
}

CuttingParameters TechnologyCalculator::calculateTrochoidal(
    const Material& material,
    const ToolDefinition& tool,
    double maxSpindleRpm,
    double minSpindleRpm) {

    // Basis-Schnittdaten berechnen
    CuttingParameters params = calculate(material, tool, maxSpindleRpm, minSpindleRpm);

    const double d = std::max(0.5, tool.diameter);
    const double engagement = std::clamp(material.trochoidalEngagement, 0.02, 0.25);
    const double feedFactor = std::clamp(material.trochoidalFeedFactor, 1.0, 4.0);

    // Trochoidale Zustellung: ae = d * engagement (z.B. 5-15% je nach Material)
    params.recommendedStepOver = std::max(0.1, d * engagement);

    // Volle Schneidtiefe: ap = Schneidlänge * 0.8 (nahezu volle Nutlänge)
    const double fluteLen = tool.fluteLength > 0.5 ? tool.fluteLength : d * 2.0;
    params.recommendedStepDown = std::max(0.5, fluteLen * 0.8);

    // Vorschub erhöhen (geringere Schnittkräfte bei niedrigem ae)
    params.feedRate = std::round(params.feedRate * feedFactor / 10.0) * 10.0;

    // Eintauchvorschub bleibt konservativ
    params.plungeFeedRate = std::round(params.feedRate * material.plungeRatio / 10.0) * 10.0;

    return params;
}

MaterialDatabase::MaterialDatabase() {
    initializeDefaults();
}

Material MaterialDatabase::findById(int id) const {
    for (const auto& m : m_materials) {
        if (m.id == id) return m;
    }
    return m_materials.isEmpty() ? Material{} : m_materials.first();
}

Material MaterialDatabase::findByName(const QString& name) const {
    for (const auto& m : m_materials) {
        if (m.name.compare(name, Qt::CaseInsensitive) == 0) return m;
    }
    return findById(1);
}

void MaterialDatabase::addMaterial(const Material& mat) {
    m_materials.append(mat);
}

void MaterialDatabase::removeMaterial(int id) {
    for (int i = 0; i < m_materials.size(); ++i) {
        if (m_materials[i].id == id) {
            m_materials.removeAt(i);
            break;
        }
    }
}

void MaterialDatabase::initializeDefaults() {
    m_materials.clear();

    // 1. Aluminium
    Material mAlu;
    mAlu.id = 1;
    mAlu.name = QStringLiteral("Aluminium (EN AW-6060 / 6082)");
    mAlu.category = QStringLiteral("Nichteisenmetall");
    mAlu.vc = 260.0;
    mAlu.fzBase = 0.045;
    mAlu.maxStepDownRatio = 0.5;
    mAlu.maxStepOverRatio = 0.45;
    mAlu.plungeRatio = 0.35;
    mAlu.trochoidalEngagement = 0.15;  // 15% — weiches Material
    mAlu.trochoidalFeedFactor = 2.5;
    addMaterial(mAlu);

    // 2. POM / Delrin
    Material mPom;
    mPom.id = 2;
    mPom.name = QStringLiteral("Kunststoff POM-C (Delrin)");
    mPom.category = QStringLiteral("Kunststoffe");
    mPom.vc = 320.0;
    mPom.fzBase = 0.07;
    mPom.maxStepDownRatio = 0.8;
    mPom.maxStepOverRatio = 0.6;
    mPom.plungeRatio = 0.45;
    mPom.trochoidalEngagement = 0.20;  // 20% — Kunststoff, sehr weich
    mPom.trochoidalFeedFactor = 2.5;
    addMaterial(mPom);

    // 3. Messing
    Material mBrass;
    mBrass.id = 3;
    mBrass.name = QStringLiteral("Messing (CuZn39Pb3)");
    mBrass.category = QStringLiteral("Nichteisenmetall");
    mBrass.vc = 180.0;
    mBrass.fzBase = 0.035;
    mBrass.maxStepDownRatio = 0.4;
    mBrass.maxStepOverRatio = 0.4;
    mBrass.plungeRatio = 0.3;
    mBrass.trochoidalEngagement = 0.12;  // 12% — mittlere Haerte
    mBrass.trochoidalFeedFactor = 2.0;
    addMaterial(mBrass);

    // 4. Acrylglas
    Material mPmma;
    mPmma.id = 4;
    mPmma.name = QStringLiteral("Acrylglas / Plexiglas (PMMA)");
    mPmma.category = QStringLiteral("Kunststoffe");
    mPmma.vc = 220.0;
    mPmma.fzBase = 0.04;
    mPmma.maxStepDownRatio = 0.5;
    mPmma.maxStepOverRatio = 0.4;
    mPmma.plungeRatio = 0.3;
    mPmma.trochoidalEngagement = 0.18;  // 18% — Kunststoff
    mPmma.trochoidalFeedFactor = 2.5;
    addMaterial(mPmma);

    // 5. Holz / MDF
    Material mWood;
    mWood.id = 5;
    mWood.name = QStringLiteral("Holz / Multiplex / MDF");
    mWood.category = QStringLiteral("Holz & Verbund");
    mWood.vc = 450.0;
    mWood.fzBase = 0.09;
    mWood.maxStepDownRatio = 1.0;
    mWood.maxStepOverRatio = 0.65;
    mWood.plungeRatio = 0.5;
    mWood.trochoidalEngagement = 0.25;  // 25% — weiches Holz
    mWood.trochoidalFeedFactor = 2.0;
    addMaterial(mWood);

    // 6. Baustahl
    Material mSteel;
    mSteel.id = 6;
    mSteel.name = QStringLiteral("Baustahl (S235JR)");
    mSteel.category = QStringLiteral("Stahl");
    mSteel.vc = 85.0;
    mSteel.fzBase = 0.025;
    mSteel.maxStepDownRatio = 0.25;
    mSteel.maxStepOverRatio = 0.35;
    mSteel.plungeRatio = 0.25;
    mSteel.trochoidalEngagement = 0.08;  // 8% — Stahl, konservativ
    mSteel.trochoidalFeedFactor = 1.8;
    addMaterial(mSteel);
}

MaterialDatabase MaterialDatabase::createDefault() {
    MaterialDatabase db;
    return db;
}

} // namespace GeminiCNC::Core
