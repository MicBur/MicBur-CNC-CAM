#include "ToolDefinition.h"
#include <QJsonArray>
#include <QJsonDocument>
#include <QFile>
#include <QFileInfo>
#include <QSaveFile>
#include <QDir>
#include <QStandardPaths>
#include <QCoreApplication>

namespace GeminiCNC::Core {

QString toolTypeToString(ToolType type) {
    switch (type) {
        case ToolType::EndMill: return QStringLiteral("Schaftfräser");
        case ToolType::BallMill: return QStringLiteral("Kugelkopffräser");
        case ToolType::ChamferMill: return QStringLiteral("Fasenfräser");
        case ToolType::FaceMill: return QStringLiteral("Planfräser");
        case ToolType::Drill: return QStringLiteral("Bohrer");
    }
    return QStringLiteral("Unbekannt");
}

ToolType stringToToolType(const QString& str) {
    if (str == QStringLiteral("Kugelkopffräser") || str == QStringLiteral("BallMill")) return ToolType::BallMill;
    if (str == QStringLiteral("Fasenfräser") || str == QStringLiteral("ChamferMill")) return ToolType::ChamferMill;
    if (str == QStringLiteral("Planfräser") || str == QStringLiteral("FaceMill")) return ToolType::FaceMill;
    if (str == QStringLiteral("Bohrer") || str == QStringLiteral("Drill")) return ToolType::Drill;
    return ToolType::EndMill;
}

ToolDefinition::ToolDefinition(int toolId, const QString& toolName, ToolType toolType, double diam)
    : id(toolId), name(toolName), type(toolType), diameter(diam), shaftDiameter(diam) {}

double ToolDefinition::effectiveStepOver() const {
    return diameter * (stepOverPercentage / 100.0);
}

QJsonObject ToolDefinition::toJson() const {
    QJsonObject obj;
    obj[QStringLiteral("id")] = id;
    obj[QStringLiteral("name")] = name;
    obj[QStringLiteral("type")] = toolTypeToString(type);
    obj[QStringLiteral("diameter")] = diameter;
    obj[QStringLiteral("fluteLength")] = fluteLength;
    obj[QStringLiteral("shaftDiameter")] = shaftDiameter;
    obj[QStringLiteral("overallLength")] = overallLength;
    obj[QStringLiteral("stickOutLength")] = stickOutLength;
    obj[QStringLiteral("holderDiameter")] = holderDiameter;
    obj[QStringLiteral("flutes")] = flutes;
    obj[QStringLiteral("defaultFeedRate")] = defaultFeedRate;
    obj[QStringLiteral("plungeFeedRate")] = plungeFeedRate;
    obj[QStringLiteral("spindleSpeed")] = spindleSpeed;
    obj[QStringLiteral("maxStepDown")] = maxStepDown;
    obj[QStringLiteral("stepOverPercentage")] = stepOverPercentage;
    obj[QStringLiteral("color")] = color;
    return obj;
}

ToolDefinition ToolDefinition::fromJson(const QJsonObject& json) {
    ToolDefinition tool;
    if (json.contains(QStringLiteral("id"))) tool.id = json[QStringLiteral("id")].toInt();
    if (json.contains(QStringLiteral("name"))) tool.name = json[QStringLiteral("name")].toString();
    if (json.contains(QStringLiteral("type"))) tool.type = stringToToolType(json[QStringLiteral("type")].toString());
    if (json.contains(QStringLiteral("diameter"))) tool.diameter = json[QStringLiteral("diameter")].toDouble();
    if (json.contains(QStringLiteral("fluteLength"))) tool.fluteLength = json[QStringLiteral("fluteLength")].toDouble();
    if (json.contains(QStringLiteral("shaftDiameter"))) tool.shaftDiameter = json[QStringLiteral("shaftDiameter")].toDouble();
    if (json.contains(QStringLiteral("overallLength"))) tool.overallLength = json[QStringLiteral("overallLength")].toDouble();
    if (json.contains(QStringLiteral("stickOutLength"))) tool.stickOutLength = json[QStringLiteral("stickOutLength")].toDouble();
    if (json.contains(QStringLiteral("holderDiameter"))) tool.holderDiameter = json[QStringLiteral("holderDiameter")].toDouble();
    if (json.contains(QStringLiteral("flutes"))) tool.flutes = json[QStringLiteral("flutes")].toInt();
    if (json.contains(QStringLiteral("defaultFeedRate"))) tool.defaultFeedRate = json[QStringLiteral("defaultFeedRate")].toDouble();
    if (json.contains(QStringLiteral("plungeFeedRate"))) tool.plungeFeedRate = json[QStringLiteral("plungeFeedRate")].toDouble();
    if (json.contains(QStringLiteral("spindleSpeed"))) tool.spindleSpeed = json[QStringLiteral("spindleSpeed")].toDouble();
    if (json.contains(QStringLiteral("maxStepDown"))) tool.maxStepDown = json[QStringLiteral("maxStepDown")].toDouble();
    if (json.contains(QStringLiteral("stepOverPercentage"))) tool.stepOverPercentage = json[QStringLiteral("stepOverPercentage")].toDouble();
    if (json.contains(QStringLiteral("color"))) tool.color = json[QStringLiteral("color")].toString();
    return tool;
}

QString ToolDefinition::defaultLibraryPath() {
    // Portable Version (USB-Stick): Datei "portable.txt" neben der .exe → Daten im Programmordner
    const QString appDir = QCoreApplication::instance() ? QCoreApplication::applicationDirPath() : QString();
    QString dir;
    if (!appDir.isEmpty() && QFileInfo::exists(QDir(appDir).filePath(QStringLiteral("portable.txt")))) {
        dir = QDir(appDir).filePath(QStringLiteral("daten"));
    } else {
        dir = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
    }
    if (dir.isEmpty()) {
        dir = QDir::homePath() + QStringLiteral("/.geminicnc");
    }
    QDir().mkpath(dir);
    return QDir(dir).filePath(QStringLiteral("werkzeuge.json"));
}

bool ToolDefinition::saveLibrary(const QString& filePath, const QList<ToolDefinition>& tools, int activeToolId) {
    QJsonArray toolArray;
    for (const auto& tool : tools) {
        toolArray.append(tool.toJson());
    }
    QJsonObject root;
    root[QStringLiteral("version")] = 1;
    root[QStringLiteral("activeToolId")] = activeToolId;
    root[QStringLiteral("tools")] = toolArray;

    QDir().mkpath(QFileInfo(filePath).absolutePath());
    // QSaveFile: erst vollständig schreiben, dann atomar ersetzen (kein halbes JSON bei Absturz)
    QSaveFile file(filePath);
    if (!file.open(QIODevice::WriteOnly)) return false;
    file.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    return file.commit();
}

bool ToolDefinition::loadLibrary(const QString& filePath, QList<ToolDefinition>& tools, int& activeToolId) {
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly)) return false;

    QJsonParseError parseError;
    const QJsonDocument doc = QJsonDocument::fromJson(file.readAll(), &parseError);
    if (parseError.error != QJsonParseError::NoError || !doc.isObject()) return false;

    const QJsonObject root = doc.object();
    QList<ToolDefinition> loaded;
    for (const auto& value : root[QStringLiteral("tools")].toArray()) {
        if (value.isObject()) loaded.append(fromJson(value.toObject()));
    }
    if (loaded.isEmpty()) return false;

    tools = loaded;
    activeToolId = root[QStringLiteral("activeToolId")].toInt(-1);
    return true;
}

QList<ToolDefinition> ToolDefinition::createDefaultLibrary() {
    QList<ToolDefinition> lib;

    // T1: 6mm Schaftfräser Alu/Holz (Standard: Signalgelb #FFE614)
    ToolDefinition t1(1, QStringLiteral("6mm Flachfräser 2-Schneider"), ToolType::EndMill, 6.0);
    t1.fluteLength = 18.0;
    t1.stickOutLength = 28.0;
    t1.defaultFeedRate = 1500.0;
    t1.plungeFeedRate = 500.0;
    t1.spindleSpeed = 18000.0;
    t1.maxStepDown = 2.5;
    t1.stepOverPercentage = 45.0;
    t1.color = QStringLiteral("#FFE614"); // Hurco Signalgelb
    lib.append(t1);

    // T2: 3mm Schaftfräser Schlichten / Konturen (Bernstein/Orange)
    ToolDefinition t2(2, QStringLiteral("3mm Feinschnittfräser"), ToolType::EndMill, 3.0);
    t2.fluteLength = 10.0;
    t2.stickOutLength = 20.0;
    t2.defaultFeedRate = 900.0;
    t2.plungeFeedRate = 300.0;
    t2.spindleSpeed = 22000.0;
    t2.maxStepDown = 1.0;
    t2.stepOverPercentage = 40.0;
    t2.color = QStringLiteral("#F59E0B"); // Bernstein-Orange
    lib.append(t2);

    // T3: 40mm Planfräser (Face Mill) (Smaragdgrün)
    ToolDefinition t3(3, QStringLiteral("40mm Planfräskopf"), ToolType::FaceMill, 40.0);
    t3.fluteLength = 6.0;
    t3.shaftDiameter = 12.0;
    t3.stickOutLength = 40.0;
    t3.holderDiameter = 50.0;
    t3.flutes = 4;
    t3.defaultFeedRate = 2000.0;
    t3.plungeFeedRate = 400.0;
    t3.spindleSpeed = 12000.0;
    t3.maxStepDown = 0.8;
    t3.stepOverPercentage = 70.0;
    t3.color = QStringLiteral("#10B981"); // Smaragdgrün
    lib.append(t3);

    // T4: 4mm Kugelkopffräser (3D-Freiform)
    ToolDefinition t4(4, QStringLiteral("4mm Kugelkopffräser"), ToolType::BallMill, 4.0);
    t4.fluteLength = 12.0;
    t4.stickOutLength = 22.0;
    t4.defaultFeedRate = 1200.0;
    t4.plungeFeedRate = 400.0;
    t4.spindleSpeed = 20000.0;
    t4.maxStepDown = 0.5;
    t4.stepOverPercentage = 15.0; // Feine Überlappung für 3D Schlichten
    lib.append(t4);

    return lib;
}

} // namespace GeminiCNC::Core
