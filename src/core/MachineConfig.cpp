#include "MachineConfig.h"

namespace GeminiCNC::Core {

QJsonObject TMC2209Config::toJson() const {
    QJsonObject obj;
    obj[QStringLiteral("runCurrentMa")] = runCurrentMa;
    obj[QStringLiteral("holdCurrentMa")] = holdCurrentMa;
    obj[QStringLiteral("microsteps")] = microsteps;
    obj[QStringLiteral("interpolate")] = interpolate;
    obj[QStringLiteral("stealthChop")] = stealthChop;
    obj[QStringLiteral("stealthThreshold")] = stealthThreshold;
    return obj;
}

TMC2209Config TMC2209Config::fromJson(const QJsonObject& json) {
    TMC2209Config cfg;
    if (json.contains(QStringLiteral("runCurrentMa"))) cfg.runCurrentMa = json[QStringLiteral("runCurrentMa")].toInt();
    if (json.contains(QStringLiteral("holdCurrentMa"))) cfg.holdCurrentMa = json[QStringLiteral("holdCurrentMa")].toInt();
    if (json.contains(QStringLiteral("microsteps"))) cfg.microsteps = json[QStringLiteral("microsteps")].toInt();
    if (json.contains(QStringLiteral("interpolate"))) cfg.interpolate = json[QStringLiteral("interpolate")].toBool();
    if (json.contains(QStringLiteral("stealthChop"))) cfg.stealthChop = json[QStringLiteral("stealthChop")].toBool();
    if (json.contains(QStringLiteral("stealthThreshold"))) cfg.stealthThreshold = json[QStringLiteral("stealthThreshold")].toInt();
    return cfg;
}

bool MachineConfig::isWithinLimits(const Vector3D& pos) const {
    if (pos.x < minLimits.x || pos.x > maxLimits.x) return false;
    if (pos.y < minLimits.y || pos.y > maxLimits.y) return false;
    if (pos.z < minLimits.z || pos.z > maxLimits.z) return false;
    if (hasFourthAxis && (pos.a < minLimits.a || pos.a > maxLimits.a)) return false;
    return true;
}

QJsonObject MachineConfig::toJson() const {
    QJsonObject obj;
    obj[QStringLiteral("machineName")] = machineName;

    QJsonObject minLim;
    minLim[QStringLiteral("x")] = minLimits.x;
    minLim[QStringLiteral("y")] = minLimits.y;
    minLim[QStringLiteral("z")] = minLimits.z;
    minLim[QStringLiteral("a")] = minLimits.a;
    obj[QStringLiteral("minLimits")] = minLim;

    QJsonObject maxLim;
    maxLim[QStringLiteral("x")] = maxLimits.x;
    maxLim[QStringLiteral("y")] = maxLimits.y;
    maxLim[QStringLiteral("z")] = maxLimits.z;
    maxLim[QStringLiteral("a")] = maxLimits.a;
    obj[QStringLiteral("maxLimits")] = maxLim;

    QJsonObject maxSpd;
    maxSpd[QStringLiteral("x")] = maxRapidSpeeds.x;
    maxSpd[QStringLiteral("y")] = maxRapidSpeeds.y;
    maxSpd[QStringLiteral("z")] = maxRapidSpeeds.z;
    maxSpd[QStringLiteral("a")] = maxRapidSpeeds.a;
    obj[QStringLiteral("maxRapidSpeeds")] = maxSpd;

    obj[QStringLiteral("maxAcceleration")] = maxAcceleration;
    obj[QStringLiteral("safeRetractZ")] = safeRetractZ;
    obj[QStringLiteral("minSpindleRpm")] = minSpindleRpm;
    obj[QStringLiteral("maxSpindleRpm")] = maxSpindleRpm;
    obj[QStringLiteral("hasFourthAxis")] = hasFourthAxis;
    obj[QStringLiteral("controllerType")] = controllerType;
    obj[QStringLiteral("crcOutputMode")] = crcOutputMode;
    obj[QStringLiteral("workOffset")] = workOffset;

    obj[QStringLiteral("driverX")] = driverX.toJson();
    obj[QStringLiteral("driverY")] = driverY.toJson();
    obj[QStringLiteral("driverZ")] = driverZ.toJson();
    obj[QStringLiteral("driverA")] = driverA.toJson();

    return obj;
}

MachineConfig MachineConfig::fromJson(const QJsonObject& json) {
    MachineConfig cfg;
    if (json.contains(QStringLiteral("machineName"))) cfg.machineName = json[QStringLiteral("machineName")].toString();

    if (json.contains(QStringLiteral("minLimits"))) {
        QJsonObject m = json[QStringLiteral("minLimits")].toObject();
        cfg.minLimits = Vector3D(m[QStringLiteral("x")].toDouble(), m[QStringLiteral("y")].toDouble(),
                                 m[QStringLiteral("z")].toDouble(), m[QStringLiteral("a")].toDouble());
    }
    if (json.contains(QStringLiteral("maxLimits"))) {
        QJsonObject m = json[QStringLiteral("maxLimits")].toObject();
        cfg.maxLimits = Vector3D(m[QStringLiteral("x")].toDouble(), m[QStringLiteral("y")].toDouble(),
                                 m[QStringLiteral("z")].toDouble(), m[QStringLiteral("a")].toDouble());
    }
    if (json.contains(QStringLiteral("maxRapidSpeeds"))) {
        QJsonObject m = json[QStringLiteral("maxRapidSpeeds")].toObject();
        cfg.maxRapidSpeeds = Vector3D(m[QStringLiteral("x")].toDouble(), m[QStringLiteral("y")].toDouble(),
                                      m[QStringLiteral("z")].toDouble(), m[QStringLiteral("a")].toDouble());
    }

    if (json.contains(QStringLiteral("maxAcceleration"))) cfg.maxAcceleration = json[QStringLiteral("maxAcceleration")].toDouble();
    if (json.contains(QStringLiteral("safeRetractZ"))) cfg.safeRetractZ = json[QStringLiteral("safeRetractZ")].toDouble();
    if (json.contains(QStringLiteral("minSpindleRpm"))) cfg.minSpindleRpm = json[QStringLiteral("minSpindleRpm")].toDouble();
    if (json.contains(QStringLiteral("maxSpindleRpm"))) cfg.maxSpindleRpm = json[QStringLiteral("maxSpindleRpm")].toDouble();
    if (json.contains(QStringLiteral("hasFourthAxis"))) cfg.hasFourthAxis = json[QStringLiteral("hasFourthAxis")].toBool();
    if (json.contains(QStringLiteral("controllerType"))) cfg.controllerType = json[QStringLiteral("controllerType")].toInt();
    if (json.contains(QStringLiteral("crcOutputMode"))) cfg.crcOutputMode = json[QStringLiteral("crcOutputMode")].toInt();
    if (json.contains(QStringLiteral("workOffset"))) {
        const int offset = json[QStringLiteral("workOffset")].toInt();
        cfg.workOffset = offset < 0 ? 0 : (offset > 5 ? 5 : offset);
    }

    if (json.contains(QStringLiteral("driverX"))) cfg.driverX = TMC2209Config::fromJson(json[QStringLiteral("driverX")].toObject());
    if (json.contains(QStringLiteral("driverY"))) cfg.driverY = TMC2209Config::fromJson(json[QStringLiteral("driverY")].toObject());
    if (json.contains(QStringLiteral("driverZ"))) cfg.driverZ = TMC2209Config::fromJson(json[QStringLiteral("driverZ")].toObject());
    if (json.contains(QStringLiteral("driverA"))) cfg.driverA = TMC2209Config::fromJson(json[QStringLiteral("driverA")].toObject());

    return cfg;
}

MachineConfig MachineConfig::presetCNC6040() {
    MachineConfig cfg;
    cfg.machineName = QStringLiteral("CNC 6040");
    cfg.minLimits = Vector3D(-300.0, -200.0, -80.0, -360.0);
    cfg.maxLimits = Vector3D(300.0, 200.0, 50.0, 360.0);
    cfg.maxRapidSpeeds = Vector3D(4000.0, 4000.0, 2000.0, 7200.0);
    cfg.maxAcceleration = 500.0;
    cfg.safeRetractZ = 5.0;
    cfg.minSpindleRpm = 3000.0;
    cfg.maxSpindleRpm = 24000.0;
    cfg.hasFourthAxis = false;
    return cfg;
}

MachineConfig MachineConfig::presetCNC3018() {
    MachineConfig cfg;
    cfg.machineName = QStringLiteral("CNC 3018");
    cfg.minLimits = Vector3D(-150.0, -90.0, -45.0, -360.0);
    cfg.maxLimits = Vector3D(150.0, 90.0, 30.0, 360.0);
    cfg.maxRapidSpeeds = Vector3D(3000.0, 3000.0, 1500.0, 3600.0);
    cfg.maxAcceleration = 300.0;
    cfg.safeRetractZ = 3.0;
    cfg.minSpindleRpm = 1000.0;
    cfg.maxSpindleRpm = 10000.0;
    cfg.hasFourthAxis = false;
    return cfg;
}

MachineConfig MachineConfig::presetLargePortal() {
    MachineConfig cfg;
    cfg.machineName = QStringLiteral("Portalfräse Groß");
    cfg.minLimits = Vector3D(-600.0, -400.0, -200.0, -360.0);
    cfg.maxLimits = Vector3D(600.0, 400.0, 100.0, 360.0);
    cfg.maxRapidSpeeds = Vector3D(8000.0, 8000.0, 4000.0, 7200.0);
    cfg.maxAcceleration = 800.0;
    cfg.safeRetractZ = 10.0;
    cfg.minSpindleRpm = 3000.0;
    cfg.maxSpindleRpm = 24000.0;
    cfg.hasFourthAxis = true;
    return cfg;
}

MachineConfig MachineConfig::presetHurcoVMX30() {
    MachineConfig cfg;
    cfg.machineName = QStringLiteral("Hurco VMX 30");

    // Hurco VMX 30 Verfahrwege: X=762mm, Y=508mm, Z=610mm
    // Limits symmetrisch um WCS-Nullpunkt (typisch auf Werkstück gesetzt)
    cfg.minLimits  = Vector3D(-381.0, -254.0, -610.0, -360.0);
    cfg.maxLimits  = Vector3D(381.0, 254.0, 50.0, 360.0);

    // Eilgang: 30.5 m/min (X/Y), 30.5 m/min (Z)
    cfg.maxRapidSpeeds = Vector3D(30500.0, 30500.0, 30500.0, 7200.0);

    // Beschleunigung: industriell (~5 m/s²)
    cfg.maxAcceleration = 5000.0;

    // Sicherheitshöhe
    cfg.safeRetractZ = 10.0;

    // Spindel: BT40, 12.000 U/min (Standard-Variante)
    cfg.minSpindleRpm = 50.0;
    cfg.maxSpindleRpm = 12000.0;

    // Keine 4. Achse im Standard
    cfg.hasFourthAxis = false;

    // Postprozessor: Hurco WinMax
    cfg.controllerType = 1;  // ControllerType::Hurco_WinMax
    cfg.crcOutputMode = 0;   // CenterLine (Standard)

    return cfg;
}

} // namespace GeminiCNC::Core
