#ifndef GEMINI_CNC_MACHINECONFIG_H
#define GEMINI_CNC_MACHINECONFIG_H

#include "Vector3D.h"
#include <QString>
#include <QJsonObject>

namespace GeminiCNC::Core {

/**
 * @brief TMC2209 Schrittmotortreiber-Konfiguration für eine Achse.
 */
struct TMC2209Config {
    int runCurrentMa{800};       // Motorstrom in mA
    int holdCurrentMa{400};      // Haltestrom in mA
    int microsteps{16};          // Mikroschritte (16, 32, 64, etc.)
    bool interpolate{true};      // 256x Mikroschritt-Interpolation
    bool stealthChop{true};      // Flüsterleiser Betrieb (StealthChop2)
    int stealthThreshold{250};   // Umschalt-Schwellwert zu SpreadCycle (mm/s oder rpm)

    [[nodiscard]] QJsonObject toJson() const;
    static TMC2209Config fromJson(const QJsonObject& json);
};

/**
 * @brief Maschinenspezifische Konfiguration (Verfahrwege, Limits, Geschwindigkeiten, 4. Achse).
 */
class MachineConfig {
public:
    QString machineName{"CNC 6040"};

    // Soft-Limits (Arbeitsraumgrenzen in mm / Grad um WCS-Nullpunkt)
    // Z=0 = Werkstückoberkante. Z+ = über Werkstück (Sicherheitshöhe), Z- = in Material
    Vector3D minLimits{-300.0, -200.0, -80.0, -360.0};
    Vector3D maxLimits{300.0, 200.0, 50.0, 360.0};

    // Maximale Eilgang-Geschwindigkeiten (mm/min bzw. deg/min)
    Vector3D maxRapidSpeeds{4000.0, 4000.0, 2000.0, 7200.0};

    // Beschleunigungen (mm/s²)
    double maxAcceleration{500.0};

    // Sicherheitshöhe (Z-Retract Plane in mm über Werkstück-Nullpunkt)
    double safeRetractZ{5.0};

    // Spindeleigenschaften
    double minSpindleRpm{3000.0};
    double maxSpindleRpm{24000.0};

    // Optionen
    bool hasFourthAxis{false};   // 4. Drehachse (A) aktiv?

    // ── Postprozessor-Konfiguration ──

    /**
     * @brief Zielsteuerung für den G-Code-Export.
     * 0 = ISO Standard, 1 = Hurco WinMax, 2 = Heidenhain, 3 = Klipper, 4 = Saeilo
     */
    int controllerType{0};

    /**
     * @brief Schneidradiuskorrektur-Modus.
     * 0 = CenterLine (Software berechnet Mittelpunktsbahn, reine XYZ-Ausgabe)
     * 1 = ControllerCRC (Originalkontur + G41/G42, Steuerung korrigiert)
     */
    int crcOutputMode{0};

    // TMC2209 Treiberkonfiguration je Achse
    TMC2209Config driverX{900, 450, 16, true, true, 200};
    TMC2209Config driverY{900, 450, 16, true, true, 200};
    TMC2209Config driverZ{700, 350, 16, true, true, 150};
    TMC2209Config driverA{600, 300, 16, true, true, 100};

    MachineConfig() = default;

    [[nodiscard]] bool isWithinLimits(const Vector3D& pos) const;
    [[nodiscard]] QJsonObject toJson() const;
    [[nodiscard]] static MachineConfig fromJson(const QJsonObject& json);

    // Maschinentyp-Presets
    [[nodiscard]] static MachineConfig presetCNC6040();
    [[nodiscard]] static MachineConfig presetCNC3018();
    [[nodiscard]] static MachineConfig presetLargePortal();
    [[nodiscard]] static MachineConfig presetHurcoVMX30();
};

} // namespace GeminiCNC::Core

#endif // GEMINI_CNC_MACHINECONFIG_H
