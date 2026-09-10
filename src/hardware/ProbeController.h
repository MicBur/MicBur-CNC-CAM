#ifndef GEMINI_CNC_PROBECONTROLLER_H
#define GEMINI_CNC_PROBECONTROLLER_H

#include <QObject>
#include <QString>
#include "IKlipperClient.h"
#include "JogController.h"
#include "core/Vector3D.h"
#include "core/ToolDefinition.h"

namespace GeminiCNC::Hardware {

enum class CornerPosition {
    BottomLeft,  // X- / Y- (Standard)
    BottomRight, // X+ / Y-
    TopLeft,     // X- / Y+
    TopRight     // X+ / Y+
};

struct ToolSetterConfig {
    Core::Vector3D position{450.0, 350.0, -10.0}; // Tisch-Position des Werkzeugtasters
    double setterHeight{40.0};                     // Bauhöhe des Tasters (mm)
    double fastFeed{300.0};                        // mm/min
    double slowFeed{30.0};                         // mm/min
    double retractDist{2.0};                       // mm
    double maxTravel{80.0};                        // mm max. Suchweg
    double breakageThreshold{0.5};                 // mm Toleranz für Werkzeugbruch-Warnung
};

struct WorkpieceProbeConfig {
    double stylusDiameter{4.0};                    // Tastkugeldurchmesser (mm)
    double fastFeed{200.0};                        // mm/min
    double slowFeed{25.0};                         // mm/min
    double searchDistance{25.0};                   // mm max. Tastweg
    double clearanceZ{5.0};                        // mm
};

/**
 * @brief Vollständiges Einmess- und Antast-System im Hurco WinMax-Stil.
 */
class ProbeController : public QObject {
    Q_OBJECT
public:
    explicit ProbeController(IKlipperClient* client, QObject* parent = nullptr);

    [[nodiscard]] const ToolSetterConfig& toolSetterConfig() const { return m_setterConfig; }
    void setToolSetterConfig(const ToolSetterConfig& config) { m_setterConfig = config; }

    [[nodiscard]] const WorkpieceProbeConfig& workpieceProbeConfig() const { return m_probeConfig; }
    void setWorkpieceProbeConfig(const WorkpieceProbeConfig& config) { m_probeConfig = config; }

    // Werkzeug-Zyklen (Tool Setter)
    void startToolLengthMeasurement(int toolId);
    void checkToolBreakage(int toolId, double expectedLength);

    // Werkstück-Zyklen (3D-Messtaster & Touch Plate)
    void probeSurfaceZ();
    void probeTouchPlateZ(double plateThickness = 10.0);
    void probeCornerTouchPlate(double plateThickness = 10.0, double lipX = 10.0, double lipY = 10.0);
    void probeEdge(JogAxis axis, int direction); // direction: +1 oder -1
    void probeCorner(CornerPosition corner);
    void probeBoreCenter(double approxDiameter = 30.0);
    void probePartSkew(double distanceX = 50.0);

signals:
    void probingStarted(const QString& cycleName);
    void probingFinished(const QString& resultMessage);
    void toolMeasured(int toolId, double measuredLength, double lengthOffset);
    void toolBreakageDetected(int toolId, double deviation);
    void centerFound(double centerX, double centerY, double measuredDiameter);
    void skewDetected(double angleDegrees);
    void probeError(const QString& error);

private slots:
    void onPositionReported(const Core::Vector3D& pos);

private:
    IKlipperClient* m_client{nullptr};
    ToolSetterConfig m_setterConfig;
    WorkpieceProbeConfig m_probeConfig;

    Core::Vector3D m_lastPos;
    int m_measuringToolId{1};
    QString m_activeCycle;
};

} // namespace GeminiCNC::Hardware

#endif // GEMINI_CNC_PROBECONTROLLER_H
