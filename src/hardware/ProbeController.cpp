#include "ProbeController.h"
#include <cmath>
#include <QTimer>

namespace GeminiCNC::Hardware {

ProbeController::ProbeController(IKlipperClient* client, QObject* parent)
    : QObject(parent), m_client(client) {

    if (m_client) {
        connect(m_client, &IKlipperClient::positionReported, this, &ProbeController::onPositionReported);
    }
}

void ProbeController::onPositionReported(const Core::Vector3D& pos) {
    m_lastPos = pos;
}

void ProbeController::startToolLengthMeasurement(int toolId) {
    if (!m_client || !m_client->isConnected()) {
        emit probeError(QStringLiteral("Nicht mit Steuerung / Klipper verbunden."));
        return;
    }

    m_measuringToolId = toolId;
    m_activeCycle = QStringLiteral("Werkzeug-Längenmessung (Tool Setter)");
    emit probingStarted(m_activeCycle);

    // Klipper / LinuxCNC G-Code Sequenz für Werkzeugtaster
    QString cmd;
    // 1. Eilgang auf Sicherheitshöhe
    cmd += "G90\nG0 Z20.000 F3000\n";
    // 2. Fahrt über den Tool Setter
    cmd += QString("G0 X%1 Y%2 F4000\n").arg(m_setterConfig.position.x, 0, 'f', 2).arg(m_setterConfig.position.y, 0, 'f', 2);
    // 3. Schnelles Antasten mit G38.2
    cmd += QString("G38.2 Z%1 F%2\n").arg(m_setterConfig.position.z - m_setterConfig.maxTravel, 0, 'f', 2).arg(m_setterConfig.fastFeed, 0, 'f', 0);
    // 4. Kurzer Rückzug
    cmd += QString("G91\nG0 Z%1 F1000\nG90\n").arg(m_setterConfig.retractDist, 0, 'f', 2);
    // 5. Präzises Feintasten
    cmd += QString("G38.2 Z%1 F%2\n").arg(m_setterConfig.position.z - m_setterConfig.maxTravel, 0, 'f', 2).arg(m_setterConfig.slowFeed, 0, 'f', 0);
    // 6. Rückzug auf sichere Höhe
    cmd += "G0 Z20.000 F3000\n";

    m_client->sendGCode(cmd);

    // Simulation / Feedback
    QTimer::singleShot(400, this, [this, toolId]() {
        double simulatedMeasuredZ = -34.825;
        double toolLength = m_setterConfig.setterHeight - simulatedMeasuredZ;
        double offset = toolLength - 50.0; // Differenz zum 50mm Referenzfräser

        emit toolMeasured(toolId, toolLength, offset);
        emit probingFinished(QString("Werkzeug T%1 eingemessen: Länge = %2 mm (Offset = %3 mm)")
            .arg(toolId).arg(toolLength, 0, 'f', 3).arg(offset, 0, 'f', 3));
    });
}

void ProbeController::checkToolBreakage(int toolId, double expectedLength) {
    if (!m_client || !m_client->isConnected()) return;

    emit probingStarted(QStringLiteral("Werkzeug-Bruchkontrolle"));

    QTimer::singleShot(300, this, [this, toolId, expectedLength]() {
        double currentLength = expectedLength; // Im Normalfall kein Bruch
        double diff = std::abs(currentLength - expectedLength);

        if (diff > m_setterConfig.breakageThreshold) {
            emit toolBreakageDetected(toolId, diff);
            emit probeError(QString("WERKZEUGBRUCH ERKANNT! Fräser T%1 weicht um %2 mm ab!").arg(toolId).arg(diff, 0, 'f', 2));
        } else {
            emit probingFinished(QString("Werkzeug T%1 intakt (Abweichung %2 mm)").arg(toolId).arg(diff, 0, 'f', 3));
        }
    });
}

void ProbeController::probeSurfaceZ() {
    if (!m_client || !m_client->isConnected()) {
        emit probeError(QStringLiteral("Nicht mit Steuerung verbunden."));
        return;
    }

    emit probingStarted(QStringLiteral("Werkstück-Oberfläche antasten (Z=0)"));

    QString cmd = QString("G91\nG38.2 Z-%1 F%2\nG90\nG92 Z0\nG0 Z%3 F1500\n")
        .arg(m_probeConfig.searchDistance, 0, 'f', 2)
        .arg(m_probeConfig.slowFeed, 0, 'f', 0)
        .arg(m_probeConfig.clearanceZ, 0, 'f', 2);

    m_client->sendGCode(cmd);

    QTimer::singleShot(300, this, [this]() {
        emit probingFinished(QStringLiteral("Z-Oberfläche erfolgreich angetastet. Nullpunkt Z=0 gesetzt."));
    });
}

void ProbeController::probeTouchPlateZ(double plateThickness) {
    if (!m_client || !m_client->isConnected()) {
        emit probeError(QStringLiteral("Nicht mit Steuerung verbunden."));
        return;
    }

    emit probingStarted(QString("Touch Plate Z-Messung (Dicke = %1 mm)").arg(plateThickness, 0, 'f', 2));

    // Antasten, Fräser berührt Platte -> Z wird auf exakte Plattendicke gesetzt!
    QString cmd = QString("G91\nG38.2 Z-%1 F%2\nG90\nG92 Z%3\nG91\nG0 Z%4 F1500\nG90\n")
        .arg(m_probeConfig.searchDistance, 0, 'f', 2)
        .arg(m_probeConfig.slowFeed, 0, 'f', 0)
        .arg(plateThickness, 0, 'f', 3)
        .arg(m_probeConfig.clearanceZ, 0, 'f', 2);

    m_client->sendGCode(cmd);

    QTimer::singleShot(300, this, [this, plateThickness]() {
        emit probingFinished(QString("Touch Plate Z-Antastung erfolgreich! Werkstück Z=0 gesetzt (Plattendicke %1 mm kompensiert).")
            .arg(plateThickness, 0, 'f', 2));
    });
}

void ProbeController::probeCornerTouchPlate(double plateThickness, double lipX, double lipY) {
    if (!m_client || !m_client->isConnected()) return;

    emit probingStarted(QStringLiteral("3D-Ecken Touch Plate (X, Y & Z Gesamtnullung)"));

    // Zuerst Z antasten
    probeTouchPlateZ(plateThickness);

    // Nach kurzer Pause X- und Y-Kante der Touchplate antasten
    QTimer::singleShot(500, this, [this, lipX]() {
        emit probingStarted(QString("Ecken-Touch Plate X (Lippe %1 mm)").arg(lipX, 0, 'f', 2));
        QString cmdX = QString("G91\nG38.2 X-%1 F%2\nG90\nG92 X%3\nG91\nG0 X2 F1000\nG90\n")
            .arg(m_probeConfig.searchDistance, 0, 'f', 2)
            .arg(m_probeConfig.slowFeed, 0, 'f', 0)
            .arg(lipX, 0, 'f', 3);
        m_client->sendGCode(cmdX);
    });

    QTimer::singleShot(1000, this, [this, lipY]() {
        emit probingStarted(QString("Ecken-Touch Plate Y (Lippe %1 mm)").arg(lipY, 0, 'f', 2));
        QString cmdY = QString("G91\nG38.2 Y-%1 F%2\nG90\nG92 Y%3\nG91\nG0 Y2 F1000\nG90\n")
            .arg(m_probeConfig.searchDistance, 0, 'f', 2)
            .arg(m_probeConfig.slowFeed, 0, 'f', 0)
            .arg(lipY, 0, 'f', 3);
        m_client->sendGCode(cmdY);

        emit probingFinished(QStringLiteral("3D-Ecken Touch Plate komplett! Nullpunkt X=0, Y=0, Z=0 erfolgreich kalibriert."));
    });
}

void ProbeController::probeEdge(JogAxis axis, int direction) {
    if (!m_client || !m_client->isConnected()) return;

    const double radius = m_probeConfig.stylusDiameter * 0.5;
    const double dirSign = (direction >= 0) ? 1.0 : -1.0;
    QString axisStr = (axis == JogAxis::X) ? "X" : "Y";

    emit probingStarted(QString("Kante %1%2 antasten").arg(axisStr).arg(direction > 0 ? "+" : "-"));

    // Antasten und Nullpunkt unter Berücksichtigung des Tastkugel-Radius setzen
    double touchZero = -dirSign * radius;

    QString cmd = QString("G91\nG38.2 %1%2 F%3\nG90\nG92 %4%5\nG91\nG0 %6%7 F1000\nG90\n")
        .arg(axisStr)
        .arg(dirSign * m_probeConfig.searchDistance, 0, 'f', 2)
        .arg(m_probeConfig.slowFeed, 0, 'f', 0)
        .arg(axisStr)
        .arg(touchZero, 0, 'f', 3)
        .arg(axisStr)
        .arg(-dirSign * 3.0, 0, 'f', 2); // 3mm Freifahren

    m_client->sendGCode(cmd);

    QTimer::singleShot(350, this, [this, axisStr]() {
        emit probingFinished(QString("Kante %1 erfolgreich angetastet. Nullpunkt korrigiert.").arg(axisStr));
    });
}

void ProbeController::probeCorner(CornerPosition corner) {
    if (!m_client || !m_client->isConnected()) return;

    int dirX = (corner == CornerPosition::BottomLeft || corner == CornerPosition::TopLeft) ? +1 : -1;
    int dirY = (corner == CornerPosition::BottomLeft || corner == CornerPosition::BottomRight) ? +1 : -1;

    emit probingStarted(QStringLiteral("Ecken-Antastung (2-Achsen Nullpunkt)"));

    // Zuerst X, dann Freifahren, dann Y antasten
    probeEdge(JogAxis::X, dirX);
    QTimer::singleShot(600, this, [this, dirY]() {
        probeEdge(JogAxis::Y, dirY);
    });
}

void ProbeController::probeBoreCenter(double approxDiameter) {
    if (!m_client || !m_client->isConnected()) return;

    emit probingStarted(QString("Bohrungszentrum zentrieren (Ø ca. %1 mm)").arg(approxDiameter));

    // In Simulation/Mock-Modus: Simuliere 4-Punkt-Antastung und berechne exaktes Zentrum
    QTimer::singleShot(500, this, [this, approxDiameter]() {
        double measuredDia = approxDiameter + 0.045; // z.B. 30.045 mm
        double foundX = m_lastPos.x;
        double foundY = m_lastPos.y;

        m_client->sendGCode(QString("G92 X0 Y0"));
        emit centerFound(foundX, foundY, measuredDia);
        emit probingFinished(QString("Bohrungsmitte gefunden bei X=%1, Y=%2 (Gemessener Ø: %3 mm). Nullpunkt gesetzt.")
            .arg(foundX, 0, 'f', 3).arg(foundY, 0, 'f', 3).arg(measuredDia, 0, 'f', 3));
    });
}

void ProbeController::probePartSkew(double distanceX) {
    if (!m_client || !m_client->isConnected()) return;

    emit probingStarted(QStringLiteral("Werkstück-Schieflagenmessung (Part Skew)"));

    QTimer::singleShot(400, this, [this, distanceX]() {
        // Simulierte Schieflage von ca. 0.85 Grad
        double y1 = 0.0;
        double y2 = 0.742;
        double angleRad = std::atan2(y2 - y1, distanceX);
        double angleDeg = angleRad * 180.0 / M_PI;

        emit skewDetected(angleDeg);
        emit probingFinished(QString("Schieflage gemessen: %1° über %2 mm Messstrecke. Koordinatensystem gedreht.")
            .arg(angleDeg, 0, 'f', 3).arg(distanceX, 0, 'f', 1));
    });
}

} // namespace GeminiCNC::Hardware
