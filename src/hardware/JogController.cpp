#include "JogController.h"
#include <cmath>

namespace GeminiCNC::Hardware {

JogController::JogController(IKlipperClient* client, QObject* parent)
    : QObject(parent), m_client(client) {}

void JogController::setStepDistance(double distanceMm) {
    m_stepDistance = std::max(0.001, distanceMm);
}

void JogController::setJogFeedRate(double feedMmMin) {
    m_jogFeedRate = std::max(50.0, feedMmMin);
}

void JogController::jogAxis(JogAxis axis, int direction) {
    if (!m_client || !m_client->isConnected()) return;

    double dist = (direction >= 0 ? 1.0 : -1.0) * m_stepDistance;

    QString axisLetter;
    switch (axis) {
        case JogAxis::X: axisLetter = "X"; break;
        case JogAxis::Y: axisLetter = "Y"; break;
        case JogAxis::Z: axisLetter = "Z"; break;
        case JogAxis::A: axisLetter = "A"; break;
    }

    // Klipper / LinuxCNC relatives Fahren: G91 -> G1 Axis Dist F Feed -> G90
    QString cmd = QString("G91\nG1 %1%2 F%3\nG90")
        .arg(axisLetter)
        .arg(dist, 0, 'f', 3)
        .arg(m_jogFeedRate, 0, 'f', 0);

    m_client->sendGCode(cmd);
}

void JogController::homeAxis(JogAxis axis) {
    if (!m_client || !m_client->isConnected()) return;
    QString axisLetter;
    switch (axis) {
        case JogAxis::X: axisLetter = "X"; break;
        case JogAxis::Y: axisLetter = "Y"; break;
        case JogAxis::Z: axisLetter = "Z"; break;
        case JogAxis::A: axisLetter = "A"; break;
    }
    m_client->homeAxes(axisLetter);
}

void JogController::homeAll() {
    if (!m_client || !m_client->isConnected()) return;
    m_client->homeAxes(QStringLiteral("XYZ"));
}

void JogController::zeroAxis(JogAxis axis) {
    if (!m_client || !m_client->isConnected()) return;
    QString axisLetter;
    switch (axis) {
        case JogAxis::X: axisLetter = "X"; break;
        case JogAxis::Y: axisLetter = "Y"; break;
        case JogAxis::Z: axisLetter = "Z"; break;
        case JogAxis::A: axisLetter = "A"; break;
    }
    m_client->sendGCode(QString("G92 %10").arg(axisLetter));
}

void JogController::zeroAll() {
    if (!m_client || !m_client->isConnected()) return;
    m_client->sendGCode(QStringLiteral("G92 X0 Y0 Z0 A0"));
}

void JogController::setSpindleState(bool on, double rpm) {
    if (!m_client || !m_client->isConnected()) return;
    m_spindleOn = on;
    if (on) {
        m_client->sendGCode(QString("M3 S%1").arg(static_cast<int>(rpm)));
    } else {
        m_client->sendGCode(QStringLiteral("M5"));
    }
}

} // namespace GeminiCNC::Hardware
