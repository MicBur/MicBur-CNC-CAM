#include "MoonrakerClient.h"
#include <QJsonDocument>
#include <QJsonObject>
#include <QUrl>
#include <QNetworkRequest>
#include <QTimer>

namespace GeminiCNC::Hardware {

MoonrakerClient::MoonrakerClient(QObject* parent) : IKlipperClient(parent) {
    m_networkManager = new QNetworkAccessManager(this);
    connect(m_networkManager, &QNetworkAccessManager::finished, this, &MoonrakerClient::onNetworkReply);
}

void MoonrakerClient::connectToHost(const QString& host, int port) {
    m_host = host;
    m_port = port;

    if (m_mockMode) {
        m_state = ConnectionState::Connecting;
        emit stateChanged(m_state);

        // Verzögerte Antwort für realistisches Feedback
        QTimer::singleShot(150, this, [this]() {
            m_state = ConnectionState::Connected;
            emit stateChanged(m_state);
            emit gcodeResponseReceived(QStringLiteral("// Klipper Moonraker Simulator v0.12.0 (Windows Autark) verbunden"));
            emit positionReported(m_simulatedPosition);
        });
        return;
    }

    // Echter Netzwerk-Modus
    m_state = ConnectionState::Connecting;
    emit stateChanged(m_state);

    QUrl url(QString("http://%1:%2/printer/info").arg(m_host).arg(m_port));
    QNetworkRequest request(url);
    request.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");
    m_networkManager->get(request);
}

void MoonrakerClient::disconnectFromHost() {
    m_state = ConnectionState::Disconnected;
    emit stateChanged(m_state);
    emit gcodeResponseReceived(QStringLiteral("// Verbindung getrennt."));
}

bool MoonrakerClient::isConnected() const {
    return m_state == ConnectionState::Connected;
}

void MoonrakerClient::sendGCode(const QString& gcode) {
    if (!isConnected()) {
        emit errorMessage(QStringLiteral("Nicht mit Klipper verbunden."));
        return;
    }

    if (m_mockMode) {
        handleMockGCode(gcode);
        return;
    }

    QUrl url(QString("http://%1:%2/printer/gcode/script").arg(m_host).arg(m_port));
    QNetworkRequest request(url);
    request.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");

    QJsonObject obj;
    obj[QStringLiteral("script")] = gcode;
    QByteArray postData = QJsonDocument(obj).toJson();

    m_networkManager->post(request, postData);
}

void MoonrakerClient::emergencyStop() {
    if (m_mockMode) {
        emit gcodeResponseReceived(QStringLiteral("!! NOT-HALT (Emergency Stop) ausgelöst !!"));
        m_state = ConnectionState::Disconnected;
        emit stateChanged(m_state);
        return;
    }

    QUrl url(QString("http://%1:%2/printer/emergency_stop").arg(m_host).arg(m_port));
    QNetworkRequest request(url);
    m_networkManager->post(request, QByteArray());
}

void MoonrakerClient::homeAxes(const QString& axes) {
    sendGCode(QString("G28 %1").arg(axes));
}

void MoonrakerClient::queryTmcStatus(const QString& stepper) {
    if (m_mockMode) {
        QJsonObject tmc;
        tmc[QStringLiteral("stepper")] = stepper;
        tmc[QStringLiteral("current_mA")] = 800;
        tmc[QStringLiteral("microsteps")] = 16;
        tmc[QStringLiteral("stealthchop")] = true;
        tmc[QStringLiteral("driver_status")] = QStringLiteral("OK (Standstill / CoolStep Active)");
        tmc[QStringLiteral("temperature_degC")] = 39.4;

        emit tmcStatusReported(stepper, tmc);
        emit gcodeResponseReceived(QString("// DUMP_TMC STEPPER=%1: RunCurrent=800mA, StealthChop=ON").arg(stepper));
        return;
    }

    sendGCode(QString("DUMP_TMC STEPPER=%1").arg(stepper));
}

void MoonrakerClient::handleMockGCode(const QString& gcode) {
    QString trimmed = gcode.trimmed();

    if (trimmed.startsWith(QStringLiteral("G28"))) {
        // Homing
        m_simulatedPosition = {0.0, 0.0, 0.0, 0.0};
        emit positionReported(m_simulatedPosition);
        emit gcodeResponseReceived(QStringLiteral("ok - Homing abgeschlossen (X0 Y0 Z0 A0)"));
        return;
    }

    if (trimmed.startsWith(QStringLiteral("G0")) || trimmed.startsWith(QStringLiteral("G1"))) {
        QStringList parts = trimmed.split(QLatin1Char(' '), Qt::SkipEmptyParts);
        for (const auto& part : parts) {
            if (part.size() < 2) continue;
            QChar axis = part[0].toUpper();
            double val = part.mid(1).toDouble();
            if (axis == 'X') m_simulatedPosition.x = val;
            else if (axis == 'Y') m_simulatedPosition.y = val;
            else if (axis == 'Z') m_simulatedPosition.z = val;
            else if (axis == 'A') m_simulatedPosition.a = val;
        }
        emit positionReported(m_simulatedPosition);
        emit gcodeResponseReceived(QString("ok - %1").arg(trimmed));
        return;
    }

    if (trimmed.startsWith(QStringLiteral("M114"))) {
        emit gcodeResponseReceived(QString("X:%1 Y:%2 Z:%3 A:%4 E:0")
            .arg(m_simulatedPosition.x, 0, 'f', 3)
            .arg(m_simulatedPosition.y, 0, 'f', 3)
            .arg(m_simulatedPosition.z, 0, 'f', 3)
            .arg(m_simulatedPosition.a, 0, 'f', 3));
        return;
    }

    emit gcodeResponseReceived(QString("ok - %1").arg(trimmed));
}

void MoonrakerClient::onNetworkReply(QNetworkReply* reply) {
    if (!reply) return;

    if (reply->error() == QNetworkReply::NoError) {
        QByteArray data = reply->readAll();
        if (m_state == ConnectionState::Connecting) {
            m_state = ConnectionState::Connected;
            emit stateChanged(m_state);
        }
        emit gcodeResponseReceived(QString::fromUtf8(data));
    } else {
        if (m_state == ConnectionState::Connecting) {
            m_state = ConnectionState::Error;
            emit stateChanged(m_state);
        }
        emit errorMessage(QString("Netzwerkfehler: %1").arg(reply->errorString()));
    }
    reply->deleteLater();
}

} // namespace GeminiCNC::Hardware
