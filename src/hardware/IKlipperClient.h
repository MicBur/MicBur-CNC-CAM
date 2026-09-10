#ifndef GEMINI_CNC_IKLIPPERCLIENT_H
#define GEMINI_CNC_IKLIPPERCLIENT_H

#include <QObject>
#include <QString>
#include <QJsonObject>
#include "core/Vector3D.h"

namespace GeminiCNC::Hardware {

enum class ConnectionState {
    Disconnected,
    Connecting,
    Connected,
    Error
};

/**
 * @brief Abstraktes Interface für Klipper / Moonraker Hardware-Anbindung.
 */
class IKlipperClient : public QObject {
    Q_OBJECT
public:
    explicit IKlipperClient(QObject* parent = nullptr) : QObject(parent) {}
    ~IKlipperClient() override = default;

    virtual void connectToHost(const QString& host, int port = 7125) = 0;
    virtual void disconnectFromHost() = 0;
    [[nodiscard]] virtual bool isConnected() const = 0;
    [[nodiscard]] virtual ConnectionState connectionState() const = 0;

    virtual void sendGCode(const QString& gcode) = 0;
    virtual void emergencyStop() = 0;
    virtual void homeAxes(const QString& axes = QStringLiteral("XYZ")) = 0;
    virtual void queryTmcStatus(const QString& stepper = QStringLiteral("stepper_x")) = 0;

signals:
    void stateChanged(ConnectionState newState);
    void positionReported(const Core::Vector3D& pos);
    void gcodeResponseReceived(const QString& response);
    void tmcStatusReported(const QString& stepper, const QJsonObject& status);
    void errorMessage(const QString& error);
};

} // namespace GeminiCNC::Hardware

#endif // GEMINI_CNC_IKLIPPERCLIENT_H
