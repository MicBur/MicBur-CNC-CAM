#ifndef GEMINI_CNC_MOONRAKERCLIENT_H
#define GEMINI_CNC_MOONRAKERCLIENT_H

#include "IKlipperClient.h"
#include <QNetworkAccessManager>
#include <QNetworkReply>

namespace GeminiCNC::Hardware {

/**
 * @brief Implementierung des Moonraker-REST/WebSocket-Clients mit integriertem Windows-Offline-Simulator.
 */
class MoonrakerClient : public IKlipperClient {
    Q_OBJECT
public:
    explicit MoonrakerClient(QObject* parent = nullptr);
    ~MoonrakerClient() override = default;

    void connectToHost(const QString& host, int port = 7125) override;
    void disconnectFromHost() override;
    [[nodiscard]] bool isConnected() const override;
    [[nodiscard]] ConnectionState connectionState() const override { return m_state; }

    void sendGCode(const QString& gcode) override;
    void emergencyStop() override;
    void homeAxes(const QString& axes = QStringLiteral("XYZ")) override;
    void queryTmcStatus(const QString& stepper = QStringLiteral("stepper_x")) override;

    [[nodiscard]] bool isMockMode() const { return m_mockMode; }
    void setMockMode(bool enable) { m_mockMode = enable; }

private slots:
    void onNetworkReply(QNetworkReply* reply);

private:
    void handleMockGCode(const QString& gcode);

    ConnectionState m_state{ConnectionState::Disconnected};
    QString m_host{"127.0.0.1"};
    int m_port{7125};
    bool m_mockMode{true}; // Standardmäßig Mock-Modus für autarken Windows-Betrieb

    QNetworkAccessManager* m_networkManager{nullptr};
    Core::Vector3D m_simulatedPosition{0.0, 0.0, 0.0, 0.0};
};

} // namespace GeminiCNC::Hardware

#endif // GEMINI_CNC_MOONRAKERCLIENT_H
