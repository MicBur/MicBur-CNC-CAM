#include <iostream>
#include <cassert>
#include <QCoreApplication>
#include <QTimer>

#include "hardware/MoonrakerClient.h"
#include "hardware/JogController.h"

using namespace GeminiCNC;

void testMoonrakerMockMode() {
    std::cout << "[TEST] Moonraker Mock Client Connection & Commands..." << std::endl;

    int argc = 1;
    char* argv[] = {(char*)"TestGeminiCNC", nullptr};
    QCoreApplication app(argc, argv);

    Hardware::MoonrakerClient client;
    client.setMockMode(true);

    bool connectedSignalReceived = false;
    QObject::connect(&client, &Hardware::MoonrakerClient::stateChanged, [&](Hardware::ConnectionState state) {
        if (state == Hardware::ConnectionState::Connected) {
            connectedSignalReceived = true;
        }
    });

    client.connectToHost("127.0.0.1", 7125);

    // Event-Loop kurz laufen lassen für den QTimer::singleShot
    QTimer::singleShot(250, &app, &QCoreApplication::quit);
    app.exec();

    assert(connectedSignalReceived);
    assert(client.isConnected());

    // Homing testen
    client.homeAxes("XYZ");

    // TMC2209 Statusabfrage testen
    bool tmcReceived = false;
    QObject::connect(&client, &Hardware::MoonrakerClient::tmcStatusReported, [&](const QString& stepper, const QJsonObject& status) {
        if (stepper == "stepper_x" && status.contains("current_mA")) {
            tmcReceived = true;
        }
    });

    client.queryTmcStatus("stepper_x");
    assert(tmcReceived);

    // Jog Controller testen
    Hardware::JogController jog(&client);
    jog.setStepDistance(5.0);
    jog.jogAxis(Hardware::JogAxis::X, +1);

    std::cout << " -> PASSED" << std::endl;
}

int main() {
    std::cout << "=== Running Hardware Test Suite ===" << std::endl;
    testMoonrakerMockMode();
    std::cout << "=== All Hardware Tests PASSED ===" << std::endl;
    return 0;
}
