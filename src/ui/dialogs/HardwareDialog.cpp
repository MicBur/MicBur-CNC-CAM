#include "HardwareDialog.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <QJsonDocument>

namespace GeminiCNC::UI {

HardwareDialog::HardwareDialog(Hardware::MoonrakerClient* client, QWidget* parent)
    : QWidget(parent), m_client(client) {

    auto* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(6, 6, 6, 6);
    mainLayout->setSpacing(8);

    // 1. Verbindungseinstellungen
    auto* connGroup = new QGroupBox(QStringLiteral("Klipper Moonraker API Verbindung"), this);
    auto* formLayout = new QFormLayout(connGroup);

    m_editHost = new QLineEdit(QStringLiteral("127.0.0.1"), this);
    formLayout->addRow(QStringLiteral("Host / IP:"), m_editHost);

    m_spinPort = new QSpinBox(this);
    m_spinPort->setRange(1, 65535);
    m_spinPort->setValue(7125);
    formLayout->addRow(QStringLiteral("Port:"), m_spinPort);

    m_chkMockMode = new QCheckBox(QStringLiteral("Windows Offline-Simulator aktivieren (ohne echten Raspberry Pi)"), this);
    m_chkMockMode->setChecked(true);
    connect(m_chkMockMode, &QCheckBox::toggled, this, &HardwareDialog::onMockModeToggled);
    formLayout->addRow(m_chkMockMode);

    m_chkEnableFourthAxis = new QCheckBox(QStringLiteral("4. Achse (A-Drehachse) aktivieren"), this);
    m_chkEnableFourthAxis->setChecked(false);
    formLayout->addRow(m_chkEnableFourthAxis);

    auto* btnConnLayout = new QHBoxLayout();
    m_btnConnect = new QPushButton(QStringLiteral("Verbinden"), this);
    m_btnConnect->setStyleSheet("background-color: #3182CE; color: white; font-weight: bold; padding: 6px; border-radius: 4px;");
    connect(m_btnConnect, &QPushButton::clicked, this, &HardwareDialog::onConnectClicked);

    m_lblConnStatus = new QLabel(QStringLiteral("Status: Getrennt"), this);
    m_lblConnStatus->setStyleSheet("font-weight: bold; color: #A0AEC0;");

    btnConnLayout->addWidget(m_btnConnect);
    btnConnLayout->addWidget(m_lblConnStatus);
    formLayout->addRow(btnConnLayout);

    mainLayout->addWidget(connGroup);

    // 2. TMC2209 Treiberstatus
    auto* tmcGroup = new QGroupBox(QStringLiteral("TMC2209 Treiber-Diagnose"), this);
    auto* tmcLayout = new QVBoxLayout(tmcGroup);

    auto* tmcBtnLayout = new QHBoxLayout();
    auto addTmcBtn = [this, tmcBtnLayout](const QString& stepperName, const QString& label) {
        auto* b = new QPushButton(label, this);
        b->setStyleSheet("background-color: #4A5568; color: white; padding: 4px; border-radius: 3px;");
        connect(b, &QPushButton::clicked, this, [this, stepperName]() { onQueryTmcClicked(stepperName); });
        tmcBtnLayout->addWidget(b);
    };

    addTmcBtn("stepper_x", "TMC X");
    addTmcBtn("stepper_y", "TMC Y");
    addTmcBtn("stepper_z", "TMC Z");
    addTmcBtn("stepper_a", "TMC A (4. Achse)");
    tmcLayout->addLayout(tmcBtnLayout);

    mainLayout->addWidget(tmcGroup);

    // 3. Kommunikationsprotokoll / Log
    auto* logGroup = new QGroupBox(QStringLiteral("Hardware-Terminal & Log"), this);
    auto* logLayout = new QVBoxLayout(logGroup);

    m_txtLog = new QTextEdit(this);
    m_txtLog->setReadOnly(true);
    m_txtLog->setStyleSheet("background-color: #1A202C; color: #A0AEC0; font-family: Consolas, monospace; font-size: 11px;");
    logLayout->addWidget(m_txtLog);

    mainLayout->addWidget(logGroup, 1);

    if (m_client) {
        connect(m_client, &Hardware::MoonrakerClient::stateChanged, this, &HardwareDialog::onClientStateChanged);
        connect(m_client, &Hardware::MoonrakerClient::gcodeResponseReceived, this, &HardwareDialog::onClientResponse);
        connect(m_client, &Hardware::MoonrakerClient::tmcStatusReported, this, &HardwareDialog::onTmcStatus);
    }
}

void HardwareDialog::onConnectClicked() {
    if (!m_client) return;

    if (m_client->isConnected()) {
        m_client->disconnectFromHost();
    } else {
        m_client->setMockMode(m_chkMockMode->isChecked());
        m_client->connectToHost(m_editHost->text(), m_spinPort->value());
    }
}

void HardwareDialog::onMockModeToggled(bool checked) {
    if (m_client) {
        m_client->setMockMode(checked);
    }
}

void HardwareDialog::onQueryTmcClicked(const QString& stepper) {
    if (m_client) {
        m_client->queryTmcStatus(stepper);
    }
}

void HardwareDialog::onClientStateChanged(Hardware::ConnectionState state) {
    switch (state) {
        case Hardware::ConnectionState::Connected:
            m_lblConnStatus->setText(QStringLiteral("Status: Verbunden ✔"));
            m_lblConnStatus->setStyleSheet("font-weight: bold; color: #48BB78;");
            m_btnConnect->setText(QStringLiteral("Trennen"));
            m_btnConnect->setStyleSheet("background-color: #E53E3E; color: white; font-weight: bold; padding: 6px; border-radius: 4px;");
            break;
        case Hardware::ConnectionState::Connecting:
            m_lblConnStatus->setText(QStringLiteral("Status: Verbinde..."));
            m_lblConnStatus->setStyleSheet("font-weight: bold; color: #ECC94B;");
            break;
        case Hardware::ConnectionState::Disconnected:
            m_lblConnStatus->setText(QStringLiteral("Status: Getrennt"));
            m_lblConnStatus->setStyleSheet("font-weight: bold; color: #A0AEC0;");
            m_btnConnect->setText(QStringLiteral("Verbinden"));
            m_btnConnect->setStyleSheet("background-color: #3182CE; color: white; font-weight: bold; padding: 6px; border-radius: 4px;");
            break;
        case Hardware::ConnectionState::Error:
            m_lblConnStatus->setText(QStringLiteral("Status: Fehler"));
            m_lblConnStatus->setStyleSheet("font-weight: bold; color: #F56565;");
            m_btnConnect->setText(QStringLiteral("Verbinden"));
            break;
    }
}

void HardwareDialog::onClientResponse(const QString& resp) {
    m_txtLog->append(resp);
}

void HardwareDialog::onTmcStatus(const QString& stepper, const QJsonObject& status) {
    QByteArray json = QJsonDocument(status).toJson(QJsonDocument::Indented);
    m_txtLog->append(QString("--- TMC Status für %1 ---\n%2").arg(stepper, QString::fromUtf8(json)));
}

} // namespace GeminiCNC::UI
