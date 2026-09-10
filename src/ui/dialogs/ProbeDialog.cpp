#include "ProbeDialog.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QFormLayout>
#include <QGroupBox>

namespace GeminiCNC::UI {

ProbeDialog::ProbeDialog(Hardware::ProbeController* probeCtrl, QWidget* parent)
    : QWidget(parent), m_probeCtrl(probeCtrl) {

    m_activeTool = Core::ToolDefinition(1, QStringLiteral("6mm Schaftfräser"), Core::ToolType::EndMill, 6.0);
    setupUi();

    if (m_probeCtrl) {
        connect(m_probeCtrl, &Hardware::ProbeController::probingStarted, this, &ProbeDialog::onProbingStarted);
        connect(m_probeCtrl, &Hardware::ProbeController::probingFinished, this, &ProbeDialog::onProbingFinished);
        connect(m_probeCtrl, &Hardware::ProbeController::probeError, this, &ProbeDialog::onProbeError);
    }
}

void ProbeDialog::setToolLibrary(const QList<Core::ToolDefinition>& tools) {
    m_toolLibrary = tools;
}

void ProbeDialog::setActiveTool(const Core::ToolDefinition& tool) {
    m_activeTool = tool;
    m_lblActiveToolInfo->setText(QString("Aktives Werkzeug: T%1 - %2 (Ø %3mm)")
        .arg(m_activeTool.id).arg(m_activeTool.name).arg(m_activeTool.diameter, 0, 'f', 1));
}

void ProbeDialog::setupUi() {
    auto* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(4, 4, 4, 4);
    mainLayout->setSpacing(6);

    // Statusbanner oben
    m_lblActiveToolInfo = new QLabel(this);
    m_lblActiveToolInfo->setStyleSheet("background-color: #2D3748; color: #63B3ED; font-weight: bold; padding: 6px; border-radius: 4px;");
    mainLayout->addWidget(m_lblActiveToolInfo);

    auto* tabWidget = new QTabWidget(this);

    // ==========================================
    // Reiter 1: Werkzeug-Einmessen (Tool Setter)
    // ==========================================
    auto* tabTool = new QWidget(this);
    auto* lTool = new QVBoxLayout(tabTool);

    auto* btnMeasure = new QPushButton(QStringLiteral("⚡ Werkzeuglänge automatisch vermessen (G38.2)"), this);
    btnMeasure->setStyleSheet("background-color: #38A169; color: white; font-weight: bold; padding: 10px; border-radius: 4px; font-size: 13px;");
    connect(btnMeasure, &QPushButton::clicked, this, &ProbeDialog::onMeasureToolClicked);
    lTool->addWidget(btnMeasure);

    auto* btnBreakage = new QPushButton(QStringLiteral("🔍 Bruchkontrolle durchführen"), this);
    btnBreakage->setStyleSheet("background-color: #D69E2E; color: white; font-weight: bold; padding: 8px; border-radius: 4px;");
    connect(btnBreakage, &QPushButton::clicked, this, &ProbeDialog::onCheckBreakageClicked);
    lTool->addWidget(btnBreakage);

    auto* setterGroup = new QGroupBox(QStringLiteral("Tool-Setter Tischposition"), this);
    auto* formSetter = new QFormLayout(setterGroup);

    m_spinSetterX = new QDoubleSpinBox(this); m_spinSetterX->setRange(-1000, 1000); m_spinSetterX->setValue(450.0); m_spinSetterX->setSuffix(" mm");
    m_spinSetterY = new QDoubleSpinBox(this); m_spinSetterY->setRange(-1000, 1000); m_spinSetterY->setValue(350.0); m_spinSetterY->setSuffix(" mm");
    m_spinSetterHeight = new QDoubleSpinBox(this); m_spinSetterHeight->setRange(1, 200); m_spinSetterHeight->setValue(40.0); m_spinSetterHeight->setSuffix(" mm");

    formSetter->addRow(QStringLiteral("X-Koordinate:"), m_spinSetterX);
    formSetter->addRow(QStringLiteral("Y-Koordinate:"), m_spinSetterY);
    formSetter->addRow(QStringLiteral("Tasterhöhe:"), m_spinSetterHeight);
    lTool->addWidget(setterGroup);
    lTool->addStretch(1);

    tabWidget->addTab(tabTool, QStringLiteral("Werkzeug-Taster"));

    // ==========================================
    // Reiter 2: Werkstück-Antasten (3D Probe & Touch Plate)
    // ==========================================
    auto* tabPart = new QWidget(this);
    auto* lPart = new QVBoxLayout(tabPart);

    // Touch Plate Box
    auto* touchPlateGroup = new QGroupBox(QStringLiteral("Touch Plate (Z- & Ecken-Tastplatte)"), this);
    auto* tpLayout = new QVBoxLayout(touchPlateGroup);

    auto* tpParamLayout = new QHBoxLayout();
    m_spinPlateThickness = new QDoubleSpinBox(this);
    m_spinPlateThickness->setRange(0.1, 100.0);
    m_spinPlateThickness->setValue(10.0);
    m_spinPlateThickness->setSuffix(" mm");

    tpParamLayout->addWidget(new QLabel(QStringLiteral("Plattendicke Z:")));
    tpParamLayout->addWidget(m_spinPlateThickness);
    tpLayout->addLayout(tpParamLayout);

    auto* tpBtnLayout = new QHBoxLayout();
    auto* btnTouchPlateZ = new QPushButton(QStringLiteral("⚡ Z-Touch Plate antasten"), this);
    btnTouchPlateZ->setStyleSheet("background-color: #2B6CB0; color: white; font-weight: bold; padding: 7px; border-radius: 3px;");
    connect(btnTouchPlateZ, &QPushButton::clicked, this, &ProbeDialog::onProbeTouchPlateZClicked);

    auto* btnCornerPlate = new QPushButton(QStringLiteral("⌖ 3D-Ecken Touch Plate (X/Y/Z)"), this);
    btnCornerPlate->setStyleSheet("background-color: #2C7A7B; color: white; font-weight: bold; padding: 7px; border-radius: 3px;");
    connect(btnCornerPlate, &QPushButton::clicked, this, &ProbeDialog::onProbeCornerTouchPlateClicked);

    tpBtnLayout->addWidget(btnTouchPlateZ);
    tpBtnLayout->addWidget(btnCornerPlate);
    tpLayout->addLayout(tpBtnLayout);

    lPart->addWidget(touchPlateGroup);

    // 3D-Messtaster Z-Fläche
    auto* btnProbeZ = new QPushButton(QStringLiteral("⬇ 3D-Messtaster: Werkstück-Z=0"), this);
    btnProbeZ->setStyleSheet("background-color: #4A5568; color: white; padding: 5px; border-radius: 3px;");
    connect(btnProbeZ, &QPushButton::clicked, this, &ProbeDialog::onProbeZClicked);
    lPart->addWidget(btnProbeZ);

    // Kanten
    auto* edgeGroup = new QGroupBox(QStringLiteral("Kanten antasten"), this);
    auto* edgeLayout = new QGridLayout(edgeGroup);

    auto* btnEdgeYPlus = new QPushButton(QStringLiteral("Kante Y+ ▲"), this);
    auto* btnEdgeYMinus = new QPushButton(QStringLiteral("Kante Y- ▼"), this);
    auto* btnEdgeXMinus = new QPushButton(QStringLiteral("◀ Kante X-"), this);
    auto* btnEdgeXPlus = new QPushButton(QStringLiteral("Kante X+ ▶"), this);

    edgeLayout->addWidget(btnEdgeYPlus, 0, 1);
    edgeLayout->addWidget(btnEdgeXMinus, 1, 0);
    edgeLayout->addWidget(btnEdgeXPlus, 1, 2);
    edgeLayout->addWidget(btnEdgeYMinus, 2, 1);

    connect(btnEdgeYPlus, &QPushButton::clicked, this, [this]() { onProbeEdgeClicked(Hardware::JogAxis::Y, +1); });
    connect(btnEdgeYMinus, &QPushButton::clicked, this, [this]() { onProbeEdgeClicked(Hardware::JogAxis::Y, -1); });
    connect(btnEdgeXPlus, &QPushButton::clicked, this, [this]() { onProbeEdgeClicked(Hardware::JogAxis::X, +1); });
    connect(btnEdgeXMinus, &QPushButton::clicked, this, [this]() { onProbeEdgeClicked(Hardware::JogAxis::X, -1); });

    lPart->addWidget(edgeGroup);

    // Bohrungsmitte & Schieflage
    auto* centerGroup = new QGroupBox(QStringLiteral("Zentrieren & Schieflage"), this);
    auto* centerLayout = new QVBoxLayout(centerGroup);

    auto* boreLayout = new QHBoxLayout();
    m_spinBoreDia = new QDoubleSpinBox(this); m_spinBoreDia->setRange(2, 500); m_spinBoreDia->setValue(30.0); m_spinBoreDia->setSuffix(" mm");
    auto* btnProbeBore = new QPushButton(QStringLiteral("◎ Bohrungsmitte finden (4-Punkt)"), this);
    btnProbeBore->setStyleSheet("background-color: #805AD5; color: white; font-weight: bold; padding: 6px; border-radius: 3px;");
    connect(btnProbeBore, &QPushButton::clicked, this, &ProbeDialog::onProbeBoreClicked);

    boreLayout->addWidget(new QLabel(QStringLiteral("Ø ca.:")));
    boreLayout->addWidget(m_spinBoreDia);
    boreLayout->addWidget(btnProbeBore, 1);
    centerLayout->addLayout(boreLayout);

    auto* skewLayout = new QHBoxLayout();
    m_spinSkewDistance = new QDoubleSpinBox(this); m_spinSkewDistance->setRange(10, 1000); m_spinSkewDistance->setValue(60.0); m_spinSkewDistance->setSuffix(" mm");
    auto* btnProbeSkew = new QPushButton(QStringLiteral("📐 Schieflage kompensieren (G68)"), this);
    btnProbeSkew->setStyleSheet("background-color: #DD6B20; color: white; font-weight: bold; padding: 6px; border-radius: 3px;");
    connect(btnProbeSkew, &QPushButton::clicked, this, &ProbeDialog::onProbeSkewClicked);

    skewLayout->addWidget(new QLabel(QStringLiteral("Messweg:")));
    skewLayout->addWidget(m_spinSkewDistance);
    skewLayout->addWidget(btnProbeSkew, 1);
    centerLayout->addLayout(skewLayout);

    lPart->addWidget(centerGroup);
    lPart->addStretch(1);

    tabWidget->addTab(tabPart, QStringLiteral("Werkstück-Antasten"));
    mainLayout->addWidget(tabWidget, 1);

    // Status & Log
    m_lblProbeStatus = new QLabel(QStringLiteral("Taster bereit. Keine Messung aktiv."), this);
    m_lblProbeStatus->setStyleSheet("color: #68D391; font-weight: bold;");
    mainLayout->addWidget(m_lblProbeStatus);

    m_txtLog = new QTextEdit(this);
    m_txtLog->setFixedHeight(70);
    m_txtLog->setReadOnly(true);
    m_txtLog->setStyleSheet("background-color: #1A202C; color: #E2E8F0; font-family: Consolas, monospace; font-size: 11px;");
    mainLayout->addWidget(m_txtLog);
}

void ProbeDialog::onMeasureToolClicked() {
    if (!m_probeCtrl) return;
    Hardware::ToolSetterConfig cfg = m_probeCtrl->toolSetterConfig();
    cfg.position.x = m_spinSetterX->value();
    cfg.position.y = m_spinSetterY->value();
    cfg.setterHeight = m_spinSetterHeight->value();
    m_probeCtrl->setToolSetterConfig(cfg);

    m_probeCtrl->startToolLengthMeasurement(m_activeTool.id);
}

void ProbeDialog::onCheckBreakageClicked() {
    if (!m_probeCtrl) return;
    m_probeCtrl->checkToolBreakage(m_activeTool.id, m_activeTool.overallLength);
}

void ProbeDialog::onProbeZClicked() {
    if (m_probeCtrl) m_probeCtrl->probeSurfaceZ();
}

void ProbeDialog::onProbeTouchPlateZClicked() {
    if (m_probeCtrl) {
        double th = m_spinPlateThickness ? m_spinPlateThickness->value() : 10.0;
        m_probeCtrl->probeTouchPlateZ(th);
    }
}

void ProbeDialog::onProbeCornerTouchPlateClicked() {
    if (m_probeCtrl) {
        double th = m_spinPlateThickness ? m_spinPlateThickness->value() : 10.0;
        m_probeCtrl->probeCornerTouchPlate(th, 10.0, 10.0);
    }
}

void ProbeDialog::onProbeEdgeClicked(Hardware::JogAxis axis, int dir) {
    if (m_probeCtrl) m_probeCtrl->probeEdge(axis, dir);
}

void ProbeDialog::onProbeCornerClicked(Hardware::CornerPosition corner) {
    if (m_probeCtrl) m_probeCtrl->probeCorner(corner);
}

void ProbeDialog::onProbeBoreClicked() {
    if (m_probeCtrl) m_probeCtrl->probeBoreCenter(m_spinBoreDia->value());
}

void ProbeDialog::onProbeSkewClicked() {
    if (m_probeCtrl) m_probeCtrl->probePartSkew(m_spinSkewDistance->value());
}

void ProbeDialog::onProbingStarted(const QString& cycleName) {
    m_lblProbeStatus->setText(QString("Tastzyklus aktiv: %1...").arg(cycleName));
    m_lblProbeStatus->setStyleSheet("color: #ECC94B; font-weight: bold;");
    m_txtLog->append(QString("▶ Starte: %1").arg(cycleName));
}

void ProbeDialog::onProbingFinished(const QString& result) {
    m_lblProbeStatus->setText(QStringLiteral("Messung erfolgreich abgeschlossen."));
    m_lblProbeStatus->setStyleSheet("color: #68D391; font-weight: bold;");
    m_txtLog->append(QString("✔ %1").arg(result));
}

void ProbeDialog::onProbeError(const QString& error) {
    m_lblProbeStatus->setText(QString("Fehler: %1").arg(error));
    m_lblProbeStatus->setStyleSheet("color: #FC8181; font-weight: bold;");
    m_txtLog->append(QString("✖ %1").arg(error));
}

} // namespace GeminiCNC::UI
