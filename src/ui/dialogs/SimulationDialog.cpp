#include "SimulationDialog.h"
#include "geometry/StlLoader.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QFileDialog>
#include <QMessageBox>
#include <QFile>

namespace GeminiCNC::UI {

SimulationDialog::SimulationDialog(Simulation::SimulationEngine* engine, QWidget* parent)
    : QWidget(parent), m_engine(engine) {

    auto* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(6, 6, 6, 6);
    mainLayout->setSpacing(8);

    // 1. Digital Readout (DRO)
    auto* droGroup = new QGroupBox(QStringLiteral("Achs-Koordinaten (DRO)"), this);
    auto* droLayout = new QGridLayout(droGroup);

    auto makeDroLabel = [this](const QString& axisName) {
        auto* lbl = new QLabel(QString("%1: +000.000").arg(axisName), this);
        lbl->setStyleSheet("background-color: #0F172A; color: #38BDF8; font-family: Consolas, monospace; "
                           "font-size: 16px; font-weight: bold; padding: 6px; border: 1px solid #1E293B; border-radius: 4px;");
        lbl->setAlignment(Qt::AlignCenter);
        return lbl;
    };

    m_droX = makeDroLabel("X");
    m_droY = makeDroLabel("Y");
    m_droZ = makeDroLabel("Z");
    m_droA = makeDroLabel("A");

    droLayout->addWidget(m_droX, 0, 0);
    droLayout->addWidget(m_droY, 0, 1);
    droLayout->addWidget(m_droZ, 1, 0);
    droLayout->addWidget(m_droA, 1, 1);
    mainLayout->addWidget(droGroup);

    // 2. Transport-Steuerung
    auto* ctrlGroup = new QGroupBox(QStringLiteral("Simulations-Steuerung"), this);
    auto* ctrlLayout = new QVBoxLayout(ctrlGroup);

    auto* btnLayout = new QHBoxLayout();
    m_btnPlay = new QPushButton(QStringLiteral("▶ Start"), this);
    m_btnPlay->setStyleSheet("padding: 8px; font-weight: bold; background-color: #38A169; color: white; border-radius: 4px;");
    connect(m_btnPlay, &QPushButton::clicked, this, &SimulationDialog::onPlayClicked);

    m_btnPause = new QPushButton(QStringLiteral("⏸ Pause"), this);
    m_btnPause->setStyleSheet("padding: 8px; background-color: #D69E2E; color: white; border-radius: 4px;");
    connect(m_btnPause, &QPushButton::clicked, this, &SimulationDialog::onPauseClicked);

    m_btnStop = new QPushButton(QStringLiteral("⏹ Reset"), this);
    m_btnStop->setStyleSheet("padding: 8px; background-color: #E53E3E; color: white; border-radius: 4px;");
    connect(m_btnStop, &QPushButton::clicked, this, &SimulationDialog::onStopClicked);

    m_btnStep = new QPushButton(QStringLiteral("⏭ Step"), this);
    m_btnStep->setStyleSheet("padding: 8px; background-color: #4A5568; color: white; border-radius: 4px;");
    connect(m_btnStep, &QPushButton::clicked, this, &SimulationDialog::onStepClicked);

    btnLayout->addWidget(m_btnPlay);
    btnLayout->addWidget(m_btnPause);
    btnLayout->addWidget(m_btnStop);
    btnLayout->addWidget(m_btnStep);
    ctrlLayout->addLayout(btnLayout);

    // Geschwindigkeitsregler
    auto* speedLayout = new QHBoxLayout();
    speedLayout->addWidget(new QLabel(QStringLiteral("Tempo:"), this));
    m_sliderSpeed = new QSlider(Qt::Horizontal, this);
    m_sliderSpeed->setRange(1, 50);
    m_sliderSpeed->setValue(5);
    connect(m_sliderSpeed, &QSlider::valueChanged, this, &SimulationDialog::onSpeedSliderChanged);
    m_lblSpeed = new QLabel(QStringLiteral("5.0x"), this);
    m_lblSpeed->setFixedWidth(40);
    speedLayout->addWidget(m_sliderSpeed);
    speedLayout->addWidget(m_lblSpeed);
    ctrlLayout->addLayout(speedLayout);

    // Fortschrittsbalken
    m_progressBar = new QProgressBar(this);
    m_progressBar->setRange(0, 100);
    m_progressBar->setValue(0);
    m_progressBar->setTextVisible(true);
    ctrlLayout->addWidget(m_progressBar);

    m_lblProgressDetail = new QLabel(QStringLiteral("Segmente: 0 / 0"), this);
    m_lblProgressDetail->setStyleSheet("color: #A0AEC0; font-size: 11px;");
    ctrlLayout->addWidget(m_lblProgressDetail);

    mainLayout->addWidget(ctrlGroup);

    // 3. Status & Kollisionsüberwachung
    auto* collGroup = new QGroupBox(QStringLiteral("Echtzeit-Kollisionsüberwachung"), this);
    auto* collLayout = new QVBoxLayout(collGroup);

    m_lblStatus = new QLabel(QStringLiteral("Status: Bereit"), this);
    m_lblStatus->setStyleSheet("font-weight: bold; color: #68D391;");
    collLayout->addWidget(m_lblStatus);

    m_collisionList = new QListWidget(this);
    m_collisionList->setStyleSheet("background-color: #1A202C; color: #FEB2B2; font-family: Consolas, monospace; font-size: 11px;");
    m_collisionList->setFixedHeight(70);
    collLayout->addWidget(m_collisionList);

    mainLayout->addWidget(collGroup);

    // 4. Postprozessor-Auswahl & G-Code Export
    auto* exportGroup = new QGroupBox(QStringLiteral("G-Code Export"), this);
    auto* exportLayout = new QVBoxLayout(exportGroup);

    auto* ppLayout = new QHBoxLayout();
    auto* lblPP = new QLabel(QStringLiteral("Steuerung:"), this);
    lblPP->setStyleSheet("font-weight: bold;");
    m_cmbPostProcessor = new QComboBox(this);
    for (const auto& profile : CAM::PostProcessorFactory::availableProfiles()) {
        m_cmbPostProcessor->addItem(profile);
    }
    m_cmbPostProcessor->setCurrentIndex(0); // ISO Standard
    ppLayout->addWidget(lblPP);
    ppLayout->addWidget(m_cmbPostProcessor, 1);
    exportLayout->addLayout(ppLayout);

    m_btnExportGCode = new QPushButton(QStringLiteral("💾 G-Code exportieren..."), this);
    m_btnExportGCode->setStyleSheet("padding: 8px; font-weight: bold; background-color: #3182CE; color: white; border-radius: 4px;");
    connect(m_btnExportGCode, &QPushButton::clicked, this, &SimulationDialog::onExportGCodeClicked);
    exportLayout->addWidget(m_btnExportGCode);

    m_btnExportSTL = new QPushButton(QStringLiteral("📦 Fräsergebnis als STL exportieren..."), this);
    m_btnExportSTL->setStyleSheet("padding: 8px; font-weight: bold; background-color: #2B6CB0; color: white; border-radius: 4px;");
    connect(m_btnExportSTL, &QPushButton::clicked, this, &SimulationDialog::onExportStockSTLClicked);
    exportLayout->addWidget(m_btnExportSTL);

    mainLayout->addWidget(exportGroup);

    mainLayout->addStretch(1);

    if (m_engine) {
        connect(m_engine, &Simulation::SimulationEngine::positionChanged, this, &SimulationDialog::onEnginePositionChanged);
        connect(m_engine, &Simulation::SimulationEngine::stateChanged, this, &SimulationDialog::onEngineStateChanged);
        connect(m_engine, &Simulation::SimulationEngine::progressChanged, this, &SimulationDialog::onEngineProgressChanged);
        connect(m_engine, &Simulation::SimulationEngine::collisionDetected, this, &SimulationDialog::onEngineCollision);
    }
}

void SimulationDialog::setToolpath(const CAM::Toolpath& toolpath) {
    // Engine wird zentral im MainWindow (programCalculated) gesetzt, hier nur für Export merken
    m_toolpath = toolpath;
    m_collisionList->clear();
}

void SimulationDialog::onPlayClicked() {
    if (m_engine) m_engine->play();
}

void SimulationDialog::onPauseClicked() {
    if (m_engine) m_engine->pause();
}

void SimulationDialog::onStopClicked() {
    if (m_engine) m_engine->reset();
}

void SimulationDialog::onStepClicked() {
    if (m_engine) m_engine->stepForward();
}

void SimulationDialog::onSpeedSliderChanged(int value) {
    double mult = value * 1.0;
    m_lblSpeed->setText(QString("%1x").arg(mult, 0, 'f', 1));
    if (m_engine) m_engine->setSpeedMultiplier(mult);
}

void SimulationDialog::onExportGCodeClicked() {
    if (m_toolpath.empty()) {
        QMessageBox::warning(this, QStringLiteral("Kein G-Code"), QStringLiteral("Es wurde noch keine Fräsbahn berechnet."));
        return;
    }

    // PostProcessor aus Dropdown erzeugen
    auto controllerType = CAM::stringToControllerType(m_cmbPostProcessor->currentText());
    auto pp = CAM::PostProcessorFactory::create(controllerType);

    // Kontext zusammenbauen
    CAM::PostProcessorContext ctx;
    ctx.crcMode = static_cast<CAM::CrcOutputMode>(m_machineConfig.crcOutputMode);
    ctx.toolNumber = 1;
    // Kontext vom ersten tatsächlich verwendeten Werkzeug (nicht vom ersten der Bibliothek)
    for (const auto& seg : m_toolpath.segments) {
        if (seg.toolId <= 0) continue;
        ctx.toolNumber = seg.toolId;
        for (const auto& t : m_toolLibrary) {
            if (t.id == seg.toolId) { ctx.toolRadius = t.diameter / 2.0; break; }
        }
        break;
    }

    // Datei-Dialog mit korrekter Endung
    QString ext = pp->fileExtension();
    QString defaultName = QStringLiteral("programm%1").arg(ext);
    QString filter = QStringLiteral("NC-Dateien (*%1);;Alle Dateien (*.*)").arg(ext);

    QString path = QFileDialog::getSaveFileName(this, QStringLiteral("G-Code speichern"), defaultName, filter);
    if (path.isEmpty()) return;

    // G-Code über PostProcessor-Pipeline generieren
    QString gcode = pp->process(m_toolpath, m_machineConfig, m_toolLibrary, ctx);

    // Datei schreiben
    QFile file(path);
    if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        file.write(gcode.toUtf8());
        file.close();
        QMessageBox::information(this, QStringLiteral("Export erfolgreich"),
            QStringLiteral("G-Code gespeichert:\n%1\n\nPostprozessor: %2\nSegmente: %3")
                .arg(path)
                .arg(pp->name())
                .arg(m_toolpath.size()));
    } else {
        QMessageBox::warning(this, QStringLiteral("Fehler"), QStringLiteral("Datei konnte nicht geschrieben werden."));
    }
}

void SimulationDialog::updatePpDropdown() {
    if (!m_cmbPostProcessor) return;
    // Controller-Type aus MachineConfig als Vorauswahl setzen
    int idx = m_machineConfig.controllerType;
    if (idx >= 0 && idx < m_cmbPostProcessor->count()) {
        m_cmbPostProcessor->setCurrentIndex(idx);
    }
}

void SimulationDialog::onEnginePositionChanged(const Core::Vector3D& pos) {
    m_droX->setText(QString("X: %1%2").arg(pos.x >= 0 ? "+" : "").arg(pos.x, 7, 'f', 3));
    m_droY->setText(QString("Y: %1%2").arg(pos.y >= 0 ? "+" : "").arg(pos.y, 7, 'f', 3));
    m_droZ->setText(QString("Z: %1%2").arg(pos.z >= 0 ? "+" : "").arg(pos.z, 7, 'f', 3));
    m_droA->setText(QString("A: %1%2°").arg(pos.a >= 0 ? "+" : "").arg(pos.a, 6, 'f', 2));
}

void SimulationDialog::onEngineStateChanged(Simulation::SimState state) {
    m_lblStatus->setText(QString("Status: %1").arg(Simulation::simStateToString(state)));
    if (state == Simulation::SimState::HaltedOnCollision) {
        m_lblStatus->setStyleSheet("font-weight: bold; color: #FC8181;");
    } else if (state == Simulation::SimState::Running) {
        m_lblStatus->setStyleSheet("font-weight: bold; color: #63B3ED;");
    } else {
        m_lblStatus->setStyleSheet("font-weight: bold; color: #68D391;");
    }
}

void SimulationDialog::onEngineProgressChanged(double percent, size_t currentSeg, size_t totalSegs) {
    m_progressBar->setValue(static_cast<int>(percent));
    m_lblProgressDetail->setText(QString("Segmente: %1 / %2 (%3 %)")
        .arg(currentSeg).arg(totalSegs).arg(percent, 0, 'f', 1));
}

void SimulationDialog::onEngineCollision(const CAM::CollisionViolation& violation) {
    m_collisionList->addItem(violation.toString());
    m_collisionList->scrollToBottom();
}

void SimulationDialog::onExportStockSTLClicked() {
    if (!m_engine) return;

    const auto& stock = m_engine->stockModel();
    auto mesh = stock.toMesh();
    if (mesh.isEmpty()) {
        QMessageBox::warning(this, QStringLiteral("Kein Ergebnis"),
                           QStringLiteral("Kein Simulationsergebnis vorhanden. Bitte zuerst die Simulation starten."));
        return;
    }

    QString path = QFileDialog::getSaveFileName(
        this,
        QStringLiteral("Fräsergebnis als STL speichern"),
        QStringLiteral("fraes_ergebnis.stl"),
        QStringLiteral("STL-Dateien (*.stl);;Alle Dateien (*.*)")
    );
    if (path.isEmpty()) return;

    if (Geometry::StlLoader::saveBinary(path, mesh)) {
        QMessageBox::information(
            this,
            QStringLiteral("Export erfolgreich"),
            QStringLiteral("Fräsergebnis erfolgreich als STL exportiert:\n%1\n\nDreiecke: %2")
                .arg(path)
                .arg(mesh.triangleCount())
        );
    } else {
        QMessageBox::warning(this, QStringLiteral("Fehler"),
                           QStringLiteral("Datei konnte nicht geschrieben werden."));
    }
}

} // namespace GeminiCNC::UI
