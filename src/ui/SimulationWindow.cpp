#include "SimulationWindow.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QCloseEvent>

namespace GeminiCNC::UI {

SimulationWindow::SimulationWindow(Simulation::SimulationEngine* engine, QWidget* parent)
    : QMainWindow(parent), m_engine(engine) {
    setupUi();
    setupToolbar();
    connectEngineSignals();
}

void SimulationWindow::setupUi() {
    setWindowTitle(QStringLiteral("GeminiCNC — 3D Frässimulation"));
    setMinimumSize(800, 600);
    resize(1024, 768);

    // Dark Theme wie Hauptfenster
    setStyleSheet(QStringLiteral(
        "QMainWindow { background-color: #0B0F14; }"
        "QToolBar { background-color: #111827; border-bottom: 1px solid #1F2937; spacing: 4px; padding: 2px; }"
        "QToolButton { color: #E2E8F0; font-family: Consolas, sans-serif; font-size: 11px; font-weight: bold; "
        "  padding: 4px 8px; border-radius: 3px; background-color: #1E293B; border: 1px solid #334155; }"
        "QToolButton:hover { background-color: #334155; border-color: #00D2FF; color: #FFFFFF; }"
        "QToolButton:checked { background-color: #2563EB; border-color: #60A5FA; color: #FFFFFF; }"
        "QLabel { color: #94A3B8; font-family: Consolas, monospace; font-size: 11px; font-weight: bold; }"
        "QStatusBar { background-color: #0B0F14; color: #00D2FF; font-family: Consolas, monospace; font-size: 12px; }"
    ));

    m_viewport = new Viewport3D(this);
    setCentralWidget(m_viewport);

    // Statusbar
    m_lblBlockInfo = new QLabel(QStringLiteral("Satz: 0 / 0"), this);
    m_lblBlockInfo->setStyleSheet("color: #00D2FF; font-weight: bold; padding: 0 12px;");
    m_lblStatus = new QLabel(QStringLiteral("BEREIT"), this);
    m_lblStatus->setStyleSheet("color: #10B981; font-weight: bold;");

    statusBar()->addWidget(m_lblStatus);
    statusBar()->addPermanentWidget(m_lblBlockInfo);
}

void SimulationWindow::setupToolbar() {
    auto* tb = addToolBar(QStringLiteral("Simulation"));
    tb->setMovable(false);

    // Transport-Controls
    m_btnPlay = new QPushButton(QStringLiteral("▶ Start"), this);
    m_btnPlay->setStyleSheet("background-color: #16A34A; color: white; font-weight: bold; padding: 5px 12px; border-radius: 3px;");
    connect(m_btnPlay, &QPushButton::clicked, this, [this]() {
        if (m_engine) m_engine->play();
    });
    tb->addWidget(m_btnPlay);

    m_btnPause = new QPushButton(QStringLiteral("⏸ Pause"), this);
    m_btnPause->setStyleSheet("background-color: #CA8A04; color: white; font-weight: bold; padding: 5px 12px; border-radius: 3px;");
    connect(m_btnPause, &QPushButton::clicked, this, [this]() {
        if (m_engine) m_engine->pause();
    });
    tb->addWidget(m_btnPause);

    m_btnStop = new QPushButton(QStringLiteral("⏹ Stop"), this);
    m_btnStop->setStyleSheet("background-color: #DC2626; color: white; font-weight: bold; padding: 5px 12px; border-radius: 3px;");
    connect(m_btnStop, &QPushButton::clicked, this, [this]() {
        if (m_engine) m_engine->stop();
    });
    tb->addWidget(m_btnStop);

    m_btnStep = new QPushButton(QStringLiteral("⏭ Satz"), this);
    m_btnStep->setStyleSheet("background-color: #2563EB; color: white; font-weight: bold; padding: 5px 12px; border-radius: 3px;");
    connect(m_btnStep, &QPushButton::clicked, this, [this]() {
        if (m_engine) m_engine->stepForward();
    });
    tb->addWidget(m_btnStep);

    tb->addSeparator();

    // Geschwindigkeitsregler
    auto* lblSpd = new QLabel(QStringLiteral(" Tempo: "), this);
    tb->addWidget(lblSpd);

    m_sliderSpeed = new QSlider(Qt::Horizontal, this);
    m_sliderSpeed->setRange(1, 100);
    m_sliderSpeed->setValue(10);
    m_sliderSpeed->setFixedWidth(120);
    m_sliderSpeed->setStyleSheet(
        "QSlider::groove:horizontal { height: 6px; background: #334155; border-radius: 3px; }"
        "QSlider::handle:horizontal { width: 14px; margin: -4px 0; background: #00D2FF; border-radius: 7px; }"
    );
    connect(m_sliderSpeed, &QSlider::valueChanged, this, [this](int val) {
        double mult = static_cast<double>(val) * 0.5;
        if (m_engine) m_engine->setSpeedMultiplier(mult);
        m_lblSpeed->setText(QString("%1×").arg(mult, 0, 'f', 1));
    });
    tb->addWidget(m_sliderSpeed);

    m_lblSpeed = new QLabel(QStringLiteral("5.0×"), this);
    m_lblSpeed->setFixedWidth(50);
    tb->addWidget(m_lblSpeed);

    tb->addSeparator();

    // Ansichten
    auto* actIso = tb->addAction(QStringLiteral("⛶ ISO"));
    connect(actIso, &QAction::triggered, m_viewport, &Viewport3D::setViewIsometric);

    auto* actTop = tb->addAction(QStringLiteral("⬓ XY"));
    connect(actTop, &QAction::triggered, m_viewport, &Viewport3D::setViewTop);

    auto* actFront = tb->addAction(QStringLiteral("◻ XZ"));
    connect(actFront, &QAction::triggered, m_viewport, &Viewport3D::setViewFront);

    auto* actSide = tb->addAction(QStringLiteral("◻ YZ"));
    connect(actSide, &QAction::triggered, m_viewport, &Viewport3D::setViewSide);

    auto* actFit = tb->addAction(QStringLiteral("🔍 Einpassen"));
    connect(actFit, &QAction::triggered, m_viewport, &Viewport3D::fitToView);

    tb->addSeparator();

    // Sichtbarkeits-Toggles
    auto* actStock = tb->addAction(QStringLiteral("Rohteil"));
    actStock->setCheckable(true);
    actStock->setChecked(true);
    connect(actStock, &QAction::toggled, m_viewport, &Viewport3D::setShowStock);

    auto* actPart = tb->addAction(QStringLiteral("Bauteil"));
    actPart->setCheckable(true);
    actPart->setChecked(true);
    connect(actPart, &QAction::toggled, m_viewport, &Viewport3D::setShowPart);

    auto* actPath = tb->addAction(QStringLiteral("Fräsbahnen"));
    actPath->setCheckable(true);
    actPath->setChecked(true);
    connect(actPath, &QAction::toggled, m_viewport, &Viewport3D::setShowToolpath);

    auto* actGrid = tb->addAction(QStringLiteral("Raster"));
    actGrid->setCheckable(true);
    actGrid->setChecked(true);
    connect(actGrid, &QAction::toggled, m_viewport, &Viewport3D::setShowGrid);
}

void SimulationWindow::connectEngineSignals() {
    if (!m_engine) return;

    connect(m_engine, &Simulation::SimulationEngine::positionChanged, this, [this](const Core::Vector3D& pos) {
        m_viewport->setToolPosition(pos);
    });

    connect(m_engine, &Simulation::SimulationEngine::stockUpdated, this, [this]() {
        m_viewport->updateDynamicStock(m_engine->stockModel());
    });

    connect(m_engine, &Simulation::SimulationEngine::progressChanged, this,
            [this](double percent, size_t curSeg, size_t totalSegs) {
        m_lblBlockInfo->setText(QString("Satz: %1 / %2 (%3%)")
            .arg(curSeg).arg(totalSegs).arg(percent, 0, 'f', 0));
    });

    connect(m_engine, &Simulation::SimulationEngine::stateChanged, this,
            [this](Simulation::SimState state) {
        switch (state) {
            case Simulation::SimState::Idle:
                m_lblStatus->setText(QStringLiteral("BEREIT"));
                m_lblStatus->setStyleSheet("color: #10B981; font-weight: bold;");
                break;
            case Simulation::SimState::Running:
                m_lblStatus->setText(QStringLiteral("▶ LÄUFT"));
                m_lblStatus->setStyleSheet("color: #00D2FF; font-weight: bold;");
                break;
            case Simulation::SimState::Paused:
                m_lblStatus->setText(QStringLiteral("⏸ PAUSE"));
                m_lblStatus->setStyleSheet("color: #F59E0B; font-weight: bold;");
                break;
            case Simulation::SimState::Finished:
                m_lblStatus->setText(QStringLiteral("✔ FERTIG"));
                m_lblStatus->setStyleSheet("color: #10B981; font-weight: bold;");
                break;
            case Simulation::SimState::HaltedOnCollision:
                m_lblStatus->setText(QStringLiteral("⚠ KOLLISION!"));
                m_lblStatus->setStyleSheet("color: #EF4444; font-weight: bold;");
                break;
        }
    });

    connect(m_engine, &Simulation::SimulationEngine::simulationFinished, this, [this]() {
        m_lblStatus->setText(QStringLiteral("✔ Simulation beendet — Bauteil fertig gefräst."));
        m_lblStatus->setStyleSheet("color: #10B981; font-weight: bold;");
    });
}

void SimulationWindow::setStockMesh(const Geometry::Mesh& mesh) {
    m_viewport->setStockMesh(mesh);
}

void SimulationWindow::setTargetPartMesh(const Geometry::Mesh& mesh) {
    m_viewport->setTargetPartMesh(mesh);
}

void SimulationWindow::setToolpath(const CAM::Toolpath& toolpath) {
    m_viewport->setToolpath(toolpath);
}

void SimulationWindow::setRenderQuality(int quality) {
    m_viewport->setRenderQuality(quality);
}

void SimulationWindow::setMaterialPreset(int preset) {
    m_viewport->setMaterialPreset(preset);
}

void SimulationWindow::closeEvent(QCloseEvent* event) {
    // Fenster nur verstecken, nicht zerstören — kann wieder geöffnet werden
    event->ignore();
    hide();
}

} // namespace GeminiCNC::UI
