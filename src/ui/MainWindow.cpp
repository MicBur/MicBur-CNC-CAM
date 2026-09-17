#include "MainWindow.h"
#include "geometry/Contour.h"
#include <QAction>
#include <QMessageBox>
#include <QHBoxLayout>

namespace GeminiCNC::UI {

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
    setWindowTitle(QStringLiteral("MicBur-CNC-CAM - by Michael Burzlaff"));
    resize(1440, 900);
    setStyleSheet(QStringLiteral("QMainWindow { background-color: #1B1F23; }"));

    // Module initialisieren
    m_simEngine = std::make_unique<Simulation::SimulationEngine>(this);
    m_moonrakerClient = std::make_unique<Hardware::MoonrakerClient>(this);
    m_jogController = std::make_unique<Hardware::JogController>(m_moonrakerClient.get(), this);
    m_probeController = std::make_unique<Hardware::ProbeController>(m_moonrakerClient.get(), this);

    setupUi();
    setupToolbar();
    connectSignals();

    statusBar()->hide();
}

// ─── UI Layout: Header → [Dialoge | 3D-Viewport | Softkeys rechts] ──
void MainWindow::setupUi() {
    auto* mainContainer = new QWidget(this);
    auto* mainLayout = new QVBoxLayout(mainContainer);
    mainLayout->setContentsMargins(0, 0, 0, 0);
    mainLayout->setSpacing(0);

    // 1. Hurco Header (DRO, Status, Werkzeug)
    m_winmaxHeader = new WinMaxHeaderBar(this);
    mainLayout->addWidget(m_winmaxHeader);

    // 2. Hauptbereich: [Dialoge | 3D-Viewport | Softkeys]
    auto* contentLayout = new QHBoxLayout();
    contentLayout->setContentsMargins(0, 0, 0, 0);
    contentLayout->setSpacing(0);

    // 2a. Links: Dialog-Seiten
    m_dialogStack = new DialogStack(this);
    m_dialogStack->setMinimumWidth(440);

    // Seiten erzeugen
    m_pageSetup = new SetupDialog(this);
    m_pageToolManager = new ToolManagerDialog(this);
    m_pageConversational = new ConversationalEditorDialog(this);
    m_pageSimulation = new SimulationDialog(m_simEngine.get(), this);
    m_pageJog = new JogDialog(m_jogController.get(), this);
    m_pageProbe = new ProbeDialog(m_probeController.get(), this);
    m_pageHardware = new HardwareDialog(m_moonrakerClient.get(), this);
    m_simWindow = new SimulationWindow(m_simEngine.get(), this);

    m_dialogStack->addPage(DialogPage::Setup, m_pageSetup, QStringLiteral("Werkstück & Setup"));
    m_dialogStack->addPage(DialogPage::ToolManager, m_pageToolManager, QStringLiteral("Werkzeugverwaltung"));
    m_dialogStack->addPage(DialogPage::Programming, m_pageConversational, QStringLiteral("WinMax Arbeitsplan"));
    m_dialogStack->addPage(DialogPage::Simulation, m_pageSimulation, QStringLiteral("Automatik & 3D-Simulation"));
    m_dialogStack->addPage(DialogPage::Jog, m_pageJog, QStringLiteral("Handbetrieb (Jog)"));
    m_dialogStack->addPage(DialogPage::Probe, m_pageProbe, QStringLiteral("Einmessen & Tasten"));
    m_dialogStack->addPage(DialogPage::Hardware, m_pageHardware, QStringLiteral("Hardware & Klipper"));

    // 2b. Mitte: 3D-Viewport
    m_viewport = new Viewport3D(this);

    // Splitter für Dialog/Viewport
    m_splitter = new QSplitter(Qt::Horizontal, this);
    m_splitter->addWidget(m_dialogStack);
    m_splitter->addWidget(m_viewport);
    m_splitter->setStretchFactor(0, 1);
    m_splitter->setStretchFactor(1, 1);
    m_splitter->setHandleWidth(3);
    m_splitter->setStyleSheet(QStringLiteral(
        "QSplitter::handle { background-color: #374151; }"));

    contentLayout->addWidget(m_splitter, 1);

    m_winmaxSoftkeys = new WinMaxSoftkeyBar(this);
    contentLayout->addWidget(m_winmaxSoftkeys);

    mainLayout->addLayout(contentLayout, 1);

    // 3. Hurco WinMax Prompt-Bereich (Screenshots 191009, 191921, etc.)
    auto* promptWidget = new QWidget(this);
    promptWidget->setFixedHeight(32);
    promptWidget->setStyleSheet(QStringLiteral("background-color: #0B0F14; border-top: 1px solid #1E293B;"));
    auto* promptLayout = new QHBoxLayout(promptWidget);
    promptLayout->setContentsMargins(14, 2, 14, 2);

    m_lblContextPrompt = new QLabel(QStringLiteral("Bereit für Eingabe. Wählen Sie einen Softkey rechts."), this);
    m_lblContextPrompt->setStyleSheet(QStringLiteral("color: #00D2FF; font-family: Consolas, monospace; font-size: 12px; font-weight: bold;"));

    m_lblMachineAlarm = new QLabel(QStringLiteral("KLIPPER BEREIT • SYSTEM AKTIV • G54 NULLPUNKT GESETZT"), this);
    m_lblMachineAlarm->setStyleSheet(QStringLiteral("color: #10B981; font-family: Consolas, monospace; font-size: 11px; font-weight: bold;"));

    promptLayout->addWidget(m_lblContextPrompt, 1);
    promptLayout->addWidget(m_lblMachineAlarm, 0);
    mainLayout->addWidget(promptWidget);

    setCentralWidget(mainContainer);

    // Standard-Rohteil im Viewport anzeigen
    m_viewport->setStockMesh(m_pageSetup->stockMesh());
    m_simWindow->setStockMesh(m_pageSetup->stockMesh());
    m_pageConversational->setStockMesh(m_pageSetup->stockMesh());
    m_machineConfig = m_pageSetup->machineConfig();
    m_pageConversational->setMachineConfig(m_machineConfig);
}

void MainWindow::setupToolbar() {
    auto* tb = addToolBar(QStringLiteral("Hurco 3D-Grafik & Simulation"));
    tb->setMovable(false);
    tb->setStyleSheet(QStringLiteral(
        "QToolBar { background-color: #111827; border-bottom: 1px solid #1F2937; spacing: 4px; padding: 2px; } "
        "QToolButton { color: #E2E8F0; font-family: Consolas, sans-serif; font-size: 11px; font-weight: bold; padding: 4px 8px; border-radius: 3px; background-color: #1E293B; border: 1px solid #334155; } "
        "QToolButton:hover { background-color: #334155; border-color: #00D2FF; color: #FFFFFF; } "
        "QToolButton:checked { background-color: #2563EB; border-color: #60A5FA; color: #FFFFFF; } "
        "QLabel { color: #94A3B8; font-family: Consolas, monospace; font-size: 11px; font-weight: bold; }"));

    // 1. Kamera-Ansichten (Hurco Presets)
    auto* actIso = tb->addAction(QStringLiteral("⛶ ISO"));
    actIso->setToolTip(QStringLiteral("3D-Isometrische Ansicht"));
    connect(actIso, &QAction::triggered, m_viewport, &Viewport3D::setViewIsometric);

    auto* actTop = tb->addAction(QStringLiteral("⬒ XY (Oben)"));
    actTop->setToolTip(QStringLiteral("Draufsicht (XY-Ebene)"));
    connect(actTop, &QAction::triggered, m_viewport, &Viewport3D::setViewTop);

    auto* actFront = tb->addAction(QStringLiteral("⬓ XZ (Vorne)"));
    actFront->setToolTip(QStringLiteral("Vorderansicht (XZ-Ebene)"));
    connect(actFront, &QAction::triggered, m_viewport, &Viewport3D::setViewFront);

    auto* actSide = tb->addAction(QStringLiteral("⬔ YZ (Seite)"));
    actSide->setToolTip(QStringLiteral("Seitenansicht (YZ-Ebene)"));
    connect(actSide, &QAction::triggered, m_viewport, &Viewport3D::setViewSide);

    auto* actFit = tb->addAction(QStringLiteral("🔍 Fit"));
    actFit->setToolTip(QStringLiteral("Ansicht an Bauteil anpassen"));
    connect(actFit, &QAction::triggered, this, &MainWindow::onFitViewClicked);

    tb->addSeparator();

    // 2. Frässimulation Transport (Hurco Style)
    auto* actPlay = tb->addAction(QStringLiteral("▶ Start"));
    actPlay->setToolTip(QStringLiteral("3D-Frässimulation starten (F1)"));
    connect(actPlay, &QAction::triggered, this, [this]() {
        if (m_simEngine->state() != Simulation::SimState::Running) {
            m_simEngine->play();
            m_lblContextPrompt->setText(QStringLiteral("Simulation läuft... Materialabtrag aktiv."));
        }
    });

    auto* actPause = tb->addAction(QStringLiteral("⏸ Pause"));
    actPause->setToolTip(QStringLiteral("Simulation anhalten (F2)"));
    connect(actPause, &QAction::triggered, this, [this]() {
        m_simEngine->pause();
        m_lblContextPrompt->setText(QStringLiteral("Simulation pausiert."));
    });

    auto* actStep = tb->addAction(QStringLiteral("⏭ Satz einzeln"));
    actStep->setToolTip(QStringLiteral("Einzelsatz ausführen (F3)"));
    connect(actStep, &QAction::triggered, this, [this]() {
        m_simEngine->stepForward();
        m_lblContextPrompt->setText(QStringLiteral("Einzelsatz ausgeführt."));
    });

    auto* actReset = tb->addAction(QStringLiteral("⏹ Reset"));
    actReset->setToolTip(QStringLiteral("Simulation auf Anfang zurücksetzen (F4)"));
    connect(actReset, &QAction::triggered, this, [this]() {
        m_simEngine->reset();
        m_lblContextPrompt->setText(QStringLiteral("Simulation zurückgesetzt."));
    });

    tb->addSeparator();

    // 3. Tempo-Regler
    auto* lblTempo = new QLabel(QStringLiteral(" Tempo: "), this);
    tb->addWidget(lblTempo);

    m_simSpeedSlider = new QSlider(Qt::Horizontal, this);
    m_simSpeedSlider->setRange(1, 50);
    m_simSpeedSlider->setValue(5);
    m_simSpeedSlider->setFixedWidth(80);
    m_simSpeedSlider->setToolTip(QStringLiteral("Simulations-Geschwindigkeit (1x bis 50x)"));
    m_simSpeedLabel = new QLabel(QStringLiteral("5.0x "), this);
    m_simSpeedLabel->setStyleSheet(QStringLiteral("color: #38BDF8; font-weight: bold; min-width: 36px;"));

    connect(m_simSpeedSlider, &QSlider::valueChanged, this, [this](int val) {
        double mult = static_cast<double>(val);
        m_simEngine->setSpeedMultiplier(mult);
        m_simSpeedLabel->setText(QString("%1x ").arg(mult, 0, 'f', 1));
    });

    tb->addWidget(m_simSpeedSlider);
    tb->addWidget(m_simSpeedLabel);

    tb->addSeparator();

    // 4. Live Satz-Monitor
    m_simBlockInfo = new QLabel(QStringLiteral("Satz: 0 / 0"), this);
    m_simBlockInfo->setStyleSheet(QStringLiteral("color: #FCD34D; font-family: Consolas, monospace; font-weight: bold; padding: 0 6px;"));
    tb->addWidget(m_simBlockInfo);

    tb->addSeparator();

    // 5. Layer Toggles
    auto* actToggleStock = tb->addAction(QStringLiteral("Rohteil"));
    actToggleStock->setCheckable(true);
    actToggleStock->setChecked(true);
    connect(actToggleStock, &QAction::toggled, m_viewport, &Viewport3D::setShowStock);

    auto* actTogglePart = tb->addAction(QStringLiteral("Bauteil"));
    actTogglePart->setCheckable(true);
    actTogglePart->setChecked(true);
    connect(actTogglePart, &QAction::toggled, m_viewport, &Viewport3D::setShowPart);

    auto* actTogglePath = tb->addAction(QStringLiteral("Wege"));
    actTogglePath->setCheckable(true);
    actTogglePath->setChecked(true);
    connect(actTogglePath, &QAction::toggled, m_viewport, &Viewport3D::setShowToolpath);

    auto* actToggleGrid = tb->addAction(QStringLiteral("Gitter"));
    actToggleGrid->setCheckable(true);
    actToggleGrid->setChecked(true);
    connect(actToggleGrid, &QAction::toggled, m_viewport, &Viewport3D::setShowGrid);

    tb->addSeparator();

    // 6. Material-Rendering-Taste (Umschalten Alu/Messing/Stahl/Edelstahl/Holz/POM)
    {
        static const QStringList matNames = {
            QStringLiteral("Alu"), QStringLiteral("Messing"), QStringLiteral("Stahl"),
            QStringLiteral("Holz"), QStringLiteral("POM"), QStringLiteral("Edelstahl")
        };
        auto* actMaterial = tb->addAction(QStringLiteral("🎨 Alu"));
        actMaterial->setToolTip(QStringLiteral("Werkstück-Material wechseln (glänzendes Rendering)"));
        connect(actMaterial, &QAction::triggered, this, [this, actMaterial]() {
            static int curMat = 0;
            curMat = (curMat + 1) % matNames.size();
            m_viewport->setMaterialPreset(curMat);
            m_simWindow->setMaterialPreset(curMat);
            actMaterial->setText(QString("🎨 %1").arg(matNames[curMat]));
            if (m_lblContextPrompt) {
                m_lblContextPrompt->setText(QString("Material-Rendering: %1").arg(matNames[curMat]));
            }
        });
    }

    // 6b. Darstellungsqualität: Schnell / Realistisch / Realistisch + Schatten
    {
        static const QStringList qualityNames = {
            QStringLiteral("Schnell"), QStringLiteral("Realistisch"), QStringLiteral("Realistisch + Schatten")
        };
        auto* actQuality = tb->addAction(QStringLiteral("✨ Realistisch + Schatten"));
        actQuality->setToolTip(QStringLiteral("Werkstück-Darstellung umschalten: Schnell (einfach), "
                                              "Realistisch (Metall, Fräserspuren), Realistisch + Schatten"));
        connect(actQuality, &QAction::triggered, this, [this, actQuality]() {
            static int quality = 2;
            quality = (quality + 1) % qualityNames.size();
            m_viewport->setRenderQuality(quality);
            m_simWindow->setRenderQuality(quality);
            actQuality->setText(QString("✨ %1").arg(qualityNames[quality]));
            if (m_lblContextPrompt) {
                m_lblContextPrompt->setText(QString("Darstellung: %1").arg(qualityNames[quality]));
            }
        });
    }

    tb->addSeparator();

    // 7. Separates Simulationsfenster öffnen
    auto* actSimWindow = tb->addAction(QStringLiteral("🖥 Sim-Fenster"));
    actSimWindow->setToolTip(QStringLiteral("Simulation in separatem Fenster öffnen (Alt+Tab zum Wechseln)"));
    connect(actSimWindow, &QAction::triggered, this, [this]() {
        // Aktuelle Daten ins Sim-Fenster synchronisieren
        m_simWindow->setStockMesh(m_pageSetup->stockMesh());
        if (!m_pageConversational->currentToolpath().empty()) {
            m_simWindow->setToolpath(m_pageConversational->currentToolpath());
        }
        m_simWindow->show();
        m_simWindow->raise();
        m_simWindow->activateWindow();
    });
}

void MainWindow::connectSignals() {
    // 1. Setup-Dialog Geometrie-Events
    connect(m_pageSetup, &SetupDialog::stockMeshChanged, this, [this](const Geometry::Mesh& mesh) {
        m_viewport->setStockMesh(mesh);
        m_simWindow->setStockMesh(mesh);
        m_pageConversational->setStockMesh(mesh);
        if (m_pageSetup->stockType() >= 1) {
            // Zylinder (jede Achse) und STL über das echte Mesh
            m_simEngine->setMeshStock(mesh);
        } else if (m_pageSetup->stockType() == 1) {
            m_simEngine->setCylinderStock(mesh.boundingBox, m_pageSetup->stockCylinderRadius());
        } else {
            m_simEngine->setStockBounds(mesh.boundingBox);
        }
        m_pageConversational->onCalculateProgramClicked();
    });

    connect(m_pageSetup, &SetupDialog::partMeshChanged, this, [this](const Geometry::Mesh& mesh) {
        m_viewport->setTargetPartMesh(mesh);
        m_simWindow->setTargetPartMesh(mesh);
        m_pageConversational->setPartMesh(mesh);
    });

    connect(m_pageConversational, &ConversationalEditorDialog::partMeshChanged, this, [this](const Geometry::Mesh& mesh) {
        m_viewport->setTargetPartMesh(mesh);
        m_simWindow->setTargetPartMesh(mesh);
    });

    connect(m_pageSetup, &SetupDialog::contoursChanged, this, [this](const std::vector<Geometry::Contour>& contours) {
        m_viewport->setSelectableContours(contours);
        m_pageConversational->setContours(contours);
    });

    // Maschinenparameter-Events
    connect(m_pageSetup, &SetupDialog::machineConfigChanged, this, [this](const Core::MachineConfig& config) {
        m_machineConfig = config;
        m_pageConversational->setMachineConfig(config);
        m_pageConversational->onCalculateProgramClicked();
        m_pageSimulation->setMachineConfig(config);
    });

    connect(m_pageSetup, &SetupDialog::materialPresetChanged, this, [this](int preset) {
        m_viewport->setMaterialPreset(preset);
        m_simWindow->setMaterialPreset(preset);
    });

    // 2. Werkzeug-Events
    // Bibliothek geändert (neu, gelöscht, bearbeitet) → alle Seiten mit Werkzeugauswahl aktualisieren
    connect(m_pageToolManager, &ToolManagerDialog::toolLibraryChanged, this, [this](const QList<Core::ToolDefinition>& tools) {
        m_pageConversational->setToolLibrary(tools);
        m_pageSimulation->setToolLibrary(tools);
        m_pageProbe->setToolLibrary(tools);
        m_simEngine->setToolLibrary(tools);
    });

    connect(m_pageToolManager, &ToolManagerDialog::activeToolChanged, this, [this](const Core::ToolDefinition& tool) {
        m_viewport->setActiveTool(tool);
        m_winmaxHeader->setActiveTool(tool);
        m_pageProbe->setActiveTool(tool);
        m_simEngine->setActiveTool(tool);
    });

    // 2b. Block-Werkzeug → Header + Viewport synchronisieren
    connect(m_pageConversational, &ConversationalEditorDialog::activeToolChanged, this, [this](const Core::ToolDefinition& tool) {
        m_viewport->setActiveTool(tool);
        m_winmaxHeader->setActiveTool(tool);
        m_simEngine->setActiveTool(tool);
    });

    // 3. Fräsbahn-Vorschau (Blockvorschau, Sichtbarkeit) – nur anzeigen, Simulation bleibt unangetastet,
    //    sonst würde jedes Umschalten das bereits gefräste Rohteil löschen
    connect(m_pageConversational, &ConversationalEditorDialog::toolpathGenerated, this, [this](const CAM::Toolpath& tp) {
        m_viewport->setToolpath(tp);
        m_simWindow->setToolpath(tp);
    });

    // 3a. Gesamtprogramm neu berechnet → Simulation neu aufsetzen
    connect(m_pageConversational, &ConversationalEditorDialog::programCalculated, this, [this](const CAM::Toolpath& tp) {
        m_pageSimulation->setToolpath(tp);
        m_pageSimulation->setToolLibrary(m_pageToolManager->toolList());

        // StockModel nur mit den echten Rohteil-Abmessungen erstellen.
        // NICHT mit Toolpath-Bounds expandieren! Der Toolpath hat clearanceZ
        // der über dem Rohteil liegt — das würde das Heightfield aufblasen
        // und den Nullpunkt scheinbar ins Material versenken.
        Core::BoundingBox stockOnly = m_pageSetup->stockMesh().boundingBox;
        if (!stockOnly.isValid()) {
            // Fallback: Toolpath-BBox verwenden wenn kein Stock definiert
            stockOnly = tp.boundingBox();
        }
        if (m_pageSetup->stockType() >= 1 && !m_pageSetup->stockMesh().isEmpty()) {
            m_simEngine->setMeshStock(m_pageSetup->stockMesh());
        } else if (stockOnly.isValid()) {
            if (m_pageSetup->stockType() == 1) {
                m_simEngine->setCylinderStock(stockOnly, m_pageSetup->stockCylinderRadius());
            } else {
                m_simEngine->setStockBounds(stockOnly);
            }
        }

        // Tool-Library an SimEngine übergeben für automatischen Werkzeugwechsel
        m_simEngine->setToolLibrary(m_pageToolManager->toolList());
        m_simEngine->setToolpath(tp);
        if (m_simBlockInfo) {
            m_simBlockInfo->setText(QString("Satz: 0 / %1").arg(tp.size()));
        }
    });

    // 3b. SimEngine Werkzeugwechsel → Viewport + Header synchronisieren
    connect(m_simEngine.get(), &Simulation::SimulationEngine::activeToolChanged, this, [this](const Core::ToolDefinition& tool) {
        m_viewport->setActiveTool(tool);
        m_winmaxHeader->setActiveTool(tool);
    });

    connect(m_pageConversational, &ConversationalEditorDialog::pickingModeRequested, this, [this](bool enabled) {
        m_viewport->setPickingEnabled(enabled);
    });

    connect(m_pageConversational, &ConversationalEditorDialog::segmentEditorVisibilityChanged, this, [this](bool visible) {
        m_winmaxSoftkeys->setMenu(visible ? SoftkeyMenu::ContourEdit : SoftkeyMenu::BlockEdit);
    });

    connect(m_viewport, &Viewport3D::contourPicked, this, [this](int idx, const Geometry::Contour& contour) {
        m_pageConversational->applyPickedContour(idx, contour);
    });

    // 4. Probing-Events
    connect(m_probeController.get(), &Hardware::ProbeController::toolMeasured, this, [this](int toolId, double length, double offset) {
        Q_UNUSED(toolId); Q_UNUSED(length); Q_UNUSED(offset);
    });

    // 5. Simulation-Events
    connect(m_simEngine.get(), &Simulation::SimulationEngine::positionChanged, this, [this](const Core::Vector3D& pos) {
        m_viewport->setToolPosition(pos);
        m_winmaxHeader->setCoordinates(pos.x, pos.y, pos.z);
    });

    connect(m_simEngine.get(), &Simulation::SimulationEngine::stockUpdated, this, [this]() {
        m_viewport->updateDynamicStock(m_simEngine->stockModel());
    });

    connect(m_simEngine.get(), &Simulation::SimulationEngine::progressChanged, this, [this](double percent, size_t curSeg, size_t totalSegs) {
        if (m_simBlockInfo) {
            m_simBlockInfo->setText(QString("Satz: %1 / %2 (%3%)").arg(curSeg).arg(totalSegs).arg(percent, 0, 'f', 0));
        }
    });

    connect(m_simEngine.get(), &Simulation::SimulationEngine::simulationFinished, this, [this]() {
        if (m_lblContextPrompt) {
            m_lblContextPrompt->setText(QStringLiteral("Frässimulation erfolgreich beendet. Bauteil fertig gefräst."));
        }
    });

    connect(m_simEngine.get(), &Simulation::SimulationEngine::collisionDetected, this, [this](const CAM::CollisionViolation& v) {
        QMessageBox::critical(this, QStringLiteral("Kollisions-Alarm!"), v.toString());
    });

    // 6. Hardware DRO
    connect(m_moonrakerClient.get(), &Hardware::MoonrakerClient::positionReported, this, [this](const Core::Vector3D& pos) {
        if (m_simEngine->state() != Simulation::SimState::Running) {
            m_viewport->setToolPosition(pos);
            m_winmaxHeader->setCoordinates(pos.x, pos.y, pos.z);
        }
    });

    // 7. WinMax Softkey-Verbindung & Hurco Prompt
    connect(m_winmaxSoftkeys, &WinMaxSoftkeyBar::softkeyTriggered, this, &MainWindow::onWinMaxSoftkeyTriggered);

    connect(m_pageConversational, &ConversationalEditorDialog::promptChanged, this, [this](const QString& p) {
        if (m_lblContextPrompt) m_lblContextPrompt->setText(p);
    });

    connect(m_dialogStack, &DialogStack::pageChanged, this, [this](DialogPage page) {
        if (page == DialogPage::Programming) {
            m_winmaxSoftkeys->setMenu(SoftkeyMenu::BlockEdit);
        } else if (page == DialogPage::Jog) {
            m_winmaxSoftkeys->setMenu(SoftkeyMenu::ManualJog);
        } else if (page == DialogPage::Simulation) {
            m_winmaxSoftkeys->setMenu(SoftkeyMenu::Simulation);
        } else {
            m_winmaxSoftkeys->setMenu(SoftkeyMenu::Main);
        }
    });

    // Initiales aktives Werkzeug synchronisieren
    // Gespeicherte Werkzeugbibliothek an alle Seiten verteilen
    const auto& library = m_pageToolManager->toolList();
    m_pageConversational->setToolLibrary(library);
    m_pageSimulation->setToolLibrary(library);
    m_pageProbe->setToolLibrary(library);
    m_simEngine->setToolLibrary(library);

    auto initialTool = m_pageToolManager->activeTool();
    m_viewport->setActiveTool(initialTool);
    m_winmaxHeader->setActiveTool(initialTool);
    m_pageProbe->setActiveTool(initialTool);
    m_simEngine->setActiveTool(initialTool);
    if (m_pageSetup->stockType() >= 1) {
        m_simEngine->setMeshStock(m_pageSetup->stockMesh());
    } else if (m_pageSetup->stockType() == 1) {
        m_simEngine->setCylinderStock(m_pageSetup->stockMesh().boundingBox, m_pageSetup->stockCylinderRadius());
    } else {
        m_simEngine->setStockBounds(m_pageSetup->stockMesh().boundingBox);
    }

    // Initiales Programm berechnen und Werkzeugwege laden
    m_pageConversational->onCalculateProgramClicked();
}

// ─── Tastatursteuerung: F1–F8 + Escape ─────────────────────────────
void MainWindow::keyPressEvent(QKeyEvent* event) {
    if (event->key() >= Qt::Key_F1 && event->key() <= Qt::Key_F8) {
        int fNum = (event->key() - Qt::Key_F1) + 1;
        m_winmaxSoftkeys->triggerKey(fNum);
        event->accept();
        return;
    }
    if (event->key() == Qt::Key_Escape) {
        m_winmaxSoftkeys->setMenu(SoftkeyMenu::Main);
        event->accept();
        return;
    }
    QMainWindow::keyPressEvent(event);
}

// ─── Softkey-Routing (zentrale Steuerung nach Hurco Vorbild) ────────
void MainWindow::onWinMaxSoftkeyTriggered(SoftkeyMenu menu, int fKeyNumber, const QString& actionKey) {
    Q_UNUSED(menu);
    Q_UNUSED(fKeyNumber);

    // ── Hauptmenü (Home) ──
    if (actionKey == "setup") {
        m_dialogStack->setCurrentPage(DialogPage::Setup);
    } else if (actionKey == "tools") {
        m_dialogStack->setCurrentPage(DialogPage::ToolManager);
    } else if (actionKey == "prog") {
        m_dialogStack->setCurrentPage(DialogPage::Programming);
        m_winmaxSoftkeys->setMenu(SoftkeyMenu::BlockEdit);
    } else if (actionKey == "probe") {
        m_dialogStack->setCurrentPage(DialogPage::Probe);
    } else if (actionKey == "offsets") {
        m_dialogStack->setCurrentPage(DialogPage::Setup);
    } else if (actionKey == "graphics") {
        m_dialogStack->setCurrentPage(DialogPage::Simulation);
        m_winmaxSoftkeys->setMenu(SoftkeyMenu::Simulation);
        m_lblContextPrompt->setText(QStringLiteral("3D-Grafik & Simulation aktiv. F1: Start, F2: Pause, F3: Satz einzeln, F5: ISO, F6: XY"));
    } else if (actionKey == "menu_jog") {
        m_dialogStack->setCurrentPage(DialogPage::Jog);
        m_winmaxSoftkeys->setMenu(SoftkeyMenu::ManualJog);
    } else if (actionKey == "estop") {
        onEmergencyStopClicked();
    }

    // ── Menü-Aufrufe & Navigation ──
    else if (actionKey == "menu_newblock") {
        m_dialogStack->setCurrentPage(DialogPage::Programming);
        m_winmaxSoftkeys->setMenu(SoftkeyMenu::NewBlock);
    } else if (actionKey == "menu_milling") {
        m_winmaxSoftkeys->setMenu(SoftkeyMenu::Milling);
    } else if (actionKey == "menu_holes") {
        m_winmaxSoftkeys->setMenu(SoftkeyMenu::Holes);
    } else if (actionKey == "menu_newseg") {
        m_winmaxSoftkeys->setMenu(SoftkeyMenu::NewSegment);
    }

    // ── Zurück-Navigation (Exit) ──
    else if (actionKey == "back_main") {
        if (m_pageConversational->isSegmentEditorVisible()) {
            m_pageConversational->hideSegmentEditor();
        }
        m_dialogStack->setCurrentPage(DialogPage::Programming);
        m_winmaxSoftkeys->setMenu(SoftkeyMenu::Main);
    } else if (actionKey == "back_prog" || actionKey == "back_block") {
        if (m_pageConversational->isSegmentEditorVisible()) {
            m_pageConversational->hideSegmentEditor();
        }
        m_dialogStack->setCurrentPage(DialogPage::Programming);
        m_winmaxSoftkeys->setMenu(SoftkeyMenu::BlockEdit);
    } else if (actionKey == "back_newblock") {
        m_winmaxSoftkeys->setMenu(SoftkeyMenu::NewBlock);
    } else if (actionKey == "back_contouredit") {
        m_winmaxSoftkeys->setMenu(SoftkeyMenu::ContourEdit);
    }

    // ── Bearbeitungsblöcke hinzufügen ──
    else if (actionKey == "prog_frame") {
        m_pageConversational->onAddPocketShapeClicked(CAM::PocketShape::Rectangle);
        m_dialogStack->setCurrentPage(DialogPage::Programming);
        m_winmaxSoftkeys->setMenu(SoftkeyMenu::BlockEdit);
    } else if (actionKey == "prog_circle") {
        m_pageConversational->onAddPocketShapeClicked(CAM::PocketShape::Circle);
        m_dialogStack->setCurrentPage(DialogPage::Programming);
        m_winmaxSoftkeys->setMenu(SoftkeyMenu::BlockEdit);
    } else if (actionKey == "prog_face") {
        m_pageConversational->onAddBlockClicked(CAM::BlockType::Facing);
        m_dialogStack->setCurrentPage(DialogPage::Programming);
        m_winmaxSoftkeys->setMenu(SoftkeyMenu::BlockEdit);
    } else if (actionKey == "prog_slot") {
        m_pageConversational->onAddBlockClicked(CAM::BlockType::Slot);
        m_dialogStack->setCurrentPage(DialogPage::Programming);
        m_winmaxSoftkeys->setMenu(SoftkeyMenu::BlockEdit);
    } else if (actionKey == "prog_helix") {
        m_pageConversational->onAddBlockClicked(CAM::BlockType::HelixThread);
        m_dialogStack->setCurrentPage(DialogPage::Programming);
        m_winmaxSoftkeys->setMenu(SoftkeyMenu::BlockEdit);
    } else if (actionKey == "prog_contour") {
        m_pageConversational->onAddBlockClicked(CAM::BlockType::Contour);
        m_pageConversational->showSegmentEditor();
        m_dialogStack->setCurrentPage(DialogPage::Programming);
        m_winmaxSoftkeys->setMenu(SoftkeyMenu::ContourEdit);
    } else if (actionKey == "hole_drill" || actionKey == "hole_tap" || actionKey == "hole_bore" || actionKey == "hole_center" || actionKey == "hole_bolt" || actionKey == "hole_grid" || actionKey == "hole_locations") {
        m_pageConversational->onAddBlockClicked(CAM::BlockType::Drill);
        m_dialogStack->setCurrentPage(DialogPage::Programming);
        m_winmaxSoftkeys->setMenu(SoftkeyMenu::BlockEdit);
    }

    // ── Block-Bearbeitung ──
    else if (actionKey == "blk_prev") {
        m_pageConversational->onMoveUpClicked();
    } else if (actionKey == "blk_next") {
        m_pageConversational->onMoveDownClicked();
    } else if (actionKey == "blk_del") {
        m_pageConversational->onRemoveBlockClicked();
    } else if (actionKey == "prog_calc") {
        m_pageConversational->onCalculateProgramClicked();
    }

    // ── Kontur-Datensatz-Aktionen (Linien & Bögen) ──
    else if (actionKey == "seg_prev") {
        m_pageConversational->navigateSegmentPrev();
    } else if (actionKey == "seg_next") {
        m_pageConversational->navigateSegmentNext();
    } else if (actionKey == "seg_store") {
        m_pageConversational->storeSegmentCalculatedValue();
    } else if (actionKey == "seg_find_alt") {
        m_pageConversational->findSegmentAlternativeSolution();
    } else if (actionKey == "seg_del") {
        m_pageConversational->deleteCurrentSegment();
    }

    // ── Neue Kontursegmente ──
    else if (actionKey == "newseg_line") {
        m_pageConversational->addContourSegment(Geometry::ContourSegmentType::Line);
        m_winmaxSoftkeys->setMenu(SoftkeyMenu::ContourEdit);
    } else if (actionKey == "newseg_arc") {
        m_pageConversational->addContourSegment(Geometry::ContourSegmentType::ArcCW);
        m_winmaxSoftkeys->setMenu(SoftkeyMenu::ContourEdit);
    } else if (actionKey == "newseg_blend") {
        m_pageConversational->addContourSegment(Geometry::ContourSegmentType::Fillet);
        m_winmaxSoftkeys->setMenu(SoftkeyMenu::ContourEdit);
    } else if (actionKey == "newseg_helix") {
        m_pageConversational->addContourSegment(Geometry::ContourSegmentType::Helix);
        m_winmaxSoftkeys->setMenu(SoftkeyMenu::ContourEdit);
    }

    // ── Handbetrieb (Jog) ──
    else if (actionKey == "jog_xm") {
        m_jogController->jogAxis(Hardware::JogAxis::X, -1);
    } else if (actionKey == "jog_xp") {
        m_jogController->jogAxis(Hardware::JogAxis::X, +1);
    } else if (actionKey == "jog_ym") {
        m_jogController->jogAxis(Hardware::JogAxis::Y, -1);
    } else if (actionKey == "jog_yp") {
        m_jogController->jogAxis(Hardware::JogAxis::Y, +1);
    } else if (actionKey == "jog_zm") {
        m_jogController->jogAxis(Hardware::JogAxis::Z, -1);
    } else if (actionKey == "jog_zp") {
        m_jogController->jogAxis(Hardware::JogAxis::Z, +1);
    } else if (actionKey == "jog_zero") {
        m_jogController->zeroAll();
        m_winmaxHeader->setCoordinates(0.0, 0.0, 0.0);
    }

    // ── Hurco 3D-Simulation & Grafik Softkeys (F1–F7) ──
    else if (actionKey == "sim_start") {
        m_simEngine->play();
        m_lblContextPrompt->setText(QStringLiteral("Simulation läuft... Materialabtrag aktiv."));
    } else if (actionKey == "sim_pause") {
        m_simEngine->pause();
        m_lblContextPrompt->setText(QStringLiteral("Simulation pausiert."));
    } else if (actionKey == "sim_step") {
        m_simEngine->stepForward();
        m_lblContextPrompt->setText(QStringLiteral("Einzelsatz ausgeführt."));
    } else if (actionKey == "sim_reset") {
        m_simEngine->reset();
        m_lblContextPrompt->setText(QStringLiteral("Simulation auf Start zurückgesetzt."));
    } else if (actionKey == "sim_view_iso") {
        m_viewport->setViewIsometric();
        m_lblContextPrompt->setText(QStringLiteral("Ansicht: 3D-Isometrie (ISO)"));
    } else if (actionKey == "sim_view_xy") {
        m_viewport->setViewTop();
        m_lblContextPrompt->setText(QStringLiteral("Ansicht: XY-Ebene (Draufsicht)"));
    } else if (actionKey == "sim_view_xz") {
        static bool toggleXz = false;
        if (toggleXz) {
            m_viewport->setViewSide();
            m_lblContextPrompt->setText(QStringLiteral("Ansicht: YZ-Ebene (Seitenansicht)"));
        } else {
            m_viewport->setViewFront();
            m_lblContextPrompt->setText(QStringLiteral("Ansicht: XZ-Ebene (Vorderansicht)"));
        }
        toggleXz = !toggleXz;
    }
}

void MainWindow::onEmergencyStopClicked() {
    m_simEngine->stop();
    m_moonrakerClient->emergencyStop();
    m_winmaxHeader->setMachineState(QStringLiteral("NOT-HALT"), QStringLiteral("#EF4444"));
    QMessageBox::warning(this, QStringLiteral("NOT-HALT"),
        QStringLiteral("NOT-HALT ausgelöst! Alle Antriebe und Spindel gestoppt."));
    statusBar()->showMessage(QStringLiteral("NOT-HALT AKTIVIERT!"), 10000);
}

void MainWindow::onFitViewClicked() {
    m_viewport->fitToView();
}

} // namespace GeminiCNC::UI
