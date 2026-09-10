#ifndef GEMINI_CNC_MAINWINDOW_H
#define GEMINI_CNC_MAINWINDOW_H

#include <QMainWindow>
#include <QSplitter>
#include <QToolBar>
#include <QStatusBar>
#include <memory>

#include "viewport/Viewport3D.h"
#include "dialogs/DialogStack.h"
#include "dialogs/SetupDialog.h"
#include "dialogs/ToolManagerDialog.h"
#include "dialogs/ContourDialog.h"
#include "dialogs/SimulationDialog.h"
#include "dialogs/JogDialog.h"
#include "dialogs/HardwareDialog.h"
#include "dialogs/ConversationalEditorDialog.h"
#include "dialogs/ProbeDialog.h"
#include "SimulationWindow.h"

#include "core/MachineConfig.h"
#include "simulation/SimulationEngine.h"
#include "hardware/MoonrakerClient.h"
#include "hardware/JogController.h"
#include "hardware/ProbeController.h"

#include "winmax/WinMaxHeaderBar.h"
#include "winmax/WinMaxSoftkeyBar.h"
#include <QKeyEvent>

namespace GeminiCNC::UI {

/**
 * @brief Hauptfenster der dialogbasierten CNC-Steuerungs- und Simulationssoftware nach Hurco WinMax Vorbild.
 */
class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override = default;

protected:
    void keyPressEvent(QKeyEvent* event) override;

private slots:
    void onEmergencyStopClicked();
    void onFitViewClicked();
    void onWinMaxSoftkeyTriggered(SoftkeyMenu menu, int fKeyNumber, const QString& actionKey);

private:
    void setupUi();
    void setupToolbar();
    void connectSignals();

    // Module
    Core::MachineConfig m_machineConfig;
    std::unique_ptr<Simulation::SimulationEngine> m_simEngine;
    std::unique_ptr<Hardware::MoonrakerClient> m_moonrakerClient;
    std::unique_ptr<Hardware::JogController> m_jogController;
    std::unique_ptr<Hardware::ProbeController> m_probeController;

    // WinMax UI Komponenten
    WinMaxHeaderBar* m_winmaxHeader{nullptr};
    WinMaxSoftkeyBar* m_winmaxSoftkeys{nullptr};
    QSplitter* m_splitter{nullptr};
    Viewport3D* m_viewport{nullptr};
    DialogStack* m_dialogStack{nullptr};

    SetupDialog* m_pageSetup{nullptr};
    ToolManagerDialog* m_pageToolManager{nullptr};
    ConversationalEditorDialog* m_pageConversational{nullptr};
    SimulationDialog* m_pageSimulation{nullptr};
    JogDialog* m_pageJog{nullptr};
    ProbeDialog* m_pageProbe{nullptr};
    HardwareDialog* m_pageHardware{nullptr};
    SimulationWindow* m_simWindow{nullptr};

    // Hurco Prompt & Status
    QLabel* m_lblContextPrompt{nullptr};
    QLabel* m_lblMachineAlarm{nullptr};

    // Simulations-Steuerung in Viewport-Toolbar
    QSlider* m_simSpeedSlider{nullptr};
    QLabel* m_simSpeedLabel{nullptr};
    QLabel* m_simBlockInfo{nullptr};
};

} // namespace GeminiCNC::UI

#endif // GEMINI_CNC_MAINWINDOW_H
