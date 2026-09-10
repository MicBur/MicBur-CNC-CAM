#ifndef GEMINI_CNC_SIMULATIONDIALOG_H
#define GEMINI_CNC_SIMULATIONDIALOG_H

#include <QWidget>
#include <QPushButton>
#include <QSlider>
#include <QProgressBar>
#include <QLabel>
#include <QListWidget>
#include <QComboBox>
#include "simulation/SimulationEngine.h"
#include "cam/Toolpath.h"
#include "cam/PostProcessor.h"
#include "core/MachineConfig.h"
#include "core/ToolDefinition.h"

namespace GeminiCNC::UI {

/**
 * @brief Dialog für Schritt 4: 3D-Simulation, Echtzeit-Achs-DRO, Kollisionsüberwachung & G-Code-Export.
 */
class SimulationDialog : public QWidget {
    Q_OBJECT
public:
    explicit SimulationDialog(Simulation::SimulationEngine* engine, QWidget* parent = nullptr);

    void setToolpath(const CAM::Toolpath& toolpath);
    void setMachineConfig(const Core::MachineConfig& config) { m_machineConfig = config; updatePpDropdown(); }
    void setToolLibrary(const QList<Core::ToolDefinition>& tools) { m_toolLibrary = tools; }

private slots:
    void onPlayClicked();
    void onPauseClicked();
    void onStopClicked();
    void onStepClicked();
    void onSpeedSliderChanged(int value);
    void onExportGCodeClicked();

    void onEnginePositionChanged(const Core::Vector3D& pos);
    void onEngineStateChanged(Simulation::SimState state);
    void onEngineProgressChanged(double percent, size_t currentSeg, size_t totalSegs);
    void onEngineCollision(const CAM::CollisionViolation& violation);

private:
    void updatePpDropdown();

    Simulation::SimulationEngine* m_engine{nullptr};
    CAM::Toolpath m_toolpath;
    Core::MachineConfig m_machineConfig;
    QList<Core::ToolDefinition> m_toolLibrary;

    QPushButton* m_btnPlay{nullptr};
    QPushButton* m_btnPause{nullptr};
    QPushButton* m_btnStop{nullptr};
    QPushButton* m_btnStep{nullptr};

    QSlider* m_sliderSpeed{nullptr};
    QLabel* m_lblSpeed{nullptr};

    QProgressBar* m_progressBar{nullptr};
    QLabel* m_lblProgressDetail{nullptr};

    // Digital Readout (DRO)
    QLabel* m_droX{nullptr};
    QLabel* m_droY{nullptr};
    QLabel* m_droZ{nullptr};
    QLabel* m_droA{nullptr};

    QLabel* m_lblStatus{nullptr};
    QListWidget* m_collisionList{nullptr};

    // PostProcessor-Auswahl
    QComboBox* m_cmbPostProcessor{nullptr};
    QPushButton* m_btnExportGCode{nullptr};
};

} // namespace GeminiCNC::UI

#endif // GEMINI_CNC_SIMULATIONDIALOG_H
