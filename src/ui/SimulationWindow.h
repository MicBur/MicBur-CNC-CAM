#ifndef GEMINI_CNC_SIMULATIONWINDOW_H
#define GEMINI_CNC_SIMULATIONWINDOW_H

#include <QMainWindow>
#include <QSlider>
#include <QLabel>
#include <QPushButton>
#include <QToolBar>
#include <QStatusBar>

#include "ui/viewport/Viewport3D.h"
#include "simulation/SimulationEngine.h"
#include "geometry/Mesh.h"
#include "cam/Toolpath.h"

namespace GeminiCNC::UI {

/**
 * @brief Separates Fenster für die 3D-Frässimulation mit eigenem Viewport.
 *
 * Kann neben dem Hauptfenster geöffnet werden, damit der Benutzer gleichzeitig
 * programmieren und die Simulation beobachten kann (Dual-Monitor oder Alt+Tab).
 */
class SimulationWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit SimulationWindow(Simulation::SimulationEngine* engine, QWidget* parent = nullptr);
    ~SimulationWindow() override = default;

    /// @brief Viewport-Daten synchron zum Hauptfenster halten
    void setStockMesh(const Geometry::Mesh& mesh);
    void setTargetPartMesh(const Geometry::Mesh& mesh);
    void setToolpath(const CAM::Toolpath& toolpath);
    void setMaterialPreset(int preset);

    /// @brief Zugriff auf den eigenen Viewport (für Signal-Routing)
    [[nodiscard]] Viewport3D* viewport() const { return m_viewport; }

protected:
    void closeEvent(QCloseEvent* event) override;

private:
    void setupUi();
    void setupToolbar();
    void connectEngineSignals();

    Simulation::SimulationEngine* m_engine{nullptr};
    Viewport3D* m_viewport{nullptr};

    // Transport-Controls
    QPushButton* m_btnPlay{nullptr};
    QPushButton* m_btnPause{nullptr};
    QPushButton* m_btnStop{nullptr};
    QPushButton* m_btnStep{nullptr};
    QSlider* m_sliderSpeed{nullptr};
    QLabel* m_lblSpeed{nullptr};

    // Status
    QLabel* m_lblBlockInfo{nullptr};
    QLabel* m_lblStatus{nullptr};
};

} // namespace GeminiCNC::UI

#endif // GEMINI_CNC_SIMULATIONWINDOW_H
