#ifndef GEMINI_CNC_SIMULATIONENGINE_H
#define GEMINI_CNC_SIMULATIONENGINE_H

#include <QObject>
#include <QTimer>
#include <QList>
#include <memory>
#include "core/Vector3D.h"
#include "core/ToolDefinition.h"
#include "cam/Toolpath.h"
#include "cam/CollisionDetector.h"
#include "StockModel.h"

namespace GeminiCNC::Simulation {

enum class SimState {
    Idle,
    Running,
    Paused,
    Finished,
    HaltedOnCollision
};

QString simStateToString(SimState state);

/**
 * @brief Virtuelle CNC-Interpolations-Engine für die 3D-Simulation.
 */
class SimulationEngine : public QObject {
    Q_OBJECT
public:
    explicit SimulationEngine(QObject* parent = nullptr);
    ~SimulationEngine() override;

    void setToolpath(const CAM::Toolpath& toolpath);
    void setActiveTool(const Core::ToolDefinition& tool);
    void setToolLibrary(const QList<Core::ToolDefinition>& tools);
    void setStockBounds(const Core::BoundingBox& stockBounds);
    void setCylinderStock(const Core::BoundingBox& stockBounds, double radius);
    void setMeshStock(const Geometry::Mesh& stockMesh);  // vorgefrästes STL-Rohteil

    [[nodiscard]] SimState state() const { return m_state; }
    [[nodiscard]] Core::Vector3D currentPosition() const { return m_currentPos; }
    [[nodiscard]] double speedMultiplier() const { return m_speedMultiplier; }
    [[nodiscard]] size_t currentSegmentIndex() const { return m_currentSegmentIdx; }
    [[nodiscard]] const StockModel& stockModel() const { return m_stockModel; }

public slots:
    void play();
    void pause();
    void stop();
    void reset();
    void stepForward();
    void setSpeedMultiplier(double multiplier);
    void jumpToSegment(size_t index);

signals:
    void positionChanged(const Core::Vector3D& pos);
    void stateChanged(SimState newState);
    void progressChanged(double percent, size_t currentSeg, size_t totalSegs);
    void collisionDetected(const CAM::CollisionViolation& violation);
    void simulationFinished();
    void stockUpdated();
    void activeToolChanged(const Core::ToolDefinition& tool);

private slots:
    void onTick();

private:
    void updateSegmentMovement(double dtSec);
    void checkToolChange(int segToolId);

    SimState m_state{SimState::Idle};
    CAM::Toolpath m_toolpath;
    Core::ToolDefinition m_activeTool;
    QList<Core::ToolDefinition> m_toolLibrary;
    int m_lastEmittedToolId{-1};
    StockModel m_stockModel;

    Core::Vector3D m_currentPos{0.0, 0.0, 20.0, 0.0};
    Core::Vector3D m_lastCarvePos{0.0, 0.0, 20.0, 0.0};
    size_t m_currentSegmentIdx{0};
    double m_segmentProgress{0.0}; // 0.0 bis 1.0 innerhalb des Segments

    double m_speedMultiplier{5.0}; // 5-fache Echtzeit als Standard
    QTimer* m_timer{nullptr};
    qint64 m_lastTickMs{0};
};

} // namespace GeminiCNC::Simulation

#endif // GEMINI_CNC_SIMULATIONENGINE_H
