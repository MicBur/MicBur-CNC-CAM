#include "SimulationEngine.h"
#include <QDateTime>
#include <cmath>

namespace GeminiCNC::Simulation {

namespace {

// Art der Fräserspur für die Darstellung
int markKindFor(const Core::ToolDefinition& tool) {
    if (tool.type == Core::ToolType::BallMill) return 2;
    if (tool.type == Core::ToolType::Drill) return 3;
    return 1;
}

// Vorschub je Umdrehung (mm) bestimmt den Abstand der Fräserspuren
double markPitchFor(const CAM::PathSegment& seg) {
    return seg.spindleRpm > 1.0 ? seg.feedRate / seg.spindleRpm : 0.1;
}

} // namespace

QString simStateToString(SimState state) {
    switch (state) {
        case SimState::Idle: return QStringLiteral("Bereit");
        case SimState::Running: return QStringLiteral("Simulation läuft");
        case SimState::Paused: return QStringLiteral("Pausiert");
        case SimState::Finished: return QStringLiteral("Abgeschlossen");
        case SimState::HaltedOnCollision: return QStringLiteral("NOT-HALT: Kollision!");
    }
    return QStringLiteral("Unbekannt");
}

SimulationEngine::SimulationEngine(QObject* parent) : QObject(parent) {
    m_timer = new QTimer(this);
    m_timer->setInterval(16); // ~60 FPS Simulations-Tick
    connect(m_timer, &QTimer::timeout, this, &SimulationEngine::onTick);
}

SimulationEngine::~SimulationEngine() {
    if (m_timer && m_timer->isActive()) {
        m_timer->stop();
    }
}

void SimulationEngine::setToolpath(const CAM::Toolpath& toolpath) {
    m_toolpath = toolpath;
    reset();
}

void SimulationEngine::setActiveTool(const Core::ToolDefinition& tool) {
    m_activeTool = tool;
    m_lastEmittedToolId = tool.id;
}

void SimulationEngine::setToolLibrary(const QList<Core::ToolDefinition>& tools) {
    m_toolLibrary = tools;
}

void SimulationEngine::checkToolChange(int segToolId) {
    if (segToolId <= 0 || segToolId == m_lastEmittedToolId) return;

    // Werkzeug aus Library auflösen
    for (const auto& t : m_toolLibrary) {
        if (t.id == segToolId) {
            m_activeTool = t;
            m_lastEmittedToolId = segToolId;
            emit activeToolChanged(t);
            return;
        }
    }
}

void SimulationEngine::setStockBounds(const Core::BoundingBox& stockBounds) {
    m_stockModel = StockModel(stockBounds);
    m_stockModel.isCylinder = false;
}

void SimulationEngine::setCylinderStock(const Core::BoundingBox& stockBounds, double radius) {
    m_stockModel = StockModel(stockBounds);
    m_stockModel.maskCylinder(radius);
}

void SimulationEngine::setMeshStock(const Geometry::Mesh& stockMesh) {
    m_stockModel.initFromMesh(stockMesh);
}

void SimulationEngine::play() {
    if (m_toolpath.empty()) return;

    if (m_state == SimState::Finished || m_state == SimState::HaltedOnCollision) {
        reset();
    }

    m_state = SimState::Running;
    m_lastTickMs = QDateTime::currentMSecsSinceEpoch();
    m_timer->start();
    emit stateChanged(m_state);
}

void SimulationEngine::pause() {
    if (m_state == SimState::Running) {
        m_timer->stop();
        m_state = SimState::Paused;
        emit stateChanged(m_state);
    }
}

void SimulationEngine::stop() {
    m_timer->stop();
    m_state = SimState::Idle;
    emit stateChanged(m_state);
}

void SimulationEngine::reset() {
    m_timer->stop();
    m_currentSegmentIdx = 0;
    m_segmentProgress = 0.0;
    m_stockModel.reset();

    // Fräser immer auf sicherer Höhe über dem Nullpunkt positionieren,
    // nicht am Startpunkt des Toolpaths (der oft weit außerhalb des Werkstücks liegt)
    double safeZ = 10.0;
    if (m_stockModel.initialTopZ > -1e6) {
        safeZ = m_stockModel.initialTopZ + 10.0;  // 10mm über Werkstück-Oberfläche
    }
    m_currentPos = {0.0, 0.0, safeZ, 0.0};

    m_state = SimState::Idle;
    emit positionChanged(m_currentPos);
    emit progressChanged(0.0, 0, m_toolpath.size());
    emit stateChanged(m_state);
    emit stockUpdated();
}

void SimulationEngine::stepForward() {
    if (m_toolpath.empty() || m_currentSegmentIdx >= m_toolpath.size()) return;

    const auto& seg = m_toolpath.segments[m_currentSegmentIdx];
    m_currentPos = seg.endPos;

    // Werkzeugwechsel erkennen und Viewport/Header aktualisieren
    checkToolChange(seg.toolId);

    // Kollision am Segment prüfen
    if (seg.hasCollision) {
        CAM::CollisionViolation v;
        v.segmentIndex = static_cast<int>(m_currentSegmentIdx);
        v.position = m_currentPos;
        v.severity = CAM::CollisionSeverity::Critical;
        v.description = seg.collisionWarning;
        emit collisionDetected(v);
        m_state = SimState::HaltedOnCollision;
        emit stateChanged(m_state);
        return;
    }

    // Materialabtrag im StockModel mit Werkzeugfarbe über gesamtes Segment (lückenlos)
    if (seg.motion != CAM::MotionType::Rapid) {
        QColor toolColor = m_activeTool.color.isEmpty() ? QColor(255, 230, 20) : QColor(m_activeTool.color);
        double carveDia = (seg.toolDiameter > 0.5) ? seg.toolDiameter : m_activeTool.diameter;
        m_stockModel.carveSegment(seg.startPos, seg.endPos, carveDia * 0.5, toolColor, markKindFor(m_activeTool), markPitchFor(seg));
        emit stockUpdated();
    }
    m_lastCarvePos = m_currentPos;

    m_currentSegmentIdx++;
    m_segmentProgress = 0.0;

    double pct = 100.0 * static_cast<double>(m_currentSegmentIdx) / m_toolpath.size();
    emit positionChanged(m_currentPos);
    emit progressChanged(pct, m_currentSegmentIdx, m_toolpath.size());

    if (m_currentSegmentIdx >= m_toolpath.size()) {
        m_state = SimState::Finished;
        emit stateChanged(m_state);
        emit simulationFinished();
    }
}

void SimulationEngine::setSpeedMultiplier(double multiplier) {
    m_speedMultiplier = std::clamp(multiplier, 0.1, 100.0);
}

void SimulationEngine::jumpToSegment(size_t index) {
    if (index >= m_toolpath.size()) return;
    m_currentSegmentIdx = index;
    m_segmentProgress = 0.0;
    m_currentPos = m_toolpath.segments[index].startPos;

    double pct = 100.0 * static_cast<double>(m_currentSegmentIdx) / m_toolpath.size();
    emit positionChanged(m_currentPos);
    emit progressChanged(pct, m_currentSegmentIdx, m_toolpath.size());
}

void SimulationEngine::onTick() {
    if (m_state != SimState::Running) return;

    qint64 nowMs = QDateTime::currentMSecsSinceEpoch();
    double dtSec = (nowMs - m_lastTickMs) / 1000.0;
    m_lastTickMs = nowMs;

    // Begrenze dt gegen extreme Ausreißer
    if (dtSec > 0.1) dtSec = 0.1;

    updateSegmentMovement(dtSec * m_speedMultiplier);
}

void SimulationEngine::updateSegmentMovement(double dtSec) {
    if (m_currentSegmentIdx >= m_toolpath.size()) {
        m_timer->stop();
        m_state = SimState::Finished;
        emit stateChanged(m_state);
        emit simulationFinished();
        return;
    }

    const auto& seg = m_toolpath.segments[m_currentSegmentIdx];

    // Kollisionsprüfung bei Betreten des Segments
    if (seg.hasCollision && m_segmentProgress < 1e-4) {
        m_timer->stop();
        m_state = SimState::HaltedOnCollision;
        emit stateChanged(m_state);

        CAM::CollisionViolation v;
        v.segmentIndex = static_cast<int>(m_currentSegmentIdx);
        v.lineNumber = seg.lineNumber;
        v.position = m_currentPos;
        v.severity = CAM::CollisionSeverity::Critical;
        v.description = seg.collisionWarning;
        emit collisionDetected(v);
        return;
    }

    double segLen = seg.length();
    if (segLen < 1e-6) {
        m_currentSegmentIdx++;
        m_segmentProgress = 0.0;
        return;
    }

    double speedMmSec = (seg.motion == CAM::MotionType::Rapid ? 5000.0 : seg.feedRate) / 60.0;
    double remainingDist = speedMmSec * dtSec;
    bool didCarve = false;

    while (remainingDist > 0.0 && m_currentSegmentIdx < m_toolpath.size()) {
        const auto& curSeg = m_toolpath.segments[m_currentSegmentIdx];

        // Werkzeugwechsel erkennen (aktualisiert m_activeTool, Viewport, Header)
        checkToolChange(curSeg.toolId);
        QColor toolColor = m_activeTool.color.isEmpty() ? QColor(255, 230, 20) : QColor(m_activeTool.color);

        double curLen = curSeg.length();
        if (curLen < 1e-6) {
            m_currentSegmentIdx++;
            m_segmentProgress = 0.0;
            continue;
        }

        double segRemaining = curLen * (1.0 - m_segmentProgress);
        Core::Vector3D prevPos = m_currentPos;
        double carveDia = (curSeg.toolDiameter > 0.5) ? curSeg.toolDiameter : m_activeTool.diameter;

        if (remainingDist >= segRemaining) {
            // Segment vollendet
            m_currentPos = curSeg.endPos;
            if (curSeg.motion != CAM::MotionType::Rapid) {
                m_stockModel.carveSegment(prevPos, m_currentPos, carveDia * 0.5, toolColor, markKindFor(m_activeTool), markPitchFor(curSeg));
                didCarve = true;
            }
            remainingDist -= segRemaining;
            m_currentSegmentIdx++;
            m_segmentProgress = 0.0;
        } else {
            // Innerhalb des Segments weiterfahren
            m_segmentProgress += remainingDist / curLen;
            m_currentPos = Core::Vector3D::lerp(curSeg.startPos, curSeg.endPos, m_segmentProgress);
            if (curSeg.motion != CAM::MotionType::Rapid) {
                m_stockModel.carveSegment(prevPos, m_currentPos, carveDia * 0.5, toolColor, markKindFor(m_activeTool), markPitchFor(curSeg));
                didCarve = true;
            }
            remainingDist = 0.0;
        }
    }

    if (didCarve) {
        emit stockUpdated();
    }

    double pct = 100.0 * (static_cast<double>(m_currentSegmentIdx) + m_segmentProgress) / m_toolpath.size();
    emit positionChanged(m_currentPos);
    emit progressChanged(pct, m_currentSegmentIdx, m_toolpath.size());
}

} // namespace GeminiCNC::Simulation
