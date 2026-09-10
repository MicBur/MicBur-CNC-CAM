#ifndef GEMINI_CNC_VIEWPORT3D_H
#define GEMINI_CNC_VIEWPORT3D_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QMatrix4x4>
#include <QMouseEvent>
#include <QWheelEvent>
#include <memory>

#include "ShaderProgram.h"
#include "core/Vector3D.h"
#include "core/ToolDefinition.h"
#include "geometry/Mesh.h"
#include "geometry/Contour.h"
#include "cam/Toolpath.h"
#include "simulation/StockModel.h"
#include "GPUStockModel.h"

namespace GeminiCNC::UI {

/**
 * @brief Hochperformantes 3D-QOpenGLWidget zur Visualisierung von Rohteil, Bauteil, Fräsbahnen und Werkzeug.
 * Speziell optimiert für flüssiges Rendering (60 FPS) auf Intel HD / mobilen i5 Systemen.
 */
class Viewport3D : public QOpenGLWidget, protected QOpenGLFunctions {
    Q_OBJECT
public:
    explicit Viewport3D(QWidget* parent = nullptr);
    ~Viewport3D() override;

    void setStockMesh(const Geometry::Mesh& mesh);
    void setTargetPartMesh(const Geometry::Mesh& mesh);
    void setToolpath(const CAM::Toolpath& toolpath);
    void setToolPosition(const Core::Vector3D& pos);
    void setActiveTool(const Core::ToolDefinition& tool);
    void updateDynamicStock(const Simulation::StockModel& stockModel);

    void resetCamera();
    void fitToView();

    // Kamera-Ansichts-Presets (wie Hurco Max5)
    void setViewIsometric(); // 3D-Isometrie
    void setViewTop();       // XY-Draufsicht
    void setViewFront();     // XZ-Vorderansicht
    void setViewSide();      // YZ-Seitenansicht

    void setShowStock(bool show) { m_showStock = show; update(); }
    void setShowPart(bool show) { m_showPart = show; update(); }
    void setShowToolpath(bool show) { m_showToolpath = show; update(); }
    void setShowGrid(bool show) { m_showGrid = show; update(); }
    void setMaterialPreset(int preset) { m_materialPreset = preset; update(); }

    // Interaktive DXF-Konturauswahl (Picking)
    void setSelectableContours(const std::vector<Geometry::Contour>& contours);
    void setPickingEnabled(bool enable) { m_pickingEnabled = enable; update(); }
    [[nodiscard]] bool isPickingEnabled() const { return m_pickingEnabled; }
    void setSelectedContourIndex(int index) { m_selectedContourIndex = index; update(); }
    [[nodiscard]] int selectedContourIndex() const { return m_selectedContourIndex; }

signals:
    void contourPicked(int index, const Geometry::Contour& contour);

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;

private:
    void renderGrid();
    void renderMesh(const Geometry::Mesh& mesh, const QColor& color, bool useVertexColors = false, bool wireframe = false);
    void renderMesh(const Geometry::Mesh& mesh, const QColor& color, float specularIntensity, float shininess, bool useVertexColors = false, bool wireframe = false);
    void renderToolpath();
    void renderTool();
    void renderAxes();
    void renderContours();

    std::unique_ptr<ShaderProgram> m_shader;

    // Daten
    Geometry::Mesh m_stockMesh;
    Geometry::Mesh m_partMesh;
    std::unique_ptr<GPUStockModel> m_gpuStockModel;
    std::unique_ptr<ShaderProgram> m_hybridShader;
    bool m_useDynamicStock{false};
    int m_renderMode{0}; // 0 = Realistic, 1 = Heatmap
    
    int m_materialPreset{1}; // 1=Alu default

    std::vector<Geometry::Contour> m_selectableContours;
    int m_selectedContourIndex{-1};
    bool m_pickingEnabled{false};

    CAM::Toolpath m_toolpath;
    Core::Vector3D m_toolPos{0.0, 0.0, 20.0, 0.0};
    Core::ToolDefinition m_activeTool;

    // Sichtbarkeit
    bool m_showStock{true};
    bool m_showPart{true};
    bool m_showToolpath{true};
    bool m_showGrid{true};

    // Kamera & Transformationen
    QMatrix4x4 m_viewMatrix;
    QMatrix4x4 m_projectionMatrix;

    float m_cameraPitch{30.0f};    // Blick von schräg oben (CNC-Konvention)
    float m_cameraYaw{-45.0f};     // ISO-Ansicht: X0/Y0 vorne-rechts (unten-rechts im Bild)
    float m_cameraDistance{180.0f}; // Abstand
    QVector3D m_cameraTarget{0.0f, 0.0f, 0.0f};

    QPoint m_lastMousePos;
    bool m_isRotating{false};
    bool m_isPanning{false};
};

} // namespace GeminiCNC::UI

#endif // GEMINI_CNC_VIEWPORT3D_H
