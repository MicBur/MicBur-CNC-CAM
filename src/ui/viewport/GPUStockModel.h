#ifndef GEMINI_CNC_GPU_STOCK_MODEL_H
#define GEMINI_CNC_GPU_STOCK_MODEL_H

#include <vector>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLShaderProgram>
#include "core/Vector3D.h"
#include "core/BoundingBox.h"
#include "cam/Toolpath.h"
#include "simulation/StockModel.h"

namespace GeminiCNC::UI {

struct GridVertex {
    float x, y, z;
    float nx, ny, nz;
    float targetZ;
};

class GPUStockModel : protected QOpenGLFunctions_3_3_Core {
public:
    GPUStockModel();
    ~GPUStockModel();

    void initializeGL();
    
    // Kopiert die initialen Höhen aus dem CPU StockModel
    void initFromCPU(const Simulation::StockModel& cpuModel);

    // Aktualisiert das GPU Modell basierend auf dem geänderten CPU Modell
    void updateFromCPU(const Simulation::StockModel& cpuModel);

    // Rendert das Grid
    void render(QOpenGLShaderProgram* shader, int renderMode);

    bool isInitialized() const { return m_initialized; }

private:
    void rebuildNormalsAndTargetZ(const Simulation::StockModel& cpuModel);

    bool m_initialized{false};
    int m_resX{0}, m_resY{0};

    std::vector<GridVertex> m_grid;
    std::vector<uint32_t> m_indices;

    QOpenGLVertexArrayObject m_vao;
    QOpenGLBuffer m_vbo;
    QOpenGLBuffer m_ibo;
    size_t m_indexCount{0};
};

} // namespace GeminiCNC::UI
#endif
