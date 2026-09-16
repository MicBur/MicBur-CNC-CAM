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

    // Baut die Rohteil-Oberfläche aus dem CPU StockModel komplett auf
    void initFromCPU(const Simulation::StockModel& cpuModel);

    // Aktualisiert das GPU Modell (Schichten/Durchbrüche können die Topologie ändern → Neuaufbau)
    void updateFromCPU(const Simulation::StockModel& cpuModel);

    // Rendert die Oberfläche
    void render(QOpenGLShaderProgram* shader, int renderMode);

    bool isInitialized() const { return m_initialized; }

private:
    void upload(const Simulation::StockModel& cpuModel);

    bool m_initialized{false};
    bool m_attribsSet{false};

    std::vector<GridVertex> m_vertices;

    QOpenGLVertexArrayObject m_vao;
    QOpenGLBuffer m_vbo;
    QOpenGLBuffer m_ibo;
    size_t m_indexCount{0};
};

} // namespace GeminiCNC::UI
#endif
