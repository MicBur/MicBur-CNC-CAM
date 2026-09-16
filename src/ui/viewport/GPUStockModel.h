#ifndef GEMINI_CNC_GPU_STOCK_MODEL_H
#define GEMINI_CNC_GPU_STOCK_MODEL_H

#include <vector>
#include <memory>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLShaderProgram>
#include <QMatrix4x4>
#include <QVector3D>
#include "core/Vector3D.h"
#include "core/BoundingBox.h"
#include "cam/Toolpath.h"
#include "simulation/StockModel.h"

namespace GeminiCNC::UI {

struct GridVertex {
    float x, y, z;
    float nx, ny, nz;
    float targetZ;
    float ao;                                    // Umgebungsverdeckung
    float markU, markD, markRadius, markPitch;   // Fräserspur (siehe StockModel::ToolMark)
    float dirX, dirY, kind;                      // Vorschubrichtung, Spurart (0 = ungefräst)
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

    // ── Schattenwurf des Hauptlichts auf das Werkstück (Shadow Mapping) ──
    bool initShadowMap(int size);
    [[nodiscard]] bool hasShadowMap() const { return m_shadowTexture != 0; }
    [[nodiscard]] QMatrix4x4 lightSpaceMatrix(const QVector3D& lightDir) const;
    void renderShadowPass(const QMatrix4x4& lightSpace, GLuint targetFramebuffer);
    void bindShadowTexture(int unit);

private:
    void upload(const Simulation::StockModel& cpuModel);

    bool m_initialized{false};
    bool m_attribsSet{false};

    std::vector<GridVertex> m_vertices;
    Core::BoundingBox m_bounds;

    QOpenGLVertexArrayObject m_vao;
    QOpenGLBuffer m_vbo;
    QOpenGLBuffer m_ibo;
    size_t m_indexCount{0};

    GLuint m_shadowFbo{0};
    GLuint m_shadowTexture{0};
    int m_shadowSize{0};
    std::unique_ptr<QOpenGLShaderProgram> m_depthProgram;
};

} // namespace GeminiCNC::UI
#endif
