#include "GPUStockModel.h"

namespace GeminiCNC::UI {

GPUStockModel::GPUStockModel()
    : m_vbo(QOpenGLBuffer::VertexBuffer), m_ibo(QOpenGLBuffer::IndexBuffer) {
}

GPUStockModel::~GPUStockModel() {
    if (m_initialized) {
        m_vbo.destroy();
        m_ibo.destroy();
        m_vao.destroy();
    }
}

void GPUStockModel::initializeGL() {
    initializeOpenGLFunctions();

    m_vao.create();
    m_vbo.create();
    m_vbo.setUsagePattern(QOpenGLBuffer::DynamicDraw);

    m_ibo.create();
    m_ibo.setUsagePattern(QOpenGLBuffer::DynamicDraw); // Durchbrüche/Schichten ändern die Topologie

    m_initialized = true;
}

void GPUStockModel::initFromCPU(const Simulation::StockModel& cpuModel) {
    upload(cpuModel);
}

void GPUStockModel::updateFromCPU(const Simulation::StockModel& cpuModel) {
    upload(cpuModel);
}

void GPUStockModel::upload(const Simulation::StockModel& cpuModel) {
    if (!m_initialized) return;

    // Gemeinsame Oberflächenerzeugung mit dem STL-Export → Anzeige und Export sind identisch
    const auto surface = cpuModel.buildSurface();

    m_vertices.resize(surface.vertices.size());
    for (size_t i = 0; i < surface.vertices.size(); ++i) {
        const auto& v = surface.vertices[i];
        // targetZ: Platzhalter für die Restmaterial-Heatmap
        m_vertices[i] = {v.x, v.y, v.z, v.nx, v.ny, v.nz, -1.0f};
    }

    m_vao.bind();
    m_vbo.bind();
    m_vbo.allocate(m_vertices.data(), static_cast<int>(m_vertices.size() * sizeof(GridVertex)));

    if (!m_attribsSet) {
        glEnableVertexAttribArray(0); // aPos
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(GridVertex), (void*)offsetof(GridVertex, x));

        glEnableVertexAttribArray(1); // aNormal
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(GridVertex), (void*)offsetof(GridVertex, nx));

        glEnableVertexAttribArray(2); // aTargetZ
        glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, sizeof(GridVertex), (void*)offsetof(GridVertex, targetZ));
        m_attribsSet = true;
    }

    m_ibo.bind();
    m_ibo.allocate(surface.indices.data(), static_cast<int>(surface.indices.size() * sizeof(uint32_t)));
    m_vao.release();

    // Puffer wieder lösen: der Upload läuft innerhalb von paintGL, danach zeichnen Werkzeug und
    // Fräsbahnen mit Client-Arrays – ein gebundener Puffer würde deren Zeiger als Offset deuten
    m_vbo.release();
    m_ibo.release();

    m_indexCount = surface.indices.size();
}

void GPUStockModel::render(QOpenGLShaderProgram* shader, int renderMode) {
    if (!m_initialized || m_indexCount == 0) return;

    shader->bind();
    shader->setUniformValue("u_RenderMode", renderMode);

    m_vao.bind();
    glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(m_indexCount), GL_UNSIGNED_INT, nullptr);
    m_vao.release();
}

} // namespace GeminiCNC::UI
