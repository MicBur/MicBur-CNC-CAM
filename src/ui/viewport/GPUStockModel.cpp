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
    m_ibo.setUsagePattern(QOpenGLBuffer::StaticDraw);

    m_initialized = true;
}

void GPUStockModel::initFromCPU(const Simulation::StockModel& cpuModel) {
    if (!m_initialized) return;

    m_resX = cpuModel.resX;
    m_resY = cpuModel.resY;

    size_t vertexCount = m_resX * m_resY;
    m_grid.resize(vertexCount);

    double cellSizeX = (cpuModel.bounds.maxPoint.x - cpuModel.bounds.minPoint.x) / (m_resX - 1);
    double cellSizeY = (cpuModel.bounds.maxPoint.y - cpuModel.bounds.minPoint.y) / (m_resY - 1);

    // Init vertices
    for (int j = 0; j < m_resY; ++j) {
        float py = cpuModel.bounds.minPoint.y + j * cellSizeY;
        for (int i = 0; i < m_resX; ++i) {
            float px = cpuModel.bounds.minPoint.x + i * cellSizeX;
            int idx = j * m_resX + i;
            
            m_grid[idx].x = px;
            m_grid[idx].y = py;
            m_grid[idx].z = cpuModel.heightField[idx];
            m_grid[idx].targetZ = cpuModel.initialTopZ; // Default
            
            // Default Normal
            m_grid[idx].nx = 0.0f;
            m_grid[idx].ny = 0.0f;
            m_grid[idx].nz = 1.0f;
        }
    }

    rebuildNormalsAndTargetZ(cpuModel);

    // Create Indices
    m_indices.clear();
    for (int j = 0; j < m_resY - 1; ++j) {
        for (int i = 0; i < m_resX - 1; ++i) {
            uint32_t i0 = j * m_resX + i;
            uint32_t i1 = i0 + 1;
            uint32_t i2 = (j + 1) * m_resX + i;
            uint32_t i3 = i2 + 1;

            m_indices.push_back(i0);
            m_indices.push_back(i1);
            m_indices.push_back(i2);

            m_indices.push_back(i2);
            m_indices.push_back(i1);
            m_indices.push_back(i3);
        }
    }
    m_indexCount = m_indices.size();

    // Upload to GPU
    m_vao.bind();
    m_vbo.bind();
    m_vbo.allocate(m_grid.data(), m_grid.size() * sizeof(GridVertex));

    // Attributes
    glEnableVertexAttribArray(0); // aPos
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(GridVertex), (void*)offsetof(GridVertex, x));

    glEnableVertexAttribArray(1); // aNormal
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(GridVertex), (void*)offsetof(GridVertex, nx));

    glEnableVertexAttribArray(2); // aTargetZ
    glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, sizeof(GridVertex), (void*)offsetof(GridVertex, targetZ));

    m_ibo.bind();
    m_ibo.allocate(m_indices.data(), m_indices.size() * sizeof(uint32_t));

    m_vao.release();
}

void GPUStockModel::updateFromCPU(const Simulation::StockModel& cpuModel) {
    if (!m_initialized || m_grid.empty()) return;

    // Fast copy of Z values
    bool changed = false;
    for (size_t i = 0; i < m_grid.size(); ++i) {
        if (m_grid[i].z != cpuModel.heightField[i]) {
            m_grid[i].z = cpuModel.heightField[i];
            changed = true;
        }
    }

    if (changed) {
        rebuildNormalsAndTargetZ(cpuModel);
        
        m_vbo.bind();
        m_vbo.write(0, m_grid.data(), m_grid.size() * sizeof(GridVertex));
        m_vbo.release();
    }
}

void GPUStockModel::rebuildNormalsAndTargetZ(const Simulation::StockModel& cpuModel) {
    // Einfache Central-Difference für Normals
    double cellSizeX = (cpuModel.bounds.maxPoint.x - cpuModel.bounds.minPoint.x) / (m_resX - 1);
    double cellSizeY = (cpuModel.bounds.maxPoint.y - cpuModel.bounds.minPoint.y) / (m_resY - 1);

    for (int j = 1; j < m_resY - 1; ++j) {
        for (int i = 1; i < m_resX - 1; ++i) {
            int idx = j * m_resX + i;
            
            float dzdx = (m_grid[j * m_resX + i + 1].z - m_grid[j * m_resX + i - 1].z) / (2.0f * cellSizeX);
            float dzdy = (m_grid[(j + 1) * m_resX + i].z - m_grid[(j - 1) * m_resX + i].z) / (2.0f * cellSizeY);

            // Normal = cross( tangentX, tangentY )
            // tangentX = (1, 0, dzdx), tangentY = (0, 1, dzdy)
            // Normal = (-dzdx, -dzdy, 1)
            float nx = -dzdx;
            float ny = -dzdy;
            float nz = 1.0f;
            float len = std::sqrt(nx*nx + ny*ny + nz*nz);

            m_grid[idx].nx = nx / len;
            m_grid[idx].ny = ny / len;
            m_grid[idx].nz = nz / len;
            
            // Set target Z for heatmap (0 is usually initialTopZ, we can improve this later)
            m_grid[idx].targetZ = -1.0f; // Placeholder, you could pass the actual target Z from toolpath here
        }
    }
}

void GPUStockModel::render(QOpenGLShaderProgram* shader, int renderMode) {
    if (!m_initialized || m_indexCount == 0) return;

    shader->bind();
    shader->setUniformValue("u_RenderMode", renderMode);

    m_vao.bind();
    glDrawElements(GL_TRIANGLES, m_indexCount, GL_UNSIGNED_INT, nullptr);
    m_vao.release();
}

} // namespace GeminiCNC::UI
