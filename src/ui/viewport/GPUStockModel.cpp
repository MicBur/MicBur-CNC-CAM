#include "GPUStockModel.h"
#include <algorithm>
#include <cmath>

namespace GeminiCNC::UI {

namespace {

const char* kDepthVertexShader = R"(
#version 330 core
layout(location = 0) in vec3 aPos;
uniform mat4 uLightSpace;
void main() {
    gl_Position = uLightSpace * vec4(aPos, 1.0);
}
)";

const char* kDepthFragmentShader = R"(
#version 330 core
void main() {
}
)";

} // namespace

GPUStockModel::GPUStockModel()
    : m_vbo(QOpenGLBuffer::VertexBuffer), m_ibo(QOpenGLBuffer::IndexBuffer) {
}

GPUStockModel::~GPUStockModel() {
    if (m_initialized) {
        if (m_shadowTexture) glDeleteTextures(1, &m_shadowTexture);
        if (m_shadowFbo) glDeleteFramebuffers(1, &m_shadowFbo);
        m_depthProgram.reset();
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
    m_bounds = cpuModel.bounds;

    m_vertices.resize(surface.vertices.size());
    for (size_t i = 0; i < surface.vertices.size(); ++i) {
        const auto& v = surface.vertices[i];
        const Simulation::StockModel::VertexShading sh = (i < surface.shading.size())
            ? surface.shading[i] : Simulation::StockModel::VertexShading{};
        // targetZ: Platzhalter für die Restmaterial-Heatmap
        m_vertices[i] = {v.x, v.y, v.z, v.nx, v.ny, v.nz, -1.0f,
                         sh.ao, sh.markU, sh.markD, sh.markRadius, sh.markPitch,
                         sh.dirX, sh.dirY, sh.kind};
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

        glEnableVertexAttribArray(3); // aAo
        glVertexAttribPointer(3, 1, GL_FLOAT, GL_FALSE, sizeof(GridVertex), (void*)offsetof(GridVertex, ao));

        glEnableVertexAttribArray(4); // aMark (u, d, Radius, Vorschub/U)
        glVertexAttribPointer(4, 4, GL_FLOAT, GL_FALSE, sizeof(GridVertex), (void*)offsetof(GridVertex, markU));

        glEnableVertexAttribArray(5); // aMarkDir (Richtung x/y, Spurart)
        glVertexAttribPointer(5, 3, GL_FLOAT, GL_FALSE, sizeof(GridVertex), (void*)offsetof(GridVertex, dirX));
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

bool GPUStockModel::initShadowMap(int size) {
    if (!m_initialized) return false;
    if (m_shadowTexture && m_shadowSize == size) return true;

    if (!m_depthProgram) {
        m_depthProgram = std::make_unique<QOpenGLShaderProgram>();
        if (!m_depthProgram->addShaderFromSourceCode(QOpenGLShader::Vertex, kDepthVertexShader)
            || !m_depthProgram->addShaderFromSourceCode(QOpenGLShader::Fragment, kDepthFragmentShader)
            || !m_depthProgram->link()) {
            m_depthProgram.reset();
            return false;
        }
    }

    if (!m_shadowTexture) glGenTextures(1, &m_shadowTexture);
    glBindTexture(GL_TEXTURE_2D, m_shadowTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, size, size, 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    const GLfloat border[] = {1.0f, 1.0f, 1.0f, 1.0f};
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, border);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
    glBindTexture(GL_TEXTURE_2D, 0);

    if (!m_shadowFbo) glGenFramebuffers(1, &m_shadowFbo);
    GLint previousFbo = 0;
    glGetIntegerv(GL_FRAMEBUFFER_BINDING, &previousFbo);
    glBindFramebuffer(GL_FRAMEBUFFER, m_shadowFbo);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, m_shadowTexture, 0);
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);
    const bool complete = glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
    if (complete) glClear(GL_DEPTH_BUFFER_BIT); // noch kein Schatten
    glBindFramebuffer(GL_FRAMEBUFFER, static_cast<GLuint>(previousFbo));

    if (!complete) {
        glDeleteTextures(1, &m_shadowTexture);
        glDeleteFramebuffers(1, &m_shadowFbo);
        m_shadowTexture = 0;
        m_shadowFbo = 0;
        m_shadowSize = 0;
        return false;
    }
    m_shadowSize = size;
    return true;
}

QMatrix4x4 GPUStockModel::lightSpaceMatrix(const QVector3D& lightDir) const {
    QMatrix4x4 result;
    if (!m_bounds.isValid()) return result;

    const auto c = m_bounds.center();
    const QVector3D center(static_cast<float>(c.x), static_cast<float>(c.y), static_cast<float>(c.z));
    const float radius = 0.5f * static_cast<float>(std::sqrt(m_bounds.widthX() * m_bounds.widthX()
                                                             + m_bounds.depthY() * m_bounds.depthY()
                                                             + m_bounds.heightZ() * m_bounds.heightZ())) + 5.0f;
    const QVector3D dir = lightDir.normalized();

    QMatrix4x4 view;
    view.lookAt(center + dir * radius * 2.0f, center,
                std::abs(dir.z()) > 0.95f ? QVector3D(0.0f, 1.0f, 0.0f) : QVector3D(0.0f, 0.0f, 1.0f));
    QMatrix4x4 projection;
    projection.ortho(-radius, radius, -radius, radius, 0.1f, radius * 4.0f);
    return projection * view;
}

void GPUStockModel::renderShadowPass(const QMatrix4x4& lightSpace, GLuint targetFramebuffer) {
    if (!m_initialized || !m_shadowTexture || !m_depthProgram || m_indexCount == 0) return;

    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);

    glBindFramebuffer(GL_FRAMEBUFFER, m_shadowFbo);
    glViewport(0, 0, m_shadowSize, m_shadowSize);
    glClear(GL_DEPTH_BUFFER_BIT);
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(2.0f, 4.0f); // gegen Schattenakne

    m_depthProgram->bind();
    m_depthProgram->setUniformValue("uLightSpace", lightSpace);
    m_vao.bind();
    glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(m_indexCount), GL_UNSIGNED_INT, nullptr);
    m_vao.release();
    m_depthProgram->release();

    glDisable(GL_POLYGON_OFFSET_FILL);
    glBindFramebuffer(GL_FRAMEBUFFER, targetFramebuffer);
    glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
}

void GPUStockModel::bindShadowTexture(int unit) {
    if (!m_initialized || !m_shadowTexture) return;
    glActiveTexture(GL_TEXTURE0 + unit);
    glBindTexture(GL_TEXTURE_2D, m_shadowTexture);
    glActiveTexture(GL_TEXTURE0);
}

} // namespace GeminiCNC::UI
