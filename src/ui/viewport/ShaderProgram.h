#ifndef GEMINI_CNC_SHADERPROGRAM_H
#define GEMINI_CNC_SHADERPROGRAM_H

#include <QOpenGLShaderProgram>
#include <QMatrix4x4>
#include <QColor>

namespace GeminiCNC::UI {

/**
 * @brief Kapselt Shader für diffuses Phong-Shading und einfache Farblinien.
 */
class ShaderProgram {
public:
    ShaderProgram() = default;
    ~ShaderProgram();

    bool init();
    bool initHybrid();
    void bind();
    void release();

    void setMatrices(const QMatrix4x4& model, const QMatrix4x4& view, const QMatrix4x4& projection);
    void setColor(const QColor& color);
    void setUseLighting(bool enable);
    void setUseVertexColor(bool enable);
    void setLightDirection(const QVector3D& dir);
    
    void setSpecular(float intensity, float shininess);
    void setViewPos(const QVector3D& pos);

    [[nodiscard]] QOpenGLShaderProgram& program() { return m_program; }

private:
    QOpenGLShaderProgram m_program;
    int m_locModel{-1};
    int m_locView{-1};
    int m_locProjection{-1};
    int m_locColor{-1};
    int m_locUseLighting{-1};
    int m_locUseVertexColor{-1};
    int m_locLightDir{-1};
    int m_locSpecularIntensity{-1};
    int m_locShininess{-1};
    int m_locViewPos{-1};
};

} // namespace GeminiCNC::UI

#endif // GEMINI_CNC_SHADERPROGRAM_H
