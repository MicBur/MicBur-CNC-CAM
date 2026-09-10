#include "ShaderProgram.h"

namespace GeminiCNC::UI {

static const char* vertexShaderSource = R"(
#version 330 core
layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec4 inColor;

uniform mat4 uModel;
uniform mat4 uView;
uniform mat4 uProjection;
uniform bool uUseVertexColor;
uniform vec4 uColor;

out vec3 fragNormal;
out vec3 fragPos;
out vec4 fragColorAttr;

void main() {
    vec4 worldPos = uModel * vec4(inPosition, 1.0);
    fragPos = worldPos.xyz;
    mat3 normalMatrix = transpose(inverse(mat3(uModel)));
    fragNormal = normalize(normalMatrix * inNormal);
    fragColorAttr = uUseVertexColor ? inColor : uColor;
    gl_Position = uProjection * uView * worldPos;
}
)";

static const char* fragmentShaderSource = R"(
#version 330 core
in vec3 fragNormal;
in vec3 fragPos;
in vec4 fragColorAttr;

uniform bool uUseLighting;
uniform vec3 uLightDir;
uniform float uSpecularIntensity;
uniform float uShininess;
uniform vec3 uViewPos;

out vec4 fragColor;

void main() {
    if (!uUseLighting) {
        fragColor = fragColorAttr;
        return;
    }

    vec3 norm = normalize(fragNormal);
    vec3 lightDir = normalize(uLightDir);

    float diff = max(dot(norm, lightDir), 0.0);
    vec3 ambient = 0.40 * fragColorAttr.rgb;
    vec3 diffuse = 0.60 * diff * fragColorAttr.rgb;

    vec3 viewDir = normalize(uViewPos - fragPos);
    vec3 halfDir = normalize(lightDir + viewDir);
    float spec = pow(max(dot(norm, halfDir), 0.0), uShininess);
    vec3 specular = uSpecularIntensity * spec * vec3(1.0);

    fragColor = vec4(ambient + diffuse + specular, fragColorAttr.a);
}
)";

ShaderProgram::~ShaderProgram() {
    m_program.removeAllShaders();
}

bool ShaderProgram::init() {
    if (!m_program.addShaderFromSourceCode(QOpenGLShader::Vertex, vertexShaderSource)) {
        return false;
    }
    if (!m_program.addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentShaderSource)) {
        return false;
    }
    if (!m_program.link()) {
        return false;
    }

    m_locModel = m_program.uniformLocation("uModel");
    m_locView = m_program.uniformLocation("uView");
    m_locProjection = m_program.uniformLocation("uProjection");
    m_locColor = m_program.uniformLocation("uColor");
    m_locUseLighting = m_program.uniformLocation("uUseLighting");
    m_locUseVertexColor = m_program.uniformLocation("uUseVertexColor");
    m_locLightDir = m_program.uniformLocation("uLightDir");
    m_locSpecularIntensity = m_program.uniformLocation("uSpecularIntensity");
    m_locShininess = m_program.uniformLocation("uShininess");
    m_locViewPos = m_program.uniformLocation("uViewPos");

    return true;
}

static const char* hybridVertexShader = R"(
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in float aTargetZ;

out vec3 v_FragPos;
out vec3 v_Normal;
out float v_TargetZ;

uniform mat4 uModel;
uniform mat4 uView;
uniform mat4 uProjection;

void main() {
    v_FragPos = vec3(uModel * vec4(aPos, 1.0));
    v_Normal = mat3(transpose(inverse(uModel))) * aNormal;
    v_TargetZ = aTargetZ;
    gl_Position = uProjection * uView * vec4(v_FragPos, 1.0);
}
)";

static const char* hybridFragmentShader = R"(
#version 330 core
in vec3 v_FragPos;
in vec3 v_Normal;
in float v_TargetZ;

out vec4 FragColor;

// Lighting Uniforms
uniform vec3 uLightDir;
uniform vec3 uViewPos;
uniform vec4 uColor;
uniform float uSpecularIntensity;
uniform float uShininess;

// Render Mode: 0 = Realistic, 1 = Restmaterial Heatmap
uniform int u_RenderMode; 

void main() {
    vec3 normal = normalize(v_Normal);
    vec3 lightDir = normalize(uLightDir);
    vec3 viewDir = normalize(uViewPos - v_FragPos);
    
    vec3 ambient = 0.40 * vec3(1.0);
    
    float diff = max(dot(normal, lightDir), 0.0);
    vec3 diffuse = 0.60 * diff * vec3(1.0);
    
    vec3 halfwayDir = normalize(lightDir + viewDir);
    float spec = pow(max(dot(normal, halfwayDir), 0.0), uShininess);
    vec3 specular = uSpecularIntensity * spec * vec3(1.0);
    
    vec3 baseColor = uColor.rgb;

    if (u_RenderMode == 1) {
        float diffZ = v_FragPos.z - v_TargetZ;
        
        if (diffZ > 0.05) {
            baseColor = vec3(0.1, 0.4, 1.0);
        } else if (diffZ < -0.05) {
            baseColor = vec3(1.0, 0.1, 0.1);
        } else {
            baseColor = vec3(0.1, 0.9, 0.2);
        }
    }
    
    vec3 result = (ambient + diffuse) * baseColor + specular;
    FragColor = vec4(result, 1.0);
}
)";

bool ShaderProgram::initHybrid() {
    if (!m_program.addShaderFromSourceCode(QOpenGLShader::Vertex, hybridVertexShader)) return false;
    if (!m_program.addShaderFromSourceCode(QOpenGLShader::Fragment, hybridFragmentShader)) return false;
    if (!m_program.link()) return false;
    
    m_locModel = m_program.uniformLocation("uModel");
    m_locView = m_program.uniformLocation("uView");
    m_locProjection = m_program.uniformLocation("uProjection");
    m_locColor = m_program.uniformLocation("uColor");
    m_locUseLighting = m_program.uniformLocation("uUseLighting");
    m_locUseVertexColor = m_program.uniformLocation("uUseVertexColor");
    m_locLightDir = m_program.uniformLocation("uLightDir");
    m_locSpecularIntensity = m_program.uniformLocation("uSpecularIntensity");
    m_locShininess = m_program.uniformLocation("uShininess");
    m_locViewPos = m_program.uniformLocation("uViewPos");

    return true;
}

void ShaderProgram::bind() {
    m_program.bind();
    setSpecular(0.0f, 32.0f); // Default for backward compatibility
}

void ShaderProgram::release() {
    m_program.release();
}

void ShaderProgram::setMatrices(const QMatrix4x4& model, const QMatrix4x4& view, const QMatrix4x4& projection) {
    m_program.setUniformValue(m_locModel, model);
    m_program.setUniformValue(m_locView, view);
    m_program.setUniformValue(m_locProjection, projection);
}

void ShaderProgram::setColor(const QColor& color) {
    QVector4D c(color.redF(), color.greenF(), color.blueF(), color.alphaF());
    m_program.setUniformValue(m_locColor, c);
}

void ShaderProgram::setUseLighting(bool enable) {
    m_program.setUniformValue(m_locUseLighting, enable);
}

void ShaderProgram::setUseVertexColor(bool enable) {
    m_program.setUniformValue(m_locUseVertexColor, enable);
}

void ShaderProgram::setLightDirection(const QVector3D& dir) {
    m_program.setUniformValue(m_locLightDir, dir.normalized());
}

void ShaderProgram::setSpecular(float intensity, float shininess) {
    m_program.setUniformValue(m_locSpecularIntensity, intensity);
    m_program.setUniformValue(m_locShininess, shininess);
}

void ShaderProgram::setViewPos(const QVector3D& pos) {
    m_program.setUniformValue(m_locViewPos, pos);
}

} // namespace GeminiCNC::UI
