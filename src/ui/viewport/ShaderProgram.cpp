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

// ═══════════════════════════════════════════════════════════
// Rohteil-Shader: realistisches Metall (PBR), Fräserspuren, Schatten, Restmaterial-Heatmap
// ═══════════════════════════════════════════════════════════

static const char* hybridVertexShader = R"(
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in float aTargetZ;
layout(location = 3) in float aAo;
layout(location = 4) in vec4 aMark;     // u, d, Radius, Vorschub je Umdrehung
layout(location = 5) in vec3 aMarkDir;  // Vorschubrichtung x/y, Spurart (0 = ungefräst)

out vec3 v_FragPos;
out vec3 v_Normal;
out float v_TargetZ;
out float v_Ao;
out vec4 v_Mark;
out vec3 v_MarkDir;
out vec4 v_LightSpacePos;

uniform mat4 uModel;
uniform mat4 uView;
uniform mat4 uProjection;
uniform mat4 uLightSpace;

void main() {
    v_FragPos = vec3(uModel * vec4(aPos, 1.0));
    v_Normal = mat3(transpose(inverse(uModel))) * aNormal;
    v_TargetZ = aTargetZ;
    v_Ao = aAo;
    v_Mark = aMark;
    v_MarkDir = aMarkDir;
    v_LightSpacePos = uLightSpace * vec4(v_FragPos, 1.0);
    gl_Position = uProjection * uView * vec4(v_FragPos, 1.0);
}
)";

static const char* hybridFragmentShader = R"(
#version 330 core
in vec3 v_FragPos;
in vec3 v_Normal;
in float v_TargetZ;
in float v_Ao;
in vec4 v_Mark;
in vec3 v_MarkDir;
in vec4 v_LightSpacePos;

out vec4 FragColor;

// Beleuchtung (Schnell-Modus)
uniform vec3 uLightDir;
uniform vec3 uViewPos;
uniform vec4 uColor;
uniform float uSpecularIntensity;
uniform float uShininess;

// Render Mode: 0 = Realistisch, 1 = Restmaterial-Heatmap
uniform int u_RenderMode;

// Qualität: 0 = Schnell (Blinn-Phong), 1 = Realistisch (PBR + Spuren), 2 = Realistisch + Schatten
uniform int uQuality;
uniform vec3 uCutColor;       // gefräste Fläche (bei Metallen Reflexionsfarbe F0)
uniform float uCutMetallic;
uniform float uCutRoughness;
uniform vec3 uRawColor;       // Rohteiloberfläche (Walzhaut, Sägeschnitt)
uniform float uRawMetallic;
uniform float uRawRoughness;
uniform sampler2DShadow uShadowMap;

const float PI = 3.14159265;

float hash(vec2 p) {
    return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453);
}

float valueNoise(vec2 p) {
    vec2 i = floor(p);
    vec2 f = fract(p);
    vec2 u = f * f * (3.0 - 2.0 * f);
    return mix(mix(hash(i), hash(i + vec2(1.0, 0.0)), u.x),
               mix(hash(i + vec2(0.0, 1.0)), hash(i + vec2(1.0, 1.0)), u.x), u.y);
}

// Prozedurale Studio-Umgebung (Z nach oben): Boden, Horizont, Himmel und Softboxen.
// Rauheit weitet die Lichtquellen auf (grobe Näherung einer vorgefilterten Umgebung).
vec3 environment(vec3 dir, float rough) {
    float up = clamp(dir.z * 0.5 + 0.5, 0.0, 1.0);
    // Hallenboden, helle Hallenwände am Horizont (spiegeln sich in senkrechten Wänden), gedämpfte Decke
    vec3 c = mix(vec3(0.05, 0.052, 0.058), vec3(0.30, 0.31, 0.33), smoothstep(0.25, 0.5, up));
    c = mix(c, vec3(0.22, 0.235, 0.26), smoothstep(0.55, 1.0, up));
    float spread = rough * rough * 0.9;
    float key = smoothstep(0.92 - spread, 0.99, dot(dir, normalize(vec3(0.45, -0.55, 0.70))));
    float fill = smoothstep(0.95 - spread, 0.995, dot(dir, normalize(vec3(-0.75, 0.35, 0.56))));
    float strip = smoothstep(0.985 - spread * 0.5, 1.0, 1.0 - abs(dir.y)) * smoothstep(0.1, 0.5, dir.z);
    float energy = 1.0 / (1.0 + 8.0 * spread);
    return c + (vec3(2.6) * key + vec3(1.0) * fill + vec3(0.6) * strip) * energy;
}

float D_GGX(float NdotH, float a) {
    float a2 = a * a;
    float d = NdotH * NdotH * (a2 - 1.0) + 1.0;
    return a2 / (PI * d * d);
}

float G_Smith(float NdotV, float NdotL, float rough) {
    float k = (rough + 1.0) * (rough + 1.0) / 8.0;
    return (NdotV / (NdotV * (1.0 - k) + k)) * (NdotL / (NdotL * (1.0 - k) + k));
}

vec3 F_Schlick(float cosTheta, vec3 F0) {
    return F0 + (1.0 - F0) * pow(1.0 - cosTheta, 5.0);
}

// Analytische Näherung der vorintegrierten Umgebungs-BRDF (Karis)
vec2 envBRDFApprox(float NdotV, float rough) {
    const vec4 c0 = vec4(-1.0, -0.0275, -0.572, 0.022);
    const vec4 c1 = vec4(1.0, 0.0425, 1.04, -0.04);
    vec4 r = rough * c0 + c1;
    float a004 = min(r.x * r.x, exp2(-9.28 * NdotV)) * r.x + r.y;
    return vec2(-1.04, 1.04) * a004 + r.zw;
}

// Fräserspuren als feine Riefen: Normale kippt entlang der Vorschubrichtung, Rauheit variiert.
// Zu feine Spuren (weit weg) werden ausgeblendet, damit nichts flimmert.
// Fräserspuren: Riefen der Schneide in drei Maßstäben (echter Vorschub je Umdrehung und gröbere Stufen,
// damit die Spuren auch aus Entfernung sichtbar bleiben) und Absätze zwischen zwei Bahnen.
// Zu feine Stufen werden über fwidth ausgeblendet, damit nichts flimmert.
vec3 applyToolMarks(vec3 N, float cut, out float roughOffset, out float albedoShade) {
    roughOffset = 0.0;
    albedoShade = 1.0;
    if (cut < 0.01) return N;

    int kind = int(v_MarkDir.z + 0.5);
    vec2 dir = v_MarkDir.xy;
    float dirLen = length(dir);
    dir = dirLen > 1e-4 ? dir / dirLen : vec2(1.0, 0.0);
    float R = max(v_Mark.z, 0.1);
    float wall = 1.0 - smoothstep(0.35, 0.8, abs(N.z));

    float coord;
    if (kind >= 3) {
        coord = v_Mark.x;                                 // Eintauchen/Bohren: konzentrische Ringe
    } else if (kind == 2) {
        coord = v_Mark.x;                                 // Kugelfräser: Riefen quer zur Bahn
    } else {
        float d = clamp(v_Mark.y, -R, R);
        coord = v_Mark.x - sqrt(max(R * R - d * d, 0.0)) * (1.0 - wall); // Boden: Bogenspur, Wand: gerade Riefen
    }

    float scale = max(v_Mark.w, 0.2);
    float tilt = 0.0;
    float shade = 0.0;
    float weight = 1.0;
    for (int o = 0; o < 3; ++o) {
        float ph = coord / scale;
        float fade = clamp(1.0 - fwidth(ph) * 1.4, 0.0, 1.0);
        tilt += cos(ph * 2.0 * PI) * fade * weight;
        shade += sin(ph * 2.0 * PI) * fade * weight;
        scale *= 4.0;
        weight *= 0.55;
    }

    // Absatz zwischen zwei Bahnen am Rand der letzten Bahn (Boden)
    float edgeCoord = abs(v_Mark.y) / R;
    float edge = smoothstep(0.78, 1.0, edgeCoord) * (1.0 - wall) * (kind <= 2 ? 1.0 : 0.0);
    edge *= clamp(1.0 - fwidth(edgeCoord) * 3.0, 0.0, 1.0);

    vec3 T = vec3(dir, 0.0);
    T -= N * dot(T, N);
    float tLen = length(T);
    T = tLen > 1e-4 ? T / tLen : vec3(0.0);
    vec3 B = vec3(-dir.y, dir.x, 0.0) * (v_Mark.y >= 0.0 ? 1.0 : -1.0);

    N = normalize(N + (T * tilt * 0.14 + B * edge * 0.3) * cut);
    roughOffset = (clamp(shade * 0.5 + 0.5, 0.0, 1.0) * 0.12 + edge * 0.15) * cut;
    albedoShade = 1.0 + (shade * 0.035 - edge * 0.12) * cut;
    return N;
}

float shadowFactor(vec3 N, vec3 L) {
    if (uQuality < 2) return 1.0;
    vec3 proj = v_LightSpacePos.xyz / v_LightSpacePos.w * 0.5 + 0.5;
    if (proj.z > 1.0) return 1.0;
    float bias = max(0.0015 * (1.0 - dot(N, L)), 0.0004);
    vec2 texel = 1.0 / vec2(textureSize(uShadowMap, 0));
    float sum = 0.0;
    for (int x = -1; x <= 1; ++x) {
        for (int y = -1; y <= 1; ++y) {
            sum += texture(uShadowMap, vec3(proj.xy + vec2(float(x), float(y)) * texel, proj.z - bias));
        }
    }
    return sum / 9.0;
}

void main() {
    vec3 N = normalize(v_Normal);
    vec3 L = normalize(uLightDir);
    vec3 V = normalize(uViewPos - v_FragPos);

    // Schnell-Modus und Restmaterial-Heatmap: einfache Blinn-Phong-Beleuchtung
    if (uQuality == 0 || u_RenderMode == 1) {
        float diff = max(dot(N, L), 0.0);
        vec3 halfwayDir = normalize(L + V);
        float spec = pow(max(dot(N, halfwayDir), 0.0), uShininess);
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
        FragColor = vec4((0.40 + 0.60 * diff) * baseColor + uSpecularIntensity * spec * vec3(1.0), 1.0);
        return;
    }

    float cut = clamp(v_MarkDir.z, 0.0, 1.0);
    float roughOffset;
    float markShade;
    N = applyToolMarks(N, cut, roughOffset, markShade);

    // Rohteiloberfläche leicht fleckig (Walzhaut / Sägeschnitt)
    float mottle = valueNoise(v_FragPos.xy * 0.35) * 0.6 + valueNoise(v_FragPos.xy * 1.7) * 0.4;
    vec3 rawAlbedo = uRawColor * (0.85 + 0.3 * mottle);
    float rawRough = clamp(uRawRoughness + (mottle - 0.5) * 0.15, 0.05, 1.0);

    vec3 albedo = mix(rawAlbedo, uCutColor, cut) * markShade;
    float metallic = mix(uRawMetallic, uCutMetallic, cut);
    float rough = clamp(mix(rawRough, uCutRoughness, cut) + roughOffset, 0.04, 1.0);

    vec3 F0 = mix(vec3(0.04), albedo, metallic);
    vec3 diffuseColor = albedo * (1.0 - metallic);
    float NdotV = max(dot(N, V), 1e-4);

    // Direktes Licht: Hauptlicht (mit Schatten), Fülllicht, Kantenlicht
    vec3 lightDirs[3] = vec3[3](L, normalize(vec3(-0.6, -0.35, 0.55)), normalize(vec3(0.0, 0.9, 0.35)));
    float intensities[3] = float[3](1.9, 0.45, 0.35);
    float shadow = shadowFactor(N, L);
    vec3 direct = vec3(0.0);
    for (int i = 0; i < 3; ++i) {
        vec3 Li = lightDirs[i];
        float NdotL = max(dot(N, Li), 0.0);
        if (NdotL <= 0.0) continue;
        vec3 H = normalize(V + Li);
        float NdotH = max(dot(N, H), 0.0);
        vec3 F = F_Schlick(max(dot(H, V), 0.0), F0);
        vec3 specular = D_GGX(NdotH, rough * rough) * G_Smith(NdotV, NdotL, rough) * F / max(4.0 * NdotV * NdotL, 1e-4);
        vec3 kd = (1.0 - F) * (1.0 - metallic);
        float visibility = (i == 0) ? shadow : 1.0;
        direct += (kd * diffuseColor / PI + specular) * intensities[i] * NdotL * visibility;
    }

    // Umgebungslicht: Spiegelung der Studio-Umgebung und diffuses Umgebungslicht, mit Verdeckung
    vec3 R = reflect(-V, N);
    vec2 brdf = envBRDFApprox(NdotV, rough);
    vec3 specEnv = environment(R, rough) * (F0 * brdf.x + brdf.y);
    vec3 diffEnv = environment(N, 1.0) * diffuseColor;
    float ao = clamp(v_Ao, 0.0, 1.0);
    vec3 ambient = (specEnv + diffEnv) * ao * mix(0.55, 1.0, shadow);

    vec3 color = (direct + ambient) * 0.85; // Belichtung

    // Filmisches Tonemapping (ACES-Näherung) und Gammakorrektur
    color = clamp((color * (2.51 * color + 0.03)) / (color * (2.43 * color + 0.59) + 0.14), 0.0, 1.0);
    color = pow(color, vec3(1.0 / 2.2));
    FragColor = vec4(color, 1.0);
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

    // Schattenkarte liegt auf Textureinheit 1
    m_program.bind();
    m_program.setUniformValue("uShadowMap", 1);
    m_program.release();

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
