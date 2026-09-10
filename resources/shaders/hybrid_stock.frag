#version 330 core

in vec3 v_FragPos;
in vec3 v_Normal;
in float v_TargetZ;

out vec4 FragColor;

// Lighting Uniforms
uniform vec3 uLightDir;
uniform vec3 uViewPos;
uniform vec4 uColor; // Base Material color
uniform float uSpecularIntensity;
uniform float uShininess;

// Render Mode: 0 = Realistic, 1 = Restmaterial Heatmap
uniform int u_RenderMode; 

void main() {
    // ── BLINN-PHONG BELEUCHTUNGSMODELL ──
    vec3 normal = normalize(v_Normal);
    vec3 lightDir = normalize(uLightDir);
    vec3 viewDir = normalize(uViewPos - v_FragPos);
    
    // Ambient
    vec3 ambient = 0.40 * vec3(1.0);
    
    // Diffuse
    float diff = max(dot(normal, lightDir), 0.0);
    vec3 diffuse = 0.60 * diff * vec3(1.0);
    
    // Specular (Blinn-Phong)
    vec3 halfwayDir = normalize(lightDir + viewDir);
    float spec = pow(max(dot(normal, halfwayDir), 0.0), uShininess);
    vec3 specular = uSpecularIntensity * spec * vec3(1.0);
    
    // ── MATERIAL-FARBE (BASE COLOR) ──
    vec3 baseColor = uColor.rgb;

    if (u_RenderMode == 1) {
        // ── MODUS B: HEATMAP (Restmaterial) ──
        float diffZ = v_FragPos.z - v_TargetZ;
        
        if (diffZ > 0.05) {
            // Blau: Schlichtaufmaß steht noch
            baseColor = vec3(0.1, 0.4, 1.0);
        } else if (diffZ < -0.05) {
            // Rot: Kollision / Untermaß (zu tief gefräst)
            baseColor = vec3(1.0, 0.1, 0.1);
        } else {
            // Grün: Maß perfekt erreicht (innerhalb 50µm Toleranz)
            baseColor = vec3(0.1, 0.9, 0.2);
        }
    }
    
    // Final Color Assembly
    vec3 result = (ambient + diffuse) * baseColor + specular;
    FragColor = vec4(result, 1.0);
}
