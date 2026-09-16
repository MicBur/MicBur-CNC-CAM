#include "Viewport3D.h"
#include <cmath>
#include <algorithm>

namespace GeminiCNC::UI {

namespace {

// Physikalische Materialwerte: gefräste Fläche und Rohteiloberfläche
struct MaterialLook {
    QVector3D cutColor;
    float cutMetallic;
    float cutRoughness;
    QVector3D rawColor;
    float rawMetallic;
    float rawRoughness;
};

MaterialLook materialLook(int preset) {
    switch (preset) {
        case 1:  return {{0.95f, 0.80f, 0.45f}, 1.0f, 0.25f, {0.72f, 0.58f, 0.32f}, 1.0f, 0.50f};  // Messing
        case 2:  return {{0.62f, 0.63f, 0.64f}, 1.0f, 0.28f, {0.22f, 0.23f, 0.25f}, 0.45f, 0.70f}; // Stahl mit Walzhaut
        case 3:  return {{0.55f, 0.38f, 0.22f}, 0.0f, 0.75f, {0.47f, 0.32f, 0.19f}, 0.0f, 0.85f};  // Holz
        case 4:  return {{0.92f, 0.92f, 0.90f}, 0.0f, 0.45f, {0.88f, 0.88f, 0.86f}, 0.0f, 0.55f};  // POM
        case 5:  return {{0.72f, 0.70f, 0.67f}, 1.0f, 0.20f, {0.55f, 0.55f, 0.56f}, 1.0f, 0.42f};  // Edelstahl
        default: return {{0.91f, 0.92f, 0.92f}, 1.0f, 0.22f, {0.76f, 0.77f, 0.78f}, 1.0f, 0.45f};  // Aluminium
    }
}

} // namespace

Viewport3D::Viewport3D(QWidget* parent) : QOpenGLWidget(parent) {
    setFocusPolicy(Qt::StrongFocus);
    m_activeTool = Core::ToolDefinition(1, QStringLiteral("6mm Fräser"), Core::ToolType::EndMill, 6.0);
}

Viewport3D::~Viewport3D() {
    makeCurrent();
    m_gpuStockModel.reset(); // GL-Objekte bei aktivem Kontext freigeben
    m_hybridShader.reset();
    m_shader.reset();
    doneCurrent();
}

void Viewport3D::setStockMesh(const Geometry::Mesh& mesh) {
    m_stockMesh = mesh;
    m_useDynamicStock = false;
    fitToView();
    update();
}

void Viewport3D::setTargetPartMesh(const Geometry::Mesh& mesh) {
    m_partMesh = mesh;
    fitToView();
    update();
}

void Viewport3D::setToolpath(const CAM::Toolpath& toolpath) {
    m_toolpath = toolpath;
    update();
}

void Viewport3D::setToolPosition(const Core::Vector3D& pos) {
    m_toolPos = pos;
    update();
}

void Viewport3D::setActiveTool(const Core::ToolDefinition& tool) {
    m_activeTool = tool;
    update();
}

void Viewport3D::setSelectableContours(const std::vector<Geometry::Contour>& contours) {
    m_selectableContours = contours;
    m_selectedContourIndex = -1;
    update();
}

void Viewport3D::updateDynamicStock(const Simulation::StockModel& stockModel) {
    // Oberfläche erst beim nächsten Zeichnen aufbauen: mehrere Abtrag-Ticks pro Bild
    // werden so zu einem einzigen GPU-Upload zusammengefasst
    m_dynamicStockSource = &stockModel;
    m_dynamicStockDirty = true;
    m_useDynamicStock = true;
    update();
}

void Viewport3D::resetCamera() {
    setViewIsometric();
}

void Viewport3D::setViewIsometric() {
    m_cameraPitch = 35.0f;    // 35° Isometrisch
    m_cameraYaw = -45.0f;     // X0/Y0 vorne-rechts (unten-rechts)
    update();
}

void Viewport3D::setViewTop() {
    m_cameraPitch = 90.0f;    // Exakte Draufsicht (XY-Ebene)
    m_cameraYaw = 0.0f;
    update();
}

void Viewport3D::setViewFront() {
    m_cameraPitch = 0.0f;     // Blick horizontal von vorne (XZ-Ebene)
    m_cameraYaw = -90.0f;
    update();
}

void Viewport3D::setViewSide() {
    m_cameraPitch = 0.0f;     // Blick horizontal von der Seite (YZ-Ebene)
    m_cameraYaw = 0.0f;
    update();
}

void Viewport3D::fitToView() {
    Core::BoundingBox totalBox;
    if (m_stockMesh.boundingBox.isValid()) totalBox.expand(m_stockMesh.boundingBox);
    if (m_partMesh.boundingBox.isValid()) totalBox.expand(m_partMesh.boundingBox);

    if (totalBox.isValid()) {
        auto c = totalBox.center();
        // CNC-Konvention: Kameraziel auf XY-Mitte + Z=0 (Werkstück-Oberkante)
        // statt auf volumetrischen BBox-Mittelpunkt, damit der Nullpunkt
        // visuell auf der Werkstückoberfläche liegt.
        float targetZ = static_cast<float>(totalBox.maxPoint.z);
        m_cameraTarget = QVector3D(static_cast<float>(c.x), static_cast<float>(c.y), targetZ);
        double maxDim = std::max({totalBox.widthX(), totalBox.depthY(), totalBox.heightZ()});
        m_cameraDistance = static_cast<float>(std::max(80.0, maxDim * 2.2));
    } else {
        resetCamera();
    }
}

void Viewport3D::initializeGL() {
    initializeOpenGLFunctions();

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glClearColor(0.02f, 0.03f, 0.05f, 1.0f); // Tiefes Obsidian-Schwarz nach Max5 Vorbild (Screenshot 192612)

    m_shader = std::make_unique<ShaderProgram>();
    m_shader->init();
    
    m_hybridShader = std::make_unique<ShaderProgram>();
    m_hybridShader->initHybrid();
}

void Viewport3D::resizeGL(int w, int h) {
    if (h == 0) h = 1;
    glViewport(0, 0, w, h);

    m_projectionMatrix.setToIdentity();
    m_projectionMatrix.perspective(45.0f, static_cast<float>(w) / static_cast<float>(h), 0.5f, 5000.0f);
}

void Viewport3D::paintGL() {
    // Dynamisches Rohteil vor dem Zeichnen aktualisieren (auch für den Schattendurchlauf)
    if (m_showStock && m_useDynamicStock && m_dynamicStockDirty && m_dynamicStockSource) {
        if (!m_gpuStockModel) m_gpuStockModel = std::make_unique<GPUStockModel>();
        if (!m_gpuStockModel->isInitialized()) m_gpuStockModel->initializeGL();
        m_gpuStockModel->updateFromCPU(*m_dynamicStockSource);
        m_dynamicStockDirty = false;
        m_shadowDirty = true;
    }

    // Schattenkarte des Hauptlichts nur neu zeichnen, wenn sich das Werkstück geändert hat
    const QVector3D keyLightDir = QVector3D(0.4f, 0.6f, 1.0f).normalized();
    if (m_renderQuality >= 2 && m_showStock && m_useDynamicStock && m_gpuStockModel && m_gpuStockModel->isInitialized()) {
        if (m_gpuStockModel->initShadowMap(2048) && m_shadowDirty) {
            m_gpuStockModel->renderShadowPass(m_gpuStockModel->lightSpaceMatrix(keyLightDir), defaultFramebufferObject());
            m_shadowDirty = false;
        }
    }

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // View-Matrix aufbauen (CNC-Konvention: Z nach oben, Blick von schräg oben)
    m_viewMatrix.setToIdentity();
    m_viewMatrix.translate(0.0f, 0.0f, -m_cameraDistance);
    m_viewMatrix.rotate(m_cameraPitch - 90.0f, 1.0f, 0.0f, 0.0f); // -90 = Draufsicht, + Pitch kippt nach schräg
    m_viewMatrix.rotate(m_cameraYaw, 0.0f, 0.0f, 1.0f);
    m_viewMatrix.translate(-m_cameraTarget);

    if (!m_shader) return;
    m_shader->bind();
    m_shader->setLightDirection(QVector3D(0.4f, 0.6f, 1.0f));

    QVector3D cameraWorldPos = m_viewMatrix.inverted().column(3).toVector3D();
    m_shader->setViewPos(cameraWorldPos);

    // 1. Gitter
    if (m_showGrid) {
        renderGrid();
    }

    // 2. Fertigteil (solides industrielles Blau/Cyan)
    if (m_showPart && !m_partMesh.isEmpty()) {
        renderMesh(m_partMesh, QColor(45, 140, 230, 240));
    }

    // 3. Rohteil (Ungefräst = Blau, Gefräst = Gelb / Werkzeugfarbe)
    if (m_showStock) {
        QColor stockColor;
        float spec = 0.0f;
        float shin = 32.0f;
        
        switch (m_materialPreset) {
            case 0: stockColor = QColor(200, 205, 215); spec = 0.7f; shin = 64.0f; break; // Alu
            case 1: stockColor = QColor(205, 175, 55); spec = 0.8f; shin = 96.0f; break; // Messing
            case 2: stockColor = QColor(160, 163, 170); spec = 0.9f; shin = 128.0f; break; // Stahl
            case 3: stockColor = QColor(160, 120, 70); spec = 0.15f; shin = 8.0f; break; // Holz
            case 4: stockColor = QColor(240, 240, 235); spec = 0.3f; shin = 16.0f; break; // POM
            case 5: stockColor = QColor(185, 195, 210); spec = 0.95f; shin = 160.0f; break; // Edelstahl
            default: stockColor = QColor(200, 205, 215); spec = 0.7f; shin = 64.0f; break; // Default
        }


        if (m_useDynamicStock && m_gpuStockModel && m_gpuStockModel->isInitialized() && m_hybridShader) {
            m_hybridShader->bind();
            m_hybridShader->setLightDirection(QVector3D(0.4f, 0.6f, 1.0f));
            m_hybridShader->setViewPos(cameraWorldPos);
            QMatrix4x4 model;
            m_hybridShader->setMatrices(model, m_viewMatrix, m_projectionMatrix);
            m_hybridShader->setColor(stockColor);
            m_hybridShader->setSpecular(spec, shin);

            // Realistisches Material (PBR) und Fräserspuren; Schatten nur mit gültiger Schattenkarte
            const MaterialLook look = materialLook(m_materialPreset);
            auto& stockProgram = m_hybridShader->program();
            const int quality = (m_renderQuality >= 2 && !m_gpuStockModel->hasShadowMap()) ? 1 : m_renderQuality;
            stockProgram.setUniformValue("uQuality", quality);
            stockProgram.setUniformValue("uCutColor", look.cutColor);
            stockProgram.setUniformValue("uCutMetallic", look.cutMetallic);
            stockProgram.setUniformValue("uCutRoughness", look.cutRoughness);
            stockProgram.setUniformValue("uRawColor", look.rawColor);
            stockProgram.setUniformValue("uRawMetallic", look.rawMetallic);
            stockProgram.setUniformValue("uRawRoughness", look.rawRoughness);
            stockProgram.setUniformValue("uLightSpace", m_gpuStockModel->lightSpaceMatrix(keyLightDir));
            m_gpuStockModel->bindShadowTexture(1);
            
            m_gpuStockModel->render(&m_hybridShader->program(), m_renderMode);
            
            m_shader->bind(); // Restore standard shader for the rest
        } else if (!m_stockMesh.isEmpty()) {
            renderMesh(m_stockMesh, stockColor, spec, shin, false);
        }
    }

    // 4. Fräsbahnen
    if (m_showToolpath && !m_toolpath.empty()) {
        renderToolpath();
    }

    // 5. Wählbare 2D-DXF Konturen
    renderContours();

    // 6. Werkzeug an aktueller Position
    renderTool();

    // 7. Achsenkreuz ZULETZT rendern, mit minimaler Z-Erhöhung (0.2mm)
    //    damit Achsen auf der Oberfläche sichtbar sind (kein Z-Fighting),
    //    aber trotzdem korrekt von den Seitenwänden verdeckt werden.
    renderAxes();

    m_shader->release();
}

void Viewport3D::renderGrid() {
    m_shader->setUseLighting(false);
    QMatrix4x4 model;
    m_shader->setMatrices(model, m_viewMatrix, m_projectionMatrix);
    m_shader->setColor(QColor(60, 68, 82, 200));

    // Grid = Maschinenbett-Oberfläche: liegt an der Unterkante des Rohteils (Z_min)
    float gridZ = 0.0f;
    if (!m_stockMesh.isEmpty() && m_stockMesh.boundingBox.isValid()) {
        gridZ = static_cast<float>(m_stockMesh.boundingBox.minPoint.z);
    }

    // Grid-Größe an Stock anpassen
    constexpr float extent = 150.0f;
    constexpr float step = 15.0f;

    std::vector<float> lines;
    for (float x = -extent; x <= extent; x += step) {
        lines.push_back(x); lines.push_back(-extent); lines.push_back(gridZ);
        lines.push_back(0); lines.push_back(0); lines.push_back(1);
        lines.push_back(x); lines.push_back(extent); lines.push_back(gridZ);
        lines.push_back(0); lines.push_back(0); lines.push_back(1);
    }
    for (float y = -extent; y <= extent; y += step) {
        lines.push_back(-extent); lines.push_back(y); lines.push_back(gridZ);
        lines.push_back(0); lines.push_back(0); lines.push_back(1);
        lines.push_back(extent); lines.push_back(y); lines.push_back(gridZ);
        lines.push_back(0); lines.push_back(0); lines.push_back(1);
    }

    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), lines.data());
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), lines.data() + 3);

    glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(lines.size() / 6));

    glDisableVertexAttribArray(0);
    glDisableVertexAttribArray(1);
}

void Viewport3D::renderAxes() {
    m_shader->setUseLighting(false);
    QMatrix4x4 model;
    // Achsen 1.0mm über der Oberfläche (Z=0) rendern:
    // → Verhindert Z-Fighting mit der Stock-Oberfläche
    // → Seitenwände verdecken die Achsen korrekt (Depth-Test bleibt an)
    model.translate(0.0f, 0.0f, 1.0f);
    m_shader->setMatrices(model, m_viewMatrix, m_projectionMatrix);

    constexpr float len = 50.0f;

    glEnableVertexAttribArray(0);

    // Dicke Achsenlinien
    glLineWidth(2.5f);

    // X-Achse (Rot)
    m_shader->setColor(QColor(255, 60, 60, 255));
    float xLine[] = {0,0,0, 0,0,1,  len,0,0, 0,0,1};
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), xLine);
    glDrawArrays(GL_LINES, 0, 2);

    // Y-Achse (Grün)
    m_shader->setColor(QColor(60, 230, 60, 255));
    float yLine[] = {0,0,0, 0,0,1,  0,len,0, 0,0,1};
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), yLine);
    glDrawArrays(GL_LINES, 0, 2);

    // Z-Achse (Blau)
    m_shader->setColor(QColor(80, 150, 255, 255));
    float zLine[] = {0,0,0, 0,0,1,  0,0,len, 0,0,1};
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), zLine);
    glDrawArrays(GL_LINES, 0, 2);

    // Nullpunkt-Markierung: Kreuz auf der XY-Ebene (weißes Fadenkreuz)
    glLineWidth(1.5f);
    m_shader->setColor(QColor(255, 255, 255, 200));
    constexpr float mk = 8.0f;  // 8mm Marker-Größe
    float crossH[] = {-mk,0,0, 0,0,1,  mk,0,0, 0,0,1};
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), crossH);
    glDrawArrays(GL_LINES, 0, 2);
    float crossV[] = {0,-mk,0, 0,0,1,  0,mk,0, 0,0,1};
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), crossV);
    glDrawArrays(GL_LINES, 0, 2);

    // Kleiner Diamant am Ursprung
    float diamond[] = {
        mk,0,0,  0,0,1,   0,mk,0,  0,0,1,
        0,mk,0,  0,0,1,  -mk,0,0,  0,0,1,
       -mk,0,0,  0,0,1,   0,-mk,0, 0,0,1,
        0,-mk,0, 0,0,1,   mk,0,0,  0,0,1
    };
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), diamond);
    glDrawArrays(GL_LINES, 0, 8);

    glLineWidth(1.0f);
    glDisableVertexAttribArray(0);
}

void Viewport3D::renderMesh(const Geometry::Mesh& mesh, const QColor& color, bool useVertexColors, bool wireframe) {
    renderMesh(mesh, color, 0.0f, 32.0f, useVertexColors, wireframe);
}

void Viewport3D::renderMesh(const Geometry::Mesh& mesh, const QColor& color, float specularIntensity, float shininess, bool useVertexColors, bool wireframe) {
    Q_UNUSED(wireframe);
    if (mesh.isEmpty() || mesh.triangles.empty()) return;

    m_shader->setUseLighting(true);
    m_shader->setUseVertexColor(useVertexColors);
    m_shader->setSpecular(specularIntensity, shininess);
    QMatrix4x4 model;
    m_shader->setMatrices(model, m_viewMatrix, m_projectionMatrix);
    m_shader->setColor(color);

    // Vertex-Buffer direkt anbinden
    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);
    if (useVertexColors) {
        glEnableVertexAttribArray(2);
    }

    const float* vPtr = reinterpret_cast<const float*>(mesh.vertices.data());
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Geometry::Vertex), vPtr);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Geometry::Vertex), vPtr + 3);
    if (useVertexColors) {
        glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, sizeof(Geometry::Vertex), vPtr + 6);
    }

    const uint32_t* idxPtr = reinterpret_cast<const uint32_t*>(mesh.triangles.data());
    glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(mesh.triangles.size() * 3), GL_UNSIGNED_INT, idxPtr);

    glDisableVertexAttribArray(0);
    glDisableVertexAttribArray(1);
    if (useVertexColors) {
        glDisableVertexAttribArray(2);
    }
    m_shader->setUseVertexColor(false);
    m_shader->setSpecular(0.0f, 32.0f); // Reset
}

void Viewport3D::renderToolpath() {
    m_shader->setUseLighting(false);
    QMatrix4x4 model;
    m_shader->setMatrices(model, m_viewMatrix, m_projectionMatrix);

    std::vector<float> rapidVerts;
    std::vector<float> feedVerts;
    std::vector<float> collVerts;

    for (const auto& s : m_toolpath.segments) {
        if (!s.visible) continue;
        if (s.hasCollision) {
            collVerts.push_back(static_cast<float>(s.startPos.x));
            collVerts.push_back(static_cast<float>(s.startPos.y));
            collVerts.push_back(static_cast<float>(s.startPos.z));
            collVerts.push_back(static_cast<float>(s.endPos.x));
            collVerts.push_back(static_cast<float>(s.endPos.y));
            collVerts.push_back(static_cast<float>(s.endPos.z));
        } else if (s.motion == CAM::MotionType::Rapid) {
            rapidVerts.push_back(static_cast<float>(s.startPos.x));
            rapidVerts.push_back(static_cast<float>(s.startPos.y));
            rapidVerts.push_back(static_cast<float>(s.startPos.z));
            rapidVerts.push_back(static_cast<float>(s.endPos.x));
            rapidVerts.push_back(static_cast<float>(s.endPos.y));
            rapidVerts.push_back(static_cast<float>(s.endPos.z));
        } else {
            feedVerts.push_back(static_cast<float>(s.startPos.x));
            feedVerts.push_back(static_cast<float>(s.startPos.y));
            feedVerts.push_back(static_cast<float>(s.startPos.z));
            feedVerts.push_back(static_cast<float>(s.endPos.x));
            feedVerts.push_back(static_cast<float>(s.endPos.y));
            feedVerts.push_back(static_cast<float>(s.endPos.z));
        }
    }

    glEnableVertexAttribArray(0);

    // Eilgang (Rot/Pink)
    if (!rapidVerts.empty()) {
        m_shader->setColor(QColor(255, 70, 70, 200));
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), rapidVerts.data());
        glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(rapidVerts.size() / 3));
    }

    // Vorschub (Leuchtendes Gelb nach Max5 Vorbild)
    if (!feedVerts.empty()) {
        m_shader->setColor(QColor(255, 230, 20, 255));
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), feedVerts.data());
        glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(feedVerts.size() / 3));
    }

    // Kollisionssegmente (Signalrot)
    if (!collVerts.empty()) {
        m_shader->setColor(QColor(255, 0, 0, 255));
        glLineWidth(3.0f);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), collVerts.data());
        glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(collVerts.size() / 3));
        glLineWidth(1.0f);
    }

    glDisableVertexAttribArray(0);
}

void Viewport3D::renderTool() {
    m_shader->setUseLighting(true);

    const float r = static_cast<float>(m_activeTool.diameter * 0.5);
    const float fluteH = static_cast<float>(m_activeTool.fluteLength);
    const float shankR = static_cast<float>(m_activeTool.shaftDiameter * 0.5);
    const float stickOut = static_cast<float>(m_activeTool.stickOutLength);
    const float holderR = static_cast<float>(m_activeTool.holderDiameter * 0.5);
    const float holderH = 25.0f;
    const Core::ToolType toolType = m_activeTool.type;

    QMatrix4x4 model;
    model.translate(static_cast<float>(m_toolPos.x),
                    static_cast<float>(m_toolPos.y),
                    static_cast<float>(m_toolPos.z));

    m_shader->setMatrices(model, m_viewMatrix, m_projectionMatrix);

    constexpr int segs = 24; // Höhere Auflösung für glattere Kugelköpfe

    // ═══════════════════════════════════════════════════════
    // Hilfslambdas für Geometrie-Aufbau
    // ═══════════════════════════════════════════════════════

    // Fügt einen Zylinder-Mantel hinzu (von zBot bis zTop, Radius rBot/rTop)
    auto addCylinderMantle = [&](std::vector<float>& verts, std::vector<uint32_t>& indices,
                                  float rBot, float rTop, float zBot, float zTop) {
        uint32_t baseIdx = static_cast<uint32_t>(verts.size() / 6);
        float dz = zTop - zBot;
        float dr = rTop - rBot;
        float slopeLen = std::sqrt(dz * dz + dr * dr);
        float nzComp = (slopeLen > 1e-6f) ? (-dr / slopeLen) : 0.0f;
        float nrComp = (slopeLen > 1e-6f) ? (dz / slopeLen) : 1.0f;

        for (int i = 0; i < segs; ++i) {
            float angle = i * 2.0f * static_cast<float>(M_PI) / segs;
            float cosA = std::cos(angle);
            float sinA = std::sin(angle);

            // Unten
            verts.push_back(rBot * cosA); verts.push_back(rBot * sinA); verts.push_back(zBot);
            verts.push_back(nrComp * cosA); verts.push_back(nrComp * sinA); verts.push_back(nzComp);
            // Oben
            verts.push_back(rTop * cosA); verts.push_back(rTop * sinA); verts.push_back(zTop);
            verts.push_back(nrComp * cosA); verts.push_back(nrComp * sinA); verts.push_back(nzComp);
        }

        for (int i = 0; i < segs; ++i) {
            uint32_t b = baseIdx + static_cast<uint32_t>(i) * 2;
            uint32_t nb = baseIdx + static_cast<uint32_t>(((i + 1) % segs)) * 2;
            indices.push_back(b); indices.push_back(nb); indices.push_back(nb + 1);
            indices.push_back(b); indices.push_back(nb + 1); indices.push_back(b + 1);
        }
    };

    // Fügt eine geschlossene Kreisscheibe (Endkappe) hinzu
    auto addDisk = [&](std::vector<float>& verts, std::vector<uint32_t>& indices,
                        float radius, float z, float nz) {
        uint32_t centerIdx = static_cast<uint32_t>(verts.size() / 6);
        verts.push_back(0.0f); verts.push_back(0.0f); verts.push_back(z);
        verts.push_back(0.0f); verts.push_back(0.0f); verts.push_back(nz);

        for (int i = 0; i < segs; ++i) {
            float angle = i * 2.0f * static_cast<float>(M_PI) / segs;
            verts.push_back(radius * std::cos(angle));
            verts.push_back(radius * std::sin(angle));
            verts.push_back(z);
            verts.push_back(0.0f); verts.push_back(0.0f); verts.push_back(nz);
        }

        for (int i = 0; i < segs; ++i) {
            uint32_t curr = centerIdx + 1 + static_cast<uint32_t>(i);
            uint32_t next = centerIdx + 1 + static_cast<uint32_t>((i + 1) % segs);
            if (nz > 0.0f) {
                indices.push_back(centerIdx); indices.push_back(curr); indices.push_back(next);
            } else {
                indices.push_back(centerIdx); indices.push_back(next); indices.push_back(curr);
            }
        }
    };

    // Fügt eine Halbkugel hinzu (für BallMill)
    auto addHemisphere = [&](std::vector<float>& verts, std::vector<uint32_t>& indices,
                              float radius, float zBase) {
        constexpr int rings = 8;
        uint32_t baseIdx = static_cast<uint32_t>(verts.size() / 6);

        for (int ring = 0; ring <= rings; ++ring) {
            float phi = static_cast<float>(M_PI) * 0.5f * static_cast<float>(ring) / static_cast<float>(rings);
            float rRing = radius * std::cos(phi);
            float zOff = -radius * std::sin(phi); // Halbkugel nach unten

            for (int i = 0; i < segs; ++i) {
                float theta = i * 2.0f * static_cast<float>(M_PI) / segs;
                float x = rRing * std::cos(theta);
                float y = rRing * std::sin(theta);
                float z = zBase + zOff;

                // Normalenvektor zeigt radial nach außen von Kugelzentrum (zBase)
                float nx = x;
                float ny = y;
                float nz = zOff;
                float len = std::sqrt(nx * nx + ny * ny + nz * nz);
                if (len > 1e-6f) { nx /= len; ny /= len; nz /= len; }

                verts.push_back(x); verts.push_back(y); verts.push_back(z);
                verts.push_back(nx); verts.push_back(ny); verts.push_back(nz);
            }
        }

        for (int ring = 0; ring < rings; ++ring) {
            for (int i = 0; i < segs; ++i) {
                uint32_t curr = baseIdx + static_cast<uint32_t>(ring * segs + i);
                uint32_t next = baseIdx + static_cast<uint32_t>(ring * segs + (i + 1) % segs);
                uint32_t currBelow = baseIdx + static_cast<uint32_t>((ring + 1) * segs + i);
                uint32_t nextBelow = baseIdx + static_cast<uint32_t>((ring + 1) * segs + (i + 1) % segs);

                indices.push_back(curr); indices.push_back(next); indices.push_back(nextBelow);
                indices.push_back(curr); indices.push_back(nextBelow); indices.push_back(currBelow);
            }
        }
    };

    // Hilfslambda zum Rendern einer Vertex/Index-Menge
    auto drawGeometry = [&](const std::vector<float>& verts, const std::vector<uint32_t>& indices) {
        if (verts.empty() || indices.empty()) return;
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), verts.data());
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), verts.data() + 3);
        glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(indices.size()), GL_UNSIGNED_INT, indices.data());
    };

    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);

    // ═══════════════════════════════════════════════════════
    // 1. Schneide (Flute) — typabhängig
    // ═══════════════════════════════════════════════════════
    {
        std::vector<float> fluteVerts;
        std::vector<uint32_t> fluteIdx;

        m_shader->setColor(QColor(220, 220, 230, 240)); // Silber / Hartmetall
        m_shader->setSpecular(0.6f, 64.0f);

        switch (toolType) {
            case Core::ToolType::BallMill: {
                // Halbkugel unten + Zylinder oben (ab Kugelradius bis fluteH)
                addHemisphere(fluteVerts, fluteIdx, r, 0.0f);
                if (fluteH > r) {
                    addCylinderMantle(fluteVerts, fluteIdx, r, r, 0.0f, fluteH);
                }
                // Obere Kappe
                addDisk(fluteVerts, fluteIdx, r, fluteH, 1.0f);
                break;
            }

            case Core::ToolType::ChamferMill: {
                // Kegelstumpf: Spitze unten (kleiner Radius), oben = voller Radius
                float tipR = r * 0.1f; // Spitze fast auf 0
                addCylinderMantle(fluteVerts, fluteIdx, tipR, r, 0.0f, fluteH);
                // Bodenkappe (kleine Kreisscheibe)
                addDisk(fluteVerts, fluteIdx, tipR, 0.0f, -1.0f);
                // Obere Kappe
                addDisk(fluteVerts, fluteIdx, r, fluteH, 1.0f);
                break;
            }

            case Core::ToolType::Drill: {
                // Kegelspitze unten (118°-Winkel ≈ Spitzenhöhe ≈ 0.3 * Durchmesser)
                float tipH = r * 0.6f;
                addCylinderMantle(fluteVerts, fluteIdx, 0.0f, r, 0.0f, tipH);
                // Zylindrischer Bohrkörper
                if (fluteH > tipH) {
                    addCylinderMantle(fluteVerts, fluteIdx, r, r, tipH, fluteH);
                }
                // Obere Kappe
                addDisk(fluteVerts, fluteIdx, r, fluteH, 1.0f);
                break;
            }

            case Core::ToolType::EndMill:
            case Core::ToolType::FaceMill:
            default: {
                // Gerader Zylinder mit flachem Boden
                addCylinderMantle(fluteVerts, fluteIdx, r, r, 0.0f, fluteH);
                // Geschlossene Bodenkappe
                addDisk(fluteVerts, fluteIdx, r, 0.0f, -1.0f);
                // Geschlossene Oberkappe
                addDisk(fluteVerts, fluteIdx, r, fluteH, 1.0f);
                break;
            }
        }

        drawGeometry(fluteVerts, fluteIdx);
    }

    // ═══════════════════════════════════════════════════════
    // 2. Schaft (Shank) — von fluteH bis stickOut
    // ═══════════════════════════════════════════════════════
    if (stickOut > fluteH + 0.5f) {
        std::vector<float> shankVerts;
        std::vector<uint32_t> shankIdx;

        m_shader->setColor(QColor(180, 185, 195, 245)); // Helles Stahl-Grau
        m_shader->setSpecular(0.4f, 48.0f);

        addCylinderMantle(shankVerts, shankIdx, shankR, shankR, fluteH, stickOut);
        // Endkappen nur wenn Schaftradius != Fräserradius
        if (std::abs(shankR - r) > 0.2f) {
            addDisk(shankVerts, shankIdx, shankR, fluteH, -1.0f);
        }
        addDisk(shankVerts, shankIdx, shankR, stickOut, 1.0f);

        drawGeometry(shankVerts, shankIdx);
    }

    // ═══════════════════════════════════════════════════════
    // 3. Werkzeughalter (Holder) — ab stickOut nach oben
    // ═══════════════════════════════════════════════════════
    {
        std::vector<float> holderVerts;
        std::vector<uint32_t> holderIdx;

        m_shader->setColor(QColor(80, 85, 95, 255)); // Dunkelgrau / Anthrazit
        m_shader->setSpecular(0.3f, 32.0f);

        addCylinderMantle(holderVerts, holderIdx, holderR, holderR, stickOut, stickOut + holderH);
        addDisk(holderVerts, holderIdx, holderR, stickOut, -1.0f);
        addDisk(holderVerts, holderIdx, holderR, stickOut + holderH, 1.0f);

        drawGeometry(holderVerts, holderIdx);
    }

    m_shader->setSpecular(0.0f, 32.0f); // Reset
    glDisableVertexAttribArray(0);
    glDisableVertexAttribArray(1);
}

void Viewport3D::renderContours() {
    if (m_selectableContours.empty()) return;

    m_shader->setUseLighting(false);
    QMatrix4x4 model;
    m_shader->setMatrices(model, m_viewMatrix, m_projectionMatrix);

    glEnableVertexAttribArray(0);

    for (size_t cIdx = 0; cIdx < m_selectableContours.size(); ++cIdx) {
        const auto& c = m_selectableContours[cIdx];
        if (c.points.size() < 2) continue;

        bool isSelected = (static_cast<int>(cIdx) == m_selectedContourIndex);
        if (isSelected) {
            m_shader->setColor(QColor(255, 220, 0, 255)); // Leuchtendes Goldgelb
            glLineWidth(3.5f);
        } else {
            m_shader->setColor(QColor(80, 210, 240, 200)); // Cyan
            glLineWidth(1.5f);
        }

        std::vector<float> lines;
        const size_t ptCount = c.points.size();
        const size_t segCount = c.isClosed ? ptCount : ptCount - 1;

        for (size_t i = 0; i < segCount; ++i) {
            size_t nextIdx = (i + 1) % ptCount;
            lines.push_back(static_cast<float>(c.points[i].x));
            lines.push_back(static_cast<float>(c.points[i].y));
            lines.push_back(0.1f); // Leicht über Gitter

            lines.push_back(static_cast<float>(c.points[nextIdx].x));
            lines.push_back(static_cast<float>(c.points[nextIdx].y));
            lines.push_back(0.1f);
        }

        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), lines.data());
        glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(lines.size() / 3));
    }

    glLineWidth(1.0f);
    glDisableVertexAttribArray(0);
}

void Viewport3D::mousePressEvent(QMouseEvent* event) {
    m_lastMousePos = event->pos();

    // Interaktives Picking im 3D-Viewport (Hurco DXF Transfer)
    if (m_pickingEnabled && event->button() == Qt::LeftButton && !(event->modifiers() & Qt::ShiftModifier)) {
        float xNdc = (2.0f * event->pos().x()) / width() - 1.0f;
        float yNdc = 1.0f - (2.0f * event->pos().y()) / height();

        QMatrix4x4 invVP = (m_projectionMatrix * m_viewMatrix).inverted();
        QVector4D nearPoint = invVP * QVector4D(xNdc, yNdc, -1.0f, 1.0f);
        QVector4D farPoint  = invVP * QVector4D(xNdc, yNdc, 1.0f, 1.0f);

        if (std::abs(nearPoint.w()) > 1e-6f && std::abs(farPoint.w()) > 1e-6f) {
            QVector3D rayStart = nearPoint.toVector3D() / nearPoint.w();
            QVector3D rayEnd   = farPoint.toVector3D() / farPoint.w();
            QVector3D rayDir   = (rayEnd - rayStart).normalized();

            // Schnitt mit Z=0 Ebene
            if (std::abs(rayDir.z()) > 1e-5f) {
                float t = -rayStart.z() / rayDir.z();
                if (t > 0.0f) {
                    QVector3D hit = rayStart + t * rayDir;
                    double hitX = hit.x();
                    double hitY = hit.y();

                    int bestIdx = -1;
                    double bestDist = 20.0; // 20mm Fangbereich

                    for (size_t cIdx = 0; cIdx < m_selectableContours.size(); ++cIdx) {
                        const auto& c = m_selectableContours[cIdx];
                        for (size_t p = 0; p < c.points.size(); ++p) {
                            size_t nextP = (p + 1) % c.points.size();
                            double x1 = c.points[p].x, y1 = c.points[p].y;
                            double x2 = c.points[nextP].x, y2 = c.points[nextP].y;
                            double dx = x2 - x1, dy = y2 - y1;
                            double lenSq = dx * dx + dy * dy;
                            double u = lenSq > 1e-8 ? std::clamp(((hitX - x1) * dx + (hitY - y1) * dy) / lenSq, 0.0, 1.0) : 0.0;
                            double px = x1 + u * dx;
                            double py = y1 + u * dy;
                            double dist = std::sqrt((hitX - px) * (hitX - px) + (hitY - py) * (hitY - py));
                            if (dist < bestDist) {
                                bestDist = dist;
                                bestIdx = static_cast<int>(cIdx);
                            }
                        }
                    }

                    if (bestIdx >= 0) {
                        m_selectedContourIndex = bestIdx;
                        update();
                        emit contourPicked(bestIdx, m_selectableContours[bestIdx]);
                        return; // Klick verarbeitet, keine Kamerarotation
                    }
                }
            }
        }
    }

    if (event->button() == Qt::LeftButton) {
        if (event->modifiers() & Qt::ShiftModifier) {
            m_isPanning = true;
        } else {
            m_isRotating = true;
        }
    } else if (event->button() == Qt::RightButton || event->button() == Qt::MiddleButton) {
        m_isPanning = true;
    }
}

void Viewport3D::mouseMoveEvent(QMouseEvent* event) {
    int dx = event->pos().x() - m_lastMousePos.x();
    int dy = event->pos().y() - m_lastMousePos.y();
    m_lastMousePos = event->pos();

    if (m_isRotating) {
        m_cameraYaw += dx * 0.4f;
        m_cameraPitch += dy * 0.4f;
        m_cameraPitch = std::clamp(m_cameraPitch, -89.0f, 89.0f);
        update();
    } else if (m_isPanning) {
        float factor = m_cameraDistance * 0.0015f;
        QMatrix4x4 rot;
        rot.rotate(-m_cameraYaw, 0.0f, 0.0f, 1.0f);
        rot.rotate(-m_cameraPitch, 1.0f, 0.0f, 0.0f);
        QVector3D panVec = rot.map(QVector3D(-dx * factor, dy * factor, 0.0f));
        m_cameraTarget += panVec;
        update();
    }
}

void Viewport3D::mouseReleaseEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton) {
        m_isRotating = false;
        m_isPanning = false;
    } else if (event->button() == Qt::RightButton || event->button() == Qt::MiddleButton) {
        m_isPanning = false;
    }
}

void Viewport3D::wheelEvent(QWheelEvent* event) {
    float numDegrees = event->angleDelta().y() / 8.0f;
    float factor = std::pow(0.95f, numDegrees / 15.0f);
    m_cameraDistance = std::clamp(m_cameraDistance * factor, 5.0f, 2000.0f);
    update();
}

} // namespace GeminiCNC::UI
