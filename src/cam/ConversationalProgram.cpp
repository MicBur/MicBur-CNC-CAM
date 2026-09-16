#include "ConversationalProgram.h"
#include <algorithm>
#include <cmath>
#include <functional>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>

namespace GeminiCNC::CAM {

void ConversationalProgram::addBlock(const ConversationalBlock& block) {
    blocks.push_back(block);
}

void ConversationalProgram::insertBlock(size_t index, const ConversationalBlock& block) {
    if (index <= blocks.size()) {
        blocks.insert(blocks.begin() + index, block);
    }
}

void ConversationalProgram::removeBlock(size_t index) {
    if (index < blocks.size()) {
        blocks.erase(blocks.begin() + index);
    }
}

bool ConversationalProgram::moveBlockUp(size_t index) {
    if (index > 0 && index < blocks.size()) {
        std::swap(blocks[index], blocks[index - 1]);
        return true;
    }
    return false;
}

bool ConversationalProgram::moveBlockDown(size_t index) {
    if (index + 1 < blocks.size()) {
        std::swap(blocks[index], blocks[index + 1]);
        return true;
    }
    return false;
}

void ConversationalProgram::duplicateBlock(size_t index) {
    if (index < blocks.size()) {
        ConversationalBlock copy = blocks[index];
        copy.id = static_cast<int>(blocks.size()) + 1;
        copy.name = QString("%1 (Kopie)").arg(copy.name);
        blocks.insert(blocks.begin() + index + 1, copy);
    }
}

namespace {

constexpr double kPi = 3.14159265358979323846;

// Ebene Affintransformation einer Musterposition: x' = a*x + b*y + tx, y' = c*x + d*y + ty
struct PatternTransform {
    double a{1.0}, b{0.0}, c{0.0}, d{1.0}, tx{0.0}, ty{0.0};

    // Erst diese Transformation, danach outer (für verschachtelte Muster)
    [[nodiscard]] PatternTransform then(const PatternTransform& outer) const {
        PatternTransform r;
        r.a = outer.a * a + outer.b * c;
        r.b = outer.a * b + outer.b * d;
        r.c = outer.c * a + outer.d * c;
        r.d = outer.c * b + outer.d * d;
        r.tx = outer.a * tx + outer.b * ty + outer.tx;
        r.ty = outer.c * tx + outer.d * ty + outer.ty;
        return r;
    }

    [[nodiscard]] bool isIdentity() const {
        return a == 1.0 && b == 0.0 && c == 0.0 && d == 1.0 && tx == 0.0 && ty == 0.0;
    }

    void apply(Core::Vector3D& p) const {
        const double x = p.x, y = p.y;
        p.x = a * x + b * y + tx;
        p.y = c * x + d * y + ty;
    }

    void apply(PathSegment& seg) const {
        apply(seg.startPos);
        apply(seg.endPos);
        apply(seg.arcCenter);
        // Spiegeln kehrt den Drehsinn von Kreisbögen um
        if (a * d - b * c < 0.0) {
            if (seg.motion == MotionType::ArcCW) seg.motion = MotionType::ArcCCW;
            else if (seg.motion == MotionType::ArcCCW) seg.motion = MotionType::ArcCW;
        }
    }

    static PatternTransform translation(double x, double y) {
        PatternTransform t;
        t.tx = x;
        t.ty = y;
        return t;
    }

    static PatternTransform rotation(double deg, double cx, double cy) {
        const double r = deg * kPi / 180.0;
        const double cs = std::cos(r), sn = std::sin(r);
        PatternTransform t;
        t.a = cs;  t.b = -sn;
        t.c = sn;  t.d = cs;
        t.tx = cx - cs * cx + sn * cy;
        t.ty = cy - sn * cx - cs * cy;
        return t;
    }

    static PatternTransform mirror(bool flipX, bool flipY, double cx, double cy) {
        PatternTransform t;
        if (flipX) { t.a = -1.0; t.tx = 2.0 * cx; }
        if (flipY) { t.d = -1.0; t.ty = 2.0 * cy; }
        return t;
    }
};

std::vector<PatternTransform> patternInstances(const ConversationalBlock& p) {
    std::vector<PatternTransform> out;
    const double ang = p.patternAngleDeg * kPi / 180.0;
    const double ux = std::cos(ang), uy = std::sin(ang);

    switch (p.patternType) {
        case PatternType::Linear: {
            const int n = std::max(1, p.patternCountX);
            for (int k = 0; k < n; ++k) {
                out.push_back(PatternTransform::translation(k * p.patternSpacingX * ux, k * p.patternSpacingX * uy));
            }
            break;
        }
        case PatternType::Rectangular: {
            const int nx = std::max(1, p.patternCountX);
            const int ny = std::max(1, p.patternCountY);
            for (int j = 0; j < ny; ++j) {
                for (int i = 0; i < nx; ++i) {
                    const double ox = i * p.patternSpacingX, oy = j * p.patternSpacingY;
                    out.push_back(PatternTransform::translation(ox * ux - oy * uy, ox * uy + oy * ux));
                }
            }
            break;
        }
        case PatternType::Circular: {
            const int n = std::max(1, p.patternCountX);
            const double step = std::abs(p.patternStepAngleDeg) > 1e-9 ? p.patternStepAngleDeg : 360.0 / n;
            for (int k = 0; k < n; ++k) {
                out.push_back(PatternTransform::rotation(p.patternAngleDeg + k * step, p.patternCenterX, p.patternCenterY));
            }
            break;
        }
        case PatternType::Mirror:
            out.emplace_back(); // Original
            if (p.patternMirrorX) out.push_back(PatternTransform::mirror(true, false, p.patternCenterX, p.patternCenterY));
            if (p.patternMirrorY) out.push_back(PatternTransform::mirror(false, true, p.patternCenterX, p.patternCenterY));
            if (p.patternMirrorX && p.patternMirrorY) out.push_back(PatternTransform::mirror(true, true, p.patternCenterX, p.patternCenterY));
            break;
    }

    if (out.empty()) out.emplace_back();
    return out;
}

// Passendes Muster Ende (verschachtelte Muster berücksichtigt); ohne Ende gilt das Muster bis Programmende
size_t findPatternEnd(const std::vector<ConversationalBlock>& blocks, size_t start) {
    int depth = 0;
    for (size_t i = start + 1; i < blocks.size(); ++i) {
        if (blocks[i].type == BlockType::PatternStart) {
            ++depth;
        } else if (blocks[i].type == BlockType::PatternEnd) {
            if (depth == 0) return i;
            --depth;
        }
    }
    return blocks.size();
}

} // namespace

Toolpath ConversationalProgram::generateFullToolpath(
    const QList<Core::ToolDefinition>& toolLibrary,
    const Core::BoundingBox& stockBounds,
    const Geometry::Mesh& partMesh) const {

    Toolpath fullTp(programName);
    Core::Vector3D currentMachinePos = {0.0, 0.0, 20.0, 0.0};
    int currentToolId = -1;

    auto addSegment = [&](const PathSegment& seg) {
        fullTp.addSegment(seg);
        currentMachinePos = seg.endPos;
    };

    // Verbindungsfahrt zum nächsten Startpunkt: hoch auf Sicherheitshöhe, im Eilgang hinüber,
    // letzte Annäherung im Vorschub (kein Eilgang ins Material)
    auto addLinkMove = [&](const PathSegment& next, double clearanceZ, double plungeFeed) {
        const Core::Vector3D target = next.startPos;
        const double safeZ = std::max({currentMachinePos.z, target.z, clearanceZ});

        PathSegment link;
        link.toolId = next.toolId;
        link.toolDiameter = next.toolDiameter;
        link.spindleRpm = next.spindleRpm;
        link.visible = next.visible;
        link.blockId = next.blockId;
        link.motion = MotionType::Rapid;
        link.feedRate = 0;

        if (currentMachinePos.z < safeZ - 1e-6) {
            link.startPos = currentMachinePos;
            link.endPos = {currentMachinePos.x, currentMachinePos.y, safeZ, currentMachinePos.a};
            addSegment(link);
        }
        if (std::abs(target.x - currentMachinePos.x) > 1e-6 || std::abs(target.y - currentMachinePos.y) > 1e-6
            || std::abs(target.a - currentMachinePos.a) > 1e-6) {
            link.startPos = currentMachinePos;
            link.endPos = {target.x, target.y, safeZ, target.a};
            addSegment(link);
        }
        if (target.z < currentMachinePos.z - 1e-6) {
            link.motion = MotionType::LinearFeed;
            link.feedRate = plungeFeed > 0.0 ? plungeFeed : std::max(1.0, next.feedRate);
            link.startPos = currentMachinePos;
            link.endPos = target;
            addSegment(link);
        }
    };

    auto appendBlock = [&](const ConversationalBlock& block, const PatternTransform& transform) {
        // Werkzeug für diesen Block (Hauptwerkzeug) suchen
        Core::ToolDefinition mainTool(block.toolId, "Standardfräser", Core::ToolType::EndMill, 6.0);
        for (const auto& t : toolLibrary) {
            if (t.id == block.toolId) {
                mainTool = t;
                break;
            }
        }

        Core::ToolDefinition finishTool = mainTool;
        if (block.finishToolId > 0 && block.finishToolId != block.toolId) {
            for (const auto& t : toolLibrary) {
                if (t.id == block.finishToolId) {
                    finishTool = t;
                    break;
                }
            }
        }

        Toolpath blockTp = block.generateToolpath(mainTool, finishTool, stockBounds, partMesh);
        const bool transformed = !transform.isIdentity();
        const double clearance = std::max(block.clearanceZ, block.startZ + block.clearanceZ);

        for (auto seg : blockTp.segments) {
            if (transformed) transform.apply(seg);

            if (!fullTp.empty()) {
                if (seg.toolId != currentToolId && currentToolId != -1) {
                    // Bei Werkzeugwechsel sicheren Rückzug auf Z=25mm einfügen
                    PathSegment retract;
                    retract.motion = MotionType::Rapid;
                    retract.startPos = currentMachinePos;
                    retract.endPos = {currentMachinePos.x, currentMachinePos.y, 25.0};
                    retract.feedRate = 0;
                    retract.toolId = currentToolId;
                    addSegment(retract);
                }
                // Lücke zum Startpunkt (nächster Block, nächste Musterposition, nach Werkzeugwechsel) schließen
                const bool gap = std::abs(seg.startPos.x - currentMachinePos.x) > 1e-3
                              || std::abs(seg.startPos.y - currentMachinePos.y) > 1e-3
                              || std::abs(seg.startPos.z - currentMachinePos.z) > 1e-3
                              || std::abs(seg.startPos.a - currentMachinePos.a) > 1e-3;
                if (gap) addLinkMove(seg, clearance, block.plungeFeedRate);
            }

            currentToolId = seg.toolId;
            addSegment(seg);
        }
    };

    // Blöcke [first, last) abarbeiten; Muster wiederholen ihren Inhalt je Musterposition
    std::function<void(size_t, size_t, const PatternTransform&)> generateRange =
        [&](size_t first, size_t last, const PatternTransform& outer) {
            for (size_t i = first; i < last; ++i) {
                const auto& block = blocks[i];
                if (block.type == BlockType::PatternEnd) continue; // Muster Ende ohne passenden Beginn

                if (block.type == BlockType::PatternStart) {
                    const size_t end = std::min(findPatternEnd(blocks, i), last);
                    const auto instances = block.enabled ? patternInstances(block) : std::vector<PatternTransform>(1);
                    for (const auto& instance : instances) {
                        generateRange(i + 1, end, instance.then(outer));
                    }
                    i = end; // Muster Ende überspringen
                    continue;
                }

                if (block.enabled) appendBlock(block, outer);
            }
        };
    generateRange(0, blocks.size(), PatternTransform{});

    // Abschließender Rückzug
    if (!fullTp.empty()) {
        PathSegment finalRetract;
        finalRetract.motion = MotionType::Rapid;
        finalRetract.startPos = currentMachinePos;
        finalRetract.endPos = {currentMachinePos.x, currentMachinePos.y, 25.0};
        finalRetract.feedRate = 0;
        finalRetract.toolId = currentToolId;
        fullTp.addSegment(finalRetract);
    }

    return fullTp;
}

bool ConversationalProgram::saveToFile(const QString& filePath) const {
    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly)) return false;

    QJsonObject root;
    root[QStringLiteral("programName")] = programName;

    QJsonArray blkArray;
    for (const auto& b : blocks) {
        blkArray.append(b.toJson());
    }
    root[QStringLiteral("blocks")] = blkArray;

    file.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    file.close();
    return true;
}

ConversationalProgram ConversationalProgram::loadFromFile(const QString& filePath) {
    ConversationalProgram prog;
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly)) return prog;

    QByteArray data = file.readAll();
    file.close();

    QJsonDocument doc = QJsonDocument::fromJson(data);
    if (!doc.isObject()) return prog;

    QJsonObject root = doc.object();
    if (root.contains(QStringLiteral("programName"))) {
        prog.programName = root[QStringLiteral("programName")].toString();
    }

    if (root.contains(QStringLiteral("blocks"))) {
        QJsonArray blkArray = root[QStringLiteral("blocks")].toArray();
        for (const auto& v : blkArray) {
            prog.addBlock(ConversationalBlock::fromJson(v.toObject()));
        }
    }

    return prog;
}

ConversationalProgram ConversationalProgram::createSampleProgram() {
    ConversationalProgram prog;
    prog.programName = QStringLiteral("WinMax_Musterbauteil");

    // Block 1: Planfräsen mit 40mm Messerkopf
    ConversationalBlock b1(1, BlockType::Facing, QStringLiteral("1: Rohteil planen (Z=0)"));
    b1.toolId = 3; // 40mm Planfräser
    b1.startZ = 0.5;
    b1.targetZ = 0.0;
    b1.stepDown = 0.5;
    b1.stepOver = 20.0;
    b1.spindleRpm = 12000.0;
    b1.feedRate = 2200.0;
    prog.addBlock(b1);

    // Block 2: Rechtecktasche 50x30mm
    ConversationalBlock b2(2, BlockType::Pocket, QStringLiteral("2: Haupttasche schruppen"));
    b2.toolId = 1; // 6mm Schaftfräser
    b2.pocketShape = PocketShape::Rectangle;
    b2.pocketWidthX = 50.0;
    b2.pocketDepthY = 30.0;
    b2.startZ = 0.0;
    b2.targetZ = -4.0;
    b2.stepDown = 1.5;
    b2.stepOver = 3.0;
    b2.spindleRpm = 18000.0;
    b2.feedRate = 1400.0;
    prog.addBlock(b2);

    // Block 3: 6-Loch-Teilkreis
    ConversationalBlock b3(3, BlockType::Drill, QStringLiteral("3: Lochkreis Ø50mm"));
    b3.toolId = 2; // 3mm Fräser / Bohrer
    b3.drillPattern = DrillPattern::BoltCircle;
    b3.boltCircleRadius = 25.0;
    b3.boltCircleHoleCount = 6;
    b3.startZ = 0.0;
    b3.targetZ = -6.0;
    b3.peckDepth = 2.0;
    b3.spindleRpm = 16000.0;
    b3.plungeFeedRate = 350.0;
    prog.addBlock(b3);

    // Block 4: Außenkontur mit Fasen (Benutzerdefinierte Segmente)
    ConversationalBlock b4(4, BlockType::Contour, QStringLiteral("4: Außenkontur mit Fasen"));
    b4.toolId = 1; // 6mm Schaftfräser
    b4.contourSide = ContourSide::Outside;
    b4.startZ = 0.0;
    b4.targetZ = -17.0;
    b4.stepDown = 2.0;
    b4.finishAllowance = 0.0;
    b4.spindleRpm = 18000.0;
    b4.feedRate = 1500.0;
    b4.plungeFeedRate = 500.0;

    // Kontur: Rechteck 70×40mm mit 3mm-Fasen rechts oben + rechts unten
    //   (5,5) → (72,5) → (75,8) → (75,42) → (72,45) → (5,45) → geschlossen
    {
        using ST = Geometry::ContourSegmentType;
        Geometry::ContourSegment s;

        // Startpunkt
        s.type = ST::StartPoint; s.x = 5.0; s.y = 5.0;
        b4.segments.push_back(s);

        // Unterkante
        s.type = ST::Line; s.x = 72.0; s.y = 5.0;
        b4.segments.push_back(s);

        // Fase rechts unten (3×3mm, 45°)
        s.type = ST::Line; s.x = 75.0; s.y = 8.0;
        b4.segments.push_back(s);

        // Rechte Seite
        s.type = ST::Line; s.x = 75.0; s.y = 42.0;
        b4.segments.push_back(s);

        // Fase rechts oben (3×3mm, 45°)
        s.type = ST::Line; s.x = 72.0; s.y = 45.0;
        b4.segments.push_back(s);

        // Oberkante
        s.type = ST::Line; s.x = 5.0; s.y = 45.0;
        b4.segments.push_back(s);

        // Linke Seite zurück zum Start (Kontur schließen)
        s.type = ST::Line; s.x = 5.0; s.y = 5.0;
        b4.segments.push_back(s);
    }

    // Kontur aus Segmenten kompilieren
    b4.contour = Geometry::Contour::createFromSegments(b4.segments, true);
    prog.addBlock(b4);

    return prog;
}

} // namespace GeminiCNC::CAM
