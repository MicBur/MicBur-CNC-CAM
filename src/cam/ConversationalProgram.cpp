#include "ConversationalProgram.h"
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

Toolpath ConversationalProgram::generateFullToolpath(
    const QList<Core::ToolDefinition>& toolLibrary,
    const Core::BoundingBox& stockBounds,
    const Geometry::Mesh& partMesh) const {

    Toolpath fullTp(programName);
    Core::Vector3D currentMachinePos = {0.0, 0.0, 20.0, 0.0};
    int currentToolId = -1;

    for (const auto& block : blocks) {
        if (!block.enabled) continue;

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
        for (const auto& seg : blockTp.segments) {
            if (seg.toolId != currentToolId && currentToolId != -1) {
                // Bei Werkzeugwechsel sicheren Rückzug auf Z=25mm einfügen
                if (!fullTp.empty()) {
                    PathSegment retract;
                    retract.motion = MotionType::Rapid;
                    retract.startPos = currentMachinePos;
                    retract.endPos = {currentMachinePos.x, currentMachinePos.y, 25.0};
                    retract.feedRate = 0;
                    retract.toolId = currentToolId;
                    retract.toolDiameter = 6.0;
                    fullTp.addSegment(retract);
                    currentMachinePos = retract.endPos;
                }
            }
            currentToolId = seg.toolId;
            fullTp.addSegment(seg);
            currentMachinePos = seg.endPos;
        }
    }

    // Abschließender Rückzug
    if (!fullTp.empty()) {
        PathSegment finalRetract;
        finalRetract.motion = MotionType::Rapid;
        finalRetract.startPos = currentMachinePos;
        finalRetract.endPos = {currentMachinePos.x, currentMachinePos.y, 25.0};
        finalRetract.feedRate = 0;
        finalRetract.toolId = currentToolId;
        finalRetract.toolDiameter = 6.0;
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
