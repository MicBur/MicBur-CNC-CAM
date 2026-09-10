#include <iostream>
#include <cassert>
#include <cmath>

#include "core/Vector3D.h"
#include "core/BoundingBox.h"
#include "core/ToolDefinition.h"
#include "core/MachineConfig.h"
#include "cam/Toolpath.h"
#include "cam/ToolpathGenerator.h"
#include "cam/CollisionDetector.h"
#include "cam/ConversationalBlock.h"

using namespace GeminiCNC;

void testToolpathGenerationFacing() {
    std::cout << "[TEST] CAM Facing Toolpath Generation..." << std::endl;
    Core::BoundingBox stock({-50, -40, -10}, {50, 40, 0});
    Core::ToolDefinition tool(1, QStringLiteral("40mm Planfräser"), Core::ToolType::FaceMill, 40.0);
    tool.defaultFeedRate = 2000.0;
    tool.spindleSpeed = 12000.0;

    CAM::FacingParams params;
    params.startZ = 0.0;
    params.targetZ = -1.0;
    params.stepDown = 0.5;
    params.clearanceZ = 5.0;

    auto tp = CAM::ToolpathGenerator::generateFacing(stock, tool, params);
    assert(!tp.empty());
    assert(tp.totalLength() > 0.0);
    assert(tp.estimatedTotalTimeSeconds() > 0.0);

    // G-Code Export testen
    QString gcode = tp.generateGCode(false);
    assert(gcode.contains("G0"));
    assert(gcode.contains("G1"));
    assert(gcode.contains("M30"));

    std::cout << " -> PASSED" << std::endl;
}

void testCollisionDetectorLimitsAndRapidDive() {
    std::cout << "[TEST] Collision Detector (Soft Limits & Rapid Dive)..." << std::endl;
    Core::MachineConfig machine;
    machine.minLimits = {-100, -100, -50, 0};
    machine.maxLimits = {100, 100, 50, 360};

    Core::BoundingBox stock({-40, -40, -15}, {40, 40, 0});
    Core::ToolDefinition tool(1, QStringLiteral("6mm Fräser"), Core::ToolType::EndMill, 6.0);
    tool.stickOutLength = 20.0; // 20mm Auskragung

    CAM::Toolpath tp("Test-Bahn");

    // 1. Gültiger Vorschubschnitt
    CAM::PathSegment segValid;
    segValid.motion = CAM::MotionType::LinearFeed;
    segValid.startPos = {0, 0, -2};
    segValid.endPos = {10, 0, -2};
    segValid.spindleRpm = 18000;
    segValid.feedRate = 1000;
    tp.addSegment(segValid);

    // 2. Gefährlicher Eilgang direkt im Rohteil (Z=-5 unter Rohteil Z=0)
    CAM::PathSegment segBadRapid;
    segBadRapid.motion = CAM::MotionType::Rapid;
    segBadRapid.startPos = {0, 0, -5};
    segBadRapid.endPos = {20, 0, -5};
    segBadRapid.spindleRpm = 18000;
    tp.addSegment(segBadRapid);

    // 3. Maschinengrenzen-Überschreitung (X=150 > maxLimit 100)
    CAM::PathSegment segOutLimit;
    segOutLimit.motion = CAM::MotionType::LinearFeed;
    segOutLimit.startPos = {90, 0, 0};
    segOutLimit.endPos = {150, 0, 0};
    segOutLimit.spindleRpm = 18000;
    tp.addSegment(segOutLimit);

    auto report = CAM::CollisionDetector::verifyToolpath(tp, machine, tool, stock);

    assert(report.hasErrors);
    assert(report.totalViolations >= 2);
    assert(tp.segments[1].hasCollision);
    assert(tp.segments[2].hasCollision);
    assert(!tp.segments[0].hasCollision);

    std::cout << " -> PASSED" << std::endl;
}

void testCollisionDetectorStickout() {
    std::cout << "[TEST] Collision Detector (Holder Collision / Stickout)..." << std::endl;
    Core::MachineConfig machine;
    Core::BoundingBox stock({-50, -50, -30}, {50, 50, 0});
    Core::ToolDefinition tool(1, QStringLiteral("Kurzer Fräser"), Core::ToolType::EndMill, 6.0);
    tool.stickOutLength = 10.0; // Nur 10mm Auskragung!

    CAM::Toolpath tp("Stickout Test");
    // Fräsen auf Z = -15mm (Tiefe 15mm > Stickout 10mm -> Halter schlägt auf!)
    CAM::PathSegment segDeep;
    segDeep.motion = CAM::MotionType::LinearFeed;
    segDeep.startPos = {0, 0, -15};
    segDeep.endPos = {20, 0, -15};
    segDeep.spindleRpm = 18000;
    tp.addSegment(segDeep);

    auto report = CAM::CollisionDetector::verifyToolpath(tp, machine, tool, stock);
    assert(report.hasErrors);
    assert(tp.segments[0].hasCollision);
    assert(tp.segments[0].collisionWarning.contains("Werkzeughalter"));

    std::cout << " -> PASSED" << std::endl;
}

void testInspectBlock1Collisions() {
    std::cout << "[DIAGNOSE] Inspecting collisions in default Contour block..." << std::endl;
    CAM::ConversationalBlock b(1, CAM::BlockType::Contour, QStringLiteral("1: Konturfräsen"));
    b.startZ = 0.0;
    b.targetZ = -2.0;
    b.stepDown = 3.0;
    b.contourSide = CAM::ContourSide::Outside;

    Core::ToolDefinition tool(1, QStringLiteral("6mm Fräser"), Core::ToolType::EndMill, 6.0);
    tool.spindleSpeed = 13800;
    tool.defaultFeedRate = 1240;
    tool.plungeFeedRate = 430;

    Core::BoundingBox stock({-50, -40, -10}, {50, 40, 0});
    auto tp = b.generateToolpath(tool, tool, stock);

    Core::MachineConfig machine;
    auto rep = CAM::CollisionDetector::verifyToolpath(tp, machine, tool, stock);

    std::cout << "Total segments: " << tp.size() << ", Violations: " << rep.totalViolations << std::endl;
    for (size_t i = 0; i < rep.violations.size() && i < 10; ++i) {
        const auto& v = rep.violations[i];
        std::cout << "  Violation #" << i << ": Seg=" << v.segmentIndex 
                  << " Pos=(" << v.position.x << ", " << v.position.y << ", " << v.position.z << ") "
                  << " Desc: " << v.description.toStdString() << std::endl;
    }
}

void test3DStlMillingAndRotation() {
    std::cout << "[TEST] 3D-STL Toolpath Generation & Mesh Rotation..." << std::endl;

    // Erzeuge ein einfaches Dach/Keil-Mesh als 3D-Testkörper
    Geometry::Mesh mesh(Geometry::MeshRole::TargetPart, QStringLiteral("Keil"));
    mesh.vertices = {
        {-20.0f, -15.0f, -10.0f,  0.0f,  0.0f, -1.0f, 1.0f, 1.0f, 1.0f, 1.0f}, // 0
        { 20.0f, -15.0f, -10.0f,  0.0f,  0.0f, -1.0f, 1.0f, 1.0f, 1.0f, 1.0f}, // 1
        { 20.0f,  15.0f, -10.0f,  0.0f,  0.0f, -1.0f, 1.0f, 1.0f, 1.0f, 1.0f}, // 2
        {-20.0f,  15.0f, -10.0f,  0.0f,  0.0f, -1.0f, 1.0f, 1.0f, 1.0f, 1.0f}, // 3
        {  0.0f,   0.0f,   0.0f,  0.0f,  0.0f,  1.0f, 1.0f, 1.0f, 1.0f, 1.0f}  // 4 Firstspitze
    };
    mesh.triangles = {
        {0, 1, 4}, // Flanke Süd
        {1, 2, 4}, // Flanke Ost
        {2, 3, 4}, // Flanke Nord
        {3, 0, 4}  // Flanke West
    };
    mesh.computeBoundingBox();

    assert(std::abs(mesh.boundingBox.widthX() - 40.0) < 1e-3);
    assert(std::abs(mesh.boundingBox.depthY() - 30.0) < 1e-3);

    // Test 1: Mesh 90° um Z drehen -> Breite und Tiefe müssen tauschen
    mesh.rotateZ(90.0, true);
    assert(std::abs(mesh.boundingBox.widthX() - 30.0) < 1e-3);
    assert(std::abs(mesh.boundingBox.depthY() - 40.0) < 1e-3);

    // Test 2: Werkzeug definieren (Kugelfräser 6mm)
    Core::ToolDefinition ballTool(2, QStringLiteral("6mm Kugelfräser"), Core::ToolType::BallMill, 6.0);
    ballTool.defaultFeedRate = 1500.0;
    ballTool.plungeFeedRate = 500.0;
    ballTool.spindleSpeed = 16000.0;

    Core::BoundingBox stockBounds({-25.0, -25.0, -12.0}, {25.0, 25.0, 2.0});

    // Test 3: 3D-STL Fräsbahn berechnen (Kombiniert: Schruppen & Schlichten)
    CAM::StlMillingParams params;
    params.mode = CAM::StlMillingMode::RoughAndFinish;
    params.finishDirection = 0; // X-Richtung
    params.roughStepDown = 3.0;
    params.roughStepOverRatio = 0.5;
    params.finishStepOver = 1.0;
    params.finishAllowance = 0.3;
    params.sampleStep = 1.0;
    params.clearanceZ = 5.0;

    auto tp = CAM::ToolpathGenerator::generateStlMilling(mesh, stockBounds, ballTool, params);
    assert(!tp.empty());
    assert(tp.size() > 50); // Sollte viele Zeilensegmente haben
    assert(tp.totalLength() > 100.0);

    // Prüfen, ob Z-Höhen variieren (folgen der 3D-Kontur)
    bool hasZVariation = false;
    for (const auto& seg : tp.segments) {
        if (std::abs(seg.startPos.z - seg.endPos.z) > 0.1) {
            hasZVariation = true;
            break;
        }
    }
    assert(hasZVariation);

    // Test 4: Block-Integration testen
    CAM::ConversationalBlock stlBlock(1, CAM::BlockType::Stl3D, QStringLiteral("1: 3D-STL Fräsen"));
    stlBlock.directStlMesh = mesh;
    stlBlock.stlStrategy = CAM::StlMillingStrategy::RasterX;
    stlBlock.stlStepOver = 1.0;
    stlBlock.stlStepDown = 3.0;

    auto blockTp = stlBlock.generateToolpath(ballTool, ballTool, stockBounds);
    assert(!blockTp.empty());
    assert(blockTp.size() > 20);

    std::cout << " -> PASSED (Total segments generated: " << tp.size() << ")" << std::endl;
}

int main() {
    std::cout << "=== Running CAM & Collision Test Suite ===" << std::endl;
    testToolpathGenerationFacing();
    testCollisionDetectorLimitsAndRapidDive();
    testCollisionDetectorStickout();
    testInspectBlock1Collisions();
    test3DStlMillingAndRotation();
    std::cout << "=== All CAM Tests PASSED ===" << std::endl;
    return 0;
}
