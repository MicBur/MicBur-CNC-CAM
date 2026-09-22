/**
 * @file TestFullMachining.cpp
 * @brief Umfassender Integrationstest: Erzeugt ein komplettes Bearbeitungsprogramm,
 *        berechnet Toolpaths, simuliert den Materialabtrag und exportiert STL.
 *
 * Testet: PlanfrÃ¤sen, Tasche, Nut, AuÃŸenkontur, Bohren (Lochkreis + Raster),
 *         Kreis-Tasche, Simulation, G-Code-Export, Programm-Serialisierung, Material-DB.
 *
 * Autor: Michael Burzlaff / MicBur-CNC-CAM Automated Test
 */

#include <QCoreApplication>
#include <QFile>
#include <QJsonDocument>
#include <cstdio>
#include <cmath>

#include "cam/ConversationalBlock.h"
#include "cam/ConversationalProgram.h"
#include "cam/Toolpath.h"
#include "cam/PostProcessor.h"
#include "core/ToolDefinition.h"
#include "core/MaterialDatabase.h"
#include "core/BoundingBox.h"
#include "core/MachineConfig.h"
#include "geometry/Mesh.h"
#include "geometry/Contour.h"
#include "geometry/StlLoader.h"
#include "simulation/StockModel.h"

using namespace GeminiCNC;
using namespace GeminiCNC::CAM;
using namespace GeminiCNC::Core;
using namespace GeminiCNC::Geometry;

static int g_passed = 0;
static int g_failed = 0;

static void check(bool cond, const char* msg) {
    if (cond) {
        printf("  [PASS] %s\n", msg);
        g_passed++;
    } else {
        printf("  [FAIL] %s\n", msg);
        g_failed++;
    }
}

// ============================================================
// Werkzeugbibliothek
// ============================================================
static QList<ToolDefinition> buildToolLibrary() {
    QList<ToolDefinition> tools;

    ToolDefinition t1;
    t1.id = 1; t1.name = "6mm SchaftfrÃ¤ser"; t1.type = ToolType::EndMill;
    t1.diameter = 6.0; t1.flutes = 2; t1.fluteLength = 20.0;
    t1.shaftDiameter = 6.0; t1.overallLength = 50.0; t1.stickOutLength = 30.0;
    t1.holderDiameter = 25.0;
    t1.defaultFeedRate = 1500; t1.plungeFeedRate = 500; t1.spindleSpeed = 18000;
    t1.maxStepDown = 2.5; t1.stepOverPercentage = 45;
    tools.append(t1);

    ToolDefinition t2;
    t2.id = 2; t2.name = "3mm SchaftfrÃ¤ser"; t2.type = ToolType::EndMill;
    t2.diameter = 3.0; t2.flutes = 2; t2.fluteLength = 12.0;
    t2.shaftDiameter = 3.0; t2.overallLength = 38.0; t2.stickOutLength = 20.0;
    t2.holderDiameter = 20.0;
    t2.defaultFeedRate = 1200; t2.plungeFeedRate = 400; t2.spindleSpeed = 22000;
    t2.maxStepDown = 1.0; t2.stepOverPercentage = 40;
    tools.append(t2);

    ToolDefinition t3;
    t3.id = 3; t3.name = "40mm PlanfrÃ¤skopf"; t3.type = ToolType::FaceMill;
    t3.diameter = 40.0; t3.flutes = 4; t3.fluteLength = 8.0;
    t3.shaftDiameter = 16.0; t3.overallLength = 60.0; t3.stickOutLength = 40.0;
    t3.holderDiameter = 50.0;
    t3.defaultFeedRate = 3000; t3.plungeFeedRate = 500; t3.spindleSpeed = 12000;
    t3.maxStepDown = 0.8; t3.stepOverPercentage = 70;
    tools.append(t3);

    ToolDefinition t4;
    t4.id = 4; t4.name = "4mm KugelfrÃ¤ser"; t4.type = ToolType::BallMill;
    t4.diameter = 4.0; t4.flutes = 2; t4.fluteLength = 16.0;
    t4.shaftDiameter = 4.0; t4.overallLength = 45.0; t4.stickOutLength = 25.0;
    t4.holderDiameter = 20.0;
    t4.defaultFeedRate = 1000; t4.plungeFeedRate = 300; t4.spindleSpeed = 20000;
    t4.maxStepDown = 0.5; t4.stepOverPercentage = 15;
    tools.append(t4);

    return tools;
}

static ToolDefinition findTool(const QList<ToolDefinition>& tools, int id) {
    for (const auto& t : tools)
        if (t.id == id) return t;
    return tools.first();
}

// ============================================================
// Test 1: PlanfrÃ¤sen
// ============================================================
static void testFacing(const QList<ToolDefinition>& tools, const BoundingBox& stock) {
    printf("\n[TEST 1] PlanfrÃ¤sen (Facing) mit T3 PlanfrÃ¤ser...\n");
    ConversationalBlock b(1, BlockType::Facing, "PlanfrÃ¤sen");
    b.toolId = 3; b.startZ = 0.0; b.targetZ = -1.0; b.stepDown = 0.5;
    b.useStockDimensions = true; b.spindleRpm = 12000; b.feedRate = 3000;

    auto tool = findTool(tools, b.toolId);
    auto tp = b.generateToolpath(tool, tool, stock);
    int segs = (int)tp.segments.size();

    check(segs > 0, "Toolpath hat Segmente");
    check(segs > 20, ("Genug Bahnen (>20): " + std::to_string(segs)).c_str());

    double minZ = 999, maxZ = -999;
    for (const auto& s : tp.segments) {
        if (s.endPos.z < minZ) minZ = s.endPos.z;
        if (s.endPos.z > maxZ) maxZ = s.endPos.z;
    }
    check(minZ <= -0.9, ("Min Z Zieltiefe: " + std::to_string(minZ)).c_str());
    check(maxZ >= 4.0, ("Sicherheitsebene: " + std::to_string(maxZ)).c_str());
    printf("  -> %d Segmente, Z: %.2f bis %.2f mm\n", segs, minZ, maxZ);
}

// ============================================================
// Test 2: Rechteck-Tasche
// ============================================================
static void testPocket(const QList<ToolDefinition>& tools, const BoundingBox& stock) {
    printf("\n[TEST 2] Rechteck-Tasche (Pocket) mit T1...\n");
    ConversationalBlock b(2, BlockType::Pocket, "Tasche");
    b.toolId = 1; b.millingType = MillingType::Pocket;
    b.pocketShape = PocketShape::Rectangle;
    b.pocketWidthX = 40.0; b.pocketDepthY = 30.0; b.pocketCornerR = 3.0;
    b.posX = 30.0; b.posY = 25.0;
    b.startZ = 0.0; b.targetZ = -10.0; b.stepDown = 2.0;
    b.stepOver = 2.7; b.pocketStrategy = 1;
    b.spindleRpm = 18000; b.feedRate = 1500;

    auto tool = findTool(tools, b.toolId);
    auto tp = b.generateToolpath(tool, tool, stock);
    int segs = (int)tp.segments.size();

    check(segs > 100, ("Genug Segmente (>100): " + std::to_string(segs)).c_str());
    double minZ = 999;
    for (const auto& s : tp.segments)
        if (s.endPos.z < minZ) minZ = s.endPos.z;
    check(minZ <= -9.5, ("Tiefe: " + std::to_string(minZ)).c_str());
    printf("  -> %d Segmente, Tiefe: %.2f mm\n", segs, minZ);
}

// ============================================================
// Test 3: Nut / Langloch
// ============================================================
static void testSlot(const QList<ToolDefinition>& tools, const BoundingBox& stock) {
    printf("\n[TEST 3] Nut (Slot) mit T2...\n");
    ConversationalBlock b(3, BlockType::Slot, "Nut");
    b.toolId = 2; b.slotLength = 40.0; b.slotWidth = 6.0; b.slotAngleDeg = 45.0;
    b.posX = 10.0; b.posY = 55.0;
    b.startZ = 0.0; b.targetZ = -5.0; b.stepDown = 1.0;
    b.spindleRpm = 22000; b.feedRate = 1200;

    auto tool = findTool(tools, b.toolId);
    auto tp = b.generateToolpath(tool, tool, stock);
    int segs = (int)tp.segments.size();

    check(segs > 10, ("Nut Segmente (>10): " + std::to_string(segs)).c_str());
    printf("  -> %d Segmente\n", segs);
}

// ============================================================
// Test 4: AuÃŸenkontur
// ============================================================
static void testContour(const QList<ToolDefinition>& tools, const BoundingBox& stock) {
    printf("\n[TEST 4] AuÃŸenkontur (Contour) mit T1...\n");
    ConversationalBlock b(4, BlockType::Contour, "Kontur");
    b.toolId = 1; b.contourSide = ContourSide::Outside;
    b.millingType = MillingType::Outside;
    b.startZ = 0.0; b.targetZ = -15.0; b.stepDown = 2.5;
    b.finishAllowance = 0.1; b.leadType = 1; b.leadRadius = 3.0;
    b.spindleRpm = 18000; b.feedRate = 1500; b.contourZForAll = true;

    using ST = ContourSegmentType;
    b.segments.clear();
    b.segments.push_back({ST::StartPoint, 0, 0, 0, 0, 0, 0, -15.0});
    b.segments.push_back({ST::Line, 80, 0, 0, 0, 0, 0, -15.0});
    b.segments.push_back({ST::Line, 80, 60, 0, 0, 0, 0, -15.0});
    b.segments.push_back({ST::Line, 0, 60, 0, 0, 0, 0, -15.0});
    b.segments.push_back({ST::Line, 0, 0, 0, 0, 0, 0, -15.0});

    auto tool = findTool(tools, b.toolId);
    auto tp = b.generateToolpath(tool, tool, stock);
    int segs = (int)tp.segments.size();

    check(segs > 20, ("Kontur Segmente (>20): " + std::to_string(segs)).c_str());
    double minZ = 999;
    for (const auto& s : tp.segments)
        if (s.endPos.z < minZ) minZ = s.endPos.z;
    check(minZ <= -14.5, ("Kontur Tiefe: " + std::to_string(minZ)).c_str());
    printf("  -> %d Segmente, Tiefe: %.2f mm\n", segs, minZ);
}

// ============================================================
// Test 5: Bohren â€” Lochkreis
// ============================================================
static void testDrillBoltCircle(const QList<ToolDefinition>& tools, const BoundingBox& stock) {
    printf("\n[TEST 5] Bohren Lochkreis (6x R20) mit T1...\n");

    ConversationalBlock bDrill(5, BlockType::Drill, "Bohren Lochkreis");
    bDrill.toolId = 1; bDrill.startZ = 0.0; bDrill.targetZ = -12.0;
    bDrill.spindleRpm = 18000; bDrill.feedRate = 500;

    DrillOperation op;
    op.type = DrillOperationType::Drill; op.toolId = 1;
    op.spindleRpm = 18000; op.plungeFeed = 500;
    op.cycleType = DrillCycleType::DeepHole; op.peckDepth = 3.0;
    bDrill.drillOps.push_back(op);

    ConversationalBlock bPos(6, BlockType::DrillPositions, "Lochkreis");
    bPos.drillPattern = DrillPattern::BoltCircle;
    bPos.boltCircleRadius = 20.0; bPos.boltCircleHoleCount = 6;
    bPos.boltCircleStartAngle = 0; bPos.posX = 50.0; bPos.posY = 40.0;

    auto positions = bPos.calculateDrillPositions();
    check((int)positions.size() == 6, ("6 Positionen: " + std::to_string(positions.size())).c_str());

    for (const auto& p : positions)
        bDrill.resolvedDrillPositions.push_back({p.x, p.y});

    auto tool = findTool(tools, bDrill.toolId);
    auto tp = bDrill.generateToolpath(tool, tool, stock);
    int segs = (int)tp.segments.size();

    check(segs > 10, ("Bohr-Segmente (>10): " + std::to_string(segs)).c_str());
    double minZ = 999;
    for (const auto& s : tp.segments)
        if (s.endPos.z < minZ) minZ = s.endPos.z;
    check(minZ <= -11.0, ("Bohrtiefe: " + std::to_string(minZ)).c_str());
    printf("  -> %d Segmente, Tiefe: %.2f mm\n", segs, minZ);
}

// ============================================================
// Test 6: Bohren â€” Raster
// ============================================================
static void testDrillGrid(const QList<ToolDefinition>& tools, const BoundingBox& stock) {
    printf("\n[TEST 6] Bohren Raster 3x2 mit T2...\n");

    ConversationalBlock bDrill(7, BlockType::Drill, "Bohren Raster");
    bDrill.toolId = 2; bDrill.startZ = 0.0; bDrill.targetZ = -8.0;
    bDrill.spindleRpm = 22000;

    DrillOperation op;
    op.type = DrillOperationType::Drill; op.toolId = 2;
    op.spindleRpm = 22000; op.plungeFeed = 400;
    op.cycleType = DrillCycleType::Standard;
    bDrill.drillOps.push_back(op);

    ConversationalBlock bPos(8, BlockType::DrillPositions, "Raster 3x2");
    bPos.drillPattern = DrillPattern::Grid;
    bPos.gridCols = 3; bPos.gridRows = 2;
    bPos.gridPitchX = 15.0; bPos.gridPitchY = 20.0;
    bPos.posX = 5.0; bPos.posY = 5.0;

    auto positions = bPos.calculateDrillPositions();
    check((int)positions.size() == 6, ("6 Raster-Positionen: " + std::to_string(positions.size())).c_str());

    for (const auto& p : positions)
        bDrill.resolvedDrillPositions.push_back({p.x, p.y});

    auto tool = findTool(tools, bDrill.toolId);
    auto tp = bDrill.generateToolpath(tool, tool, stock);
    check(tp.segments.size() > 5, "Raster-Bohr-Toolpath erzeugt");
    printf("  -> %d Segmente\n", (int)tp.segments.size());
}

// ============================================================
// Test 7: Simulation â€” PlanfrÃ¤sen + Tasche + Nut
// ============================================================
static void testSimulation(const QList<ToolDefinition>& tools, const BoundingBox& stock) {
    printf("\n[TEST 7] Simulation: PlanfrÃ¤sen + Tasche + Nut...\n");

    ConversationalBlock facing(1, BlockType::Facing, "Plan");
    facing.toolId = 3; facing.startZ = 0; facing.targetZ = -1;
    facing.stepDown = 0.5; facing.useStockDimensions = true;
    facing.spindleRpm = 12000; facing.feedRate = 3000;

    ConversationalBlock pocket(2, BlockType::Pocket, "Tasche");
    pocket.toolId = 1; pocket.posX = 20; pocket.posY = 15;
    pocket.pocketShape = PocketShape::Rectangle;
    pocket.pocketWidthX = 30; pocket.pocketDepthY = 20; pocket.pocketCornerR = 2;
    pocket.startZ = -1; pocket.targetZ = -8; pocket.stepDown = 2;
    pocket.stepOver = 2.7; pocket.pocketStrategy = 0;
    pocket.millingType = MillingType::Pocket;
    pocket.spindleRpm = 18000; pocket.feedRate = 1500;

    ConversationalBlock slot(3, BlockType::Slot, "Nut");
    slot.toolId = 2; slot.posX = 60; slot.posY = 30;
    slot.slotLength = 25; slot.slotWidth = 4; slot.slotAngleDeg = 0;
    slot.startZ = -1; slot.targetZ = -5; slot.stepDown = 1;
    slot.spindleRpm = 22000; slot.feedRate = 1200;

    auto t3 = findTool(tools, 3);
    auto t1 = findTool(tools, 1);
    auto t2 = findTool(tools, 2);

    auto tpFacing = facing.generateToolpath(t3, t3, stock);
    auto tpPocket = pocket.generateToolpath(t1, t1, stock);
    auto tpSlot   = slot.generateToolpath(t2, t2, stock);

    check(!tpFacing.segments.empty(), "Facing Toolpath OK");
    check(!tpPocket.segments.empty(), "Pocket Toolpath OK");
    check(!tpSlot.segments.empty(), "Slot Toolpath OK");

    // StockModel mit Konstruktor
    Simulation::StockModel stockModel(stock, 200);

    int totalSegs = 0;
    auto runSim = [&](const Toolpath& tp, const ToolDefinition& tool) {
        for (size_t i = 1; i < tp.segments.size(); i++) {
            const auto& p0 = tp.segments[i-1];
            const auto& p1 = tp.segments[i];
            if (p1.feedRate > 0) {
                stockModel.carveSegment(p0.endPos, p1.endPos,
                                         tool.diameter / 2.0);
            }
            totalSegs++;
        }
    };

    runSim(tpFacing, t3);
    runSim(tpPocket, t1);
    runSim(tpSlot, t2);

    printf("  -> %d Segmente simuliert\n", totalSegs);

    auto mesh = stockModel.toMesh();
    check(mesh.vertices.size() > 100, ("Mesh: " + std::to_string(mesh.vertices.size()) + " Vertices").c_str());

    QString stlPath = "e:/cnc/build/test_simulation_result.stl";
    bool saved = Geometry::StlLoader::saveBinary(stlPath, mesh);
    check(saved, "STL Export erfolgreich");

    double minH = 999;
    for (const auto& v : mesh.vertices)
        if (v.z < minH) minH = v.z;
    check(minH < -4.0, ("Material abgetragen (minZ<-4): " + std::to_string(minH)).c_str());

    printf("  -> STL: %s\n", stlPath.toUtf8().constData());
    printf("  -> Mesh: %d Vertices, %d Dreiecke, MinZ=%.2f\n",
           (int)mesh.vertices.size(), (int)mesh.triangles.size(), minH);
}

// ============================================================
// Test 8: G-Code Export (alle Postprozessoren)
// ============================================================
static void testGCodeExport(const QList<ToolDefinition>& tools, const BoundingBox& stock) {
    printf("\n[TEST 8] G-Code Export (Hurco, Heidenhain, Klipper)...\n");

    ConversationalBlock facing(1, BlockType::Facing, "Plan");
    facing.toolId = 1; facing.startZ = 0; facing.targetZ = -2;
    facing.stepDown = 1; facing.useStockDimensions = true;
    facing.areaWidth = 100; facing.areaDepth = 80;
    facing.spindleRpm = 18000; facing.feedRate = 1500;

    auto tool = findTool(tools, 1);
    auto tp = facing.generateToolpath(tool, tool, stock);

    MachineConfig mc;
    PostProcessorContext ctx;

    struct PPTest { ControllerType type; const char* name; const char* checkStr; };
    PPTest tests[] = {
        {ControllerType::Hurco_WinMax, "Hurco", "M30"},
        {ControllerType::Heidenhain, "Heidenhain", "G"},
        {ControllerType::Klipper, "Klipper", "G1"},
    };

    for (const auto& t : tests) {
        auto pp = PostProcessorFactory::create(t.type);
        auto code = pp->process(tp, mc, tools, ctx);
        int lines = code.count('\n');
        check(code.contains(t.checkStr), (std::string(t.name) + ": EnthÃ¤lt '" + t.checkStr + "'").c_str());
        check(lines > 10, (std::string(t.name) + ": " + std::to_string(lines) + " Zeilen").c_str());
        printf("  -> %s: %d Zeilen G-Code\n", t.name, lines);
    }
}

// ============================================================
// Test 9: Programm speichern/laden (JSON Round-Trip)
// ============================================================
static void testProgramRoundTrip() {
    printf("\n[TEST 9] Programm speichern/laden (JSON Round-Trip)...\n");

    ConversationalProgram prog;
    prog.programName = "TestProgramm";

    ConversationalBlock b1(1, BlockType::Facing, "PlanfrÃ¤sen");
    b1.toolId = 3; b1.targetZ = -1.5; b1.stepDown = 0.5;
    prog.addBlock(b1);

    ConversationalBlock b2(2, BlockType::Pocket, "Tasche");
    b2.toolId = 1; b2.pocketWidthX = 25; b2.pocketDepthY = 18; b2.targetZ = -6;
    prog.addBlock(b2);

    ConversationalBlock b3(3, BlockType::Slot, "Nut");
    b3.toolId = 2; b3.slotLength = 35; b3.slotWidth = 5; b3.slotAngleDeg = 30;
    prog.addBlock(b3);

    // Speichern
    QJsonObject json = prog.toJson();
    QJsonDocument doc(json);
    QString path = "e:/cnc/build/test_program.gprog";
    QFile f(path);
    if (f.open(QIODevice::WriteOnly))
        f.write(doc.toJson());
    f.close();

    // Laden
    auto loaded = ConversationalProgram::loadFromFile(path);

    check(loaded.size() == 3, ("3 BlÃ¶cke: " + std::to_string(loaded.size())).c_str());
    if (loaded.size() >= 3) {
        check(loaded.blocks[0].type == BlockType::Facing, "Block 1 = Facing");
        check(loaded.blocks[1].type == BlockType::Pocket, "Block 2 = Pocket");
        check(loaded.blocks[2].type == BlockType::Slot, "Block 3 = Slot");
        check(std::abs(loaded.blocks[1].pocketWidthX - 25.0) < 0.001, "Tasche Breite=25");
        check(std::abs(loaded.blocks[2].slotAngleDeg - 30.0) < 0.001, "Nut Winkel=30");
    }
    printf("  -> JSON Round-Trip OK\n");
}

// ============================================================
// Test 10: Materialdatenbank
// ============================================================
static void testMaterialDatabase() {
    printf("\n[TEST 10] Materialdatenbank...\n");

    MaterialDatabase db;
    auto& mats = db.materials();
    check(mats.size() >= 6, ("Mind. 6 Materialien: " + std::to_string(mats.size())).c_str());

    auto alu = db.findById(1);
    check(alu.vc > 100, ("Alu Vc > 100: " + std::to_string(alu.vc)).c_str());

    auto steel = db.findById(6);
    check(steel.vc > 0, ("Stahl Vc > 0: " + std::to_string(steel.vc)).c_str());
    check(steel.vc < alu.vc, "Stahl Vc < Alu Vc");

    double d = 6.0;
    double n = (alu.vc * 1000.0) / (M_PI * d);
    check(n > 5000, ("RPM > 5000: " + std::to_string((int)n)).c_str());
    printf("  -> Alu Vc=%.0f, 6mm: n=%.0f RPM\n", alu.vc, n);
}

// ============================================================
// Test 11: Kreis-Tasche
// ============================================================
static void testCirclePocket(const QList<ToolDefinition>& tools, const BoundingBox& stock) {
    printf("\n[TEST 11] Kreis-Tasche (Circle Pocket) mit T1...\n");
    ConversationalBlock b(10, BlockType::Pocket, "Kreistasche");
    b.toolId = 1; b.millingType = MillingType::Pocket;
    b.pocketShape = PocketShape::Circle; b.pocketRadius = 15.0;
    b.posX = 50.0; b.posY = 40.0;
    b.startZ = 0.0; b.targetZ = -6.0; b.stepDown = 2.0;
    b.stepOver = 2.7; b.pocketStrategy = 1;
    b.spindleRpm = 18000; b.feedRate = 1500;

    auto tool = findTool(tools, b.toolId);
    auto tp = b.generateToolpath(tool, tool, stock);
    int segs = (int)tp.segments.size();

    check(segs > 50, ("Kreistasche Segmente (>50): " + std::to_string(segs)).c_str());
    printf("  -> %d Segmente\n", segs);
}

// ============================================================
// MAIN
// ============================================================
int main(int argc, char* argv[]) {
    QCoreApplication app(argc, argv);

    printf("\n");
    printf("================================================================\n");
    printf("  MicBur-CNC-CAM  --  Umfassender Integrationstest\n");
    printf("  Alle Bearbeitungsarten + Simulation + Export\n");
    printf("================================================================\n");

    auto tools = buildToolLibrary();
    BoundingBox stock;
    stock.minPoint = {0, 0, -20};
    stock.maxPoint = {100, 80, 0};

    printf("\nRohteil: 100 x 80 x 20 mm (Z: 0 bis -20)\n");
    printf("Werkzeuge: T1=6mm, T2=3mm, T3=40mm Plan, T4=4mm Kugel\n");

    testFacing(tools, stock);
    testPocket(tools, stock);
    testSlot(tools, stock);
    testContour(tools, stock);
    testDrillBoltCircle(tools, stock);
    testDrillGrid(tools, stock);
    testSimulation(tools, stock);
    testGCodeExport(tools, stock);
    testProgramRoundTrip();
    testMaterialDatabase();
    testCirclePocket(tools, stock);

    printf("\n================================================================\n");
    printf("  Ergebnis: %d PASSED, %d FAILED (von %d)\n",
           g_passed, g_failed, g_passed + g_failed);
    printf("================================================================\n");

    if (g_failed > 0) {
        printf("  *** FEHLER GEFUNDEN ***\n");
        return 1;
    }
    printf("  *** ALLE TESTS BESTANDEN ***\n");
    return 0;
}
