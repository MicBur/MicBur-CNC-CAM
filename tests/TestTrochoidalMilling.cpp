// ═══════════════════════════════════════════════════════════════════════════
// TestTrochoidalMilling.cpp — Tests für trochoidale Fräsbahnen
// Programmierer: Michael Burzlaff
// ═══════════════════════════════════════════════════════════════════════════

#include "cam/ToolpathGenerator.h"
#include "cam/ConversationalBlock.h"
#include "core/MaterialDatabase.h"
#include "core/ToolDefinition.h"
#include "geometry/Contour.h"
#include <QCoreApplication>
#include <iostream>
#include <cmath>
#include <cassert>

using namespace GeminiCNC;
using namespace GeminiCNC::CAM;
using namespace GeminiCNC::Core;

static int g_passed = 0;
static int g_failed = 0;

#define TEST(name) \
    do { \
        std::cout << "  Test: " << name << " ... "; \
    } while(0)

#define PASS() \
    do { \
        std::cout << "BESTANDEN" << std::endl; \
        ++g_passed; \
    } while(0)

#define FAIL(msg) \
    do { \
        std::cout << "FEHLGESCHLAGEN: " << msg << std::endl; \
        ++g_failed; \
    } while(0)

#define EXPECT_TRUE(cond, msg) \
    if (!(cond)) { FAIL(msg); return; } 

#define EXPECT_GT(a, b, msg) \
    if (!((a) > (b))) { FAIL(msg " (erwartet " #a " > " #b ", aber " + std::to_string(a) + " <= " + std::to_string(b) + ")"); return; }

#define EXPECT_NEAR(a, b, eps, msg) \
    if (std::abs((a) - (b)) > (eps)) { FAIL(msg " (erwartet " + std::to_string(a) + " ≈ " + std::to_string(b) + ")"); return; }


// ═══ Material-Tests ═══

static void testMaterialTrochoidalDefaults() {
    TEST("Material: Trochoidale Standardwerte vorhanden");
    auto db = MaterialDatabase::createDefault();

    // Alu: 15%
    auto alu = db.findById(1);
    EXPECT_NEAR(alu.trochoidalEngagement, 0.15, 0.001, "Alu ae/d");
    EXPECT_GT(alu.trochoidalFeedFactor, 1.0, "Alu Feed-Faktor");

    // Messing: 12%
    auto brass = db.findById(3);
    EXPECT_NEAR(brass.trochoidalEngagement, 0.12, 0.001, "Messing ae/d");

    // Baustahl: 8%
    auto steel = db.findById(6);
    EXPECT_NEAR(steel.trochoidalEngagement, 0.08, 0.001, "Stahl ae/d");

    PASS();
}

static void testMaterialTrochoidalEngagementVaries() {
    TEST("Material: ae/d variiert je Werkstoff");
    auto db = MaterialDatabase::createDefault();
    auto alu = db.findById(1);
    auto steel = db.findById(6);

    // Alu muss deutlich mehr Eingriff haben als Stahl
    EXPECT_TRUE(alu.trochoidalEngagement > steel.trochoidalEngagement,
                "Alu ae > Stahl ae");
    EXPECT_TRUE(alu.trochoidalEngagement >= 0.12, "Alu min 12%");
    EXPECT_TRUE(steel.trochoidalEngagement <= 0.10, "Stahl max 10%");

    PASS();
}

static void testMaterialSerialization() {
    TEST("Material: JSON-Serialisierung trochoidaler Felder");
    Material m;
    m.id = 99;
    m.name = "TestMaterial";
    m.trochoidalEngagement = 0.07;
    m.trochoidalFeedFactor = 2.3;

    auto json = m.toJson();
    Material restored = Material::fromJson(json);
    EXPECT_NEAR(restored.trochoidalEngagement, 0.07, 0.001, "ae roundtrip");
    EXPECT_NEAR(restored.trochoidalFeedFactor, 2.3, 0.001, "feed roundtrip");

    PASS();
}


// ═══ TechnologyCalculator-Tests ═══

static void testCalculateTrochoidal() {
    TEST("TechCalc: Trochoidale Schnittdaten");
    auto db = MaterialDatabase::createDefault();
    auto alu = db.findById(1);

    ToolDefinition tool;
    tool.id = 1;
    tool.diameter = 6.0;
    tool.flutes = 3;
    tool.fluteLength = 12.0;
    tool.defaultFeedRate = 800.0;
    tool.plungeFeedRate = 250.0;
    tool.spindleSpeed = 15000.0;

    auto params = TechnologyCalculator::calculateTrochoidal(alu, tool);

    // ae muss kleiner sein als konventionell
    auto conv = TechnologyCalculator::calculate(alu, tool);
    EXPECT_TRUE(params.recommendedStepOver < conv.recommendedStepOver,
                "Trochoidales ae < konventionelles ae");

    // ae = d * engagement = 6 * 0.15 = 0.9
    EXPECT_NEAR(params.recommendedStepOver, 0.9, 0.1, "ae = d * 15%");

    // Vorschub muss höher sein als konventionell
    EXPECT_GT(params.feedRate, conv.feedRate, "Trochoidaler Vorschub höher");

    // ap muss fast volle Schneidlänge sein
    EXPECT_GT(params.recommendedStepDown, 5.0, "Volle Tiefe");

    PASS();
}


// ═══ Slot-Tests ═══

static void testTrochoidalSlotBasic() {
    TEST("Slot: Grundlegende trochoidale Nut");
    ToolDefinition tool;
    tool.id = 1;
    tool.diameter = 6.0;
    tool.flutes = 3;
    tool.fluteLength = 12.0;
    tool.defaultFeedRate = 800.0;
    tool.plungeFeedRate = 250.0;
    tool.spindleSpeed = 15000.0;

    auto tp = ToolpathGenerator::generateTrochoidalSlot(
        0.0, 0.0,    // start
        50.0, 0.0,   // end
        10.0,         // slotWidth
        tool,
        0.0, -5.0, 5.0,  // startZ, targetZ, clearanceZ
        0.15, 2.0,        // engagement, feedFactor
        true);            // climbMilling

    EXPECT_GT(tp.segments.size(), 50u, "Genug Segmente erzeugt");

    // Alle Segmente müssen gültige Positionen haben
    for (const auto& seg : tp.segments) {
        EXPECT_TRUE(std::isfinite(seg.endPos.x), "endPos.x endlich");
        EXPECT_TRUE(std::isfinite(seg.endPos.y), "endPos.y endlich");
        EXPECT_TRUE(std::isfinite(seg.endPos.z), "endPos.z endlich");
    }

    // Erstes Segment startet am Anfang
    EXPECT_NEAR(tp.segments.front().startPos.x, 0.0, 1.0, "Start X");

    // Letztes Segment auf clearanceZ
    EXPECT_NEAR(tp.segments.back().endPos.z, 5.0, 0.1, "Rückzug auf clearanceZ");

    PASS();
}

static void testTrochoidalSlotFeedMultiplier() {
    TEST("Slot: Trochoidaler Vorschub > Basisvorschub");
    ToolDefinition tool;
    tool.id = 1;
    tool.diameter = 6.0;
    tool.flutes = 3;
    tool.fluteLength = 12.0;
    tool.defaultFeedRate = 800.0;
    tool.plungeFeedRate = 250.0;
    tool.spindleSpeed = 15000.0;

    auto tp = ToolpathGenerator::generateTrochoidalSlot(
        0.0, 0.0, 30.0, 0.0, 10.0,
        tool, 0.0, -3.0, 5.0,
        0.10, 2.5, true);

    // Mindestens ein Feed-Segment mit erhöhtem Vorschub
    bool foundHighFeed = false;
    for (const auto& seg : tp.segments) {
        if (seg.motion == MotionType::LinearFeed && seg.feedRate > tool.defaultFeedRate * 1.5) {
            foundHighFeed = true;
            break;
        }
    }
    EXPECT_TRUE(foundHighFeed, "Trochoidaler Vorschub erhöht");
    PASS();
}

static void testTrochoidalSlotAngled() {
    TEST("Slot: Trochoidale Nut im 45°-Winkel");
    ToolDefinition tool;
    tool.id = 1;
    tool.diameter = 6.0;
    tool.flutes = 3;
    tool.fluteLength = 12.0;
    tool.defaultFeedRate = 800.0;
    tool.plungeFeedRate = 250.0;
    tool.spindleSpeed = 15000.0;

    // 45°-Nut: (0,0) -> (30, 30)
    auto tp = ToolpathGenerator::generateTrochoidalSlot(
        0.0, 0.0, 30.0, 30.0, 10.0,
        tool, 0.0, -5.0, 5.0,
        0.12, 2.0, true);

    EXPECT_GT(tp.segments.size(), 30u, "Genug Segmente (angewinkelt)");
    PASS();
}


// ═══ Pocket-Tests ═══

static void testTrochoidalPocketBasic() {
    TEST("Pocket: Trochoidale Tasche 40x30");
    ToolDefinition tool;
    tool.id = 1;
    tool.diameter = 6.0;
    tool.flutes = 3;
    tool.fluteLength = 12.0;
    tool.defaultFeedRate = 800.0;
    tool.plungeFeedRate = 250.0;
    tool.spindleSpeed = 15000.0;

    Geometry::Contour boundary = Geometry::Contour::createRectangle(0.0, 0.0, 40.0, 30.0);

    auto tp = ToolpathGenerator::generateTrochoidalPocket(
        boundary, tool,
        0.0, -5.0, 5.0,
        0.15, 2.0, true);

    EXPECT_GT(tp.segments.size(), 100u, "Genug Segmente (Tasche)");

    // Alle Segmente innerhalb oder nahe der Tasche
    for (const auto& seg : tp.segments) {
        if (seg.motion == MotionType::Rapid) continue;
        EXPECT_TRUE(seg.endPos.x > -10.0 && seg.endPos.x < 50.0, "X in Taschenbereich");
        EXPECT_TRUE(seg.endPos.y > -10.0 && seg.endPos.y < 40.0, "Y in Taschenbereich");
    }

    PASS();
}

static void testTrochoidalPocketCircular() {
    TEST("Pocket: Trochoidale kreisförmige Tasche");
    ToolDefinition tool;
    tool.id = 1;
    tool.diameter = 6.0;
    tool.flutes = 3;
    tool.fluteLength = 12.0;
    tool.defaultFeedRate = 800.0;
    tool.plungeFeedRate = 250.0;
    tool.spindleSpeed = 15000.0;

    // Kreisförmige Tasche Ø30
    Geometry::Contour boundary;
    constexpr double kPi = 3.14159265358979323846;
    for (int i = 0; i < 36; ++i) {
        const double a = 2.0 * kPi * i / 36.0;
        boundary.addPoint(15.0 * std::cos(a), 15.0 * std::sin(a));
    }
    boundary.isClosed = true;

    auto tp = ToolpathGenerator::generateTrochoidalPocket(
        boundary, tool,
        0.0, -4.0, 5.0,
        0.10, 2.0, true);

    EXPECT_GT(tp.segments.size(), 50u, "Genug Segmente (Kreistasche)");
    PASS();
}


// ═══ Block-Integration-Tests ═══

static void testBlockSlotTrochoidal() {
    TEST("Block: Slot mit useTrochoidal erzeugt trochoidale Bahnen");

    ConversationalBlock block;
    block.id = 1;
    block.type = BlockType::Slot;
    block.name = "Trochoidal-Nut";
    block.enabled = true;
    block.toolId = 1;
    block.materialId = 1;  // Aluminium
    block.startZ = 0.0;
    block.targetZ = -5.0;
    block.clearanceZ = 5.0;
    block.posX = 0.0;
    block.posY = 0.0;
    block.slotLength = 40.0;
    block.slotWidth = 10.0;
    block.slotAngleDeg = 0.0;
    block.millingDirection = 0;
    block.useTrochoidal = true;
    block.trochoidEngagement = 0.0;  // auto aus Material
    block.trochoidFeedFactor = 0.0;  // auto aus Material

    ToolDefinition tool;
    tool.id = 1;
    tool.diameter = 6.0;
    tool.flutes = 3;
    tool.fluteLength = 12.0;
    tool.defaultFeedRate = 800.0;
    tool.plungeFeedRate = 250.0;
    tool.spindleSpeed = 15000.0;

    ToolDefinition finishTool = tool;
    finishTool.id = 2;

    BoundingBox stock(Vector3D(-50, -40, -10), Vector3D(50, 40, 0));
    Geometry::Mesh emptyMesh;
    QList<ToolDefinition> toolLib;
    toolLib.append(tool);

    auto tp = block.generateToolpath(tool, finishTool, stock, emptyMesh, toolLib);

    EXPECT_GT(tp.segments.size(), 50u, "Trochoidale Nut hat Segmente");
    PASS();
}

static void testBlockPocketTrochoidalStrategy3() {
    TEST("Block: Pocket pocketStrategy=3 erzeugt trochoidale Bahnen");

    ConversationalBlock block;
    block.id = 2;
    block.type = BlockType::Pocket;
    block.name = "Trochoidal-Tasche";
    block.enabled = true;
    block.toolId = 1;
    block.materialId = 1;  // Aluminium
    block.startZ = 0.0;
    block.targetZ = -5.0;
    block.clearanceZ = 5.0;
    block.millingType = MillingType::Pocket;
    block.pocketShape = PocketShape::Rectangle;
    block.pocketWidthX = 40.0;
    block.pocketDepthY = 30.0;
    block.millingDirection = 0;
    block.pocketStrategy = 3;  // Trochoidal

    ToolDefinition tool;
    tool.id = 1;
    tool.diameter = 6.0;
    tool.flutes = 3;
    tool.fluteLength = 12.0;
    tool.defaultFeedRate = 800.0;
    tool.plungeFeedRate = 250.0;
    tool.spindleSpeed = 15000.0;

    ToolDefinition finishTool = tool;
    finishTool.id = 2;

    BoundingBox stock(Vector3D(-50, -40, -10), Vector3D(50, 40, 0));
    Geometry::Mesh emptyMesh;
    QList<ToolDefinition> toolLib;
    toolLib.append(tool);

    auto tp = block.generateToolpath(tool, finishTool, stock, emptyMesh, toolLib);

    EXPECT_GT(tp.segments.size(), 100u, "Trochoidale Tasche hat Segmente");
    PASS();
}


// ═══ Serialisierungs-Test ═══

static void testBlockTrochoidalSerialization() {
    TEST("Block: JSON-Serialisierung trochoidaler Felder");

    ConversationalBlock block;
    block.id = 10;
    block.type = BlockType::Slot;
    block.useTrochoidal = true;
    block.trochoidEngagement = 0.08;
    block.trochoidFeedFactor = 1.9;
    block.trochoidFullDepth = false;

    auto json = block.toJson();
    auto restored = ConversationalBlock::fromJson(json);

    EXPECT_TRUE(restored.useTrochoidal, "useTrochoidal roundtrip");
    EXPECT_NEAR(restored.trochoidEngagement, 0.08, 0.001, "engagement roundtrip");
    EXPECT_NEAR(restored.trochoidFeedFactor, 1.9, 0.01, "feedFactor roundtrip");
    EXPECT_TRUE(!restored.trochoidFullDepth, "fullDepth roundtrip");

    PASS();
}


// ═══ Vergleichstest: Trochoidal vs. Konventionell ═══

static void testTrochoidalVsConventionalDifference() {
    TEST("Vergleich: Trochoidale Nut vs. konventionelle Taschenfräsung");

    ToolDefinition tool;
    tool.id = 1;
    tool.diameter = 6.0;
    tool.flutes = 3;
    tool.fluteLength = 12.0;
    tool.defaultFeedRate = 800.0;
    tool.plungeFeedRate = 250.0;
    tool.spindleSpeed = 15000.0;

    // Konventionelle Nut (über PocketMilling)
    Geometry::Contour slotContour = Geometry::Contour::createSlot(0.0, 0.0, 40.0, 10.0, 0.0);
    PocketParams pp;
    pp.startZ = 0.0;
    pp.targetZ = -5.0;
    pp.stepDown = 2.0;
    pp.stepOverRatio = 0.5;
    pp.clearanceZ = 5.0;
    auto convTp = ToolpathGenerator::generatePocketMilling(slotContour, tool, pp);

    // Trochoidale Nut
    auto trochTp = ToolpathGenerator::generateTrochoidalSlot(
        -20.0, 0.0, 20.0, 0.0, 10.0,
        tool, 0.0, -5.0, 5.0,
        0.10, 2.0, true);

    // Beide müssen Segmente haben
    EXPECT_GT(convTp.segments.size(), 10u, "Konventionell hat Segmente");
    EXPECT_GT(trochTp.segments.size(), 10u, "Trochoidal hat Segmente");

    // Trochoidal hat typischerweise MEHR Segmente (viele kleine Bögen)
    EXPECT_TRUE(trochTp.segments.size() > convTp.segments.size() * 0.5,
                "Trochoidal erzeugt mehr/ähnlich viele Bahnsegmente");

    std::cout << "[Konventionell: " << convTp.segments.size()
              << " Segmente, Trochoidal: " << trochTp.segments.size()
              << " Segmente] ";

    PASS();
}


int main(int argc, char* argv[]) {
    QCoreApplication app(argc, argv);

    std::cout << "\n══════════════════════════════════════════════" << std::endl;
    std::cout << " MicBur-CNC-CAM — Trochoidales Fräsen Tests" << std::endl;
    std::cout << "══════════════════════════════════════════════\n" << std::endl;

    // Material-Tests
    std::cout << "--- Material & Technologie ---" << std::endl;
    testMaterialTrochoidalDefaults();
    testMaterialTrochoidalEngagementVaries();
    testMaterialSerialization();
    testCalculateTrochoidal();

    // Slot-Tests
    std::cout << "\n--- Trochoidale Nut ---" << std::endl;
    testTrochoidalSlotBasic();
    testTrochoidalSlotFeedMultiplier();
    testTrochoidalSlotAngled();

    // Pocket-Tests
    std::cout << "\n--- Trochoidale Tasche ---" << std::endl;
    testTrochoidalPocketBasic();
    testTrochoidalPocketCircular();

    // Block-Integration
    std::cout << "\n--- Block-Integration ---" << std::endl;
    testBlockSlotTrochoidal();
    testBlockPocketTrochoidalStrategy3();
    testBlockTrochoidalSerialization();

    // Vergleich
    std::cout << "\n--- Vergleich ---" << std::endl;
    testTrochoidalVsConventionalDifference();

    // Ergebnis
    std::cout << "\n══════════════════════════════════════════════" << std::endl;
    std::cout << " Ergebnis: " << g_passed << " BESTANDEN, " << g_failed << " FEHLGESCHLAGEN" << std::endl;
    std::cout << "══════════════════════════════════════════════\n" << std::endl;

    return g_failed > 0 ? 1 : 0;
}
