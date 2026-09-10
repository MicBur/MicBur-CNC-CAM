#include <cassert>
#include <iostream>
#include <cmath>
#include <QApplication>
#include <QTemporaryFile>
#include <QThread>

#include "geometry/ObjLoader.h"
#include "core/MaterialDatabase.h"
#include "cam/ConversationalProgram.h"
#include "cam/ToolpathGenerator.h"
#include "hardware/ProbeController.h"
#include "hardware/MoonrakerClient.h"
#include "geometry/ContourSolver.h"
#include "ui/winmax/WinMaxSoftkeyBar.h"

using namespace GeminiCNC;

void testObjLoader() {
    std::cout << "Running testObjLoader..." << std::endl;

    // 1. Erstelle ein einfaches Dreiecks-Mesh
    Geometry::Mesh mesh(Geometry::MeshRole::TargetPart);
    mesh.name = "TestCube";
    // 4 Vertices für zwei Dreiecke (Quad)
    mesh.vertices.push_back({0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f});
    mesh.vertices.push_back({20.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f});
    mesh.vertices.push_back({20.0f, 15.0f, 5.0f, 0.0f, 0.0f, 1.0f});
    mesh.vertices.push_back({0.0f, 15.0f, 5.0f, 0.0f, 0.0f, 1.0f});

    mesh.triangles.push_back({0, 1, 2});
    mesh.triangles.push_back({0, 2, 3});
    mesh.computeBoundingBox();

    // 2. Exportieren als OBJ
    QString tmpPath = "test_mesh.obj";
    bool saved = Geometry::ObjLoader::saveToFile(tmpPath, mesh);
    assert(saved && "OBJ saveToFile failed");

    // 3. Wieder einlesen
    auto res = Geometry::ObjLoader::loadFromFile(tmpPath, Geometry::MeshRole::TargetPart);
    assert(res.success && "OBJ loadFromFile failed");
    assert(res.mesh.vertices.size() >= 4);
    assert(res.mesh.triangles.size() >= 2);
    assert(res.mesh.boundingBox.isValid());
    assert(std::abs(res.mesh.boundingBox.maxPoint.x - 20.0) < 1e-3);
    assert(std::abs(res.mesh.boundingBox.maxPoint.z - 5.0) < 1e-3);

    QFile::remove(tmpPath);
    std::cout << "testObjLoader PASSED!" << std::endl;
}

void testMaterialDatabase() {
    std::cout << "Running testMaterialDatabase..." << std::endl;

    auto db = Core::MaterialDatabase::createDefault();
    assert(db.materials().size() >= 5);

    auto alu = db.findByName("Aluminium (EN AW-6060 / 6082)");
    assert(alu.vc > 200.0);

    Core::ToolDefinition endMill6(1, "6mm Fräser", Core::ToolType::EndMill, 6.0);
    endMill6.flutes = 2;

    auto tech = Core::TechnologyCalculator::calculate(alu, endMill6);
    std::cout << "Alu tech: RPM=" << tech.spindleRpm << ", Feed=" << tech.feedRate 
              << ", Plunge=" << tech.plungeFeedRate << ", StepDown=" << tech.recommendedStepDown << std::endl;
    // n = (260 * 1000) / (pi * 6) = ~13793 U/min
    assert(tech.spindleRpm >= 10000.0 && tech.spindleRpm <= 20000.0);
    assert(tech.feedRate > 500.0);
    assert(tech.plungeFeedRate < tech.feedRate);
    assert(tech.recommendedStepDown > 0.5);

    // Test POM: Höheres Vc, höherer Vorschub
    auto pom = db.findByName("Kunststoff POM-C (Delrin)");
    auto techPom = Core::TechnologyCalculator::calculate(pom, endMill6);
    std::cout << "POM tech: RPM=" << techPom.spindleRpm << ", Feed=" << techPom.feedRate << std::endl;
    assert(techPom.spindleRpm >= tech.spindleRpm);

    std::cout << "testMaterialDatabase PASSED!" << std::endl;
}

void testConversationalProgram() {
    std::cout << "Running testConversationalProgram..." << std::endl;

    auto prog = CAM::ConversationalProgram::createSampleProgram();
    assert(prog.size() == 4);

    auto toolLib = Core::ToolDefinition::createDefaultLibrary();
    Core::BoundingBox stockBounds({-40, -30, -10}, {40, 30, 0});

    auto fullTp = prog.generateFullToolpath(toolLib, stockBounds);
    assert(!fullTp.empty());
    assert(fullTp.size() > 50);
    assert(fullTp.totalLength() > 100.0);

    // Serialisierung (.gprog)
    QString progPath = "test_prog.gprog";
    assert(prog.saveToFile(progPath));

    auto reloadedProg = CAM::ConversationalProgram::loadFromFile(progPath);
    assert(reloadedProg.size() == 4);
    assert(reloadedProg[0].type == CAM::BlockType::Facing);
    assert(reloadedProg[1].type == CAM::BlockType::Pocket);
    assert(reloadedProg[2].type == CAM::BlockType::Drill);
    assert(reloadedProg[3].type == CAM::BlockType::Contour);

    QFile::remove(progPath);
    std::cout << "testConversationalProgram PASSED!" << std::endl;
}

void test3DSurfaceFinishing() {
    std::cout << "Running test3DSurfaceFinishing..." << std::endl;

    // Erstelle ein gewölbtes 3D-Mesh
    Geometry::Mesh mesh(Geometry::MeshRole::TargetPart);
    mesh.vertices.push_back({-15.0f, -10.0f, -5.0f, 0.0f, 0.0f, 1.0f});
    mesh.vertices.push_back({15.0f, -10.0f, -5.0f, 0.0f, 0.0f, 1.0f});
    mesh.vertices.push_back({15.0f, 10.0f, 0.0f, 0.0f, 0.0f, 1.0f});
    mesh.vertices.push_back({-15.0f, 10.0f, 0.0f, 0.0f, 0.0f, 1.0f});

    mesh.triangles.push_back({0, 1, 2});
    mesh.triangles.push_back({0, 2, 3});
    mesh.computeBoundingBox();

    Core::ToolDefinition ballMill(2, "4mm Kugelkopffräser", Core::ToolType::BallMill, 4.0);

    CAM::SurfaceFinishingParams params;
    params.stepOver = 2.0;
    params.sampleStep = 1.0;
    params.clearanceZ = 5.0;

    auto tp = CAM::ToolpathGenerator::generate3DSurfaceFinishing(mesh, ballMill, params);
    assert(!tp.empty());
    assert(tp.size() > 10);
    assert(tp.totalLength() > 50.0);

    std::cout << "test3DSurfaceFinishing PASSED!" << std::endl;
}

void testProbeController() {
    std::cout << "Running testProbeController..." << std::endl;

    Hardware::MoonrakerClient client;
    client.setMockMode(true);
    client.connectToHost(QStringLiteral("127.0.0.1"), 7125);

    // Auf Mock-Verbindung warten (150ms Verzögerung)
    for (int i = 0; i < 50; ++i) {
        QCoreApplication::processEvents();
        QThread::msleep(10);
        if (client.isConnected()) break;
    }
    assert(client.isConnected());

    Hardware::ProbeController probeCtrl(&client);

    bool toolMeasuredFired = false;
    QObject::connect(&probeCtrl, &Hardware::ProbeController::toolMeasured, [&](int id, double len, double off) {
        assert(id == 1);
        assert(len > 10.0);
        toolMeasuredFired = true;
    });

    probeCtrl.startToolLengthMeasurement(1);

    // Event Loop kurz drehen lassen
    for (int i = 0; i < 60; ++i) {
        QCoreApplication::processEvents();
        QThread::msleep(10);
        if (toolMeasuredFired) break;
    }

    assert(toolMeasuredFired && "Tool measured signal should fire in mock mode");
    std::cout << "testProbeController PASSED!" << std::endl;
}

void testContourSegmentsAndPrimitives() {
    std::cout << "Running testContourSegmentsAndPrimitives..." << std::endl;

    // 1. Freie Kontur aus Segmenten (Start -> Gerade -> Bogen CW -> Gerade)
    std::vector<Geometry::ContourSegment> segs;
    segs.push_back({Geometry::ContourSegmentType::StartPoint, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    segs.push_back({Geometry::ContourSegmentType::Line, 60.0, 0.0, 0.0, 0.0, 0.0, 0.0, 60.0});
    segs.push_back({Geometry::ContourSegmentType::ArcCW, 80.0, 20.0, 0.0, 20.0, 0.0, 0.0, 0.0});
    segs.push_back({Geometry::ContourSegmentType::Line, 80.0, 50.0, 0.0, 0.0, 0.0, 0.0, 30.0});
    segs.push_back({Geometry::ContourSegmentType::Line, 0.0, 50.0, 0.0, 0.0, 0.0, 0.0, 80.0});
    segs.push_back({Geometry::ContourSegmentType::Line, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 50.0});

    auto contour = Geometry::Contour::createFromSegments(segs, true);
    assert(contour.points.size() > 10);
    assert(contour.perimeter() > 200.0);
    assert(std::abs(contour.signedArea()) > 1000.0);

    // 2. Langloch (Slot)
    auto slot = Geometry::Contour::createSlot(0.0, 0.0, 60.0, 20.0, 45.0);
    assert(slot.points.size() >= 30);
    assert(slot.perimeter() > 100.0);

    // 3. Abgerundetes Rechteck (Mill Frame)
    auto frame = Geometry::Contour::createRoundedRectangle(0.0, 0.0, 80.0, 50.0, 8.0);
    assert(frame.points.size() >= 30);

    // 4. Helix / Gewindefräspunkte
    auto helixPts = Geometry::Contour::createHelixPoints(0.0, 0.0, 0.0, -10.0, 12.0, 2.0, 32);
    assert(helixPts.size() > 100);
    assert(std::abs(helixPts.front().z - 0.0) < 1e-3);
    assert(std::abs(helixPts.back().z - (-10.0)) < 1e-3);

    // 5. Conversational Block mit Slot und Helix
    CAM::ConversationalBlock blockSlot(10, CAM::BlockType::Slot, "TestSlot");
    blockSlot.slotLength = 60.0;
    blockSlot.slotWidth = 20.0;
    Core::ToolDefinition mill(1, "6mm Fräser", Core::ToolType::EndMill, 6.0);
    Core::BoundingBox bb({-50, -50, -20}, {50, 50, 0});
    auto tpSlot = blockSlot.generateToolpath(mill, mill, bb);
    assert(!tpSlot.segments.empty());

    CAM::ConversationalBlock blockHelix(11, CAM::BlockType::HelixThread, "TestHelix");
    blockHelix.helixDiameter = 24.0;
    blockHelix.helixPitch = 1.5;
    blockHelix.targetZ = -12.0;
    auto tpHelix = blockHelix.generateToolpath(mill, mill, bb);
    assert(tpHelix.segments.size() > 50);

    std::cout << "testContourSegmentsAndPrimitives PASSED!" << std::endl;
}

void testWinMaxSoftkeys() {
    std::cout << "Running testWinMaxSoftkeys..." << std::endl;

    UI::WinMaxSoftkeyBar bar;
    assert(bar.currentMenu() == UI::SoftkeyMenu::Main);

    bool signalFired = false;
    QString capturedAction;
    QObject::connect(&bar, &UI::WinMaxSoftkeyBar::softkeyTriggered, [&](UI::SoftkeyMenu menu, int fKey, const QString& action) {
        assert(menu == UI::SoftkeyMenu::Main);
        assert(fKey == 3);
        capturedAction = action;
        signalFired = true;
    });

    bar.triggerKey(3); // F3 = Arbeitsplan
    assert(signalFired);
    assert(capturedAction == "prog");

    // Menü auf BlockEdit umschalten
    bar.setMenu(UI::SoftkeyMenu::BlockEdit);
    assert(bar.currentMenu() == UI::SoftkeyMenu::BlockEdit);

    std::cout << "testWinMaxSoftkeys PASSED!" << std::endl;
}

void testContourSolver() {
    std::cout << "Running testContourSolver..." << std::endl;
    // 1. Line Solver Test: Start (0,0), Länge 50, Winkel 0° -> Ende (50, 0)
    Geometry::LineSolveInput lIn;
    lIn.startX = 0.0;
    lIn.startY = 0.0;
    lIn.length = 50.0;
    lIn.angleDeg = 0.0;
    auto lRes = Geometry::ContourSolver::solveLine(lIn);
    assert(lRes.solved);
    assert(std::abs(lRes.endX - 50.0) < 1e-4);
    assert(std::abs(lRes.endY - 0.0) < 1e-4);

    // 2. Line Solver Test: Start (10, 10), X-End 60, Winkel 45° -> Y-End 60, Länge ~70.71
    Geometry::LineSolveInput lIn2;
    lIn2.startX = 10.0;
    lIn2.startY = 10.0;
    lIn2.endX = 60.0;
    lIn2.angleDeg = 45.0;
    auto lRes2 = Geometry::ContourSolver::solveLine(lIn2);
    assert(lRes2.solved);
    assert(std::abs(lRes2.endY - 60.0) < 1e-4);
    assert(std::abs(lRes2.length - 70.710678) < 1e-3);

    // 3. Arc Solver Test: Start (0, 0), Ende (20, 20), Radius 20, CW -> Mittelpunkt (20, 0)
    Geometry::ArcSolveInput aIn;
    aIn.startX = 0.0;
    aIn.startY = 0.0;
    aIn.endX = 20.0;
    aIn.endY = 20.0;
    aIn.radius = 20.0;
    aIn.isCW = true;
    auto aRes = Geometry::ContourSolver::solveArc(aIn);
    assert(aRes.solved);
    assert(std::abs(aRes.centerX - 20.0) < 1e-4);
    assert(std::abs(aRes.centerY - 0.0) < 1e-4);

    std::cout << "testContourSolver PASSED!" << std::endl;
}

int main(int argc, char* argv[]) {
    qputenv("QT_PLUGIN_PATH", "G:/Qt/6.12.0/mingw_64/plugins");
    qputenv("QT_QPA_PLATFORM", "offscreen");
    std::cout << "=== Running Gemini CNC WinMax Test Suite ===" << std::endl << std::flush;

    QApplication app(argc, argv);

    std::cout << "[1/8] Starting testObjLoader..." << std::endl << std::flush;
    testObjLoader();

    std::cout << "[2/8] Starting testMaterialDatabase..." << std::endl << std::flush;
    testMaterialDatabase();

    std::cout << "[3/8] Starting testConversationalProgram..." << std::endl << std::flush;
    testConversationalProgram();

    std::cout << "[4/8] Starting test3DSurfaceFinishing..." << std::endl << std::flush;
    test3DSurfaceFinishing();

    std::cout << "[5/8] Starting testProbeController..." << std::endl << std::flush;
    testProbeController();

    std::cout << "[6/8] Starting testContourSegmentsAndPrimitives..." << std::endl << std::flush;
    testContourSegmentsAndPrimitives();

    std::cout << "[7/8] Starting testContourSolver..." << std::endl << std::flush;
    testContourSolver();

    std::cout << "[8/8] Starting testWinMaxSoftkeys..." << std::endl << std::flush;
    testWinMaxSoftkeys();

    std::cout << "=== All WinMax Tests PASSED Successfully! ===" << std::endl << std::flush;
    return 0;
}
