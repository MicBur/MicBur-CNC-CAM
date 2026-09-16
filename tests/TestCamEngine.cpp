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
#include "cam/ConversationalProgram.h"
#include "simulation/StockModel.h"
#include <algorithm>
#include <set>
#include "geometry/StlLoader.h"
#include <QDir>
#include <QRegularExpression>
#include <QStringList>
#include <cstdlib>

using namespace GeminiCNC;

// assert() ist im Release-Build (-DNDEBUG) wirkungslos → harte Prüfung
static void require(bool cond, const char* msg) {
    if (!cond) {
        std::cout << " -> FAILED: " << msg << std::endl;
        std::exit(1);
    }
}

void testStockStlRoundTrip() {
    std::cout << "[TEST] Fräsergebnis STL-Export → als Rohteil laden..." << std::endl;
    Core::BoundingBox stockBounds({-50, -40, -10}, {50, 40, 0});

    Simulation::StockModel carved(stockBounds);
    carved.carveSegment({-20, 0, -5}, {20, 0, -5}, 5.0);   // Nut 5 mm tief
    carved.carveSegment({30, 20, -15}, {30, 20, -15}, 6.0); // Durchbruch (unter Boden)

    int throughPoints = 0;
    for (float z : carved.heightField) {
        require(z >= -10.0f - 1e-4f, "Heightfield liegt unter Rohteilboden");
        if (z <= -10.0f + 1e-4f) ++throughPoints;
    }
    require(throughPoints > 0, "Durchbruch wurde nicht erzeugt");

    const QString path = QDir::temp().filePath(QStringLiteral("gemini_stock_roundtrip.stl"));
    require(Geometry::StlLoader::saveBinary(path, carved.toMesh()), "STL konnte nicht geschrieben werden");

    auto loaded = Geometry::StlLoader::loadFromFile(path, Geometry::MeshRole::Stock);
    QFile::remove(path);
    require(loaded.success, "STL konnte nicht geladen werden");

    Simulation::StockModel reloaded;
    reloaded.initFromMesh(loaded.mesh);
    require(reloaded.resX == carved.resX && reloaded.resY == carved.resY, "Auflösung weicht ab");

    // Einzelne Randpunkte ohne angrenzende Fläche können beim Export entfallen → kleine Toleranz
    int mismatched = 0;
    double maxDiff = 0.0;
    for (size_t i = 0; i < carved.layerCount.size(); ++i) {
        if (carved.layerCount[i] != reloaded.layerCount[i]) { ++mismatched; continue; }
        maxDiff = std::max(maxDiff, static_cast<double>(std::abs(carved.heightField[i] - reloaded.heightField[i])));
    }
    require(mismatched <= static_cast<int>(carved.layerCount.size() / 200), "Schichtanzahl nach dem Laden weicht ab");
    require(maxDiff < 1e-3, "Geladene Rohteilform weicht vom Fräsergebnis ab");
    const double volCarved = carved.materialVolume();
    const double volReloaded = reloaded.materialVolume();
    require(std::abs(volCarved - volReloaded) / volCarved < 0.01, "Volumen nach dem Laden weicht ab");

    // Reset muss die geladene Form wiederherstellen, nicht den vollen Block
    const auto loadedHeights = reloaded.heightField;
    reloaded.carveSegment({0, -30, -8}, {0, 30, -8}, 3.0);
    reloaded.reset();
    require(reloaded.heightField == loadedHeights, "Reset stellt geladene Form nicht wieder her");

    std::cout << " -> PASSED (max. Abweichung " << maxDiff << " mm, abweichende Punkte " << mismatched
              << ", Durchbruch-Punkte " << throughPoints << ")" << std::endl;
}

void testTiltedAndCylinderStockVolume() {
    std::cout << "[TEST] Gekipptes Rohteil & Zylinder: Unterseite bleibt frei..." << std::endl;
    constexpr int L = Simulation::StockModel::kMaxLayers;

    auto box = Geometry::Mesh::createBoxStock(80.0, 50.0, 15.0, {0.0, 0.0, -15.0});
    Simulation::StockModel flat;
    flat.initFromMesh(box);
    const double boxVolume = 80.0 * 50.0 * 15.0;
    require(std::abs(flat.materialVolume() - boxVolume) / boxVolume < 0.01, "Volumen des geraden Quaders falsch");

    box.rotateX(150.0, true);
    Simulation::StockModel tilted;
    tilted.initFromMesh(box);
    const double tiltedVolume = tilted.materialVolume();
    require(std::abs(tiltedVolume - boxVolume) / boxVolume < 0.03, "Volumen des gekippten Quaders falsch (Unterseite aufgefüllt?)");

    const float floorZ = static_cast<float>(tilted.bounds.minPoint.z);
    int floating = 0;
    for (size_t p = 0; p < tilted.layerCount.size(); ++p) {
        if (tilted.layerCount[p] > 0 && tilted.layerLo[p * L] > floorZ + 1.0f) ++floating;
    }
    require(floating > 0, "Unter dem Überhang liegt Material bis zum Boden");

    const double r = 20.0, h = 60.0;
    auto cyl = Geometry::Mesh::createCylinderStock(r, h, 48, 1, false);
    Simulation::StockModel cylStock;
    cylStock.initFromMesh(cyl);
    const double cylVolume = 3.14159265358979 * r * r * h;
    require(std::abs(cylStock.materialVolume() - cylVolume) / cylVolume < 0.03, "Volumen des liegenden Zylinders falsch");

    // Schnitt von oben durch den höchsten Punkt (First) des gekippten Teils muss Material entfernen
    const double before = tilted.materialVolume();
    const auto& b = tilted.bounds;
    const size_t topIdx = static_cast<size_t>(std::max_element(tilted.heightField.begin(), tilted.heightField.end()) - tilted.heightField.begin());
    const double topY = b.minPoint.y + static_cast<double>(topIdx / tilted.resX) * b.depthY() / (tilted.resY - 1);
    const double cutZ = tilted.heightField[topIdx] - 3.0;
    tilted.carveSegment({b.minPoint.x - 5, topY, cutZ}, {b.maxPoint.x + 5, topY, cutZ}, 10.0);
    require(tilted.materialVolume() < before - 1.0, "Abtrag am gekippten Teil wirkungslos");

    std::cout << " -> PASSED (Quader gekippt " << tiltedVolume << " / " << boxVolume
              << " mm3, schwebende Punkte " << floating << ")" << std::endl;
}

using PosKey = std::pair<long, long>;

static PosKey posKey(double x, double y) {
    return {std::lround(x * 10.0), std::lround(y * 10.0)};
}

// XY-Position jeder Eintauchbewegung auf Zieltiefe (eine je Bohrung)
static std::vector<PosKey> plungeBottoms(const CAM::Toolpath& tp, double targetZ) {
    std::vector<PosKey> pts;
    for (const auto& s : tp.segments) {
        if (s.motion != CAM::MotionType::Rapid && s.endPos.z <= targetZ + 0.01 && s.startPos.z > targetZ + 0.01) {
            pts.push_back(posKey(s.endPos.x, s.endPos.y));
        }
    }
    return pts;
}

// Umlaufsinn der Vorschubbahn auf Höhe z (Fläche > 0 = gegen den Uhrzeigersinn)
static double feedLoopArea(const CAM::Toolpath& tp, double z) {
    std::vector<std::pair<double, double>> pts;
    for (const auto& s : tp.segments) {
        if (s.motion != CAM::MotionType::Rapid && std::abs(s.startPos.z - z) < 1e-6 && std::abs(s.endPos.z - z) < 1e-6) {
            pts.push_back({s.endPos.x, s.endPos.y});
        }
    }
    double area = 0.0;
    for (size_t i = 0; i < pts.size(); ++i) {
        const auto& a = pts[i];
        const auto& b = pts[(i + 1) % pts.size()];
        area += a.first * b.second - b.first * a.second;
    }
    return 0.5 * area;
}

void testMachiningOptionsAffectToolpath() {
    std::cout << "[TEST] Bearbeitungsoptionen wirken auf die Fräsbahn..." << std::endl;
    Core::ToolDefinition tool(1, QStringLiteral("6mm Schaftfräser"), Core::ToolType::EndMill, 6.0);
    tool.defaultFeedRate = 1000.0;
    tool.plungeFeedRate = 300.0;
    const Core::BoundingBox stock({-60, -60, -10}, {100, 100, 0});

    // ── Kontur: Gleichlauf / Gegenlauf ──
    const auto rect = Geometry::Contour::createRectangle(-20.0, -15.0, 40.0, 30.0);
    CAM::ContourParams cp;
    cp.startZ = 0.0;  cp.targetZ = -5.0;  cp.stepDown = 1.0;
    cp.side = CAM::ContourSide::Outside;  cp.finishAllowance = 0.0;  cp.clearanceZ = 5.0;
    cp.climbMilling = true;
    const double areaClimb = feedLoopArea(CAM::ToolpathGenerator::generateContourMilling(rect, tool, cp), -5.0);
    cp.climbMilling = false;
    const double areaConv = feedLoopArea(CAM::ToolpathGenerator::generateContourMilling(rect, tool, cp), -5.0);
    require(areaClimb < 0.0 && areaConv > 0.0, "Gleichlauf außen muss im Uhrzeigersinn fahren, Gegenlauf gegen den Uhrzeigersinn");
    cp.climbMilling = true;

    // ── Haltestege ──
    cp.useTabs = true;  cp.tabCount = 4;  cp.tabWidth = 5.0;  cp.tabHeight = 1.3;
    int tabMoves = 0;
    for (const auto& s : CAM::ToolpathGenerator::generateContourMilling(rect, tool, cp).segments) {
        if (s.motion != CAM::MotionType::Rapid && std::abs(s.startPos.z + 3.7) < 1e-6 && std::abs(s.endPos.z + 3.7) < 1e-6) ++tabMoves;
    }
    require(tabMoves >= 8, "Haltestege fehlen (erwartet: 4 Stege in 2 Ebenen unter der Steghöhe)");
    cp.useTabs = false;

    // ── Tangentialbogen: Eintauchen neben der Kontur ──
    cp.leadType = 1;  cp.leadRadius = 4.0;
    bool plungeOutside = false;
    for (const auto& s : CAM::ToolpathGenerator::generateContourMilling(rect, tool, cp).segments) {
        if (s.motion != CAM::MotionType::Rapid && s.endPos.z < s.startPos.z - 1e-6
            && std::hypot(s.endPos.x - s.startPos.x, s.endPos.y - s.startPos.y) < 1e-6) {
            plungeOutside = std::abs(s.endPos.x) > 24.0 || std::abs(s.endPos.y) > 19.0;
            break;
        }
    }
    require(plungeOutside, "Tangentialbogen: Eintauchen muss neben der Kontur erfolgen");
    cp.leadType = 0;

    // ── Rampe: kein senkrechtes Eintauchen ins Material ──
    cp.entryType = 2;
    bool ramped = false;
    bool verticalInMaterial = false;
    for (const auto& s : CAM::ToolpathGenerator::generateContourMilling(rect, tool, cp).segments) {
        if (s.motion == CAM::MotionType::Rapid || s.endPos.z >= s.startPos.z - 1e-6) continue;
        const double dxy = std::hypot(s.endPos.x - s.startPos.x, s.endPos.y - s.startPos.y);
        if (dxy > 0.5) ramped = true;
        if (dxy < 1e-6 && s.endPos.z < -0.01) verticalInMaterial = true;
    }
    require(ramped && !verticalInMaterial, "Rampe: Zustellung muss schräg entlang der Bahn erfolgen");
    cp.entryType = 0;

    // ── Tasche mit Insel: alle Strategien bleiben im zulässigen Bereich ──
    const auto pocket = Geometry::Contour::createRectangle(-30.0, -20.0, 60.0, 40.0);
    const auto island = Geometry::Contour::createRectangle(-5.0, -5.0, 10.0, 10.0);
    CAM::PocketParams pp;
    pp.startZ = 0.0;  pp.targetZ = -2.0;  pp.stepDown = 1.0;  pp.stepOverRatio = 0.5;
    pp.finishAllowance = 0.0;  pp.clearanceZ = 5.0;
    std::vector<size_t> segmentCounts;
    for (int strategy = 0; strategy <= 2; ++strategy) {
        pp.strategy = strategy;
        pp.entryType = (strategy == 1) ? 1 : 2;
        const auto ptp = CAM::ToolpathGenerator::generatePocketMilling(pocket, {island}, tool, pp);
        require(!ptp.empty(), "Taschenbahn leer");
        segmentCounts.push_back(ptp.size());
        for (const auto& s : ptp.segments) {
            if (s.motion == CAM::MotionType::Rapid) continue;
            for (double t : {0.0, 0.5, 1.0}) {
                const double x = s.startPos.x + t * (s.endPos.x - s.startPos.x);
                const double y = s.startPos.y + t * (s.endPos.y - s.startPos.y);
                const double z = s.startPos.z + t * (s.endPos.z - s.startPos.z);
                if (z > -0.01) continue; // über dem Material
                require(std::abs(x) <= 27.02 && std::abs(y) <= 17.02, "Taschenbahn verletzt die Taschenwand");
                const double dx = std::max(0.0, std::abs(x) - 5.0);
                const double dy = std::max(0.0, std::abs(y) - 5.0);
                require(std::hypot(dx, dy) >= 2.98, "Taschenbahn verletzt die Insel");
            }
        }
    }
    require(segmentCounts[0] != segmentCounts[1] || segmentCounts[1] != segmentCounts[2], "Räumstrategien erzeugen identische Bahnen");

    // ── Bohrzyklen ──
    CAM::ConversationalBlock drill(1, CAM::BlockType::Drill, QStringLiteral("Bohrung"));
    drill.drillPattern = CAM::DrillPattern::Single;
    drill.startZ = 0.0;  drill.targetZ = -6.0;  drill.peckDepth = 2.0;  drill.clearanceZ = 5.0;  drill.toolId = 1;
    auto drillStats = [&](int cycle, int& pecks, int& rapidsToR, bool& feedOut) {
        drill.drillCycle = cycle;
        pecks = 0;  rapidsToR = 0;  feedOut = false;
        for (const auto& s : drill.generateToolpath(tool, tool, stock).segments) {
            const double z = s.endPos.z;
            const bool down = z < s.startPos.z - 1e-6;
            if (s.motion != CAM::MotionType::Rapid && down
                && (std::abs(z + 2.0) < 1e-6 || std::abs(z + 4.0) < 1e-6 || std::abs(z + 6.0) < 1e-6)) ++pecks;
            if (s.motion == CAM::MotionType::Rapid && std::abs(z - 1.0) < 1e-6) ++rapidsToR;
            if (s.motion != CAM::MotionType::Rapid && z > s.startPos.z + 1e-6) feedOut = true;
        }
    };
    int pecks = 0, rapidsToR = 0;
    bool feedOut = false;
    drillStats(0, pecks, rapidsToR, feedOut);
    require(pecks == 1 && !feedOut, "G81: genau eine Bohrbewegung erwartet");
    drillStats(1, pecks, rapidsToR, feedOut);
    const int g83Retracts = rapidsToR;
    require(pecks == 3, "G83: drei Zustellungen erwartet");
    drillStats(2, pecks, rapidsToR, feedOut);
    require(pecks == 3 && rapidsToR < g83Retracts, "G73: Spanbruch ohne Rückzug zur R-Ebene erwartet");
    drillStats(4, pecks, rapidsToR, feedOut);
    require(pecks == 1 && feedOut, "G85: Rückzug im Vorschub erwartet");

    // ── Langloch: Position und Wiederholung ──
    CAM::ConversationalBlock slot(2, CAM::BlockType::Slot, QStringLiteral("Nut"));
    slot.posX = 40.0;  slot.posY = 10.0;  slot.slotLength = 30.0;  slot.slotWidth = 10.0;  slot.slotAngleDeg = 0.0;
    slot.slotCount = 3;  slot.slotSpacing = 20.0;  slot.millingType = CAM::MillingType::OnContour;
    slot.startZ = 0.0;  slot.targetZ = -1.0;  slot.stepDown = 1.0;  slot.leadType = 0;  slot.approachType = 0;
    double minX = 1e9, maxX = -1e9, minY = 1e9, maxY = -1e9;
    for (const auto& s : slot.generateToolpath(tool, tool, stock).segments) {
        if (s.motion == CAM::MotionType::Rapid || s.endPos.z > -0.5) continue;
        minX = std::min(minX, s.endPos.x);  maxX = std::max(maxX, s.endPos.x);
        minY = std::min(minY, s.endPos.y);  maxY = std::max(maxY, s.endPos.y);
    }
    require(std::abs(0.5 * (minX + maxX) - 40.0) < 0.5, "Langloch: X-Position wird nicht verwendet");
    require(maxY - minY > 49.0, "Langloch: Wiederholungen fehlen");

    // ── Helix: Position, Drehrichtung, Gänge ──
    CAM::ConversationalBlock helix(3, CAM::BlockType::HelixThread, QStringLiteral("Gewinde"));
    helix.posX = 10.0;  helix.posY = -10.0;  helix.helixDiameter = 20.0;  helix.helixPitch = 1.5;  helix.helixInternal = true;
    helix.startZ = 0.0;  helix.targetZ = -3.0;  helix.clearanceZ = 5.0;
    auto helixStats = [&](double& turnSign, int& plunges, double& radiusError) {
        turnSign = 0.0;  plunges = 0;  radiusError = 0.0;
        for (const auto& s : helix.generateToolpath(tool, tool, stock).segments) {
            if (s.motion == CAM::MotionType::Rapid) continue;
            if (std::abs(s.startPos.z - 5.0) < 1e-6 && s.endPos.z < 5.0) { ++plunges; continue; }
            const double ax = s.startPos.x - 10.0, ay = s.startPos.y + 10.0;
            const double bx = s.endPos.x - 10.0, by = s.endPos.y + 10.0;
            turnSign += ax * by - ay * bx;
            radiusError = std::max(radiusError, std::abs(std::hypot(bx, by) - 7.0));
        }
    };
    double turnSign = 0.0, radiusError = 0.0;
    int plunges = 0;
    helix.helixCW = true;
    helix.helixStarts = 1;
    helixStats(turnSign, plunges, radiusError);
    require(turnSign < 0.0 && radiusError < 1e-4, "Helix: Rechtsgewinde muss im Uhrzeigersinn um die Position laufen");
    helix.helixCW = false;
    helixStats(turnSign, plunges, radiusError);
    require(turnSign > 0.0 && plunges == 1, "Helix: Linksgewinde muss gegen den Uhrzeigersinn laufen");
    helix.helixStarts = 2;
    helixStats(turnSign, plunges, radiusError);
    require(plunges == 2, "Helix: zweigängig erwartet zwei Einstiege");

    // ── Schnittwerte und Kühlmittel aus dem Block ──
    CAM::ConversationalBlock face(4, CAM::BlockType::Facing, QStringLiteral("Planen"));
    face.feedRate = 777.0;  face.plungeFeedRate = 222.0;  face.spindleRpm = 9000.0;  face.coolantOn = true;
    face.startZ = 0.0;  face.targetZ = -1.0;  face.stepDown = 1.0;  face.useStockDimensions = true;
    const auto faceTp = face.generateToolpath(tool, tool, stock);
    bool hasFeed = false, hasPlunge = false, rpmOk = true, coolantOk = true;
    for (const auto& s : faceTp.segments) {
        if (s.motion != CAM::MotionType::Rapid && s.feedRate == 777.0) hasFeed = true;
        if (s.motion != CAM::MotionType::Rapid && s.feedRate == 222.0) hasPlunge = true;
        rpmOk = rpmOk && s.spindleRpm == 9000.0;
        coolantOk = coolantOk && s.coolantOn;
    }
    require(hasFeed && hasPlunge && rpmOk && coolantOk, "Block-Schnittwerte oder Kühlmittel werden nicht verwendet");
    const QString gcode = faceTp.exportWithPostProcessor(0, Core::MachineConfig(), {tool});
    require(gcode.contains(QStringLiteral("M08")) && gcode.contains(QStringLiteral("M09")), "G-Code: Kühlmittel M08/M09 fehlt");

    std::cout << " -> PASSED" << std::endl;
}

void testGCodeExportArcsCyclesOffsets() {
    std::cout << "[TEST] G-Code: Kreisbögen, Bohrzyklen, Nullpunkt, Längenkorrektur..." << std::endl;
    Core::ToolDefinition tool(1, QStringLiteral("6mm Schaftfräser"), Core::ToolType::EndMill, 6.0);
    tool.defaultFeedRate = 1000.0;
    tool.plungeFeedRate = 300.0;
    const QList<Core::ToolDefinition> tools{tool};
    const Core::BoundingBox stock({-60, -60, -10}, {60, 60, 0});
    Core::MachineConfig machine;
    machine.workOffset = 2; // G56

    // ── Kreis innen fräsen → Bogen-Erkennung ──
    CAM::ConversationalBlock circle(1, CAM::BlockType::Pocket, QStringLiteral("Kreis"));
    circle.pocketShape = CAM::PocketShape::Circle;
    circle.pocketRadius = 20.0;
    circle.millingType = CAM::MillingType::Inside;
    circle.startZ = 0.0;  circle.targetZ = -2.0;  circle.stepDown = 2.0;
    circle.leadType = 0;  circle.approachType = 0;  circle.coolantOn = false;  circle.finishPass = false;
    const auto circleTp = circle.generateToolpath(tool, tool, stock);
    const QString iso = circleTp.exportWithPostProcessor(0, machine, tools);

    // Bögen nachrechnen: Start und Ende gleich weit vom Mittelpunkt (I/J relativ zum Start)
    const QRegularExpression wordRe(QStringLiteral("([XYIJ])(-?\\d+(?:\\.\\d+)?)"));
    double curX = 0.0, curY = 0.0;
    int arcCount = 0;
    for (const QString& rawLine : iso.split(QLatin1Char('\n'))) {
        const QString line = rawLine.section(QLatin1Char(';'), 0, 0);
        double x = curX, y = curY, i = 0.0, j = 0.0;
        bool hasI = false, hasJ = false;
        auto it = wordRe.globalMatch(line);
        while (it.hasNext()) {
            const auto m = it.next();
            const double v = m.captured(2).toDouble();
            const QString w = m.captured(1);
            if (w == QStringLiteral("X")) x = v;
            else if (w == QStringLiteral("Y")) y = v;
            else if (w == QStringLiteral("I")) { i = v; hasI = true; }
            else if (w == QStringLiteral("J")) { j = v; hasJ = true; }
        }
        if (hasI && hasJ) {
            ++arcCount;
            const double cx = curX + i, cy = curY + j;
            require(std::abs(std::hypot(curX - cx, curY - cy) - std::hypot(x - cx, y - cy)) < 0.02,
                    "G-Code: Kreisbogen mit ungültigem Mittelpunkt");
        }
        curX = x;
        curY = y;
    }
    require(arcCount >= 2, "G-Code: Kreis wird nicht als G2/G3 ausgegeben");
    require(iso.contains(QStringLiteral("G56")), "G-Code: Nullpunkt G56 fehlt");
    require(iso.contains(QStringLiteral("G43 H1")), "G-Code: Werkzeuglängenkorrektur fehlt");

    const QString klipperCircle = circleTp.exportWithPostProcessor(3, machine, tools);
    require(!QRegularExpression(QStringLiteral("\\sI-?\\d")).match(klipperCircle).hasMatch(),
            "Klipper: Kreisbögen dürfen nicht ausgegeben werden");
    const QString hurco = circleTp.exportWithPostProcessor(1, machine, tools);
    require(hurco.contains(QStringLiteral(" G56 ")) && !hurco.contains(QStringLiteral(" G54 ")),
            "Hurco: Nullpunkt aus der Maschinenkonfiguration fehlt");

    // ── Bohrzyklus G83 statt Einzelbewegungen ──
    CAM::ConversationalBlock drill(2, CAM::BlockType::Drill, QStringLiteral("Lochkreis"));
    drill.drillPattern = CAM::DrillPattern::BoltCircle;
    drill.boltCircleHoleCount = 6;  drill.boltCircleRadius = 25.0;
    drill.drillCycle = 1;  drill.peckDepth = 2.0;
    drill.startZ = 0.0;  drill.targetZ = -6.0;  drill.coolantOn = false;
    const auto drillTp = drill.generateToolpath(tool, tool, stock);
    const QString drillIso = drillTp.exportWithPostProcessor(0, machine, tools);
    require(drillIso.contains(QStringLiteral("G98 G83")) && drillIso.contains(QStringLiteral("Q2.000"))
            && drillIso.contains(QStringLiteral("G80")), "G-Code: G83-Zyklus fehlt");
    const QStringList lines = drillIso.split(QLatin1Char('\n'));
    int cycleStart = -1, cycleEnd = -1;
    for (int k = 0; k < lines.size(); ++k) {
        if (cycleStart < 0 && lines[k].contains(QStringLiteral("G83"))) cycleStart = k;
        if (cycleStart >= 0 && k > cycleStart && lines[k].contains(QStringLiteral("G80"))) { cycleEnd = k; break; }
    }
    require(cycleStart >= 0 && cycleEnd - cycleStart - 1 == 5, "G-Code: G83-Zyklus muss alle 6 Bohrungen enthalten");
    require(!drillIso.contains(QStringLiteral("Z-4.000")), "G-Code: Einzelbewegungen des Zyklus dürfen nicht zusätzlich erscheinen");

    const QString drillKlipper = drillTp.exportWithPostProcessor(3, machine, tools);
    require(!drillKlipper.contains(QStringLiteral("G83")) && drillKlipper.contains(QStringLiteral("Z-4.000")),
            "Klipper: Bohrung muss als Einzelbewegungen ausgegeben werden");

    std::cout << " -> PASSED (" << arcCount << " Kreisbögen)" << std::endl;
}

void testContourArcsAndSegmentDepth() {
    std::cout << "[TEST] Kontur: Kreisbogen mit Mittelpunkt und Tiefe je Segment..." << std::endl;
    using ST = Geometry::ContourSegmentType;

    // 40 × 20 mit Halbkreis rechts (Mittelpunkt vorgegeben); obere Kante 2 mm tiefer
    std::vector<Geometry::ContourSegment> segs(5);
    segs[0].type = ST::StartPoint;  segs[0].x = 0.0;   segs[0].y = 0.0;   segs[0].z = -2.0;  segs[0].zStart = 0.0;
    segs[1].type = ST::Line;        segs[1].x = 40.0;  segs[1].y = 0.0;   segs[1].z = -2.0;
    segs[2].type = ST::ArcCCW;      segs[2].x = 40.0;  segs[2].y = 20.0;  segs[2].z = -4.0;
    segs[2].hasCenter = true;       segs[2].centerX = 40.0;  segs[2].centerY = 10.0;
    segs[3].type = ST::Line;        segs[3].x = 0.0;   segs[3].y = 20.0;  segs[3].z = -4.0;
    segs[4].type = ST::Line;        segs[4].x = 0.0;   segs[4].y = 0.0;   segs[4].z = -2.0;

    std::vector<double> z;
    const auto c = Geometry::Contour::createFromSegments(segs, true, &z);
    require(z.size() == c.points.size(), "Tiefe je Konturpunkt fehlt");
    int arcPoints = 0;
    for (size_t k = 0; k < c.points.size(); ++k) {
        const auto& p = c.points[k];
        if (p.x > 40.0 + 1e-6) {
            ++arcPoints;
            require(std::abs(std::hypot(p.x - 40.0, p.y - 10.0) - 10.0) < 1e-6, "Bogenpunkt liegt nicht auf dem Kreis");
            require(z[k] < -2.0 && z[k] > -4.0 - 1e-9, "Bogentiefe wird nicht interpoliert");
        }
    }
    require(arcPoints >= 8, "Bogen zu grob oder falsche Drehrichtung");

    CAM::ConversationalBlock stored(9, CAM::BlockType::Contour, QStringLiteral("Profil"));
    stored.segments = segs;
    const auto reloaded = CAM::ConversationalBlock::fromJson(stored.toJson());
    require(reloaded.segments.size() == segs.size() && reloaded.segments[2].hasCenter
            && reloaded.segments[2].centerX == 40.0 && reloaded.segments[2].centerY == 10.0,
            "Bogenmittelpunkt wird nicht gespeichert");

    // Fräsbahn folgt dem Tiefenprofil
    CAM::ConversationalBlock block(1, CAM::BlockType::Contour, QStringLiteral("Profil"));
    block.segments = segs;
    block.contourSide = CAM::ContourSide::OnLine;
    block.startZ = 0.0;  block.targetZ = -2.0;  block.stepDown = 1.0;
    block.leadType = 0;  block.approachType = 0;  block.finishPass = false;
    Core::ToolDefinition tool(1, QStringLiteral("6mm"), Core::ToolType::EndMill, 6.0);
    const Core::BoundingBox stock({-20, -20, -10}, {70, 40, 0});
    double minZ = 0.0;
    bool shallowEdgeOk = true;
    bool deepEdgeReached = false;
    for (const auto& s : block.generateToolpath(tool, tool, stock).segments) {
        if (s.motion == CAM::MotionType::Rapid) continue;
        minZ = std::min(minZ, s.endPos.z);
        const bool onBottomEdge = std::abs(s.startPos.y) < 1e-6 && std::abs(s.endPos.y) < 1e-6;
        if (onBottomEdge && (s.startPos.z < -2.0 - 1e-6 || s.endPos.z < -2.0 - 1e-6)) shallowEdgeOk = false;
        if (std::abs(s.endPos.y - 20.0) < 1e-6 && std::abs(s.endPos.z + 4.0) < 1e-6) deepEdgeReached = true;
    }
    require(std::abs(minZ + 4.0) < 1e-6, "Tiefe je Segment: tiefster Punkt muss -4 mm sein");
    require(shallowEdgeOk, "Tiefe je Segment: Unterkante (Z -2) wird zu tief gefräst");
    require(deepEdgeReached, "Tiefe je Segment: Oberkante (Z -4) wird nicht erreicht");

    std::cout << " -> PASSED" << std::endl;
}

void testBlockSerializationComplete() {
    std::cout << "[TEST] Programm speichern: alle Blockparameter bleiben erhalten..." << std::endl;
    CAM::ConversationalBlock b(7, CAM::BlockType::Pocket, QStringLiteral("Tasche komplett"));
    b.visible = false;
    b.millingType = CAM::MillingType::Outside;
    b.startSide = CAM::FrameStartSide::Left;
    b.posX = 12.5;  b.posY = -7.25;
    b.millingDirection = 1;  b.coolantOn = false;  b.finishPass = true;  b.finishStepDown = 0.35;  b.approachType = 2;
    b.areaWidth = 123.0;  b.areaDepth = 45.0;  b.useStockDimensions = false;
    b.contourSide = CAM::ContourSide::Inside;  b.finishAllowance = 0.4;  b.leadRadius = 3.5;  b.leadType = 2;
    b.useTabs = true;  b.tabWidth = 6.5;  b.tabHeight = 1.5;  b.tabCount = 5;
    b.pocketCornerR = 4.0;  b.pocketStrategy = 2;  b.enableFinishing = true;
    b.finishAllowanceXY = 0.3;  b.finishAllowanceZ = 0.1;  b.finishFeedRate = 900.0;  b.finishSpindleRpm = 15000.0;
    b.slotCornerR = 2.0;  b.slotCount = 3;  b.slotSpacing = 18.0;
    b.helixCW = false;  b.helixStarts = 2;
    b.drillCycle = 3;  b.dwellTimeSec = 1.25;

    Geometry::ContourSegment s0;
    s0.type = Geometry::ContourSegmentType::StartPoint;  s0.x = 1.0;  s0.y = 2.0;
    Geometry::ContourSegment s1;
    s1.type = Geometry::ContourSegmentType::Line;  s1.x = 10.0;  s1.y = 2.0;
    b.pocketIslands = {{s0, s1}, {s1}};

    const auto l = CAM::ConversationalBlock::fromJson(b.toJson());
    require(!l.visible && l.millingType == CAM::MillingType::Outside && l.startSide == CAM::FrameStartSide::Left, "Sichtbarkeit/Fräsart/Startseite verloren");
    require(l.posX == 12.5 && l.posY == -7.25, "Position X/Y verloren");
    require(l.millingDirection == 1 && !l.coolantOn && l.finishPass && l.finishStepDown == 0.35 && l.approachType == 2, "Bearbeitungsoptionen verloren");
    require(l.areaWidth == 123.0 && l.areaDepth == 45.0 && !l.useStockDimensions, "Planfräs-Fläche verloren");
    require(l.contourSide == CAM::ContourSide::Inside && l.finishAllowance == 0.4 && l.leadRadius == 3.5 && l.leadType == 2, "Kontur-Parameter verloren");
    require(l.useTabs && l.tabWidth == 6.5 && l.tabHeight == 1.5 && l.tabCount == 5, "Haltestege verloren");
    require(l.pocketCornerR == 4.0 && l.pocketStrategy == 2 && l.enableFinishing, "Taschen-Parameter verloren");
    require(l.finishAllowanceXY == 0.3 && l.finishAllowanceZ == 0.1 && l.finishFeedRate == 900.0 && l.finishSpindleRpm == 15000.0, "Schlicht-Parameter verloren");
    require(l.pocketIslands.size() == 2 && l.pocketIslands[0].size() == 2 && l.pocketIslands[0][1].x == 10.0, "Inseln verloren");
    require(l.slotCornerR == 2.0 && l.slotCount == 3 && l.slotSpacing == 18.0, "Langloch-Parameter verloren");
    require(!l.helixCW && l.helixStarts == 2 && l.drillCycle == 3 && l.dwellTimeSec == 1.25, "Helix/Bohrzyklus verloren");

    // Ältere Dateien ohne neue Schlüssel: Standardwerte bleiben
    const auto old = CAM::ConversationalBlock::fromJson(QJsonObject{{QStringLiteral("type"), QStringLiteral("Pocket")}});
    require(old.visible && old.coolantOn && old.tabCount == 4, "Standardwerte bei alten Dateien falsch");

    std::cout << " -> PASSED" << std::endl;
}

void testToolLibraryPersistence() {
    std::cout << "[TEST] Werkzeugbibliothek speichern/laden..." << std::endl;
    auto tools = Core::ToolDefinition::createDefaultLibrary();
    Core::ToolDefinition custom(12, QStringLiteral("10mm Schruppfräser 3-Schneider"), Core::ToolType::EndMill, 10.0);
    custom.stickOutLength = 42.0;
    custom.holderDiameter = 40.0;
    custom.flutes = 3;
    custom.color = QStringLiteral("#FF00AA");
    tools.append(custom);

    const QString path = QDir::temp().filePath(QStringLiteral("gemini_tool_library_test.json"));
    require(Core::ToolDefinition::saveLibrary(path, tools, 12), "Werkzeugbibliothek konnte nicht gespeichert werden");

    QList<Core::ToolDefinition> loaded;
    int activeId = -1;
    require(Core::ToolDefinition::loadLibrary(path, loaded, activeId), "Werkzeugbibliothek konnte nicht geladen werden");
    QFile::remove(path);

    require(loaded.size() == tools.size() && activeId == 12, "Anzahl oder aktives Werkzeug falsch");
    const auto& t = loaded.last();
    require(t.id == 12 && t.name == custom.name && t.type == Core::ToolType::EndMill && t.diameter == 10.0, "Eigenes Werkzeug falsch geladen");
    require(t.stickOutLength == 42.0 && t.holderDiameter == 40.0 && t.flutes == 3 && t.color == custom.color, "Werkzeuggeometrie falsch geladen");
    require(loaded[2].type == Core::ToolType::FaceMill && loaded[2].diameter == 40.0, "Planfräser falsch geladen");

    QList<Core::ToolDefinition> untouched = tools;
    int untouchedId = 99;
    require(!Core::ToolDefinition::loadLibrary(path, untouched, untouchedId) && untouched.size() == tools.size() && untouchedId == 99,
            "Fehlende Datei darf die Bibliothek nicht verändern");

    std::cout << " -> PASSED" << std::endl;
}

void testContourStartDepth() {
    std::cout << "[TEST] Kontur: Tiefe aus Segment 0 gilt für Folgesegmente..." << std::endl;
    using ST = Geometry::ContourSegmentType;

    std::vector<Geometry::ContourSegment> segs(4);
    segs[0].type = ST::StartPoint; segs[0].z = -5.0;
    segs[1].type = ST::Line;       segs[1].x = 50.0; segs[1].z = -5.0;
    segs[2].type = ST::ArcCW;      segs[2].x = 50.0; segs[2].y = 30.0; segs[2].radius = 15.0; segs[2].z = -2.0; // eigene Tiefe
    segs[3].type = ST::Line;       segs[3].z = -5.0;

    Geometry::Contour::applyStartDepth(segs, 1.0, -8.0);
    require(segs[0].zStart == 1.0 && segs[0].z == -8.0, "Segment 0: Z START / Z UNTEN nicht gesetzt");
    require(segs[1].z == -8.0 && segs[3].z == -8.0, "Folgesegmente übernehmen die neue Tiefe nicht");
    require(segs[2].z == -2.0, "Individuell geänderte Segmenttiefe wurde überschrieben");

    CAM::ConversationalBlock block(1, CAM::BlockType::Contour, QStringLiteral("Kontur"));
    block.segments = segs;
    const auto loaded = CAM::ConversationalBlock::fromJson(block.toJson());
    require(loaded.segments.size() == 4, "Segmente nicht gespeichert");
    require(loaded.segments[0].zStart == 1.0 && loaded.segments[0].z == -8.0, "Z START / Z UNTEN nicht gespeichert");
    require(loaded.segments[2].z == -2.0, "Segmenttiefe nicht gespeichert");

    std::cout << " -> PASSED" << std::endl;
}

void testPatternBlocks() {
    std::cout << "[TEST] Muster Start / Muster Ende..." << std::endl;
    auto tools = Core::ToolDefinition::createDefaultLibrary();
    Core::BoundingBox stock({-100, -100, -10}, {150, 100, 0});

    CAM::ConversationalBlock drill(2, CAM::BlockType::Drill, QStringLiteral("Bohrung"));
    drill.drillPattern = CAM::DrillPattern::Single;
    drill.posX = 20.0;
    drill.posY = 10.0;
    drill.toolId = 2;
    drill.targetZ = -5.0;

    CAM::ConversationalProgram single;
    single.addBlock(drill);
    const auto base = plungeBottoms(single.generateFullToolpath(tools, stock), drill.targetZ);
    require(base.size() == 1, "Einzelbohrung ohne Muster: genau eine Eintauchbewegung erwartet");
    const double bx = base.front().first / 10.0;
    const double by = base.front().second / 10.0;

    CAM::ConversationalBlock start(1, CAM::BlockType::PatternStart, QStringLiteral("Muster"));
    CAM::ConversationalBlock end(3, CAM::BlockType::PatternEnd, QStringLiteral("Muster Ende"));
    auto runPattern = [&](const CAM::ConversationalBlock& patternBlock, bool withEnd, bool withAfter) {
        CAM::ConversationalProgram prog;
        prog.addBlock(patternBlock);
        prog.addBlock(drill);
        if (withEnd) prog.addBlock(end);
        if (withAfter) prog.addBlock(drill); // Block nach Muster Ende darf nicht wiederholt werden
        return plungeBottoms(prog.generateFullToolpath(tools, stock), drill.targetZ);
    };
    auto asSet = [](const std::vector<PosKey>& v) { return std::set<PosKey>(v.begin(), v.end()); };

    // Linear: 3 × 30 mm in X, danach ein Block außerhalb des Musters
    start.patternType = CAM::PatternType::Linear;
    start.patternCountX = 3;
    start.patternSpacingX = 30.0;
    start.patternAngleDeg = 0.0;
    auto lin = runPattern(start, true, true);
    require(lin.size() == 4, "Lineares Muster: 3 Wiederholungen + 1 Block nach Muster Ende erwartet");
    require(asSet(lin) == std::set<PosKey>{posKey(bx, by), posKey(bx + 30, by), posKey(bx + 60, by)}, "Lineares Muster: falsche Positionen");

    // Ohne Muster Ende gilt das Muster bis Programmende
    require(runPattern(start, false, true).size() == 6, "Fehlendes Muster Ende: Muster muss bis Programmende gelten");

    // Deaktiviertes Muster: Blöcke genau einmal
    auto disabled = start;
    disabled.enabled = false;
    require(runPattern(disabled, true, false).size() == 1, "Deaktiviertes Muster darf nicht wiederholen");

    // Raster 2 × 2
    auto rect = start;
    rect.patternType = CAM::PatternType::Rectangular;
    rect.patternCountX = 2;
    rect.patternCountY = 2;
    rect.patternSpacingX = 15.0;
    rect.patternSpacingY = 25.0;
    require(asSet(runPattern(rect, true, false)) == std::set<PosKey>{posKey(bx, by), posKey(bx + 15, by), posKey(bx, by + 25), posKey(bx + 15, by + 25)},
            "Rastermuster: falsche Positionen");

    // Kreis: 4 × gleichmäßig um (0,0)
    auto circ = start;
    circ.patternType = CAM::PatternType::Circular;
    circ.patternCountX = 4;
    circ.patternCenterX = 0.0;
    circ.patternCenterY = 0.0;
    circ.patternAngleDeg = 0.0;
    circ.patternStepAngleDeg = 0.0;
    require(asSet(runPattern(circ, true, false)) == std::set<PosKey>{posKey(bx, by), posKey(-by, bx), posKey(-bx, -by), posKey(by, -bx)},
            "Kreismuster: falsche Positionen");

    // Spiegeln an beiden Achsen um (0,0)
    auto mirror = start;
    mirror.patternType = CAM::PatternType::Mirror;
    mirror.patternMirrorX = true;
    mirror.patternMirrorY = true;
    mirror.patternCenterX = 0.0;
    mirror.patternCenterY = 0.0;
    require(asSet(runPattern(mirror, true, false)) == std::set<PosKey>{posKey(bx, by), posKey(-bx, by), posKey(bx, -by), posKey(-bx, -by)},
            "Spiegelmuster: falsche Positionen");

    // Speichern/Laden der Musterparameter
    CAM::ConversationalProgram saved;
    saved.addBlock(circ);
    saved.addBlock(drill);
    saved.addBlock(end);
    const QString path = QDir::temp().filePath(QStringLiteral("gemini_pattern_test.gprog"));
    require(saved.saveToFile(path), "Programm mit Muster konnte nicht gespeichert werden");
    auto loaded = CAM::ConversationalProgram::loadFromFile(path);
    QFile::remove(path);
    require(loaded.size() == 3, "Geladenes Programm: Blockanzahl falsch");
    require(loaded[0].type == CAM::BlockType::PatternStart && loaded[2].type == CAM::BlockType::PatternEnd, "Geladenes Programm: Mustertypen falsch");
    require(loaded[0].patternType == CAM::PatternType::Circular && loaded[0].patternCountX == 4, "Geladenes Programm: Musterparameter falsch");
    require(loaded[1].type == CAM::BlockType::Drill && loaded[1].name == drill.name, "Geladenes Programm: Bohrblock falsch");

    std::cout << " -> PASSED" << std::endl;
}

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
    testStockStlRoundTrip();
    testTiltedAndCylinderStockVolume();
    testPatternBlocks();
    testContourStartDepth();
    testBlockSerializationComplete();
    testToolLibraryPersistence();
    testMachiningOptionsAffectToolpath();
    testGCodeExportArcsCyclesOffsets();
    testContourArcsAndSegmentDepth();
    std::cout << "=== All CAM Tests PASSED ===" << std::endl;
    return 0;
}
