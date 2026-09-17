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
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
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

void testToolMarksAndShading() {
    std::cout << "[TEST] Darstellung: Fräserspuren und Umgebungsverdeckung..." << std::endl;
    const Core::BoundingBox bounds({0, 0, -10}, {60, 40, 0});
    Simulation::StockModel stock(bounds);
    const double dx = bounds.widthX() / (stock.resX - 1);
    const double dy = bounds.depthY() / (stock.resY - 1);
    auto indexAt = [&](double x, double y) {
        const long i = std::lround(x / dx);
        const long j = std::lround(y / dy);
        return static_cast<size_t>(j) * stock.resX + static_cast<size_t>(i);
    };

    // Nut in X-Richtung, 5 mm tief, Fräser-Radius 5, Vorschub 0,4 mm/U
    stock.carveSegment({10, 20, -3}, {50, 20, -3}, 5.0, QColor(255, 230, 20), 1, 0.4);
    const size_t inside = indexAt(30.0, 22.0);
    const auto& mark = stock.marks[inside];
    const double px = std::lround(30.0 / dx) * dx;
    const double py = std::lround(22.0 / dy) * dy;
    require(mark.kind == 1, "Fräserspur in der Nut fehlt");
    require(std::abs(mark.dirX - 1.0f) < 1e-6 && std::abs(mark.dirY) < 1e-6, "Vorschubrichtung der Spur falsch");
    require(std::abs(mark.u - px) < 1e-3 && std::abs(mark.d - (py - 20.0)) < 1e-3, "Spurkoordinaten falsch");
    require(std::abs(mark.radius - 5.0f) < 1e-6 && std::abs(mark.pitch - 0.4f) < 1e-6, "Spur: Radius oder Vorschub falsch");
    require(stock.marks[indexAt(30.0, 35.0)].kind == 0, "Ungefräste Fläche darf keine Spur haben");

    // Schlichtgang auf gleicher Tiefe quer dazu: Spur wird ohne weiteren Abtrag überschrieben
    stock.carveSegment({30, 12, -3}, {30, 28, -3}, 3.0, QColor(255, 230, 20), 2, 0.2);
    require(stock.marks[inside].kind == 2 && std::abs(stock.marks[inside].dirY - 1.0f) < 1e-6,
            "Schlichtgang muss die Spur überschreiben");

    // Eintauchen: Ringspur
    stock.carveSegment({15, 8, -2}, {15, 8, -2}, 2.0, QColor(255, 230, 20), 1, 0.3);
    require(stock.marks[indexAt(15.0, 8.0)].kind == 3, "Eintauchen muss eine Ringspur erzeugen");

    const auto surface = stock.buildSurface();
    require(surface.shading.size() == surface.vertices.size(), "Darstellungsdaten passen nicht zur Oberfläche");

    float floorNearWallAo = 1.0f;
    float openTopAo = 0.0f;
    bool cutVertexFound = false;
    for (size_t k = 0; k < surface.vertices.size(); ++k) {
        const auto& v = surface.vertices[k];
        const auto& sh = surface.shading[k];
        if (std::abs(v.z + 3.0f) < 1e-4f && std::abs(v.x - 45.0f) < 1.0f && std::abs(v.y - 24.5f) < 0.3f && v.nz > 0.5f) {
            floorNearWallAo = std::min(floorNearWallAo, sh.ao);
            cutVertexFound = cutVertexFound || sh.kind > 0.5f;
        }
        if (std::abs(v.z) < 1e-4f && v.x < 3.0f && v.y > 36.0f && v.nz > 0.5f) {
            openTopAo = std::max(openTopAo, sh.ao);
        }
    }
    require(cutVertexFound, "Oberfläche trägt die Fräserspur nicht");
    require(floorNearWallAo < 0.95f, "Umgebungsverdeckung am Wandfuß fehlt");
    require(openTopAo > 0.98f, "Offene Oberfläche darf nicht verdeckt sein");

    std::cout << " -> PASSED (Verdeckung Wandfuß " << floorNearWallAo << ")" << std::endl;
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

void testHurcoDrillingAndContourRoles() {
    std::cout << "[TEST] Hurco: Bohrungen + Bohrpositionen, Kontur-Tasche mit Insel..." << std::endl;
    auto tools = Core::ToolDefinition::createDefaultLibrary();
    Core::BoundingBox stock({-100, -100, -20}, {150, 100, 0});
    Core::MachineConfig machine;

    // ── Bohrbild folgt X/Y (Fehler: Bohrbild ließ sich nicht verschieben) ──
    CAM::ConversationalBlock legacy(1, CAM::BlockType::Drill, QStringLiteral("Lochkreis alt"));
    legacy.drillPattern = CAM::DrillPattern::BoltCircle;
    legacy.boltCircleHoleCount = 4;
    legacy.boltCircleRadius = 10.0;
    legacy.posX = 30.0;
    legacy.posY = -5.0;
    legacy.targetZ = -5.0;
    legacy.toolId = 1;
    CAM::ConversationalProgram legacyProg;
    legacyProg.addBlock(legacy);
    const auto legacyHoles = plungeBottoms(legacyProg.generateFullToolpath(tools, stock), -5.0);
    require(legacyHoles.size() == 4, "Lochkreis: 4 Bohrungen erwartet");
    for (const auto& [kx, ky] : legacyHoles) {
        require(std::abs(std::hypot(kx / 10.0 - 30.0, ky / 10.0 + 5.0) - 10.0) < 0.15, "Bohrbild liegt nicht um X/Y des Blocks");
    }

    // ── Bohrungen: Zentrieren → Tieflochbohren → Gewinde, Positionen aus zwei Bohrpositionen-Blöcken ──
    CAM::ConversationalBlock holes(2, CAM::BlockType::Drill, QStringLiteral("Bohrungen"));
    holes.startZ = 0.0;
    holes.targetZ = -12.0;
    auto center = CAM::DrillOperation::createDefault(CAM::DrillOperationType::NcSpotDrill, 2);
    center.diameter = 4.0;
    center.tipAngleDeg = 90.0;
    auto drill = CAM::DrillOperation::createDefault(CAM::DrillOperationType::PeckDrill, 1);
    drill.peckDepth = 4.0;
    auto tap = CAM::DrillOperation::createDefault(CAM::DrillOperationType::Tap, 3);
    tap.spindleRpm = 400.0;
    tap.threadPitch = 1.25;
    tap.ownDepth = true;
    tap.depthZ = -10.0;
    holes.drillOps = {center, drill, tap};

    CAM::ConversationalBlock list(3, CAM::BlockType::DrillPositions, QStringLiteral("Liste"));
    list.drillPattern = CAM::DrillPattern::Manual;
    list.manualPositions = {{10.0, 10.0}, {20.0, 10.0}};
    CAM::ConversationalBlock circle(4, CAM::BlockType::DrillPositions, QStringLiteral("Teilkreis"));
    circle.drillPattern = CAM::DrillPattern::BoltCircle;
    circle.boltCircleHoleCount = 3;
    circle.boltCircleRadius = 15.0;
    circle.posX = 60.0;
    circle.posY = 20.0;

    CAM::ConversationalProgram prog;
    prog.addBlock(holes);
    prog.addBlock(list);
    prog.addBlock(circle);
    require(prog.resolvedBlock(0).resolvedDrillPositions.size() == 5, "Bohrungen: 5 Positionen aus den Folgeblöcken erwartet");

    const auto tp = prog.generateFullToolpath(tools, stock);
    auto bottoms = [&tp](int toolId, double z) {
        std::set<PosKey> keys;
        for (const auto& s : tp.segments) {
            if (s.toolId == toolId && s.motion != CAM::MotionType::Rapid
                && std::abs(s.endPos.z - z) < 1e-6 && s.startPos.z > z + 1e-6) {
                keys.insert(posKey(s.endPos.x, s.endPos.y));
            }
        }
        return keys;
    };
    const auto spotted = bottoms(2, -2.0);
    require(spotted.size() == 5, "NC-Anbohren: Tiefe aus Ø4 mm / 90° = 2 mm an allen Positionen");
    require(spotted.count(posKey(10.0, 10.0)) && spotted.count(posKey(20.0, 10.0)) && spotted.count(posKey(75.0, 20.0)),
            "Bohrpositionen (Liste / Teilkreis um Mitte X/Y) falsch");
    require(bottoms(1, -12.0).size() == 5, "Tieflochbohren muss bis Z UNTEN bohren");
    require(bottoms(3, -10.0).size() == 5, "Gewinde: eigene Tiefe nicht verwendet");
    bool tapFeedOk = false;
    for (const auto& s : tp.segments) {
        if (s.toolId == 3 && s.motion == CAM::MotionType::LinearFeed) tapFeedOk = std::abs(s.feedRate - 500.0) < 1e-6;
    }
    require(tapFeedOk, "Gewinde: Vorschub muss Drehzahl × Steigung sein");

    const QString iso = tp.exportWithPostProcessor(0, machine, tools);
    const auto t2 = iso.indexOf(QStringLiteral("T2 M06"));
    const auto t1 = iso.indexOf(QStringLiteral("T1 M06"));
    const auto t3 = iso.indexOf(QStringLiteral("T3 M06"));
    require(t2 >= 0 && t1 > t2 && t3 > t1, "Werkzeugwechsel in der Reihenfolge der Bohrvorgänge fehlen");
    require(iso.contains(QStringLiteral("G98 G81")) && iso.contains(QStringLiteral("G98 G83")) && iso.contains(QStringLiteral("G98 G84")),
            "G-Code: Zyklen G81/G83/G84 fehlen");
    require(iso.count(QStringLiteral(" G80 ")) == 3, "G-Code: jeder Bohrvorgang braucht einen eigenen Zyklus");

    CAM::ConversationalProgram noPositions;
    noPositions.addBlock(holes);
    require(noPositions.generateFullToolpath(tools, stock).empty(), "Bohrungen ohne Bohrpositionen dürfen nicht bohren");

    // Speichern / Laden der Bohrvorgänge
    const auto reloaded = CAM::ConversationalBlock::fromJson(holes.toJson());
    require(reloaded.drillOps.size() == 3 && reloaded.drillOps[2].type == CAM::DrillOperationType::Tap
            && reloaded.drillOps[2].ownDepth && std::abs(reloaded.drillOps[2].threadPitch - 1.25) < 1e-9
            && std::abs(reloaded.drillOps[0].diameter - 4.0) < 1e-9,
            "Bohrvorgänge nicht vollständig gespeichert");

    // ── Ältere .gprog-Datei: Zyklus + Bohrbild in einem Block → Bohrungen + Bohrpositionen ──
    QJsonObject legacyJson = legacy.toJson();
    legacyJson.remove(QStringLiteral("drillOps"));
    legacyJson[QStringLiteral("drillCycle")] = 1;
    QJsonObject root;
    root[QStringLiteral("programName")] = QStringLiteral("alt");
    root[QStringLiteral("blocks")] = QJsonArray{legacyJson};
    const QString legacyPath = QDir::temp().filePath(QStringLiteral("gemini_legacy_drill.gprog"));
    {
        QFile f(legacyPath);
        require(f.open(QIODevice::WriteOnly), "Testdatei nicht schreibbar");
        f.write(QJsonDocument(root).toJson());
    }
    const auto upgraded = CAM::ConversationalProgram::loadFromFile(legacyPath);
    QFile::remove(legacyPath);
    require(upgraded.size() == 2 && upgraded[0].type == CAM::BlockType::Drill && upgraded[1].type == CAM::BlockType::DrillPositions,
            "Alter Bohrblock muss in Bohrungen + Bohrpositionen aufgeteilt werden");
    require(upgraded[0].drillOps.size() == 1 && upgraded[0].drillOps[0].cycleType == CAM::DrillCycleType::DeepHole,
            "Alter Bohrzyklus G83 nicht übernommen");
    auto upgradedHoles = plungeBottoms(upgraded.generateFullToolpath(tools, stock), -5.0);
    auto expectedHoles = legacyHoles;
    std::sort(upgradedHoles.begin(), upgradedHoles.end());
    std::sort(expectedHoles.begin(), expectedHoles.end());
    upgradedHoles.erase(std::unique(upgradedHoles.begin(), upgradedHoles.end()), upgradedHoles.end());
    require(upgradedHoles == expectedHoles, "Umgewandeltes Programm bohrt an anderen Positionen");

    // ── Kontur als Tasche mit folgender Insel ──
    auto rectSegs = [](double x0, double y0, double w, double h) {
        using ST = Geometry::ContourSegmentType;
        std::vector<Geometry::ContourSegment> segs(5);
        segs[0].type = ST::StartPoint; segs[0].x = x0;     segs[0].y = y0;
        segs[1].type = ST::Line;       segs[1].x = x0 + w; segs[1].y = y0;
        segs[2].type = ST::Line;       segs[2].x = x0 + w; segs[2].y = y0 + h;
        segs[3].type = ST::Line;       segs[3].x = x0;     segs[3].y = y0 + h;
        segs[4].type = ST::Line;       segs[4].x = x0;     segs[4].y = y0;
        return segs;
    };
    CAM::ConversationalBlock pocket(10, CAM::BlockType::Contour, QStringLiteral("Tasche"));
    pocket.contourRole = CAM::ContourRole::Pocket;
    pocket.contourZForAll = true;
    pocket.segments = rectSegs(0.0, 0.0, 60.0, 40.0);
    pocket.startZ = 0.0;
    pocket.targetZ = -3.0;
    pocket.stepDown = 3.0;
    pocket.stepOver = 3.0;
    pocket.toolId = 1;
    CAM::ConversationalBlock island(11, CAM::BlockType::Contour, QStringLiteral("Insel"));
    island.contourRole = CAM::ContourRole::Island;
    island.segments = rectSegs(20.0, 15.0, 20.0, 10.0);
    require(island.generateToolpath(tools.first(), tools.first(), stock).empty(), "Insel-Block darf selbst nicht fräsen");

    auto islandDistance = [](double x, double y) {
        const double dx = std::max({20.0 - x, 0.0, x - 40.0});
        const double dy = std::max({15.0 - y, 0.0, y - 25.0});
        return std::hypot(dx, dy);
    };
    CAM::ConversationalProgram pocketProg;
    pocketProg.addBlock(pocket);
    pocketProg.addBlock(island);
    const auto pocketTp = pocketProg.generateFullToolpath(tools, stock);
    int floorPoints = 0;
    for (const auto& s : pocketTp.segments) {
        if (s.motion == CAM::MotionType::Rapid || std::abs(s.endPos.z + 3.0) > 1e-6) continue;
        ++floorPoints;
        require(s.endPos.x > 2.9 && s.endPos.x < 57.1 && s.endPos.y > 2.9 && s.endPos.y < 37.1, "Taschenbahn verlässt die Kontur");
        if (std::abs(s.startPos.z + 3.0) > 1e-6) continue;
        for (int k = 0; k <= 20; ++k) {
            const double t = k / 20.0;
            const double x = s.startPos.x + (s.endPos.x - s.startPos.x) * t;
            const double y = s.startPos.y + (s.endPos.y - s.startPos.y) * t;
            require(islandDistance(x, y) >= 2.9, "Taschenbahn fräst in die Insel");
        }
    }
    require(floorPoints > 20, "Kontur-Tasche wird nicht ausgeräumt");

    CAM::ConversationalProgram pocketOnly;
    pocketOnly.addBlock(pocket);
    bool crossesIsland = false;
    for (const auto& s : pocketOnly.generateFullToolpath(tools, stock).segments) {
        if (s.motion == CAM::MotionType::Rapid || std::abs(s.endPos.z + 3.0) > 1e-6 || std::abs(s.startPos.z + 3.0) > 1e-6) continue;
        for (int k = 0; k <= 20; ++k) {
            const double t = k / 20.0;
            const double x = s.startPos.x + (s.endPos.x - s.startPos.x) * t;
            const double y = s.startPos.y + (s.endPos.y - s.startPos.y) * t;
            if (islandDistance(x, y) < 1.0) crossesIsland = true;
        }
    }
    require(crossesIsland, "Ohne Insel-Block muss die ganze Tasche ausgeräumt werden");

    // ── Z für alle Segmente: Segmenttiefen werden ignoriert ──
    CAM::ConversationalBlock profile(12, CAM::BlockType::Contour, QStringLiteral("Kontur"));
    profile.segments = rectSegs(0.0, 0.0, 40.0, 20.0);
    profile.segments[2].z = -6.0; // abweichende Segmenttiefe
    profile.startZ = 0.0;
    profile.targetZ = -3.0;
    profile.stepDown = 3.0;
    profile.contourSide = CAM::ContourSide::OnLine;
    auto deepest = [&](const CAM::ConversationalBlock& b) {
        double z = 0.0;
        for (const auto& s : b.generateToolpath(tools.first(), tools.first(), stock).segments) z = std::min(z, s.endPos.z);
        return z;
    };
    profile.contourZForAll = false;
    require(deepest(profile) < -5.9, "Z je Segment: abweichende Tiefe fehlt");
    profile.contourZForAll = true;
    require(std::abs(deepest(profile) + 3.0) < 1e-6, "Z für alle Segmente: nur Z UNTEN von Segment 0 erwartet");
    const auto roleReloaded = CAM::ConversationalBlock::fromJson(pocket.toJson());
    require(roleReloaded.contourRole == CAM::ContourRole::Pocket && roleReloaded.contourZForAll, "Konturart nicht gespeichert");

    std::cout << " -> PASSED" << std::endl;
}

void testHurcoPocketIslands() {
    std::cout << "[TEST] Hurco: Taschengrenze mit Inseln aus Kreis und Kontur, Links/Rechts..." << std::endl;
    auto tools = Core::ToolDefinition::createDefaultLibrary();
    Core::BoundingBox stock({-20, -20, -25}, {90, 60, 0});
    using ST = Geometry::ContourSegmentType;
    auto rectSegs = [](double x0, double y0, double w, double h) {
        std::vector<Geometry::ContourSegment> segs(5);
        segs[0].type = ST::StartPoint; segs[0].x = x0;     segs[0].y = y0;
        segs[1].type = ST::Line;       segs[1].x = x0 + w; segs[1].y = y0;
        segs[2].type = ST::Line;       segs[2].x = x0 + w; segs[2].y = y0 + h;
        segs[3].type = ST::Line;       segs[3].x = x0;     segs[3].y = y0 + h;
        segs[4].type = ST::Line;       segs[4].x = x0;     segs[4].y = y0;
        return segs;
    };

    // Rahmen als Taschengrenze (auswärts gewählt → mit Inseln einwärts)
    CAM::ConversationalBlock frame(1, CAM::BlockType::Pocket, QStringLiteral("Rahmen"));
    frame.pocketShape = CAM::PocketShape::Rectangle;
    frame.posX = 0.0; frame.posY = 0.0; frame.pocketWidthX = 60.0; frame.pocketDepthY = 40.0; frame.pocketCornerR = 0.0;
    frame.millingType = CAM::MillingType::Pocket;
    frame.pocketStrategy = 1;
    frame.startZ = 0.0; frame.targetZ = -3.0; frame.stepDown = 3.0; frame.stepOver = 3.0; frame.toolId = 1;

    CAM::ConversationalBlock circleIsland(2, CAM::BlockType::Pocket, QStringLiteral("Kreis-Insel"));
    circleIsland.pocketShape = CAM::PocketShape::Circle;
    circleIsland.posX = 15.0; circleIsland.posY = 20.0; circleIsland.pocketRadius = 5.0;
    circleIsland.millingType = CAM::MillingType::Island;
    circleIsland.targetZ = -20.0; // eigene Tiefe wird nicht verwendet

    CAM::ConversationalBlock contourIsland(3, CAM::BlockType::Contour, QStringLiteral("Kontur-Insel"));
    contourIsland.segments = rectSegs(32.0, 14.0, 13.0, 12.0);
    contourIsland.setEffectiveMillingType(CAM::MillingType::Island);

    CAM::ConversationalBlock nc(4, CAM::BlockType::RawNC, QStringLiteral("NC"));
    CAM::ConversationalBlock orphan(5, CAM::BlockType::Pocket, QStringLiteral("Insel nach NC"));
    orphan.pocketShape = CAM::PocketShape::Circle;
    orphan.posX = 52.0; orphan.posY = 20.0; orphan.pocketRadius = 3.0;
    orphan.millingType = CAM::MillingType::Island;

    CAM::ConversationalProgram prog;
    for (const auto& b : {frame, circleIsland, contourIsland, nc, orphan}) prog.addBlock(b);
    require(prog.pocketBoundaryFor(1) == 0 && prog.pocketBoundaryFor(2) == 0, "Inseln direkt nach der Taschengrenze gehören zu ihr");
    require(prog.pocketBoundaryFor(4) == -1, "Insel nach einem anderen Block gehört zu keiner Taschengrenze");
    require(prog.resolvedBlock(0).pocketIslands.size() == 2, "Taschengrenze muss genau zwei Inseln haben");
    require(circleIsland.generateToolpath(tools.first(), tools.first(), stock).empty()
            && contourIsland.generateToolpath(tools.first(), tools.first(), stock).empty(), "Insel-Blöcke fräsen selbst nicht");

    const auto tp = prog.generateFullToolpath(tools, stock);
    bool nearOrphan = false;
    int floorSegments = 0;
    for (const auto& s : tp.segments) {
        require(s.endPos.z > -3.001, "Inseln dürfen keine eigene Tiefe haben");
        if (s.motion == CAM::MotionType::Rapid || std::abs(s.startPos.z + 3.0) > 1e-6 || std::abs(s.endPos.z + 3.0) > 1e-6) continue;
        ++floorSegments;
        for (int k = 0; k <= 20; ++k) {
            const double t = k / 20.0;
            const double x = s.startPos.x + (s.endPos.x - s.startPos.x) * t;
            const double y = s.startPos.y + (s.endPos.y - s.startPos.y) * t;
            require(std::hypot(x - 15.0, y - 20.0) >= 7.9, "Taschenbahn fräst in die Kreis-Insel");
            const double dx = std::max({32.0 - x, 0.0, x - 45.0});
            const double dy = std::max({14.0 - y, 0.0, y - 26.0});
            require(std::hypot(dx, dy) >= 2.9, "Taschenbahn fräst in die Kontur-Insel");
            if (std::hypot(x - 52.0, y - 20.0) < 5.5) nearOrphan = true; // ausgespart wäre >= 6 mm (Insel R3 + Fräser R3)
        }
    }
    require(floorSegments > 10, "Taschengrenze wird nicht ausgeräumt");
    require(nearOrphan, "Insel nach einem anderen Block darf nicht ausgespart werden");

    // Speichern: Fräsart Insel; ältere Programme mit Inseln im Taschenblock werden umgewandelt
    require(CAM::ConversationalBlock::fromJson(circleIsland.toJson()).isPocketIsland()
            && CAM::ConversationalBlock::fromJson(contourIsland.toJson()).isPocketIsland(), "Fräsart Insel nicht gespeichert");
    CAM::ConversationalBlock legacyPocket = frame;
    legacyPocket.pocketIslands = {rectSegs(20.0, 10.0, 10.0, 10.0)};
    CAM::ConversationalProgram legacyProg;
    legacyProg.addBlock(legacyPocket);
    const QString path = QDir::temp().filePath(QStringLiteral("gemini_legacy_islands.gprog"));
    require(legacyProg.saveToFile(path), "Testprogramm nicht gespeichert");
    const auto upgraded = CAM::ConversationalProgram::loadFromFile(path);
    QFile::remove(path);
    require(upgraded.size() == 2 && upgraded[0].pocketIslands.empty() && upgraded[1].isPocketIsland()
            && upgraded.pocketBoundaryFor(1) == 0, "Eingebettete Insel muss ein eigener Insel-Block werden");

    // Links / Rechts einer gegen den Uhrzeigersinn programmierten Kontur
    CAM::ConversationalBlock profile(6, CAM::BlockType::Contour, QStringLiteral("Kontur"));
    profile.segments = rectSegs(0.0, 0.0, 40.0, 20.0);
    profile.startZ = 0.0; profile.targetZ = -2.0; profile.stepDown = 2.0; profile.leadType = 0;
    auto extentX = [&](CAM::MillingType type) {
        profile.setEffectiveMillingType(type);
        require(profile.effectiveMillingType() == type, "Fräsart der Kontur nicht übernommen");
        double minX = 1e9, maxX = -1e9;
        for (const auto& s : profile.generateToolpath(tools.first(), tools.first(), stock).segments) {
            if (s.motion == CAM::MotionType::Rapid || std::abs(s.endPos.z + 2.0) > 1e-6) continue;
            minX = std::min(minX, s.endPos.x);
            maxX = std::max(maxX, s.endPos.x);
        }
        return std::make_pair(minX, maxX);
    };
    const auto left = extentX(CAM::MillingType::Left);
    const auto right = extentX(CAM::MillingType::Right);
    require(left.first > 2.0 && left.second < 38.0, "Links (gegen Uhrzeigersinn) muss innen fräsen");
    require(right.first < -2.0 && right.second > 42.0, "Rechts (gegen Uhrzeigersinn) muss außen fräsen");

    std::cout << " -> PASSED" << std::endl;
}

void testOpenContourOffset() {
    std::cout << "[TEST] Offene Kontur mit Bogen: Bahnkorrektur ohne Schließkante..." << std::endl;
    using ST = Geometry::ContourSegmentType;
    auto tools = Core::ToolDefinition::createDefaultLibrary();
    const Core::BoundingBox stock({-100, -20, -40}, {20, 60, 0});

    // Gerade nach links bis (-65|25), dann Viertelbogen GUZS um (-65|35) nach (-55|35) – offen
    std::vector<Geometry::ContourSegment> segs(3);
    segs[0].type = ST::StartPoint; segs[0].x = -20.0; segs[0].y = 25.0;
    segs[1].type = ST::Line;       segs[1].x = -65.0; segs[1].y = 25.0;
    segs[2].type = ST::ArcCCW;     segs[2].x = -55.0; segs[2].y = 35.0;
    segs[2].hasCenter = true; segs[2].centerX = -65.0; segs[2].centerY = 35.0; segs[2].radius = 10.0;
    for (auto& s : segs) s.z = -3.0;

    const auto contour = Geometry::Contour::createFromSegments(segs, false);
    require(!contour.isClosed && contour.points.size() > 4, "Offene Kontur erwartet");
    const auto& last = contour.points.back();
    require(std::abs(last.x + 55.0) < 1e-9 && std::abs(last.y - 35.0) < 1e-9, "Bogen muss in (-55|35) enden");

    // Abstand eines Punkts zur programmierten (offenen) Kontur
    auto distToContour = [&contour](double x, double y) {
        double best = 1e9;
        for (size_t k = 0; k + 1 < contour.points.size(); ++k) {
            const auto& a = contour.points[k];
            const auto& b = contour.points[k + 1];
            const double ex = b.x - a.x, ey = b.y - a.y;
            const double l2 = ex * ex + ey * ey;
            const double t = l2 > 1e-12 ? std::clamp(((x - a.x) * ex + (y - a.y) * ey) / l2, 0.0, 1.0) : 0.0;
            best = std::min(best, std::hypot(x - (a.x + t * ex), y - (a.y + t * ey)));
        }
        return best;
    };

    for (const auto side : {CAM::MillingType::Outside, CAM::MillingType::Inside, CAM::MillingType::Left, CAM::MillingType::Right}) {
        CAM::ConversationalBlock block(1, CAM::BlockType::Contour, QStringLiteral("offen"));
        block.segments = segs;
        block.contour = contour;
        block.startZ = 0.0; block.targetZ = -3.0; block.stepDown = 3.0; block.leadType = 0;
        block.contourZForAll = true;
        block.setEffectiveMillingType(side);

        const auto offset = contour.createOffset(3.0);
        require(std::hypot(offset.points.front().x - (-20.0), offset.points.front().y - 25.0) < 3.0 + 1e-6
                && std::hypot(offset.points.back().x - (-55.0), offset.points.back().y - 35.0) < 3.0 + 1e-6,
                "Offset einer offenen Kontur: Endpunkte dürfen nur um den Abstand verschoben werden");

        double lastX = 0.0, lastY = 0.0;
        bool any = false;
        for (const auto& s : block.generateToolpath(tools.first(), tools.first(), stock).segments) {
            if (s.motion == CAM::MotionType::Rapid || std::abs(s.endPos.z + 3.0) > 1e-6) continue;
            // Außenseite der Kehre: überall im Fräserradius. Innen passt der Ø6-Fräser nicht zwischen Gerade und Bogen.
            if (side == CAM::MillingType::Outside) {
                require(std::abs(distToContour(s.endPos.x, s.endPos.y) - 3.0) < 0.25,
                        "Fräsbahn einer offenen Kontur muss im Abstand des Fräserradius bleiben");
            }
            lastX = s.endPos.x;
            lastY = s.endPos.y;
            any = true;
        }
        require(any, "Keine Fräsbahn für die offene Kontur");
        require(std::hypot(lastX + 55.0, lastY - 35.0) < 3.0 + 0.05, "Fräsbahn muss am Bogenende (-55|35) enden");
    }
    std::cout << " -> PASSED" << std::endl;
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
    testToolMarksAndShading();
    testHurcoDrillingAndContourRoles();
    testHurcoPocketIslands();
    testOpenContourOffset();
    std::cout << "=== All CAM Tests PASSED ===" << std::endl;
    return 0;
}
