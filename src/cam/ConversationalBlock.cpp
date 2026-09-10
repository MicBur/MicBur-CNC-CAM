#include "ConversationalBlock.h"
#include "ToolpathGenerator.h"
#include "geometry/StlLoader.h"
#include <cmath>
#include <QJsonArray>

namespace GeminiCNC::CAM {

QString blockTypeToString(BlockType type) {
    switch (type) {
        case BlockType::Facing: return QStringLiteral("PlanfrÃ¤sen");
        case BlockType::Contour: return QStringLiteral("KonturfrÃ¤sen");
        case BlockType::Pocket: return QStringLiteral("TaschenfrÃ¤sen");
        case BlockType::Slot: return QStringLiteral("Langloch");
        case BlockType::HelixThread: return QStringLiteral("Helix / Gewinde");
        case BlockType::Drill: return QStringLiteral("Bohrbild");
        case BlockType::Stl3D: return QStringLiteral("3D-STL FrÃ¤sen");
        case BlockType::RawNC: return QStringLiteral("NC-Merge");
    }
    return QStringLiteral("Unbekannt");
}

BlockType stringToBlockType(const QString& str) {
    if (str == QStringLiteral("PlanfrÃ¤sen") || str == QStringLiteral("Facing")) return BlockType::Facing;
    if (str == QStringLiteral("KonturfrÃ¤sen") || str == QStringLiteral("Contour")) return BlockType::Contour;
    if (str == QStringLiteral("TaschenfrÃ¤sen") || str == QStringLiteral("Pocket")) return BlockType::Pocket;
    if (str == QStringLiteral("Langloch") || str == QStringLiteral("Slot")) return BlockType::Slot;
    if (str == QStringLiteral("Helix / Gewinde") || str == QStringLiteral("HelixThread")) return BlockType::HelixThread;
    if (str == QStringLiteral("Bohrbild") || str == QStringLiteral("Drill")) return BlockType::Drill;
    if (str == QStringLiteral("3D-STL FrÃ¤sen") || str == QStringLiteral("Stl3D")) return BlockType::Stl3D;
    return BlockType::RawNC;
}

QString stlStrategyToString(StlMillingStrategy strat) {
    switch (strat) {
        case StlMillingStrategy::RoughAndFinishX: return QStringLiteral("RoughAndFinishX");
        case StlMillingStrategy::RoughAndFinishY: return QStringLiteral("RoughAndFinishY");
        case StlMillingStrategy::RoughOnly:       return QStringLiteral("RoughOnly");
        case StlMillingStrategy::RasterX:         return QStringLiteral("RasterX");
        case StlMillingStrategy::RasterY:         return QStringLiteral("RasterY");
        case StlMillingStrategy::RasterXY:        return QStringLiteral("RasterXY");
        case StlMillingStrategy::WaterlineFinish: return QStringLiteral("WaterlineFinish");
    }
    return QStringLiteral("RoughAndFinishX");
}

StlMillingStrategy stringToStlStrategy(const QString& str) {
    if (str == "RoughAndFinishY") return StlMillingStrategy::RoughAndFinishY;
    if (str == "RoughOnly" || str == "WaterlineRoughing") return StlMillingStrategy::RoughOnly;
    if (str == "RasterY") return StlMillingStrategy::RasterY;
    if (str == "RasterX") return StlMillingStrategy::RasterX;
    if (str == "RasterXY") return StlMillingStrategy::RasterXY;
    if (str == "WaterlineFinish") return StlMillingStrategy::WaterlineFinish;
    return StlMillingStrategy::RoughAndFinishX;
}

ConversationalBlock::ConversationalBlock(int blockId, BlockType blockType, const QString& blockName)
    : id(blockId), type(blockType), name(blockName) {}

std::vector<Core::Vector3D> ConversationalBlock::calculateDrillPositions() const {
    std::vector<Core::Vector3D> positions;

    if (drillPattern == DrillPattern::Single) {
        positions.push_back({0.0, 0.0, 0.0});

    } else if (drillPattern == DrillPattern::BoltCircle) {
        const int count = std::max(1, boltCircleHoleCount);
        const double stepAngle = 2.0 * M_PI / count;
        const double startRad = boltCircleStartAngle * M_PI / 180.0;

        for (int i = 0; i < count; ++i) {
            double ang = startRad + i * stepAngle;
            double px = boltCircleRadius * std::cos(ang);
            double py = boltCircleRadius * std::sin(ang);
            positions.push_back({px, py, 0.0});
        }

    } else if (drillPattern == DrillPattern::Grid) {
        const int cols = std::max(1, gridCols);
        const int rows = std::max(1, gridRows);
        const double startX = -(cols - 1) * gridPitchX * 0.5;
        const double startY = -(rows - 1) * gridPitchY * 0.5;

        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < cols; ++c) {
                positions.push_back({startX + c * gridPitchX, startY + r * gridPitchY, 0.0});
            }
        }

    } else if (drillPattern == DrillPattern::Line) {
        // Lochreihe: Bohrungen auf einer Linie mit Winkel
        const int count = std::max(1, lineHoleCount);
        const double angRad = lineAngleDeg * M_PI / 180.0;
        const double totalLen = (count - 1) * lineSpacing;
        const double startOff = -totalLen * 0.5;

        for (int i = 0; i < count; ++i) {
            double d = startOff + i * lineSpacing;
            double px = d * std::cos(angRad);
            double py = d * std::sin(angRad);
            positions.push_back({px, py, 0.0});
        }

    } else if (drillPattern == DrillPattern::Arc) {
        // Bogenreihe: Bohrungen auf einem Kreisbogen
        const int count = std::max(2, arcHoleCount);
        const double startRad = arcStartAngle * M_PI / 180.0;
        const double endRad = arcEndAngle * M_PI / 180.0;
        const double stepAng = (endRad - startRad) / (count - 1);

        for (int i = 0; i < count; ++i) {
            double ang = startRad + i * stepAng;
            double px = arcRadius * std::cos(ang);
            double py = arcRadius * std::sin(ang);
            positions.push_back({px, py, 0.0});
        }

    } else if (drillPattern == DrillPattern::Frame) {
        // Rahmen: Bohrungen auf dem Umfang eines Rechtecks
        const double hw = frameWidth * 0.5;
        const double hh = frameHeight * 0.5;
        const int cx = std::max(2, frameCountX);
        const int cy = std::max(0, frameCountY);

        // Untere Seite (links → rechts, inkl. Ecken)
        for (int i = 0; i < cx; ++i) {
            double x = -hw + i * frameWidth / (cx - 1);
            positions.push_back({x, -hh, 0.0});
        }
        // Rechte Seite (ohne Ecken)
        for (int i = 1; i <= cy; ++i) {
            double y = -hh + i * frameHeight / (cy + 1);
            positions.push_back({hw, y, 0.0});
        }
        // Obere Seite (rechts → links, inkl. Ecken)
        for (int i = 0; i < cx; ++i) {
            double x = hw - i * frameWidth / (cx - 1);
            positions.push_back({x, hh, 0.0});
        }
        // Linke Seite (ohne Ecken)
        for (int i = 1; i <= cy; ++i) {
            double y = hh - i * frameHeight / (cy + 1);
            positions.push_back({-hw, y, 0.0});
        }

    } else if (drillPattern == DrillPattern::Manual) {
        // Manuelle Positionen
        for (const auto& [mx, my] : manualPositions) {
            positions.push_back({mx, my, 0.0});
        }
        if (positions.empty()) {
            positions.push_back({0.0, 0.0, 0.0}); // Mindestens eine Position
        }
    }

    return positions;
}

Toolpath ConversationalBlock::generateToolpath(const Core::ToolDefinition& tool, 
                                               const Core::ToolDefinition& finishTool,
                                               const Core::BoundingBox& stockBounds, 
                                               const Geometry::Mesh& partMesh) const {
    Toolpath tp(name);
    if (!enabled) return tp;

    if (type == BlockType::Facing) {
        FacingParams fp;
        fp.startZ = startZ;
        fp.targetZ = targetZ;
        fp.stepDown = stepDown;
        fp.stepOver = stepOver;
        fp.clearanceZ = clearanceZ;

        Core::BoundingBox bounds;
        if (useStockDimensions && stockBounds.isValid()) {
            bounds = stockBounds;
        } else {
            bounds = Core::BoundingBox({posX, posY, std::min(startZ, targetZ)}, {posX + areaWidth, posY + areaDepth, std::max(startZ, targetZ)});
        }
        if (!bounds.isValid()) {
            bounds = stockBounds.isValid() ? stockBounds : Core::BoundingBox({-50, -40, -10}, {50, 40, 0});
        }
        tp = ToolpathGenerator::generateFacing(bounds, tool, fp);
        tp.operationName = name;
    } else if (type == BlockType::Contour) {
        ContourParams cp;
        cp.startZ = startZ;
        cp.targetZ = targetZ;
        cp.stepDown = stepDown;
        cp.side = contourSide;
        cp.finishAllowance = finishAllowance;
        cp.clearanceZ = clearanceZ;

        Geometry::Contour c = contour;
        if (c.empty() && !segments.empty()) {
            c = Geometry::Contour::createFromSegments(segments, true);
        }
        if (c.empty()) {
            c = Geometry::Contour::createRectangle(-30.0, -20.0, 60.0, 40.0);
        }
        tp = ToolpathGenerator::generateContourMilling(c, tool, cp);
        tp.operationName = name;
    } else if (type == BlockType::Slot) {
        // Langloch / Nut FrÃ¤sen
        Geometry::Contour slotContour = Geometry::Contour::createSlot(0.0, 0.0, slotLength, slotWidth, slotAngleDeg);
        ContourParams cp;
        cp.startZ = startZ;
        cp.targetZ = targetZ;
        cp.stepDown = stepDown;
        cp.side = ContourSide::OnLine;
        cp.clearanceZ = clearanceZ;

        tp = ToolpathGenerator::generateContourMilling(slotContour, tool, cp);
        tp.operationName = name;
    } else if (type == BlockType::HelixThread) {
        // 3D-Helix / GewindefrÃ¤sen
        double pathRadius = helixInternal
            ? std::max(0.5, (helixDiameter - tool.diameter) * 0.5)
            : std::max(0.5, (helixDiameter + tool.diameter) * 0.5);

        auto hPoints = Geometry::Contour::createHelixPoints(0.0, 0.0, startZ, targetZ, pathRadius, helixPitch);

        if (!hPoints.empty()) {
            Core::Vector3D startPt = hPoints.front();
            // Anfahrt auf Eilgang Ã¼ber Startpunkt
            tp.addSegment({MotionType::Rapid, {0, 0, clearanceZ}, {startPt.x, startPt.y, clearanceZ}, {}, 0, spindleRpm, false, tool.id, tool.diameter, 0, false, {}});
            tp.addSegment({MotionType::LinearFeed, {startPt.x, startPt.y, clearanceZ}, startPt, {}, plungeFeedRate, spindleRpm, false, tool.id, tool.diameter, 0, false, {}});

            Core::Vector3D prevPt = startPt;
            for (size_t i = 1; i < hPoints.size(); ++i) {
                tp.addSegment({MotionType::LinearFeed, prevPt, hPoints[i], {}, feedRate, spindleRpm, false, tool.id, tool.diameter, 0, false, {}});
                prevPt = hPoints[i];
            }

            // RÃ¼ckzug
            tp.addSegment({MotionType::Rapid, prevPt, {prevPt.x, prevPt.y, clearanceZ}, {}, 0, spindleRpm, false, tool.id, tool.diameter, 0, false, {}});
        }
        tp.operationName = name;
    } else if (type == BlockType::Pocket) {
        Geometry::Contour boundary;
        double bx = posX;
        double by = posY;
        if (pocketShape == PocketShape::Rectangle) {
            // Wenn startSide == Bottom (Standard in Hurco: X Corner / Y Corner)
            boundary = Geometry::Contour::createRectangle(bx, by, pocketWidthX, pocketDepthY);
        } else if (pocketShape == PocketShape::Circle) {
            boundary = Geometry::Contour::createCircle(bx, by, pocketRadius);
        } else {
            boundary = contour.empty() ? Geometry::Contour::createRectangle(bx - 25, by - 25, 50, 50) : contour;
        }

        if (millingType == MillingType::Pocket) {
            // 1. Inseln kompilieren
            std::vector<Geometry::Contour> compiledIslands;
            for (const auto& islandSegs : pocketIslands) {
                auto contour = Geometry::Contour::createFromSegments(islandSegs, true);
                if (!contour.empty()) {
                    compiledIslands.push_back(contour);
                }
            }

            PocketParams pp;
            pp.startZ = startZ;
            pp.targetZ = targetZ;
            pp.stepDown = stepDown;
            pp.stepOverRatio = (tool.diameter > 0) ? (stepOver / tool.diameter) : 0.5;
            pp.finishAllowance = enableFinishing ? finishAllowanceXY : 0.0;
            pp.clearanceZ = clearanceZ;
            
            // 2. Schruppen (mit oder ohne Aufmaß)
            tp = ToolpathGenerator::generatePocketMilling(boundary, compiledIslands, tool, pp);
            // Override Feed & Speed for roughing if needed (we keep the standard block ones here)
            for (auto& seg : tp.segments) {
                if (seg.motion != MotionType::Rapid) {
                    seg.feedRate = feedRate;
                    seg.spindleRpm = spindleRpm;
                }
            }

            // 3. Schlichten (Räumt Aufmaß auf Null entlang Konturen)
            if (enableFinishing) {
                ContourParams cpFinish;
                cpFinish.startZ = startZ;
                cpFinish.targetZ = targetZ;
                // Beim reinen Wand-Schlichten typischerweise volle Tiefe oder eigener StepDown.
                // Wir gehen erstmal auf volle Tiefe, wenn nichts anderes definiert ist (stepDown als Fallback).
                cpFinish.stepDown = stepDown; 
                cpFinish.finishAllowance = 0.0;
                cpFinish.clearanceZ = clearanceZ;
                
                // 3.1 Außenkontur der Tasche schlichten (Innen abfahren)
                cpFinish.side = ContourSide::Inside;
                Toolpath finishTpOuter = ToolpathGenerator::generateContourMilling(boundary, finishTool, cpFinish);
                
                // 3.2 Inseln schlichten (Außen abfahren)
                cpFinish.side = ContourSide::Outside;
                for (const auto& island : compiledIslands) {
                    Toolpath finishTpIsland = ToolpathGenerator::generateContourMilling(island, finishTool, cpFinish);
                    finishTpOuter.segments.insert(finishTpOuter.segments.end(), finishTpIsland.segments.begin(), finishTpIsland.segments.end());
                }

                // Apply finishing feed/speed
                for (auto& seg : finishTpOuter.segments) {
                    if (seg.motion != MotionType::Rapid) {
                        seg.feedRate = finishFeedRate;
                        seg.spindleRpm = finishSpindleRpm;
                    }
                }
                
                // Anhängen
                tp.segments.insert(tp.segments.end(), finishTpOuter.segments.begin(), finishTpOuter.segments.end());
            }
        } else {
            ContourParams cp;
            cp.startZ = startZ;
            cp.targetZ = targetZ;
            cp.stepDown = stepDown;
            cp.finishAllowance = finishAllowance;
            cp.clearanceZ = clearanceZ;
            if (millingType == MillingType::Outside) {
                cp.side = ContourSide::Outside;
            } else if (millingType == MillingType::Inside) {
                cp.side = ContourSide::Inside;
            } else {
                cp.side = ContourSide::OnLine;
            }
            tp = ToolpathGenerator::generateContourMilling(boundary, tool, cp);
        }
        tp.operationName = name;
    } else if (type == BlockType::Drill) {
        // Bohrzyklus mit Spanbruch (Peck Drilling)
        auto holes = calculateDrillPositions();
        const double peck = std::max(0.2, peckDepth);
        Core::Vector3D currentPos = {0.0, 0.0, clearanceZ};

        for (const auto& hole : holes) {
            // Eilgang Ã¼ber Bohrung
            tp.addSegment({MotionType::Rapid, currentPos, {hole.x, hole.y, clearanceZ}, {}, 0, spindleRpm, false, tool.id, tool.diameter, 0, false, {}});
            currentPos = {hole.x, hole.y, clearanceZ};

            double curZ = startZ;
            while (curZ > targetZ - 1e-4) {
                double nextZ = curZ - peck;
                if (nextZ < targetZ) nextZ = targetZ;

                // Vorschubbohrung ins Material
                tp.addSegment({MotionType::LinearFeed, currentPos, {hole.x, hole.y, nextZ}, {}, plungeFeedRate, spindleRpm, false, tool.id, tool.diameter, 0, false, {}});
                currentPos = {hole.x, hole.y, nextZ};

                // RÃ¼ckzug zum Spanentspanen
                tp.addSegment({MotionType::Rapid, currentPos, {hole.x, hole.y, startZ + 1.0}, {}, 0, spindleRpm, false, tool.id, tool.diameter, 0, false, {}});
                // Wiederanfahrt bis knapp vor Schnitt
                tp.addSegment({MotionType::Rapid, {hole.x, hole.y, startZ + 1.0}, {hole.x, hole.y, nextZ + 0.3}, {}, 0, spindleRpm, false, tool.id, tool.diameter, 0, false, {}});
                currentPos = {hole.x, hole.y, nextZ + 0.3};

                curZ = nextZ;
                if (std::abs(curZ - targetZ) < 1e-4) break;
            }

            // RÃ¼ckzug auf SicherheitshÃ¶he
            tp.addSegment({MotionType::Rapid, currentPos, {hole.x, hole.y, clearanceZ}, {}, 0, spindleRpm, false, tool.id, tool.diameter, 0, false, {}});
            currentPos = {hole.x, hole.y, clearanceZ};
        }
    } else if (type == BlockType::Stl3D) {
        // Welches Mesh verwenden: direktes Mesh, STL-Dateipfad oder aktives partMesh
        Geometry::Mesh activeMesh = directStlMesh;
        if (activeMesh.isEmpty() && !stlFilePath.isEmpty()) {
            auto res = Geometry::StlLoader::loadFromFile(stlFilePath, Geometry::MeshRole::TargetPart);
            if (res.success) {
                activeMesh = res.mesh;
            }
        }
        if (activeMesh.isEmpty() && !partMesh.isEmpty()) {
            activeMesh = partMesh;
        }

        if (!activeMesh.isEmpty()) {
            // Nullpunkt-Absicherung: Unausgerichtete CAD-Modelle sofort auf Z=0 oben und XY-Mitte zentrieren
            if (activeMesh.boundingBox.maxPoint.z > 0.05 ||
                std::abs(activeMesh.boundingBox.center().x) > 400.0 ||
                std::abs(activeMesh.boundingBox.center().y) > 400.0) {
                activeMesh.alignToOrigin(true, true);
            }

            // Maximale Bearbeitungstiefe (Z-Boden) intelligent ermitteln:
            double floorZ = activeMesh.boundingBox.minPoint.z;

            // 1. Wenn ein Rohteilquader definiert ist: Werkzeug darf NIEMALS tiefer als der Quaderboden frÃ¤sen!
            if (stockBounds.isValid()) {
                floorZ = std::max(floorZ, stockBounds.minPoint.z);
            }

            // 2. Nur wenn eine explizite Zieltiefe im Block eingegeben wurde (nicht der 2D-Default -2.0 mm):
            if (targetZ < -0.01 && std::abs(targetZ - (-2.0)) > 1e-3 && targetZ > floorZ) {
                floorZ = targetZ;
            }

            // 3. Maschinensicherheit: Niemals tiefer als das Z-Softlimit der CNC-Maschine (-280.0 mm)
            floorZ = std::max(floorZ, -280.0);

            Core::BoundingBox bounds;
            if (stlUseStockDims && stockBounds.isValid()) {
                bounds = stockBounds;
                bounds.minPoint.z = floorZ;
            } else {
                const double margin = std::max(1.5, tool.diameter * 0.5 + 0.5);
                bounds = Core::BoundingBox(
                    {activeMesh.boundingBox.minPoint.x - margin, activeMesh.boundingBox.minPoint.y - margin, floorZ},
                    {activeMesh.boundingBox.maxPoint.x + margin, activeMesh.boundingBox.maxPoint.y + margin, std::max(activeMesh.boundingBox.maxPoint.z, 0.0)}
                );
            }

            StlMillingParams params;
            params.roughStepDown = stlStepDown;
            params.finishStepOver = stlStepOver;
            params.finishAllowance = stlAllowance;
            params.sampleStep = stlSampleStep;
            params.clearanceZ = clearanceZ;

            if (stlStrategy == StlMillingStrategy::RoughAndFinishX || 
                stlStrategy == StlMillingStrategy::RoughAndFinishY) {
                
                // Wir trennen Rough und Finish auf, falls finishTool unterschiedlich ist oder sowieso, um flexibel zu bleiben
                params.mode = StlMillingMode::RoughOnly;
                tp = ToolpathGenerator::generateStlMilling(activeMesh, bounds, tool, params);
                
                params.mode = StlMillingMode::FinishOnly;
                params.finishDirection = (stlStrategy == StlMillingStrategy::RoughAndFinishX) ? 0 : 1;
                Toolpath finishTp = ToolpathGenerator::generateStlMilling(activeMesh, bounds, finishTool, params);
                for(auto& s : finishTp.segments) {
                    s.toolId = finishTool.id;
                    tp.addSegment(s);
                }
            } else if (stlStrategy == StlMillingStrategy::RasterXY) {
                params.mode = StlMillingMode::FinishRasterXY;
                tp = ToolpathGenerator::generateStlMilling(activeMesh, bounds, finishTool, params);
                for(auto& s : tp.segments) s.toolId = finishTool.id;
            } else if (stlStrategy == StlMillingStrategy::WaterlineFinish) {
                params.mode = StlMillingMode::WaterlineFinish;
                tp = ToolpathGenerator::generateStlMilling(activeMesh, bounds, finishTool, params);
                for(auto& s : tp.segments) s.toolId = finishTool.id;
            } else if (stlStrategy == StlMillingStrategy::RoughOnly) {
                params.mode = StlMillingMode::RoughOnly;
                tp = ToolpathGenerator::generateStlMilling(activeMesh, bounds, tool, params);
            } else {
                params.mode = StlMillingMode::FinishOnly;
                params.finishDirection = (stlStrategy == StlMillingStrategy::RasterY) ? 1 : 0;
                tp = ToolpathGenerator::generateStlMilling(activeMesh, bounds, finishTool, params);
                for(auto& s : tp.segments) s.toolId = finishTool.id;
            }
        }
        tp.operationName = name;
    } else if (type == BlockType::RawNC) {
        tp = Toolpath::parseGCode(rawGCode, name);
    }

    tp.visible = visible;
    tp.blockIndex = id;

    // Vorschübe, Sichtbarkeit und Werkzeug synchronisieren
    for (auto& seg : tp.segments) {
        if (seg.toolId <= 0) {
            seg.toolId = tool.id;
        }
        seg.spindleRpm = spindleRpm;
        seg.visible = visible;
        seg.blockId = id;
        if (seg.motion != MotionType::Rapid && seg.feedRate <= 0.0) {
            seg.feedRate = feedRate;
        }
    }

    return tp;
}

QJsonObject ConversationalBlock::toJson() const {
    QJsonObject obj;
    obj[QStringLiteral("id")] = id;
    obj[QStringLiteral("type")] = blockTypeToString(type);
    obj[QStringLiteral("name")] = name;
    obj[QStringLiteral("enabled")] = enabled;
    obj[QStringLiteral("toolId")] = toolId;
    obj[QStringLiteral("finishToolId")] = finishToolId;
    obj[QStringLiteral("materialId")] = materialId;
    obj[QStringLiteral("spindleRpm")] = spindleRpm;
    obj[QStringLiteral("feedRate")] = feedRate;
    obj[QStringLiteral("plungeFeedRate")] = plungeFeedRate;
    obj[QStringLiteral("startZ")] = startZ;
    obj[QStringLiteral("targetZ")] = targetZ;
    obj[QStringLiteral("stepDown")] = stepDown;
    obj[QStringLiteral("stepOver")] = stepOver;
    obj[QStringLiteral("clearanceZ")] = clearanceZ;

    obj[QStringLiteral("pocketShape")] = static_cast<int>(pocketShape);
    obj[QStringLiteral("pocketWidthX")] = pocketWidthX;
    obj[QStringLiteral("pocketDepthY")] = pocketDepthY;
    obj[QStringLiteral("pocketRadius")] = pocketRadius;

    obj[QStringLiteral("slotLength")] = slotLength;
    obj[QStringLiteral("slotWidth")] = slotWidth;
    obj[QStringLiteral("slotAngleDeg")] = slotAngleDeg;

    obj[QStringLiteral("helixDiameter")] = helixDiameter;
    obj[QStringLiteral("helixPitch")] = helixPitch;
    obj[QStringLiteral("helixInternal")] = helixInternal;

    obj[QStringLiteral("drillPattern")] = static_cast<int>(drillPattern);
    obj[QStringLiteral("peckDepth")] = peckDepth;
    obj[QStringLiteral("boltCircleRadius")] = boltCircleRadius;
    obj[QStringLiteral("boltCircleHoleCount")] = boltCircleHoleCount;
    obj[QStringLiteral("boltCircleStartAngle")] = boltCircleStartAngle;
    obj[QStringLiteral("gridCols")] = gridCols;
    obj[QStringLiteral("gridRows")] = gridRows;
    obj[QStringLiteral("gridPitchX")] = gridPitchX;
    obj[QStringLiteral("gridPitchY")] = gridPitchY;
    // Lochreihe
    obj[QStringLiteral("lineHoleCount")] = lineHoleCount;
    obj[QStringLiteral("lineSpacing")] = lineSpacing;
    obj[QStringLiteral("lineAngleDeg")] = lineAngleDeg;
    // Bogenreihe
    obj[QStringLiteral("arcRadius")] = arcRadius;
    obj[QStringLiteral("arcHoleCount")] = arcHoleCount;
    obj[QStringLiteral("arcStartAngle")] = arcStartAngle;
    obj[QStringLiteral("arcEndAngle")] = arcEndAngle;
    // Rahmen
    obj[QStringLiteral("frameWidth")] = frameWidth;
    obj[QStringLiteral("frameHeight")] = frameHeight;
    obj[QStringLiteral("frameCountX")] = frameCountX;
    obj[QStringLiteral("frameCountY")] = frameCountY;
    // Manuell
    QJsonArray manArr;
    for (const auto& [mx, my] : manualPositions) {
        QJsonObject pt;
        pt["x"] = mx;
        pt["y"] = my;
        manArr.append(pt);
    }
    if (!manArr.isEmpty()) obj[QStringLiteral("manualPositions")] = manArr;

    obj[QStringLiteral("rawGCode")] = rawGCode;

    obj[QStringLiteral("stlFilePath")] = stlFilePath;
    obj[QStringLiteral("stlStrategy")] = stlStrategyToString(stlStrategy);
    obj[QStringLiteral("stlStepOver")] = stlStepOver;
    obj[QStringLiteral("stlStepDown")] = stlStepDown;
    obj[QStringLiteral("stlAllowance")] = stlAllowance;
    obj[QStringLiteral("stlSampleStep")] = stlSampleStep;
    obj[QStringLiteral("stlUseStockDims")] = stlUseStockDims;

    // Segmente serialisieren falls vorhanden
    if (!segments.empty()) {
        QJsonArray segArray;
        for (const auto& s : segments) {
            QJsonObject sObj;
            sObj[QStringLiteral("type")] = static_cast<int>(s.type);
            sObj[QStringLiteral("x")] = s.x;
            sObj[QStringLiteral("y")] = s.y;
            sObj[QStringLiteral("z")] = s.z;
            sObj[QStringLiteral("radius")] = s.radius;
            sObj[QStringLiteral("pitch")] = s.pitch;
            sObj[QStringLiteral("angleDeg")] = s.angleDeg;
            sObj[QStringLiteral("length")] = s.length;
            segArray.append(sObj);
        }
        obj[QStringLiteral("segments")] = segArray;
    }

    // Konturpunkte serialisieren falls vorhanden
    if (!contour.empty()) {
        QJsonArray ptsArray;
        for (const auto& pt : contour.points) {
            QJsonObject pObj;
            pObj[QStringLiteral("x")] = pt.x;
            pObj[QStringLiteral("y")] = pt.y;
            pObj[QStringLiteral("bulge")] = pt.bulge;
            ptsArray.append(pObj);
        }
        obj[QStringLiteral("contourPoints")] = ptsArray;
    }

    return obj;
}

ConversationalBlock ConversationalBlock::fromJson(const QJsonObject& json) {
    ConversationalBlock b;
    if (json.contains(QStringLiteral("id"))) b.id = json[QStringLiteral("id")].toInt();
    if (json.contains(QStringLiteral("type"))) b.type = stringToBlockType(json[QStringLiteral("type")].toString());
    if (json.contains(QStringLiteral("name"))) b.name = json[QStringLiteral("name")].toString();
    if (json.contains(QStringLiteral("enabled"))) b.enabled = json[QStringLiteral("enabled")].toBool();
    if (json.contains(QStringLiteral("toolId"))) b.toolId = json[QStringLiteral("toolId")].toInt();
    if (json.contains(QStringLiteral("finishToolId"))) b.finishToolId = json[QStringLiteral("finishToolId")].toInt();
    if (json.contains(QStringLiteral("materialId"))) b.materialId = json[QStringLiteral("materialId")].toInt();
    if (json.contains(QStringLiteral("spindleRpm"))) b.spindleRpm = json[QStringLiteral("spindleRpm")].toDouble();
    if (json.contains(QStringLiteral("feedRate"))) b.feedRate = json[QStringLiteral("feedRate")].toDouble();
    if (json.contains(QStringLiteral("plungeFeedRate"))) b.plungeFeedRate = json[QStringLiteral("plungeFeedRate")].toDouble();
    if (json.contains(QStringLiteral("startZ"))) b.startZ = json[QStringLiteral("startZ")].toDouble();
    if (json.contains(QStringLiteral("targetZ"))) b.targetZ = json[QStringLiteral("targetZ")].toDouble();
    if (json.contains(QStringLiteral("stepDown"))) b.stepDown = json[QStringLiteral("stepDown")].toDouble();
    if (json.contains(QStringLiteral("stepOver"))) b.stepOver = json[QStringLiteral("stepOver")].toDouble();
    if (json.contains(QStringLiteral("clearanceZ"))) b.clearanceZ = json[QStringLiteral("clearanceZ")].toDouble();

    if (json.contains(QStringLiteral("pocketShape"))) b.pocketShape = static_cast<PocketShape>(json[QStringLiteral("pocketShape")].toInt());
    if (json.contains(QStringLiteral("pocketWidthX"))) b.pocketWidthX = json[QStringLiteral("pocketWidthX")].toDouble();
    if (json.contains(QStringLiteral("pocketDepthY"))) b.pocketDepthY = json[QStringLiteral("pocketDepthY")].toDouble();
    if (json.contains(QStringLiteral("pocketRadius"))) b.pocketRadius = json[QStringLiteral("pocketRadius")].toDouble();

    if (json.contains(QStringLiteral("slotLength"))) b.slotLength = json[QStringLiteral("slotLength")].toDouble();
    if (json.contains(QStringLiteral("slotWidth"))) b.slotWidth = json[QStringLiteral("slotWidth")].toDouble();
    if (json.contains(QStringLiteral("slotAngleDeg"))) b.slotAngleDeg = json[QStringLiteral("slotAngleDeg")].toDouble();

    if (json.contains(QStringLiteral("helixDiameter"))) b.helixDiameter = json[QStringLiteral("helixDiameter")].toDouble();
    if (json.contains(QStringLiteral("helixPitch"))) b.helixPitch = json[QStringLiteral("helixPitch")].toDouble();
    if (json.contains(QStringLiteral("helixInternal"))) b.helixInternal = json[QStringLiteral("helixInternal")].toBool();

    if (json.contains(QStringLiteral("drillPattern"))) b.drillPattern = static_cast<DrillPattern>(json[QStringLiteral("drillPattern")].toInt());
    if (json.contains(QStringLiteral("peckDepth"))) b.peckDepth = json[QStringLiteral("peckDepth")].toDouble();
    if (json.contains(QStringLiteral("boltCircleRadius"))) b.boltCircleRadius = json[QStringLiteral("boltCircleRadius")].toDouble();
    if (json.contains(QStringLiteral("boltCircleHoleCount"))) b.boltCircleHoleCount = json[QStringLiteral("boltCircleHoleCount")].toInt();
    if (json.contains(QStringLiteral("boltCircleStartAngle"))) b.boltCircleStartAngle = json[QStringLiteral("boltCircleStartAngle")].toDouble();
    if (json.contains(QStringLiteral("gridCols"))) b.gridCols = json[QStringLiteral("gridCols")].toInt();
    if (json.contains(QStringLiteral("gridRows"))) b.gridRows = json[QStringLiteral("gridRows")].toInt();
    if (json.contains(QStringLiteral("gridPitchX"))) b.gridPitchX = json[QStringLiteral("gridPitchX")].toDouble();
    if (json.contains(QStringLiteral("gridPitchY"))) b.gridPitchY = json[QStringLiteral("gridPitchY")].toDouble();
    // Lochreihe
    if (json.contains(QStringLiteral("lineHoleCount"))) b.lineHoleCount = json[QStringLiteral("lineHoleCount")].toInt();
    if (json.contains(QStringLiteral("lineSpacing"))) b.lineSpacing = json[QStringLiteral("lineSpacing")].toDouble();
    if (json.contains(QStringLiteral("lineAngleDeg"))) b.lineAngleDeg = json[QStringLiteral("lineAngleDeg")].toDouble();
    // Bogenreihe
    if (json.contains(QStringLiteral("arcRadius"))) b.arcRadius = json[QStringLiteral("arcRadius")].toDouble();
    if (json.contains(QStringLiteral("arcHoleCount"))) b.arcHoleCount = json[QStringLiteral("arcHoleCount")].toInt();
    if (json.contains(QStringLiteral("arcStartAngle"))) b.arcStartAngle = json[QStringLiteral("arcStartAngle")].toDouble();
    if (json.contains(QStringLiteral("arcEndAngle"))) b.arcEndAngle = json[QStringLiteral("arcEndAngle")].toDouble();
    // Rahmen
    if (json.contains(QStringLiteral("frameWidth"))) b.frameWidth = json[QStringLiteral("frameWidth")].toDouble();
    if (json.contains(QStringLiteral("frameHeight"))) b.frameHeight = json[QStringLiteral("frameHeight")].toDouble();
    if (json.contains(QStringLiteral("frameCountX"))) b.frameCountX = json[QStringLiteral("frameCountX")].toInt();
    if (json.contains(QStringLiteral("frameCountY"))) b.frameCountY = json[QStringLiteral("frameCountY")].toInt();
    // Manuell
    if (json.contains(QStringLiteral("manualPositions"))) {
        b.manualPositions.clear();
        for (const auto& val : json[QStringLiteral("manualPositions")].toArray()) {
            auto pt = val.toObject();
            b.manualPositions.push_back({pt["x"].toDouble(), pt["y"].toDouble()});
        }
    }

    if (json.contains(QStringLiteral("rawGCode"))) b.rawGCode = json[QStringLiteral("rawGCode")].toString();

    if (json.contains(QStringLiteral("stlFilePath"))) b.stlFilePath = json[QStringLiteral("stlFilePath")].toString();
    if (json.contains(QStringLiteral("stlStrategy"))) b.stlStrategy = stringToStlStrategy(json[QStringLiteral("stlStrategy")].toString());
    if (json.contains(QStringLiteral("stlStepOver"))) b.stlStepOver = json[QStringLiteral("stlStepOver")].toDouble();
    if (json.contains(QStringLiteral("stlStepDown"))) b.stlStepDown = json[QStringLiteral("stlStepDown")].toDouble();
    if (json.contains(QStringLiteral("stlAllowance"))) b.stlAllowance = json[QStringLiteral("stlAllowance")].toDouble();
    if (json.contains(QStringLiteral("stlSampleStep"))) b.stlSampleStep = json[QStringLiteral("stlSampleStep")].toDouble();
    if (json.contains(QStringLiteral("stlUseStockDims"))) b.stlUseStockDims = json[QStringLiteral("stlUseStockDims")].toBool();

    if (json.contains(QStringLiteral("segments"))) {
        QJsonArray segArray = json[QStringLiteral("segments")].toArray();
        b.segments.clear();
        for (const auto& v : segArray) {
            QJsonObject s = v.toObject();
            Geometry::ContourSegment seg;
            seg.type = static_cast<Geometry::ContourSegmentType>(s[QStringLiteral("type")].toInt());
            seg.x = s[QStringLiteral("x")].toDouble();
            seg.y = s[QStringLiteral("y")].toDouble();
            seg.z = s[QStringLiteral("z")].toDouble();
            seg.radius = s[QStringLiteral("radius")].toDouble();
            seg.pitch = s[QStringLiteral("pitch")].toDouble();
            seg.angleDeg = s[QStringLiteral("angleDeg")].toDouble();
            seg.length = s[QStringLiteral("length")].toDouble();
            b.segments.push_back(seg);
        }
    }

    if (json.contains(QStringLiteral("contourPoints"))) {
        QJsonArray ptsArray = json[QStringLiteral("contourPoints")].toArray();
        b.contour.clear();
        for (const auto& v : ptsArray) {
            QJsonObject p = v.toObject();
            b.contour.addPoint(p[QStringLiteral("x")].toDouble(), p[QStringLiteral("y")].toDouble(), p[QStringLiteral("bulge")].toDouble());
        }
    }

    return b;
}

} // namespace GeminiCNC::CAM
