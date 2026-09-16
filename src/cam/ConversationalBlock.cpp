#include "ConversationalBlock.h"
#include "ToolpathGenerator.h"
#include "geometry/StlLoader.h"
#include <cmath>
#include <QJsonArray>

namespace GeminiCNC::CAM {

namespace {

QJsonArray segmentsToJson(const std::vector<Geometry::ContourSegment>& segments) {
    QJsonArray segArray;
    for (const auto& s : segments) {
        QJsonObject sObj;
        sObj[QStringLiteral("type")] = static_cast<int>(s.type);
        sObj[QStringLiteral("x")] = s.x;
        sObj[QStringLiteral("y")] = s.y;
        sObj[QStringLiteral("z")] = s.z;
        sObj[QStringLiteral("zStart")] = s.zStart;
        sObj[QStringLiteral("radius")] = s.radius;
        sObj[QStringLiteral("pitch")] = s.pitch;
        sObj[QStringLiteral("angleDeg")] = s.angleDeg;
        sObj[QStringLiteral("length")] = s.length;
        if (s.hasCenter) {
            sObj[QStringLiteral("centerX")] = s.centerX;
            sObj[QStringLiteral("centerY")] = s.centerY;
            sObj[QStringLiteral("hasCenter")] = true;
        }
        segArray.append(sObj);
    }
    return segArray;
}

std::vector<Geometry::ContourSegment> segmentsFromJson(const QJsonArray& segArray) {
    std::vector<Geometry::ContourSegment> segments;
    segments.reserve(segArray.size());
    for (const auto& v : segArray) {
        QJsonObject s = v.toObject();
        Geometry::ContourSegment seg;
        seg.type = static_cast<Geometry::ContourSegmentType>(s[QStringLiteral("type")].toInt());
        seg.x = s[QStringLiteral("x")].toDouble();
        seg.y = s[QStringLiteral("y")].toDouble();
        seg.z = s[QStringLiteral("z")].toDouble();
        seg.zStart = s[QStringLiteral("zStart")].toDouble();
        seg.radius = s[QStringLiteral("radius")].toDouble();
        seg.pitch = s[QStringLiteral("pitch")].toDouble();
        seg.angleDeg = s[QStringLiteral("angleDeg")].toDouble();
        seg.length = s[QStringLiteral("length")].toDouble();
        seg.hasCenter = s[QStringLiteral("hasCenter")].toBool();
        seg.centerX = s[QStringLiteral("centerX")].toDouble();
        seg.centerY = s[QStringLiteral("centerY")].toDouble();
        segments.push_back(seg);
    }
    return segments;
}

} // namespace

QString blockTypeToString(BlockType type) {
    switch (type) {
        case BlockType::Facing: return QStringLiteral("Planfräsen");
        case BlockType::Contour: return QStringLiteral("Konturfräsen");
        case BlockType::Pocket: return QStringLiteral("Taschenfräsen");
        case BlockType::Slot: return QStringLiteral("Langloch");
        case BlockType::HelixThread: return QStringLiteral("Helix / Gewinde");
        case BlockType::Drill: return QStringLiteral("Bohrbild");
        case BlockType::Stl3D: return QStringLiteral("3D-STL Fräsen");
        case BlockType::RawNC: return QStringLiteral("NC-Merge");
        case BlockType::PatternStart: return QStringLiteral("Muster Start");
        case BlockType::PatternEnd: return QStringLiteral("Muster Ende");
    }
    return QStringLiteral("Unbekannt");
}

// Stabiler Schlüssel für .gprog-Dateien (unabhängig vom Anzeigetext)
static QString blockTypeKey(BlockType type) {
    switch (type) {
        case BlockType::Facing: return QStringLiteral("Facing");
        case BlockType::Contour: return QStringLiteral("Contour");
        case BlockType::Pocket: return QStringLiteral("Pocket");
        case BlockType::Slot: return QStringLiteral("Slot");
        case BlockType::HelixThread: return QStringLiteral("HelixThread");
        case BlockType::Drill: return QStringLiteral("Drill");
        case BlockType::Stl3D: return QStringLiteral("Stl3D");
        case BlockType::RawNC: return QStringLiteral("RawNC");
        case BlockType::PatternStart: return QStringLiteral("PatternStart");
        case BlockType::PatternEnd: return QStringLiteral("PatternEnd");
    }
    return QStringLiteral("RawNC");
}

BlockType stringToBlockType(const QString& str) {
    // Schlüssel, Anzeigetext und die fehlerhaft kodierten Texte älterer .gprog-Dateien akzeptieren
    if (str == QStringLiteral("Facing") || str == QStringLiteral("Planfräsen") || str == QStringLiteral("PlanfrÃ¤sen")) return BlockType::Facing;
    if (str == QStringLiteral("Contour") || str == QStringLiteral("Konturfräsen") || str == QStringLiteral("KonturfrÃ¤sen")) return BlockType::Contour;
    if (str == QStringLiteral("Pocket") || str == QStringLiteral("Taschenfräsen") || str == QStringLiteral("TaschenfrÃ¤sen")) return BlockType::Pocket;
    if (str == QStringLiteral("Slot") || str == QStringLiteral("Langloch")) return BlockType::Slot;
    if (str == QStringLiteral("HelixThread") || str == QStringLiteral("Helix / Gewinde")) return BlockType::HelixThread;
    if (str == QStringLiteral("Drill") || str == QStringLiteral("Bohrbild")) return BlockType::Drill;
    if (str == QStringLiteral("Stl3D") || str == QStringLiteral("3D-STL Fräsen") || str == QStringLiteral("3D-STL FrÃ¤sen")) return BlockType::Stl3D;
    if (str == QStringLiteral("PatternStart") || str == QStringLiteral("Muster Start")) return BlockType::PatternStart;
    if (str == QStringLiteral("PatternEnd") || str == QStringLiteral("Muster Ende")) return BlockType::PatternEnd;
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

    constexpr double kPi = 3.14159265358979323846;

    // Schnittwerte aus dem Block (Technologie-Reiter) statt der Werkzeug-Standardwerte
    Core::ToolDefinition cutTool = tool;
    if (feedRate > 0.0) cutTool.defaultFeedRate = feedRate;
    if (plungeFeedRate > 0.0) cutTool.plungeFeedRate = plungeFeedRate;
    if (spindleRpm > 0.0) cutTool.spindleSpeed = spindleRpm;

    Core::ToolDefinition finTool = finishTool;
    if (finishFeedRate > 0.0) finTool.defaultFeedRate = finishFeedRate;
    if (plungeFeedRate > 0.0) finTool.plungeFeedRate = plungeFeedRate;
    if (finishSpindleRpm > 0.0) finTool.spindleSpeed = finishSpindleRpm;

    auto appendPath = [](Toolpath& target, Toolpath part, int toolIdOverride) {
        for (auto& s : part.segments) {
            if (toolIdOverride > 0) s.toolId = toolIdOverride;
            target.addSegment(s);
        }
    };

    auto makeSeg = [&cutTool](MotionType motion, const Core::Vector3D& from, const Core::Vector3D& to, double feedValue) {
        PathSegment seg;
        seg.motion = motion;
        seg.startPos = from;
        seg.endPos = to;
        seg.feedRate = (motion == MotionType::Rapid) ? 0.0 : feedValue;
        seg.spindleRpm = cutTool.spindleSpeed;
        seg.toolId = cutTool.id;
        seg.toolDiameter = cutTool.diameter;
        return seg;
    };

    // Gemeinsame Kontur-Parameter: Fräsrichtung, An-/Abfahrt, Eintauchart, Haltestege
    auto contourParams = [this](ContourSide side) {
        ContourParams cp;
        cp.startZ = startZ;
        cp.targetZ = targetZ;
        cp.stepDown = stepDown;
        cp.side = side;
        cp.finishAllowance = 0.0;
        cp.clearanceZ = clearanceZ;
        cp.climbMilling = (millingDirection == 0);
        cp.leadType = leadType;
        cp.leadRadius = leadRadius;
        cp.entryType = approachType;
        cp.useTabs = useTabs;
        cp.tabCount = tabCount;
        cp.tabWidth = tabWidth;
        cp.tabHeight = tabHeight;
        return cp;
    };

    auto sideForMillingType = [this]() {
        if (millingType == MillingType::Outside) return ContourSide::Outside;
        if (millingType == MillingType::Inside) return ContourSide::Inside;
        return ContourSide::OnLine;
    };

    // Kontur: Schruppen mit Aufmaß, Schlichtgang auf Endmaß (volle Tiefe in einer Umrundung)
    auto millContour = [&](const Geometry::Contour& c, ContourSide side, const std::vector<double>& profileZ = {}) {
        const bool finishing = finishPass && side != ContourSide::OnLine;
        double deepest = targetZ;
        for (double z : profileZ) deepest = std::min(deepest, z);

        ContourParams rough = contourParams(side);
        rough.vertexZ = profileZ;
        if (finishing) {
            const double floorAllowance = std::max(0.0, finishStepDown);
            rough.finishAllowance = std::max(0.0, finishAllowance);
            rough.targetZ = std::min(startZ, targetZ + floorAllowance);
            for (double& z : rough.vertexZ) z = std::min(startZ, z + floorAllowance);
        }
        Toolpath result = ToolpathGenerator::generateContourMilling(c, cutTool, rough);
        if (finishing) {
            ContourParams fin = contourParams(side);
            fin.vertexZ = profileZ;
            fin.stepDown = std::max(0.1, startZ - deepest);
            appendPath(result, ToolpathGenerator::generateContourMilling(c, finTool, fin), finTool.id);
        }
        return result;
    };

    // Tasche: Schruppen mit Aufmaß, danach Boden und Wände schlichten
    auto millPocket = [&](const Geometry::Contour& boundary, const std::vector<Geometry::Contour>& pocketIslandContours) {
        const bool finishing = enableFinishing || finishPass;
        const double allowXY = finishing ? std::max(0.0, enableFinishing ? finishAllowanceXY : finishAllowance) : 0.0;
        const double allowZ = enableFinishing ? std::max(0.0, finishAllowanceZ) : 0.0;

        PocketParams pp;
        pp.startZ = startZ;
        pp.targetZ = std::min(startZ, targetZ + allowZ);
        pp.stepDown = stepDown;
        pp.stepOverRatio = (cutTool.diameter > 0.0) ? (stepOver / cutTool.diameter) : 0.5;
        pp.finishAllowance = allowXY;
        pp.clearanceZ = clearanceZ;
        pp.strategy = pocketStrategy;
        pp.climbMilling = (millingDirection == 0);
        pp.entryType = approachType;

        Toolpath result = ToolpathGenerator::generatePocketMilling(boundary, pocketIslandContours, cutTool, pp);
        if (!finishing) return result;

        if (allowZ > 1e-6) {
            PocketParams floorPass = pp;
            floorPass.startZ = pp.targetZ;
            floorPass.targetZ = targetZ;
            floorPass.stepDown = allowZ + 1.0; // eine Ebene
            floorPass.stepOverRatio = (finTool.diameter > 0.0) ? (stepOver / finTool.diameter) : 0.5;
            appendPath(result, ToolpathGenerator::generatePocketMilling(boundary, pocketIslandContours, finTool, floorPass), finTool.id);
        }

        ContourParams wall = contourParams(ContourSide::Inside);
        wall.stepDown = std::max(0.1, startZ - targetZ);
        wall.useTabs = false;
        appendPath(result, ToolpathGenerator::generateContourMilling(boundary, finTool, wall), finTool.id);
        wall.side = ContourSide::Outside;
        for (const auto& island : pocketIslandContours) {
            appendPath(result, ToolpathGenerator::generateContourMilling(island, finTool, wall), finTool.id);
        }
        return result;
    };

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
        tp = ToolpathGenerator::generateFacing(bounds, cutTool, fp);
        tp.operationName = name;
    } else if (type == BlockType::Contour) {
        Geometry::Contour c = contour;
        std::vector<double> profileZ;
        if (!segments.empty()) {
            // Kontur und Tiefe je Punkt aus den Segmenten (Z ENDE); ältere Dateien ohne Tiefen
            // übernehmen über applyStartDepth die Blocktiefe
            std::vector<Geometry::ContourSegment> segs = segments;
            Geometry::Contour::applyStartDepth(segs, startZ, targetZ);
            std::vector<double> segZ;
            const bool closed = contour.empty() ? true : contour.isClosed;
            Geometry::Contour fromSegments = Geometry::Contour::createFromSegments(segs, closed, &segZ);
            if (c.empty() || c.points.size() == fromSegments.points.size()) {
                c = fromSegments;
                bool varies = false;
                for (double z : segZ) varies = varies || std::abs(z - targetZ) > 1e-6;
                if (varies && segZ.size() == c.points.size()) profileZ = segZ;
            }
        }
        if (c.empty()) {
            c = Geometry::Contour::createRectangle(-30.0, -20.0, 60.0, 40.0);
        }
        tp = millContour(c, contourSide, profileZ);
        tp.operationName = name;
    } else if (type == BlockType::Slot) {
        // Langloch / Nut: Position, Eckenradius und parallele Wiederholungen
        const double rad = slotAngleDeg * kPi / 180.0;
        const double cs = std::cos(rad);
        const double sn = std::sin(rad);
        const int count = std::max(1, slotCount);
        for (int k = 0; k < count; ++k) {
            const double cx = posX - sn * k * slotSpacing;
            const double cy = posY + cs * k * slotSpacing;
            Geometry::Contour slotContour;
            if (slotCornerR > 1e-6 && slotCornerR < slotWidth * 0.5 - 1e-6) {
                slotContour = Geometry::Contour::createRoundedRectangle(cx, cy, slotLength, slotWidth, slotCornerR);
                for (auto& p : slotContour.points) {
                    const double lx = p.x - cx;
                    const double ly = p.y - cy;
                    p.x = cx + lx * cs - ly * sn;
                    p.y = cy + lx * sn + ly * cs;
                }
            } else {
                slotContour = Geometry::Contour::createSlot(cx, cy, slotLength, slotWidth, slotAngleDeg);
            }
            const Toolpath part = (millingType == MillingType::Pocket)
                ? millPocket(slotContour, {})
                : millContour(slotContour, sideForMillingType());
            appendPath(tp, part, 0);
        }
        tp.operationName = name;
    } else if (type == BlockType::HelixThread) {
        // Helix / Gewindefräsen: Position, Drehrichtung und Mehrgängigkeit
        const double pathRadius = helixInternal
            ? std::max(0.5, (helixDiameter - cutTool.diameter) * 0.5)
            : std::max(0.5, (helixDiameter + cutTool.diameter) * 0.5);
        const int starts = std::max(1, helixStarts);
        const double lead = std::max(0.1, helixPitch) * starts; // Steigung je Umdrehung

        for (int k = 0; k < starts; ++k) {
            auto hPoints = Geometry::Contour::createHelixPoints(0.0, 0.0, startZ, targetZ, pathRadius, lead);
            if (hPoints.empty()) continue;
            const double rot = 2.0 * kPi * k / starts;
            const double cr = std::cos(rot);
            const double sr = std::sin(rot);
            for (auto& p : hPoints) {
                const double lx = p.x;
                const double ly = helixCW ? -p.y : p.y; // Uhrzeigersinn: Drehsinn spiegeln
                p.x = posX + lx * cr - ly * sr;
                p.y = posY + lx * sr + ly * cr;
            }

            const Core::Vector3D startPt = hPoints.front();
            const Core::Vector3D above(startPt.x, startPt.y, clearanceZ);
            tp.addSegment(makeSeg(MotionType::Rapid, Core::Vector3D(posX, posY, clearanceZ), above, 0.0));
            tp.addSegment(makeSeg(MotionType::LinearFeed, above, startPt, cutTool.plungeFeedRate));
            for (size_t i = 1; i < hPoints.size(); ++i) {
                tp.addSegment(makeSeg(MotionType::LinearFeed, hPoints[i - 1], hPoints[i], cutTool.defaultFeedRate));
            }
            const Core::Vector3D endPt = hPoints.back();
            tp.addSegment(makeSeg(MotionType::Rapid, endPt, Core::Vector3D(endPt.x, endPt.y, clearanceZ), 0.0));
        }
        tp.operationName = name;
    } else if (type == BlockType::Pocket) {
        Geometry::Contour boundary;
        if (pocketShape == PocketShape::Rectangle) {
            boundary = (pocketCornerR > 1e-6)
                ? Geometry::Contour::createRoundedRectangle(posX + pocketWidthX * 0.5, posY + pocketDepthY * 0.5, pocketWidthX, pocketDepthY, pocketCornerR)
                : Geometry::Contour::createRectangle(posX, posY, pocketWidthX, pocketDepthY);
        } else if (pocketShape == PocketShape::Circle) {
            boundary = Geometry::Contour::createCircle(posX, posY, pocketRadius);
        } else {
            boundary = contour.empty() ? Geometry::Contour::createRectangle(posX - 25, posY - 25, 50, 50) : contour;
        }

        if (millingType == MillingType::Pocket) {
            std::vector<Geometry::Contour> compiledIslands;
            for (const auto& islandSegs : pocketIslands) {
                auto islandContour = Geometry::Contour::createFromSegments(islandSegs, true);
                if (!islandContour.empty()) compiledIslands.push_back(islandContour);
            }
            tp = millPocket(boundary, compiledIslands);
        } else {
            tp = millContour(boundary, sideForMillingType());
        }
        tp.operationName = name;
    } else if (type == BlockType::Drill) {
        // Bohrzyklen: 0 G81 Einfach, 1 G83 Spanbruch (zur R-Ebene), 2 G73 Tiefloch (kurzer Rückzug),
        //             3 G84 Gewinde, 4 G85 Ausbohren, 5 G86 Ausspindeln
        const auto holes = calculateDrillPositions();
        const double peck = std::max(0.2, peckDepth);
        const double rPlane = startZ + 1.0; // R-Ebene über dem Werkstück
        const double drillFeed = cutTool.plungeFeedRate;
        Core::Vector3D currentPos(0.0, 0.0, clearanceZ);
        auto moveTo = [&](MotionType motion, double x, double y, double z, double feedValue) {
            const Core::Vector3D target(x, y, z);
            tp.addSegment(makeSeg(motion, currentPos, target, feedValue));
            currentPos = target;
        };

        int holeIndex = 0;
        for (const auto& hole : holes) {
            const size_t holeFirstSeg = tp.segments.size();
            moveTo(MotionType::Rapid, hole.x, hole.y, clearanceZ, 0.0);
            moveTo(MotionType::Rapid, hole.x, hole.y, rPlane, 0.0);

            switch (drillCycle) {
                case 1:   // G83: nach jeder Zustellung zur R-Ebene
                case 2: { // G73: nur kurz zurückziehen (Spanbruch)
                    double curZ = std::min(startZ, rPlane);
                    while (curZ > targetZ + 1e-4) {
                        const double nextZ = std::max(targetZ, curZ - peck);
                        moveTo(MotionType::LinearFeed, hole.x, hole.y, nextZ, drillFeed);
                        curZ = nextZ;
                        if (curZ <= targetZ + 1e-4) break;
                        if (drillCycle == 1) {
                            moveTo(MotionType::Rapid, hole.x, hole.y, rPlane, 0.0);
                            moveTo(MotionType::LinearFeed, hole.x, hole.y, curZ + 0.3, cutTool.defaultFeedRate);
                        } else {
                            moveTo(MotionType::LinearFeed, hole.x, hole.y, curZ + 0.5, cutTool.defaultFeedRate);
                        }
                    }
                    moveTo(MotionType::Rapid, hole.x, hole.y, rPlane, 0.0);
                    break;
                }
                case 3:   // G84 Gewinde: hinein und im Vorschub heraus (Spindelumkehr an der Steuerung)
                case 4:   // G85 Ausbohren: hinein und im Vorschub heraus
                    moveTo(MotionType::LinearFeed, hole.x, hole.y, targetZ, drillFeed);
                    moveTo(MotionType::LinearFeed, hole.x, hole.y, rPlane, drillFeed);
                    break;
                default:  // G81 Einfach / G86 Ausspindeln: hinein, im Eilgang heraus
                    moveTo(MotionType::LinearFeed, hole.x, hole.y, targetZ, drillFeed);
                    moveTo(MotionType::Rapid, hole.x, hole.y, rPlane, 0.0);
                    break;
            }
            moveTo(MotionType::Rapid, hole.x, hole.y, clearanceZ, 0.0);

            // Bohrung für die Zyklus-Ausgabe im Postprozessor kennzeichnen
            for (size_t k = holeFirstSeg; k < tp.segments.size(); ++k) {
                auto& s = tp.segments[k];
                s.drillCycle = std::clamp(drillCycle, 0, 5);
                s.drillHole = holeIndex;
                s.drillDepthZ = targetZ;
                s.drillRPlaneZ = rPlane;
                s.drillPeck = peck;
                s.drillDwellSec = std::max(0.0, dwellTimeSec);
            }
            ++holeIndex;
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
                tp = ToolpathGenerator::generateStlMilling(activeMesh, bounds, cutTool, params);
                
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
                tp = ToolpathGenerator::generateStlMilling(activeMesh, bounds, cutTool, params);
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
        if (seg.toolDiameter <= 0.5) {
            seg.toolDiameter = (seg.toolId == finishTool.id) ? finishTool.diameter : tool.diameter;
        }
        // Kontur, Tasche und Langloch bringen eigene Drehzahlen mit (Schlichtwerkzeug), sonst Block-Drehzahl
        const bool ownRpm = (type == BlockType::Contour || type == BlockType::Pocket || type == BlockType::Slot);
        if (!ownRpm || seg.spindleRpm <= 0.0) seg.spindleRpm = spindleRpm;
        seg.coolantOn = coolantOn;
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
    obj[QStringLiteral("type")] = blockTypeKey(type);
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

    // Allgemeine Bearbeitungsoptionen
    obj[QStringLiteral("visible")] = visible;
    obj[QStringLiteral("millingType")] = static_cast<int>(millingType);
    obj[QStringLiteral("startSide")] = static_cast<int>(startSide);
    obj[QStringLiteral("posX")] = posX;
    obj[QStringLiteral("posY")] = posY;
    obj[QStringLiteral("millingDirection")] = millingDirection;
    obj[QStringLiteral("coolantOn")] = coolantOn;
    obj[QStringLiteral("finishPass")] = finishPass;
    obj[QStringLiteral("finishStepDown")] = finishStepDown;
    obj[QStringLiteral("approachType")] = approachType;

    // Planfräsen
    obj[QStringLiteral("areaWidth")] = areaWidth;
    obj[QStringLiteral("areaDepth")] = areaDepth;
    obj[QStringLiteral("useStockDimensions")] = useStockDimensions;

    // Kontur
    obj[QStringLiteral("contourSide")] = static_cast<int>(contourSide);
    obj[QStringLiteral("finishAllowance")] = finishAllowance;
    obj[QStringLiteral("leadRadius")] = leadRadius;
    obj[QStringLiteral("leadType")] = leadType;
    obj[QStringLiteral("useTabs")] = useTabs;
    obj[QStringLiteral("tabWidth")] = tabWidth;
    obj[QStringLiteral("tabHeight")] = tabHeight;
    obj[QStringLiteral("tabCount")] = tabCount;

    // Tasche: Schruppen/Schlichten und Inseln
    obj[QStringLiteral("pocketCornerR")] = pocketCornerR;
    obj[QStringLiteral("pocketStrategy")] = pocketStrategy;
    obj[QStringLiteral("enableFinishing")] = enableFinishing;
    obj[QStringLiteral("finishAllowanceXY")] = finishAllowanceXY;
    obj[QStringLiteral("finishAllowanceZ")] = finishAllowanceZ;
    obj[QStringLiteral("finishFeedRate")] = finishFeedRate;
    obj[QStringLiteral("finishSpindleRpm")] = finishSpindleRpm;
    if (!pocketIslands.empty()) {
        QJsonArray islandArray;
        for (const auto& island : pocketIslands) {
            islandArray.append(segmentsToJson(island));
        }
        obj[QStringLiteral("pocketIslands")] = islandArray;
    }

    // Langloch
    obj[QStringLiteral("slotCornerR")] = slotCornerR;
    obj[QStringLiteral("slotCount")] = slotCount;
    obj[QStringLiteral("slotSpacing")] = slotSpacing;

    // Helix / Gewinde
    obj[QStringLiteral("helixCW")] = helixCW;
    obj[QStringLiteral("helixStarts")] = helixStarts;

    // Bohrzyklus
    obj[QStringLiteral("drillCycle")] = drillCycle;
    obj[QStringLiteral("dwellTimeSec")] = dwellTimeSec;

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

    if (type == BlockType::PatternStart) {
        obj[QStringLiteral("patternType")] = static_cast<int>(patternType);
        obj[QStringLiteral("patternCountX")] = patternCountX;
        obj[QStringLiteral("patternCountY")] = patternCountY;
        obj[QStringLiteral("patternSpacingX")] = patternSpacingX;
        obj[QStringLiteral("patternSpacingY")] = patternSpacingY;
        obj[QStringLiteral("patternAngleDeg")] = patternAngleDeg;
        obj[QStringLiteral("patternStepAngleDeg")] = patternStepAngleDeg;
        obj[QStringLiteral("patternCenterX")] = patternCenterX;
        obj[QStringLiteral("patternCenterY")] = patternCenterY;
        obj[QStringLiteral("patternMirrorX")] = patternMirrorX;
        obj[QStringLiteral("patternMirrorY")] = patternMirrorY;
    }

    obj[QStringLiteral("stlFilePath")] = stlFilePath;
    obj[QStringLiteral("stlStrategy")] = stlStrategyToString(stlStrategy);
    obj[QStringLiteral("stlStepOver")] = stlStepOver;
    obj[QStringLiteral("stlStepDown")] = stlStepDown;
    obj[QStringLiteral("stlAllowance")] = stlAllowance;
    obj[QStringLiteral("stlSampleStep")] = stlSampleStep;
    obj[QStringLiteral("stlUseStockDims")] = stlUseStockDims;

    // Segmente serialisieren falls vorhanden
    if (!segments.empty()) {
        obj[QStringLiteral("segments")] = segmentsToJson(segments);
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

    // Fehlende Schlüssel (ältere .gprog-Dateien) behalten den Standardwert
    auto readDouble = [&json](const QString& key, double& value) {
        if (json.contains(key)) value = json[key].toDouble();
    };
    auto readInt = [&json](const QString& key, int& value) {
        if (json.contains(key)) value = json[key].toInt();
    };
    auto readBool = [&json](const QString& key, bool& value) {
        if (json.contains(key)) value = json[key].toBool();
    };

    // Allgemeine Bearbeitungsoptionen
    readBool(QStringLiteral("visible"), b.visible);
    if (json.contains(QStringLiteral("millingType"))) b.millingType = static_cast<MillingType>(json[QStringLiteral("millingType")].toInt());
    if (json.contains(QStringLiteral("startSide"))) b.startSide = static_cast<FrameStartSide>(json[QStringLiteral("startSide")].toInt());
    readDouble(QStringLiteral("posX"), b.posX);
    readDouble(QStringLiteral("posY"), b.posY);
    readInt(QStringLiteral("millingDirection"), b.millingDirection);
    readBool(QStringLiteral("coolantOn"), b.coolantOn);
    readBool(QStringLiteral("finishPass"), b.finishPass);
    readDouble(QStringLiteral("finishStepDown"), b.finishStepDown);
    readInt(QStringLiteral("approachType"), b.approachType);

    // Planfräsen
    readDouble(QStringLiteral("areaWidth"), b.areaWidth);
    readDouble(QStringLiteral("areaDepth"), b.areaDepth);
    readBool(QStringLiteral("useStockDimensions"), b.useStockDimensions);

    // Kontur
    if (json.contains(QStringLiteral("contourSide"))) b.contourSide = static_cast<ContourSide>(json[QStringLiteral("contourSide")].toInt());
    readDouble(QStringLiteral("finishAllowance"), b.finishAllowance);
    readDouble(QStringLiteral("leadRadius"), b.leadRadius);
    readInt(QStringLiteral("leadType"), b.leadType);
    readBool(QStringLiteral("useTabs"), b.useTabs);
    readDouble(QStringLiteral("tabWidth"), b.tabWidth);
    readDouble(QStringLiteral("tabHeight"), b.tabHeight);
    readInt(QStringLiteral("tabCount"), b.tabCount);

    // Tasche: Schruppen/Schlichten und Inseln
    readDouble(QStringLiteral("pocketCornerR"), b.pocketCornerR);
    readInt(QStringLiteral("pocketStrategy"), b.pocketStrategy);
    readBool(QStringLiteral("enableFinishing"), b.enableFinishing);
    readDouble(QStringLiteral("finishAllowanceXY"), b.finishAllowanceXY);
    readDouble(QStringLiteral("finishAllowanceZ"), b.finishAllowanceZ);
    readDouble(QStringLiteral("finishFeedRate"), b.finishFeedRate);
    readDouble(QStringLiteral("finishSpindleRpm"), b.finishSpindleRpm);
    if (json.contains(QStringLiteral("pocketIslands"))) {
        b.pocketIslands.clear();
        for (const auto& island : json[QStringLiteral("pocketIslands")].toArray()) {
            b.pocketIslands.push_back(segmentsFromJson(island.toArray()));
        }
    }

    // Langloch
    readDouble(QStringLiteral("slotCornerR"), b.slotCornerR);
    readInt(QStringLiteral("slotCount"), b.slotCount);
    readDouble(QStringLiteral("slotSpacing"), b.slotSpacing);

    // Helix / Gewinde
    readBool(QStringLiteral("helixCW"), b.helixCW);
    readInt(QStringLiteral("helixStarts"), b.helixStarts);

    // Bohrzyklus
    readInt(QStringLiteral("drillCycle"), b.drillCycle);
    readDouble(QStringLiteral("dwellTimeSec"), b.dwellTimeSec);

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

    // Muster
    if (json.contains(QStringLiteral("patternType"))) b.patternType = static_cast<PatternType>(json[QStringLiteral("patternType")].toInt());
    if (json.contains(QStringLiteral("patternCountX"))) b.patternCountX = json[QStringLiteral("patternCountX")].toInt();
    if (json.contains(QStringLiteral("patternCountY"))) b.patternCountY = json[QStringLiteral("patternCountY")].toInt();
    if (json.contains(QStringLiteral("patternSpacingX"))) b.patternSpacingX = json[QStringLiteral("patternSpacingX")].toDouble();
    if (json.contains(QStringLiteral("patternSpacingY"))) b.patternSpacingY = json[QStringLiteral("patternSpacingY")].toDouble();
    if (json.contains(QStringLiteral("patternAngleDeg"))) b.patternAngleDeg = json[QStringLiteral("patternAngleDeg")].toDouble();
    if (json.contains(QStringLiteral("patternStepAngleDeg"))) b.patternStepAngleDeg = json[QStringLiteral("patternStepAngleDeg")].toDouble();
    if (json.contains(QStringLiteral("patternCenterX"))) b.patternCenterX = json[QStringLiteral("patternCenterX")].toDouble();
    if (json.contains(QStringLiteral("patternCenterY"))) b.patternCenterY = json[QStringLiteral("patternCenterY")].toDouble();
    if (json.contains(QStringLiteral("patternMirrorX"))) b.patternMirrorX = json[QStringLiteral("patternMirrorX")].toBool();
    if (json.contains(QStringLiteral("patternMirrorY"))) b.patternMirrorY = json[QStringLiteral("patternMirrorY")].toBool();

    if (json.contains(QStringLiteral("stlFilePath"))) b.stlFilePath = json[QStringLiteral("stlFilePath")].toString();
    if (json.contains(QStringLiteral("stlStrategy"))) b.stlStrategy = stringToStlStrategy(json[QStringLiteral("stlStrategy")].toString());
    if (json.contains(QStringLiteral("stlStepOver"))) b.stlStepOver = json[QStringLiteral("stlStepOver")].toDouble();
    if (json.contains(QStringLiteral("stlStepDown"))) b.stlStepDown = json[QStringLiteral("stlStepDown")].toDouble();
    if (json.contains(QStringLiteral("stlAllowance"))) b.stlAllowance = json[QStringLiteral("stlAllowance")].toDouble();
    if (json.contains(QStringLiteral("stlSampleStep"))) b.stlSampleStep = json[QStringLiteral("stlSampleStep")].toDouble();
    if (json.contains(QStringLiteral("stlUseStockDims"))) b.stlUseStockDims = json[QStringLiteral("stlUseStockDims")].toBool();

    if (json.contains(QStringLiteral("segments"))) {
        b.segments = segmentsFromJson(json[QStringLiteral("segments")].toArray());
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
