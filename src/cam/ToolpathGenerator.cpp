#include "ToolpathGenerator.h"
#include <cmath>
#include <algorithm>
#include "geometry/PolygonOffset.h"

namespace GeminiCNC::CAM {

Toolpath ToolpathGenerator::generateFacing(const Core::BoundingBox& stockBounds,
                                          const Core::ToolDefinition& tool,
                                          const FacingParams& params) {
    Toolpath tp(QStringLiteral("PlanfrÃ¤sen (Facing)"));
    if (!stockBounds.isValid()) return tp;

    const double toolRadius = tool.diameter * 0.5;
    const double minX = stockBounds.minPoint.x - toolRadius - params.extension;
    const double maxX = stockBounds.maxPoint.x + toolRadius + params.extension;
    // Y-Grenzen: Fräser-Mittelpunkt fährt von minY bis maxY,
    // dabei überragt der Fräserradius die Stock-Kante → volle Abdeckung
    const double minY = stockBounds.minPoint.y - toolRadius;
    const double maxY = stockBounds.maxPoint.y + toolRadius;

    double stepOver = (params.stepOver > 0.1) ? params.stepOver : tool.effectiveStepOver();
    if (stepOver < 0.5) stepOver = tool.diameter * 0.5;

    double currentZ = params.startZ;
    const double targetZ = params.targetZ;
    const double stepDown = std::max(0.1, params.stepDown);

    Core::Vector3D currentPos(minX, minY, params.clearanceZ);

    while (currentZ > targetZ - 1e-5) {
        currentZ -= stepDown;
        if (currentZ < targetZ) currentZ = targetZ;

        // Anfahrt auf Eilgang Ã¼ber Startpunkt
        tp.addSegment({MotionType::Rapid, currentPos, {minX, minY, params.clearanceZ}, {}, 0, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
        currentPos = {minX, minY, params.clearanceZ};

        // Eintauchen auf Z-Tiefe
        tp.addSegment({MotionType::LinearFeed, currentPos, {minX, minY, currentZ}, {}, tool.plungeFeedRate, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
        currentPos = {minX, minY, currentZ};

        // MÃ¤ander-Zickzack-Bahnen in Y
        double y = minY;
        bool leftToRight = true;

        while (y <= maxY + 1e-4) {
            double targetX = leftToRight ? maxX : minX;
            tp.addSegment({MotionType::LinearFeed, currentPos, {targetX, y, currentZ}, {}, tool.defaultFeedRate, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
            currentPos = {targetX, y, currentZ};

            y += stepOver;
            if (y <= maxY + 1e-4) {
                // Parallel-Versatz
                tp.addSegment({MotionType::LinearFeed, currentPos, {targetX, y, currentZ}, {}, tool.defaultFeedRate, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
                currentPos = {targetX, y, currentZ};
            }
            leftToRight = !leftToRight;
        }

        // RÃ¼ckzug auf SicherheitshÃ¶he
        tp.addSegment({MotionType::Rapid, currentPos, {currentPos.x, currentPos.y, params.clearanceZ}, {}, 0, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
        currentPos = {currentPos.x, currentPos.y, params.clearanceZ};

        if (std::abs(currentZ - targetZ) < 1e-5) break;
    }

    return tp;
}

Toolpath ToolpathGenerator::generateContourMilling(const Geometry::Contour& contour,
                                                  const Core::ToolDefinition& tool,
                                                  const ContourParams& params) {
    Toolpath tp(QStringLiteral("KonturfrÃ¤sen"));
    if (contour.points.size() < 2) return tp;

    // Offset-Kontur berechnen
    double offsetDist = 0.0;
    const double r = tool.diameter * 0.5 + params.finishAllowance;
    if (params.side == ContourSide::Outside) {
        offsetDist = contour.isClockwise() ? -r : r;
    } else if (params.side == ContourSide::Inside) {
        offsetDist = contour.isClockwise() ? r : -r;
    }

    Geometry::Contour pathContour = (params.side != ContourSide::OnLine)
                                    ? contour.createOffset(offsetDist)
                                    : contour;

    if (pathContour.points.empty()) return tp;

    const double clearanceZ = params.clearanceZ;
    const double stepDown = std::max(0.1, params.stepDown);
    double currentZ = params.startZ;
    const double targetZ = params.targetZ;

    const auto& startPt = pathContour.points.front();
    Core::Vector3D currentPos(startPt.x, startPt.y, clearanceZ);

    // Initialer Eilgang Ã¼ber ersten Konturpunkt
    tp.addSegment({MotionType::Rapid, currentPos, {startPt.x, startPt.y, clearanceZ}, {}, 0, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
    currentPos = {startPt.x, startPt.y, clearanceZ};

    while (currentZ > targetZ - 1e-5) {
        currentZ -= stepDown;
        if (currentZ < targetZ) currentZ = targetZ;

        // Eintauchen auf aktuelle Z-Ebene
        tp.addSegment({MotionType::LinearFeed, currentPos, {currentPos.x, currentPos.y, currentZ}, {}, tool.plungeFeedRate, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
        currentPos = {currentPos.x, currentPos.y, currentZ};

        // Entlang der Kontur abfahren
        const size_t ptCount = pathContour.points.size();
        for (size_t i = 1; i < ptCount; ++i) {
            const auto& pt = pathContour.points[i];
            Core::Vector3D nextPos(pt.x, pt.y, currentZ);
            tp.addSegment({MotionType::LinearFeed, currentPos, nextPos, {}, tool.defaultFeedRate, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
            currentPos = nextPos;
        }

        // Falls geschlossen, zurÃ¼ck zum Startpunkt
        if (pathContour.isClosed) {
            Core::Vector3D loopClosePos(startPt.x, startPt.y, currentZ);
            tp.addSegment({MotionType::LinearFeed, currentPos, loopClosePos, {}, tool.defaultFeedRate, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
            currentPos = loopClosePos;
        }

        if (std::abs(currentZ - targetZ) < 1e-5) break;
    }

    // Rückzug
    tp.addSegment({MotionType::Rapid, currentPos, {currentPos.x, currentPos.y, clearanceZ}, {}, 0, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
    return tp;
}

Toolpath ToolpathGenerator::generatePocketMilling(const Geometry::Contour& boundary,
                                                 const Core::ToolDefinition& tool,
                                                 const PocketParams& params) {
    // Wrapper für Taschen ohne Inseln
    return generatePocketMilling(boundary, {}, tool, params);
}

Toolpath ToolpathGenerator::generatePocketMilling(const Geometry::Contour& boundary,
                                                 const std::vector<Geometry::Contour>& islands,
                                                 const Core::ToolDefinition& tool,
                                                 const PocketParams& params) {
    Toolpath tp(QStringLiteral("Taschenfräsen (Pocketing)"));
    if (boundary.points.size() < 3) return tp;

    const double toolRadius = tool.diameter * 0.5;
    const double stepOver = tool.diameter * std::clamp(params.stepOverRatio, 0.1, 0.9);
    const double clearanceZ = params.clearanceZ;
    const double stepDown = std::max(0.1, params.stepDown);
    const double targetZ = params.targetZ;

    // 1. Konzentrische Inset-Rings über PolygonOffset berechnen
    std::vector<std::vector<Geometry::Contour>> rings = 
        Geometry::PolygonOffset::generatePocketContours(boundary, islands, toolRadius, stepOver, params.finishAllowance);

    if (rings.empty()) {
        return tp; // Tasche zu klein für diesen Fräser oder ungültig
    }

    // 2. Von innen nach außen abfahren (Conventional)
    std::reverse(rings.begin(), rings.end());

    double currentZ = params.startZ;
    while (currentZ > targetZ - 1e-5) {
        currentZ -= stepDown;
        if (currentZ < targetZ) currentZ = targetZ;

        // Eintauchen im Zentrum des allersten (innersten) Rings
        const auto& firstRingContours = rings.front();
        if (firstRingContours.empty()) break;
        
        const auto& centerPt = firstRingContours.front().points.front();
        Core::Vector3D currentPos(centerPt.x, centerPt.y, clearanceZ);

        // Anfahren Eilgang
        tp.addSegment({MotionType::Rapid, currentPos, {centerPt.x, centerPt.y, clearanceZ}, {}, 0, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
        // Eintauchen Vorschub
        tp.addSegment({MotionType::LinearFeed, {centerPt.x, centerPt.y, clearanceZ}, {centerPt.x, centerPt.y, currentZ}, {}, tool.plungeFeedRate, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
        currentPos = {centerPt.x, centerPt.y, currentZ};

        // 3. Alle Ringe abfahren
        for (const auto& ringContours : rings) {
            for (const auto& contour : ringContours) {
                if (contour.points.empty()) continue;

                // Zum Startpunkt der aktuellen Kontur fahren (G0 knapp über Werkstück oder G1 wenn nahe)
                const auto& startPt = contour.points.front();
                Core::Vector3D nextStart(startPt.x, startPt.y, currentZ);
                
                // Simpler Retract-Move zum nächsten Ringteil wenn weit weg
                double distSq = (currentPos.x - nextStart.x) * (currentPos.x - nextStart.x) + 
                                (currentPos.y - nextStart.y) * (currentPos.y - nextStart.y);
                if (distSq > stepOver * stepOver * 4.0) {
                    tp.addSegment({MotionType::Rapid, currentPos, {currentPos.x, currentPos.y, currentZ + 1.0}, {}, 0, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
                    tp.addSegment({MotionType::Rapid, {currentPos.x, currentPos.y, currentZ + 1.0}, {nextStart.x, nextStart.y, currentZ + 1.0}, {}, 0, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
                    tp.addSegment({MotionType::LinearFeed, {nextStart.x, nextStart.y, currentZ + 1.0}, nextStart, {}, tool.plungeFeedRate, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
                } else {
                    // Direkte Verbindung
                    tp.addSegment({MotionType::LinearFeed, currentPos, nextStart, {}, tool.defaultFeedRate, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
                }
                currentPos = nextStart;

                // Konturpunkte abfahren
                for (size_t i = 1; i < contour.points.size(); ++i) {
                    const auto& pt = contour.points[i];
                    Core::Vector3D nextPos(pt.x, pt.y, currentZ);
                    tp.addSegment({MotionType::LinearFeed, currentPos, nextPos, {}, tool.defaultFeedRate, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
                    currentPos = nextPos;
                }

                // Kontur schließen (isClosed)
                if (contour.isClosed) {
                    Core::Vector3D closePos(startPt.x, startPt.y, currentZ);
                    tp.addSegment({MotionType::LinearFeed, currentPos, closePos, {}, tool.defaultFeedRate, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
                    currentPos = closePos;
                }
            }
        }

        // Rückzug auf clearanceZ am Ende jeder Ebene
        tp.addSegment({MotionType::Rapid, currentPos, {currentPos.x, currentPos.y, clearanceZ}, {}, 0, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
        
        if (std::abs(currentZ - targetZ) < 1e-5) break;
    }

    return tp;
}

Toolpath ToolpathGenerator::generateStockRoughing(const Geometry::Mesh& stockMesh,
                                                 const Geometry::Mesh& targetPartMesh,
                                                 const Core::ToolDefinition& tool,
                                                 const StockRoughingParams& params) {
    Toolpath tp(QStringLiteral("3D Rohteilschruppen (Stock Roughing)"));

    const auto& stockBox = stockMesh.boundingBox;
    const auto& partBox = targetPartMesh.boundingBox;

    if (!stockBox.isValid()) return tp;

    const double startZ = stockBox.maxPoint.z;
    const double targetZ = partBox.isValid() ? partBox.minPoint.z : stockBox.minPoint.z;
    const double stepDown = std::max(0.2, params.stepDown);
    const double stepOver = tool.diameter * std::clamp(params.stepOverRatio, 0.2, 0.8);

    double currentZ = startZ;
    const double toolRadius = tool.diameter * 0.5 + params.finishAllowance;

    const double minX = stockBox.minPoint.x + toolRadius;
    const double maxX = stockBox.maxPoint.x - toolRadius;
    const double minY = stockBox.minPoint.y + toolRadius;
    const double maxY = stockBox.maxPoint.y - toolRadius;

    Core::Vector3D currentPos(minX, minY, params.clearanceZ);

    while (currentZ > targetZ - 1e-5) {
        currentZ -= stepDown;
        if (currentZ < targetZ) currentZ = targetZ;

        // Anfahrt auf Eilgang Ã¼ber Startpunkt der Schicht
        tp.addSegment({MotionType::Rapid, currentPos, {minX, minY, params.clearanceZ}, {}, 0, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
        tp.addSegment({MotionType::LinearFeed, {minX, minY, params.clearanceZ}, {minX, minY, currentZ}, {}, tool.plungeFeedRate, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
        currentPos = {minX, minY, currentZ};

        double y = minY;
        bool leftToRight = true;

        while (y <= maxY + 1e-4) {
            double targetX = leftToRight ? maxX : minX;

            // In dieser Schicht abtragen
            tp.addSegment({MotionType::LinearFeed, currentPos, {targetX, y, currentZ}, {}, tool.defaultFeedRate, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
            currentPos = {targetX, y, currentZ};

            y += stepOver;
            if (y <= maxY + 1e-4) {
                tp.addSegment({MotionType::LinearFeed, currentPos, {targetX, y, currentZ}, {}, tool.defaultFeedRate, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
                currentPos = {targetX, y, currentZ};
            }
            leftToRight = !leftToRight;
        }

        // RÃ¼ckzug
        tp.addSegment({MotionType::Rapid, currentPos, {currentPos.x, currentPos.y, params.clearanceZ}, {}, 0, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
        currentPos = {currentPos.x, currentPos.y, params.clearanceZ};

        if (std::abs(currentZ - targetZ) < 1e-5) break;
    }

    return tp;
}

Toolpath ToolpathGenerator::generate3DSurfaceFinishing(const Geometry::Mesh& targetPartMesh,
                                                      const Core::ToolDefinition& tool,
                                                      const SurfaceFinishingParams& params) {
    Toolpath tp(QStringLiteral("3D-FreiformflÃ¤chen-Schlichten"));
    if (targetPartMesh.isEmpty()) return tp;

    const auto& bbox = targetPartMesh.boundingBox;
    if (!bbox.isValid()) return tp;

    const double toolRadius = tool.diameter * 0.5;
    const double stepOver = std::max(0.05, params.stepOver);
    const double sampleDx = std::max(0.1, params.sampleStep);

    const double minX = bbox.minPoint.x - toolRadius;
    const double maxX = bbox.maxPoint.x + toolRadius;
    const double minY = bbox.minPoint.y - toolRadius;
    const double maxY = bbox.maxPoint.y + toolRadius;

    // Lambda zur Bestimmung der 3D-Z-HÃ¶he auf dem Mesh fÃ¼r (x, y)
    auto queryMeshZ = [&](double qx, double qy) -> double {
        double maxZ = bbox.minPoint.z;
        bool found = false;

        for (const auto& tri : targetPartMesh.triangles) {
            const auto& v0 = targetPartMesh.vertices[tri.i0];
            const auto& v1 = targetPartMesh.vertices[tri.i1];
            const auto& v2 = targetPartMesh.vertices[tri.i2];

            // Bounding Box Check 2D
            float triMinX = std::min({v0.x, v1.x, v2.x});
            float triMaxX = std::max({v0.x, v1.x, v2.x});
            float triMinY = std::min({v0.y, v1.y, v2.y});
            float triMaxY = std::max({v0.y, v1.y, v2.y});

            if (qx < triMinX || qx > triMaxX || qy < triMinY || qy > triMaxY) continue;

            // Baryzentrische Koordinaten in 2D
            double det = (v1.y - v2.y) * (v0.x - v2.x) + (v2.x - v1.x) * (v0.y - v2.y);
            if (std::abs(det) < 1e-9) continue;

            double w0 = ((v1.y - v2.y) * (qx - v2.x) + (v2.x - v1.x) * (qy - v2.y)) / det;
            double w1 = ((v2.y - v0.y) * (qx - v2.x) + (v0.x - v2.x) * (qy - v2.y)) / det;
            double w2 = 1.0 - w0 - w1;

            if (w0 >= -1e-4 && w1 >= -1e-4 && w2 >= -1e-4) {
                double z = w0 * v0.z + w1 * v1.z + w2 * v2.z;
                if (!found || z > maxZ) {
                    maxZ = z;
                    found = true;
                }
            }
        }
        return found ? maxZ : bbox.minPoint.z;
    };

    Core::Vector3D currentPos(minX, minY, params.clearanceZ);
    tp.addSegment({MotionType::Rapid, currentPos, {minX, minY, params.clearanceZ}, {}, 0, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});

    double y = minY;
    bool leftToRight = true;

    while (y <= maxY + 1e-4) {
        double startLineX = leftToRight ? minX : maxX;
        double endLineX = leftToRight ? maxX : minX;
        double dir = leftToRight ? 1.0 : -1.0;

        // Anfahrt Ã¼ber ersten Punkt der Zeile
        double firstZ = queryMeshZ(startLineX, y);
        tp.addSegment({MotionType::Rapid, currentPos, {startLineX, y, params.clearanceZ}, {}, 0, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
        tp.addSegment({MotionType::LinearFeed, {startLineX, y, params.clearanceZ}, {startLineX, y, firstZ}, {}, tool.plungeFeedRate, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
        currentPos = {startLineX, y, firstZ};

        double curX = startLineX;
        while ((dir > 0 && curX < endLineX) || (dir < 0 && curX > endLineX)) {
            curX += dir * sampleDx;
            if ((dir > 0 && curX > endLineX) || (dir < 0 && curX < endLineX)) curX = endLineX;

            double surfaceZ = queryMeshZ(curX, y);
            Core::Vector3D nextPos(curX, y, surfaceZ);
            tp.addSegment({MotionType::LinearFeed, currentPos, nextPos, {}, tool.defaultFeedRate, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
            currentPos = nextPos;
        }

        // RÃ¼ckzug am Zeilenende
        tp.addSegment({MotionType::Rapid, currentPos, {currentPos.x, currentPos.y, params.clearanceZ}, {}, 0, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
        currentPos = {currentPos.x, currentPos.y, params.clearanceZ};

        y += stepOver;
        leftToRight = !leftToRight;
    }

    return tp;
}

Toolpath ToolpathGenerator::generateStlMilling(const Geometry::Mesh& mesh,
                                              const Core::BoundingBox& stockBounds,
                                              const Core::ToolDefinition& tool,
                                              const StlMillingParams& params) {
    Toolpath tp(QStringLiteral("3D-STL FrÃ¤sen"));
    if (mesh.isEmpty() || mesh.triangles.empty()) return tp;

    Core::BoundingBox areaBox = stockBounds.isValid() ? stockBounds : mesh.boundingBox;
    if (!areaBox.isValid()) return tp;

    const double toolRadius = std::max(0.5, tool.diameter * 0.5);
    const double sampleDx = std::max(0.1, params.sampleStep);
    const double clearanceZ = std::max(areaBox.maxPoint.z + 1.0, params.clearanceZ);

    // 1. Spatial Acceleration Grid fÃ¼r blitzschnelle Triangeldurchsuchung
    const double gridMinX = mesh.boundingBox.minPoint.x;
    const double gridMinY = mesh.boundingBox.minPoint.y;
    const double gridWidth = std::max(1.0, mesh.boundingBox.widthX());
    const double gridDepth = std::max(1.0, mesh.boundingBox.depthY());

    const int gridCols = std::clamp(static_cast<int>(gridWidth / 4.0), 16, 128);
    const int gridRows = std::clamp(static_cast<int>(gridDepth / 4.0), 16, 128);
    const double cellW = gridWidth / gridCols;
    const double cellH = gridDepth / gridRows;

    std::vector<std::vector<uint32_t>> spatialGrid(gridCols * gridRows);
    for (size_t tIdx = 0; tIdx < mesh.triangles.size(); ++tIdx) {
        const auto& tri = mesh.triangles[tIdx];
        const auto& v0 = mesh.vertices[tri.i0];
        const auto& v1 = mesh.vertices[tri.i1];
        const auto& v2 = mesh.vertices[tri.i2];

        float minTx = std::min({v0.x, v1.x, v2.x});
        float maxTx = std::max({v0.x, v1.x, v2.x});
        float minTy = std::min({v0.y, v1.y, v2.y});
        float maxTy = std::max({v0.y, v1.y, v2.y});

        int c0 = std::clamp(static_cast<int>((minTx - gridMinX) / cellW), 0, gridCols - 1);
        int c1 = std::clamp(static_cast<int>((maxTx - gridMinX) / cellW), 0, gridCols - 1);
        int r0 = std::clamp(static_cast<int>((minTy - gridMinY) / cellH), 0, gridRows - 1);
        int r1 = std::clamp(static_cast<int>((maxTy - gridMinY) / cellH), 0, gridRows - 1);

        for (int r = r0; r <= r1; ++r) {
            for (int c = c0; c <= c1; ++c) {
                spatialGrid[r * gridCols + c].push_back(static_cast<uint32_t>(tIdx));
            }
        }
    }

    // Punkt-Strahlschnitt mit STL-OberflÃ¤che
    auto queryPointZ = [&](double qx, double qy) -> double {
        int c = static_cast<int>((qx - gridMinX) / cellW);
        int r = static_cast<int>((qy - gridMinY) / cellH);
        if (c < 0 || c >= gridCols || r < 0 || r >= gridRows) {
            return areaBox.minPoint.z;
        }

        const auto& cellTris = spatialGrid[r * gridCols + c];
        double highestZ = areaBox.minPoint.z;
        bool hit = false;

        for (uint32_t tIdx : cellTris) {
            const auto& tri = mesh.triangles[tIdx];
            const auto& v0 = mesh.vertices[tri.i0];
            const auto& v1 = mesh.vertices[tri.i1];
            const auto& v2 = mesh.vertices[tri.i2];

            double det = (v1.y - v2.y) * (v0.x - v2.x) + (v2.x - v1.x) * (v0.y - v2.y);
            if (std::abs(det) < 1e-9) continue;

            double w0 = ((v1.y - v2.y) * (qx - v2.x) + (v2.x - v1.x) * (qy - v2.y)) / det;
            double w1 = ((v2.y - v0.y) * (qx - v2.x) + (v0.x - v2.x) * (qy - v2.y)) / det;
            double w2 = 1.0 - w0 - w1;

            if (w0 >= -1e-4 && w1 >= -1e-4 && w2 >= -1e-4) {
                double z = w0 * v0.z + w1 * v1.z + w2 * v2.z;
                if (!hit || z > highestZ) {
                    highestZ = z;
                    hit = true;
                }
            }
        }
        return hit ? std::max(highestZ, areaBox.minPoint.z) : areaBox.minPoint.z;
    };

    // Werkzeugradius-Kompensation (untere StirnflÃ¤che bzw. Kugelradius)
    auto queryToolZ = [&](double cx, double cy) -> double {
        bool isBallEnd = (tool.type == Core::ToolType::BallMill);
        double maxZ = queryPointZ(cx, cy);

        // Mehrere Abtastpunkte auf dem Werkzeugradius
        constexpr int numRings = 2;
        constexpr int ptsPerRing = 6;
        for (int ring = 1; ring <= numRings; ++ring) {
            double rFrac = static_cast<double>(ring) / numRings;
            double curR = toolRadius * rFrac;
            double zOffset = isBallEnd ? (toolRadius - std::sqrt(std::max(0.0, toolRadius * toolRadius - curR * curR))) : 0.0;

            for (int p = 0; p < ptsPerRing; ++p) {
                double ang = p * (2.0 * M_PI / ptsPerRing);
                double sx = cx + curR * std::cos(ang);
                double sy = cy + curR * std::sin(ang);
                double sZ = queryPointZ(sx, sy) - zOffset;
                if (sZ > maxZ) maxZ = sZ;
            }
        }

        return std::clamp(maxZ + params.finishAllowance, areaBox.minPoint.z, areaBox.maxPoint.z + 10.0);
    };

    const double minX = areaBox.minPoint.x;
    const double maxX = areaBox.maxPoint.x;
    const double minY = areaBox.minPoint.y;
    const double maxY = areaBox.maxPoint.y;

    Core::Vector3D currentPos(minX, minY, clearanceZ);
    tp.addSegment({MotionType::Rapid, currentPos, {minX, minY, clearanceZ}, {}, 0, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});

    bool doRoughing = (params.mode == StlMillingMode::RoughAndFinish || params.mode == StlMillingMode::RoughOnly);
    bool doFinishing = (params.mode == StlMillingMode::RoughAndFinish || params.mode == StlMillingMode::FinishOnly ||
                        params.mode == StlMillingMode::FinishRasterXY || params.mode == StlMillingMode::WaterlineFinish);

    // â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
    // PHASE 1: Intelligentes 3D Z-Ebenen Schruppen (Materialabtrag)
    // â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
    if (doRoughing) {
        double curZ = areaBox.maxPoint.z;
        const double roughStepDown = std::max(0.2, params.roughStepDown);
        const double roughStepOver = std::max(0.5, tool.diameter * params.roughStepOverRatio);

        while (curZ > areaBox.minPoint.z - 1e-4) {
            curZ -= roughStepDown;
            if (curZ < areaBox.minPoint.z) curZ = areaBox.minPoint.z;

            double y = minY;
            bool leftToRight = true;

            while (y <= maxY + 1e-4) {
                double startX = leftToRight ? minX : maxX;
                double endX = leftToRight ? maxX : minX;
                double dir = leftToRight ? 1.0 : -1.0;

                // AnfahrhÃ¶he mit SchutzaufmaÃŸ
                double surfaceZ = queryToolZ(startX, y);
                double targetCutZ = std::max(curZ, surfaceZ);

                tp.addSegment({MotionType::Rapid, currentPos, {startX, y, clearanceZ}, {}, 0, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
                tp.addSegment({MotionType::LinearFeed, {startX, y, clearanceZ}, {startX, y, targetCutZ}, {}, tool.plungeFeedRate, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
                currentPos = {startX, y, targetCutZ};

                double curX = startX;
                while ((dir > 0 && curX < endX - 1e-4) || (dir < 0 && curX > endX + 1e-4)) {
                    curX += dir * sampleDx;
                    if ((dir > 0 && curX > endX) || (dir < 0 && curX < endX)) curX = endX;

                    double sZ = queryToolZ(curX, y);
                    double cutZ = std::max(curZ, sZ);
                    Core::Vector3D nextPt(curX, y, cutZ);
                    tp.addSegment({MotionType::LinearFeed, currentPos, nextPt, {}, tool.defaultFeedRate, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
                    currentPos = nextPt;
                }

                tp.addSegment({MotionType::Rapid, currentPos, {currentPos.x, currentPos.y, clearanceZ}, {}, 0, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
                currentPos = {currentPos.x, currentPos.y, clearanceZ};

                y += roughStepOver;
                leftToRight = !leftToRight;
            }

            if (std::abs(curZ - areaBox.minPoint.z) < 1e-4) break;
        }
    }

    // â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
    // PHASE 2: Intelligentes 3D FreiformflÃ¤chen-Schlichten (Endkontur)
    // â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
    if (doFinishing) {
        const double finishStepOver = std::max(0.05, params.finishStepOver);

        // Werkzeug-Z-Abfrage ohne AufmaÃŸ fÃ¼r exakte BauteiloberflÃ¤che
        auto queryFinishZ = [&](double cx, double cy) -> double {
            bool isBallEnd = (tool.type == Core::ToolType::BallMill);
            double maxZ = queryPointZ(cx, cy);

            constexpr int numRings = 2;
            constexpr int ptsPerRing = 6;
            for (int ring = 1; ring <= numRings; ++ring) {
                double rFrac = static_cast<double>(ring) / numRings;
                double curR = toolRadius * rFrac;
                double zOffset = isBallEnd ? (toolRadius - std::sqrt(std::max(0.0, toolRadius * toolRadius - curR * curR))) : 0.0;

                for (int p = 0; p < ptsPerRing; ++p) {
                    double ang = p * (2.0 * M_PI / ptsPerRing);
                    double sx = cx + curR * std::cos(ang);
                    double sy = cy + curR * std::sin(ang);
                    double sZ = queryPointZ(sx, sy) - zOffset;
                    if (sZ > maxZ) maxZ = sZ;
                }
            }
            return std::clamp(maxZ, areaBox.minPoint.z, areaBox.maxPoint.z + 5.0);
        };

        auto rasterX = [&]() {
            double y = minY;
            bool leftToRight = true;

            while (y <= maxY + 1e-4) {
                double startX = leftToRight ? minX : maxX;
                double endX = leftToRight ? maxX : minX;
                double dir = leftToRight ? 1.0 : -1.0;

                double firstZ = queryFinishZ(startX, y);
                tp.addSegment({MotionType::Rapid, currentPos, {startX, y, clearanceZ}, {}, 0, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
                tp.addSegment({MotionType::LinearFeed, {startX, y, clearanceZ}, {startX, y, firstZ}, {}, tool.plungeFeedRate, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
                currentPos = {startX, y, firstZ};

                double curX = startX;
                while ((dir > 0 && curX < endX - 1e-4) || (dir < 0 && curX > endX + 1e-4)) {
                    curX += dir * sampleDx;
                    if ((dir > 0 && curX > endX) || (dir < 0 && curX < endX)) curX = endX;

                    double nextZ = queryFinishZ(curX, y);
                    Core::Vector3D nextPt(curX, y, nextZ);
                    tp.addSegment({MotionType::LinearFeed, currentPos, nextPt, {}, tool.defaultFeedRate, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
                    currentPos = nextPt;
                }

                tp.addSegment({MotionType::Rapid, currentPos, {currentPos.x, currentPos.y, clearanceZ}, {}, 0, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
                currentPos = {currentPos.x, currentPos.y, clearanceZ};

                y += finishStepOver;
                leftToRight = !leftToRight;
            }
        };

        auto rasterY = [&]() {
            double x = minX;
            bool bottomToTop = true;

            while (x <= maxX + 1e-4) {
                double startY = bottomToTop ? minY : maxY;
                double endY = bottomToTop ? maxY : minY;
                double dir = bottomToTop ? 1.0 : -1.0;

                double firstZ = queryFinishZ(x, startY);
                tp.addSegment({MotionType::Rapid, currentPos, {x, startY, clearanceZ}, {}, 0, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
                tp.addSegment({MotionType::LinearFeed, {x, startY, clearanceZ}, {x, startY, firstZ}, {}, tool.plungeFeedRate, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
                currentPos = {x, startY, firstZ};

                double curY = startY;
                while ((dir > 0 && curY < endY - 1e-4) || (dir < 0 && curY > endY + 1e-4)) {
                    curY += dir * sampleDx;
                    if ((dir > 0 && curY > endY) || (dir < 0 && curY < endY)) curY = endY;

                    double nextZ = queryFinishZ(x, curY);
                    Core::Vector3D nextPt(x, curY, nextZ);
                    tp.addSegment({MotionType::LinearFeed, currentPos, nextPt, {}, tool.defaultFeedRate, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
                    currentPos = nextPt;
                }

                tp.addSegment({MotionType::Rapid, currentPos, {currentPos.x, currentPos.y, clearanceZ}, {}, 0, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
                currentPos = {currentPos.x, currentPos.y, clearanceZ};

                x += finishStepOver;
                bottomToTop = !bottomToTop;
            }
        };

        if (params.mode == StlMillingMode::FinishRasterXY) {
            rasterX();
            rasterY();
        } else if (params.mode == StlMillingMode::WaterlineFinish) {
            // Einfacher Fallback: Für Waterline nehmen wir RasterXY, um eine gute Oberfläche zu kriegen,
            // da echte Kontur-Verfolgung (Marching Squares) hier nicht implementiert ist.
            rasterX();
            rasterY();
        } else if (params.finishDirection == 0) {
            rasterX();
        } else {
            rasterY();
        }
    }

    // Finaler RÃ¼ckzug auf Sicherheitsebene
    tp.addSegment({MotionType::Rapid, currentPos, {currentPos.x, currentPos.y, clearanceZ}, {}, 0, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});

    return tp;
}

} // namespace GeminiCNC::CAM
