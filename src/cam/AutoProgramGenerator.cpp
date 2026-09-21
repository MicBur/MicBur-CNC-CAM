#include "AutoProgramGenerator.h"
#include <cmath>

namespace GeminiCNC::CAM {

ConversationalProgram AutoProgramGenerator::generate(
    const RecognitionResult& recognitionResult,
    const QList<Core::ToolDefinition>& toolLibrary,
    const Core::BoundingBox& stockBounds,
    const Geometry::Mesh& partMesh,
    const Settings& settings
) {
    ConversationalProgram program;
    program.programName = QStringLiteral("AutoCAM_Programm");
    int nextId = 1;

    // Werkzeugdaten für Schnittparameter-Defaults
    auto findTool = [&](int id) -> Core::ToolDefinition {
        for (const auto& t : toolLibrary) {
            if (t.id == id) return t;
        }
        if (!toolLibrary.isEmpty()) return toolLibrary.first();
        return Core::ToolDefinition(1, QStringLiteral("Standard"), Core::ToolType::EndMill, 6.0);
    };

    // Hilfsfunktion: Segmente aus Kontourpunkten erzeugen
    auto buildSegments = [](const Geometry::Contour& contour) -> std::vector<Geometry::ContourSegment> {
        std::vector<Geometry::ContourSegment> segs;
        if (contour.points.empty()) return segs;

        Geometry::ContourSegment start;
        start.type = Geometry::ContourSegmentType::StartPoint;
        start.x = contour.points[0].x;
        start.y = contour.points[0].y;
        segs.push_back(start);

        for (size_t i = 1; i < contour.points.size(); ++i) {
            Geometry::ContourSegment seg;
            seg.type = Geometry::ContourSegmentType::Line;
            seg.x = contour.points[i].x;
            seg.y = contour.points[i].y;
            segs.push_back(seg);
        }
        // Kontur schließen
        if (contour.isClosed && contour.points.size() > 2) {
            Geometry::ContourSegment close;
            close.type = Geometry::ContourSegmentType::Line;
            close.x = contour.points[0].x;
            close.y = contour.points[0].y;
            segs.push_back(close);
        }
        return segs;
    };

    for (const auto& feature : recognitionResult.features) {
        if (feature.type == FeatureType::Facing && !settings.generateFacing) {
            continue;
        }

        const auto tool = findTool(feature.suggestedToolId);
        const int toolId = feature.suggestedToolId > 0 ? feature.suggestedToolId : tool.id;

        switch (feature.type) {
            case FeatureType::Facing: {
                const int bid = nextId++;
                ConversationalBlock b(bid, BlockType::Facing,
                    QStringLiteral("%1: %2").arg(bid).arg(feature.name));
                b.toolId = toolId;
                b.materialId = settings.materialId;
                b.startZ = feature.topZ;
                b.targetZ = feature.bottomZ;
                b.stepDown = std::min(1.0, feature.topZ - feature.bottomZ);
                b.stepOver = tool.diameter * 0.6;
                b.clearanceZ = settings.clearanceZ;
                b.useStockDimensions = true;
                b.areaWidth = stockBounds.widthX();
                b.areaDepth = stockBounds.depthY();
                b.spindleRpm = 12000.0;
                b.feedRate = 2000.0;
                program.addBlock(b);
                break;
            }

            case FeatureType::ExternalContour: {
                const int bid = nextId++;
                ConversationalBlock b(bid, BlockType::Contour,
                    QStringLiteral("%1: %2").arg(bid).arg(feature.name));
                b.toolId = toolId;
                b.materialId = settings.materialId;
                b.contourSide = ContourSide::Outside;
                b.startZ = feature.topZ;
                b.targetZ = feature.bottomZ;
                b.stepDown = std::min(2.0, std::abs(feature.topZ - feature.bottomZ) / 3.0);
                b.clearanceZ = settings.clearanceZ;
                b.finishAllowance = settings.finishAllowance;
                b.spindleRpm = 18000.0;
                b.feedRate = 1500.0;
                b.plungeFeedRate = 500.0;
                b.contour = feature.boundary;
                b.segments = buildSegments(feature.boundary);
                b.contourZForAll = true;
                program.addBlock(b);
                break;
            }

            case FeatureType::Pocket:
            case FeatureType::ThroughPocket: {
                const int bid = nextId++;
                ConversationalBlock b(bid, BlockType::Pocket,
                    QStringLiteral("%1: %2").arg(bid).arg(feature.name));
                b.toolId = toolId;
                b.materialId = settings.materialId;
                b.pocketShape = PocketShape::CustomContour;
                b.contour = feature.boundary;
                b.startZ = feature.topZ;
                b.targetZ = feature.bottomZ;
                if (feature.type == FeatureType::ThroughPocket) {
                    b.targetZ -= 1.0; // Unter Stock-Boden durchfräsen
                }
                b.stepDown = std::min(2.0, std::abs(feature.topZ - feature.bottomZ) / 3.0);
                b.stepOver = tool.diameter * 0.5;
                b.clearanceZ = settings.clearanceZ;
                b.pocketStrategy = 1; // Spiral
                b.spindleRpm = 18000.0;
                b.feedRate = 1400.0;
                b.plungeFeedRate = 500.0;
                b.segments = buildSegments(feature.boundary);

                // Schlichtaufmaß
                if (settings.generateFinishPasses) {
                    b.enableFinishing = true;
                    b.finishAllowanceXY = settings.finishAllowance;
                    b.finishAllowanceZ = settings.finishAllowance;
                }

                b.setEffectiveMillingType(MillingType::Pocket);
                program.addBlock(b);

                // Inseln als eigene Blöcke dahinter (Hurco-Stil)
                for (const auto& island : feature.islands) {
                    const int ibid = nextId++;
                    ConversationalBlock ib(ibid, BlockType::Contour,
                        QStringLiteral("%1: Insel").arg(ibid));
                    ib.toolId = b.toolId;
                    ib.materialId = settings.materialId;
                    ib.setEffectiveMillingType(MillingType::Island);
                    ib.contour = island;
                    ib.contourZForAll = true;
                    ib.startZ = feature.topZ;
                    ib.targetZ = feature.bottomZ;
                    ib.clearanceZ = settings.clearanceZ;
                    ib.segments = buildSegments(island);
                    program.addBlock(ib);
                }
                break;
            }

            case FeatureType::Slot: {
                const int bid = nextId++;
                ConversationalBlock b(bid, BlockType::Slot,
                    QStringLiteral("%1: %2").arg(bid).arg(feature.name));
                b.toolId = toolId;
                b.materialId = settings.materialId;
                b.startZ = feature.topZ;
                b.targetZ = feature.bottomZ;
                b.stepDown = std::min(2.0, std::abs(feature.topZ - feature.bottomZ) / 3.0);
                b.clearanceZ = settings.clearanceZ;
                b.spindleRpm = 18000.0;
                b.feedRate = 1200.0;

                // Dimensionen aus der Boundary-BBox
                auto bbox = feature.boundary.getBoundingBox();
                double w = bbox.widthX();
                double h = bbox.depthY();
                b.slotLength = std::max(w, h);
                b.slotWidth = std::min(w, h);
                b.slotAngleDeg = (w > h) ? 0.0 : 90.0;

                // Position = Schwerpunkt
                b.posX = (bbox.minPoint.x + bbox.maxPoint.x) / 2.0;
                b.posY = (bbox.minPoint.y + bbox.maxPoint.y) / 2.0;

                program.addBlock(b);
                break;
            }

            case FeatureType::CircularHole: {
                // Drill-Block
                const int bid = nextId++;
                ConversationalBlock b(bid, BlockType::Drill,
                    QStringLiteral("%1: %2").arg(bid).arg(feature.name));
                b.toolId = toolId;
                b.materialId = settings.materialId;
                b.startZ = feature.topZ;
                b.targetZ = feature.bottomZ;
                b.clearanceZ = settings.clearanceZ;
                b.spindleRpm = 16000.0;
                b.plungeFeedRate = 350.0;

                // Bohrzyklus anlegen
                DrillOperation op = DrillOperation::createDefault(DrillOperationType::Drill, b.toolId);
                double depth = std::abs(feature.topZ - feature.bottomZ);
                if (depth > 10.0) {
                    op.cycleType = DrillCycleType::DeepHole;
                    op.peckDepth = 3.0;
                } else {
                    op.cycleType = DrillCycleType::Standard;
                }
                b.drillOps.push_back(op);
                program.addBlock(b);

                // Bohrpositionen-Block
                const int pbid = nextId++;
                ConversationalBlock pos(pbid, BlockType::DrillPositions,
                    QStringLiteral("%1: Bohrposition").arg(pbid));
                pos.drillPattern = DrillPattern::Single;

                // Mittelpunkt der Bohrung berechnen
                double cx = 0.0, cy = 0.0;
                if (!feature.boundary.points.empty()) {
                    for (const auto& pt : feature.boundary.points) {
                        cx += pt.x;
                        cy += pt.y;
                    }
                    cx /= static_cast<double>(feature.boundary.points.size());
                    cy /= static_cast<double>(feature.boundary.points.size());
                }
                pos.posX = cx;
                pos.posY = cy;
                program.addBlock(pos);
                break;
            }

            case FeatureType::FreeformSurface: {
                const int bid = nextId++;
                ConversationalBlock b(bid, BlockType::Stl3D,
                    QStringLiteral("%1: 3D-Schlichten").arg(bid));
                b.toolId = toolId;
                b.materialId = settings.materialId;
                b.startZ = feature.topZ;
                b.targetZ = feature.bottomZ;
                b.clearanceZ = settings.clearanceZ;
                b.stlStrategy = StlMillingStrategy::RoughAndFinishX;
                b.stlStepOver = 1.0;
                b.stlStepDown = 2.0;
                b.directStlMesh = partMesh;
                b.spindleRpm = 18000.0;
                b.feedRate = 1500.0;
                program.addBlock(b);
                break;
            }

            case FeatureType::Step:
            default:
                break;
        }
    }

    return program;
}

} // namespace GeminiCNC::CAM
