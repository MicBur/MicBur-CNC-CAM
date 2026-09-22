/**
 * @file TestStlMilling.cpp
 * @brief Lädt eine STL-Datei (Aschenbecher), verwendet den direkten Stl3D-Block
 *        mit verschiedenen Strategien, simuliert den Materialabtrag und exportiert.
 *
 * Testet: Schruppen (Waterline) + Schlichten X + Schlichten Y
 */

#include <QCoreApplication>
#include <QFile>
#include <cstdio>
#include <cmath>
#include <utility>

#include "cam/ConversationalBlock.h"
#include "cam/ConversationalProgram.h"
#include "cam/Toolpath.h"
#include "core/ToolDefinition.h"
#include "core/BoundingBox.h"
#include "geometry/Mesh.h"
#include "geometry/StlLoader.h"
#include "simulation/StockModel.h"

using namespace GeminiCNC;
using namespace GeminiCNC::CAM;
using namespace GeminiCNC::Core;
using namespace GeminiCNC::Geometry;

static QList<ToolDefinition> buildToolLibrary() {
    QList<ToolDefinition> tools;

    ToolDefinition t1;
    t1.id = 1; t1.name = "6mm Schaftfräser"; t1.type = ToolType::EndMill;
    t1.diameter = 6.0; t1.flutes = 2; t1.fluteLength = 20.0;
    t1.shaftDiameter = 6.0; t1.overallLength = 50.0; t1.stickOutLength = 30.0;
    t1.holderDiameter = 25.0;
    t1.defaultFeedRate = 1500; t1.plungeFeedRate = 500; t1.spindleSpeed = 18000;
    t1.maxStepDown = 2.5; t1.stepOverPercentage = 45;
    tools.append(t1);

    ToolDefinition t4;
    t4.id = 4; t4.name = "4mm Kugelfräser"; t4.type = ToolType::BallMill;
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

int main(int argc, char* argv[]) {
    QCoreApplication app(argc, argv);

    printf("\n");
    printf("================================================================\n");
    printf("  MicBur-CNC-CAM  STL 3D-Fraes-Test (Aschenbecher)\n");
    printf("  Schruppen + Schlichten mit verschiedenen Strategien\n");
    printf("================================================================\n\n");

    // ============================================================
    // 1. STL laden
    // ============================================================
    QString stlPath = "e:/cnc/test_stl/aschenbecher.stl";
    printf("[1] STL laden: %s\n", stlPath.toUtf8().constData());

    auto loadResult = StlLoader::loadFromFile(stlPath);
    auto& partMesh = loadResult.mesh;
    if (partMesh.vertices.empty()) {
        printf("  FEHLER: Konnte nicht geladen werden!\n");
        return 1;
    }
    partMesh.computeBoundingBox();
    auto partBB = partMesh.boundingBox;
    printf("  -> %d Vertices, %d Dreiecke\n",
           (int)partMesh.vertices.size(), (int)partMesh.triangles.size());
    printf("  -> BB: X[%.1f..%.1f] Y[%.1f..%.1f] Z[%.1f..%.1f]\n",
           partBB.minPoint.x, partBB.maxPoint.x,
           partBB.minPoint.y, partBB.maxPoint.y,
           partBB.minPoint.z, partBB.maxPoint.z);

    // Rohteil (etwas groesser als das Teil)
    BoundingBox stock(
        Vector3D(partBB.minPoint.x - 3, partBB.minPoint.y - 3, partBB.minPoint.z),
        Vector3D(partBB.maxPoint.x + 3, partBB.maxPoint.y + 3, partBB.maxPoint.z)
    );
    printf("  -> Rohteil: %.0f x %.0f x %.0f mm\n\n",
           stock.maxPoint.x - stock.minPoint.x,
           stock.maxPoint.y - stock.minPoint.y,
           stock.maxPoint.z - stock.minPoint.z);

    auto tools = buildToolLibrary();

    // ============================================================
    // 2. Bearbeitungsbloecke erstellen (verschiedene Strategien)
    // ============================================================
    printf("[2] Bearbeitungsbloecke erstellen...\n");

    struct BlockDef {
        const char* name;
        int toolId;
        StlMillingStrategy strategy;
    };
    BlockDef blockDefs[] = {
        {"Schruppen Z-Ebenen",        1, StlMillingStrategy::RoughOnly},
        {"Schlichten Raster X",        4, StlMillingStrategy::RasterX},
        {"Schlichten Raster Y",        4, StlMillingStrategy::RasterY},
        {"Schlichten Waterline",       4, StlMillingStrategy::WaterlineFinish},
    };

    struct TpEntry { Toolpath tp; ToolDefinition tool; };
    std::vector<TpEntry> toolpaths;
    int totalSegs = 0;

    for (const auto& def : blockDefs) {
        ConversationalBlock b(0, BlockType::Stl3D, def.name);
        b.toolId = def.toolId;
        b.stlStrategy = def.strategy;
        b.stlStepDown = (def.toolId == 1) ? 2.0 : 0.5;
        b.stlStepOver = (def.toolId == 1) ? 2.7 : 0.3;
        b.stlSampleStep = 0.5;
        b.stlAllowance = (def.strategy == StlMillingStrategy::RoughOnly) ? 0.2 : 0.0;
        b.startZ = partBB.maxPoint.z;
        b.targetZ = partBB.minPoint.z;
        b.clearanceZ = 5.0;
        b.spindleRpm = (def.toolId == 1) ? 18000 : 20000;
        b.feedRate = (def.toolId == 1) ? 1500 : 1000;
        b.directStlMesh = partMesh;

        auto tool = findTool(tools, def.toolId);
        auto tp = b.generateToolpath(tool, tool, stock, partMesh, tools);
        int segs = (int)tp.segments.size();
        totalSegs += segs;

        const char* stratStr = stlStrategyToString(def.strategy).toUtf8().constData();
        printf("    %s (T%d, %s): %d Segmente\n",
               def.name, def.toolId, stratStr, segs);

        TpEntry entry;
        entry.tp = tp;
        entry.tool = tool;
        toolpaths.push_back(std::move(entry));
    }
    printf("  -> Gesamt: %d Segmente\n\n", totalSegs);

    // ============================================================
    // 3. Simulation
    // ============================================================
    printf("[3] Simulation (Materialabtrag)...\n");
    Simulation::StockModel stockModel(stock, 300);

    int simSegs = 0;
    for (const auto& entry : toolpaths) {
        for (size_t i = 1; i < entry.tp.segments.size(); i++) {
            const auto& p0 = entry.tp.segments[i-1];
            const auto& p1 = entry.tp.segments[i];
            if (p1.feedRate > 0) {
                stockModel.carveSegment(p0.endPos, p1.endPos,
                                         entry.tool.diameter / 2.0);
            }
            simSegs++;
        }
    }
    printf("  -> %d Segmente simuliert\n\n", simSegs);

    // ============================================================
    // 4. Ergebnis STL exportieren
    // ============================================================
    printf("[4] Ergebnis exportieren...\n");
    auto resultMesh = stockModel.toMesh();
    printf("  -> Mesh: %d Vertices, %d Dreiecke\n",
           (int)resultMesh.vertices.size(), (int)resultMesh.triangles.size());

    QString resultPath = "e:/cnc/test_stl/aschenbecher_gefraest.stl";
    bool saved = StlLoader::saveBinary(resultPath, resultMesh);
    printf("  -> %s (%s)\n\n", resultPath.toUtf8().constData(),
           saved ? "OK" : "FEHLER");

    // ============================================================
    // 5. Zusammenfassung
    // ============================================================
    printf("================================================================\n");
    printf("  ERGEBNIS:\n");
    printf("  Original:  %d Dreiecke, %lld Bytes\n",
           (int)partMesh.triangles.size(), QFile(stlPath).size());
    printf("  Gefraest:  %d Dreiecke, %lld Bytes\n",
           (int)resultMesh.triangles.size(), QFile(resultPath).size());
    printf("  Strategien: Schruppen, Raster X, Raster Y, Waterline\n");
    printf("  Werkzeuge:  T1 6mm Schaft (Schruppen), T4 4mm Kugel (Schlichten)\n");
    printf("  Segmente:   %d total\n", totalSegs);
    printf("================================================================\n");

    return 0;
}
