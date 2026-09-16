// Rendert ein simuliert gefrästes Beispielteil mit dem echten 3D-Viewport in PNG-Dateien.
// Aufruf: RenderPreview <Ausgabeordner> [Rasterauflösung=200] [Zoomschritte nah=14] [Fokus X] [Fokus Y]
// (zur Sichtprüfung von Material, Schatten, Fräserspuren und Kanten)
#include <QApplication>
#include <QDir>
#include <QElapsedTimer>
#include <QWheelEvent>
#include <cstdlib>
#include <iostream>

#include "cam/ConversationalProgram.h"
#include "core/ToolDefinition.h"
#include "simulation/StockModel.h"
#include "ui/viewport/Viewport3D.h"

using namespace GeminiCNC;

namespace {

void waitFrames(int ms) {
    QElapsedTimer timer;
    timer.start();
    while (timer.elapsed() < ms) QApplication::processEvents(QEventLoop::AllEvents, 20);
}

void zoom(UI::Viewport3D& vp, int steps) {
    for (int i = 0; i < steps; ++i) {
        QWheelEvent ev(QPointF(700, 450), vp.mapToGlobal(QPoint(700, 450)), QPoint(), QPoint(0, 120),
                       Qt::NoButton, Qt::NoModifier, Qt::NoScrollPhase, false);
        QApplication::sendEvent(&vp, &ev);
    }
}

} // namespace

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);
    const QString outDir = argc > 1 ? QString::fromLocal8Bit(argv[1]) : QDir::currentPath();
    const int resolution = argc > 2 ? std::atoi(argv[2]) : 200;   // wie im Programm
    const int zoomSteps = argc > 3 ? std::atoi(argv[3]) : 14;
    // Kamera blickt auf den Nullpunkt: das Teil so verschieben, dass der Fokuspunkt dort liegt
    const double focusX = argc > 4 ? std::atof(argv[4]) : 0.0;
    const double focusY = argc > 5 ? std::atof(argv[5]) : 0.0;

    // Beispielprogramm fräsen (Planen, Tasche, Bohrungen, Außenkontur)
    const auto tools = Core::ToolDefinition::createDefaultLibrary();
    const Core::BoundingBox partBounds({-5, -5, -20}, {85, 55, 0.5});
    auto program = CAM::ConversationalProgram::createSampleProgram();
    auto toolpath = program.generateFullToolpath(tools, partBounds);
    const Core::BoundingBox stockBounds({-5 - focusX, -5 - focusY, -20}, {85 - focusX, 55 - focusY, 0.5});
    for (auto& seg : toolpath.segments) {
        seg.startPos.x -= focusX; seg.endPos.x -= focusX;
        seg.startPos.y -= focusY; seg.endPos.y -= focusY;
    }

    Simulation::StockModel stock(stockBounds, resolution);
    for (const auto& seg : toolpath.segments) {
        double diameter = seg.toolDiameter;
        int kind = 1;
        for (const auto& t : tools) {
            if (t.id == seg.toolId) {
                if (diameter < 0.5) diameter = t.diameter;
                if (t.type == Core::ToolType::BallMill) kind = 2;
                if (t.type == Core::ToolType::Drill) kind = 3;
            }
        }
        const double pitch = seg.spindleRpm > 1.0 ? seg.feedRate / seg.spindleRpm : 0.1;
        stock.carveSegment(seg.startPos, seg.endPos, std::max(0.5, diameter) * 0.5, QColor(255, 230, 20), kind, pitch);
    }

    UI::Viewport3D vp;
    vp.resize(1400, 900);
    vp.show();
    waitFrames(300);
    vp.setShowToolpath(false);
    vp.setShowPart(false);
    vp.updateDynamicStock(stock);
    vp.setViewIsometric();
    vp.fitToView();

    const int presets[] = {0, 2};
    const char* presetNames[] = {"alu", "stahl"};
    for (int m = 0; m < 2; ++m) {
        vp.setMaterialPreset(presets[m]);
        for (int quality : {0, 2}) {
            vp.setRenderQuality(quality);
            vp.setViewIsometric();
            vp.fitToView();
            waitFrames(400);
            vp.grabFramebuffer().save(QDir(outDir).filePath(QString("render_%1_q%2.png").arg(presetNames[m]).arg(quality)));
        }
        vp.setRenderQuality(2);
        zoom(vp, zoomSteps);
        waitFrames(400);
        vp.grabFramebuffer().save(QDir(outDir).filePath(QString("render_%1_nah.png").arg(presetNames[m])));
    }
    std::cout << "Bilder gespeichert in " << outDir.toStdString() << std::endl;
    return 0;
}
