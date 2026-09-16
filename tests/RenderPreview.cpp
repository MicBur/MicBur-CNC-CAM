// Rendert ein simuliert gefrästes Beispielteil mit dem echten 3D-Viewport in PNG-Dateien.
// Aufruf: RenderPreview <Ausgabeordner>   (zur Sichtprüfung von Material, Schatten und Fräserspuren)
#include <QApplication>
#include <QDir>
#include <QElapsedTimer>
#include <QWheelEvent>
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

    // Beispielprogramm fräsen (Planen, Tasche, Bohrungen, Außenkontur)
    const auto tools = Core::ToolDefinition::createDefaultLibrary();
    const Core::BoundingBox stockBounds({-5, -5, -20}, {85, 55, 0.5});
    auto program = CAM::ConversationalProgram::createSampleProgram();
    const auto toolpath = program.generateFullToolpath(tools, stockBounds);

    Simulation::StockModel stock(stockBounds, 360);
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
        zoom(vp, 14);
        waitFrames(400);
        vp.grabFramebuffer().save(QDir(outDir).filePath(QString("render_%1_nah.png").arg(presetNames[m])));
    }
    std::cout << "Bilder gespeichert in " << outDir.toStdString() << std::endl;
    return 0;
}
