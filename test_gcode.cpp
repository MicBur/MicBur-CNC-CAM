#include "cam/ToolpathGenerator.h"
#include "cam/PostProcessorFactory.h"
#include "core/MachineConfig.h"
#include "core/ToolDefinition.h"
#include <iostream>

using namespace GeminiCNC;
using namespace GeminiCNC::CAM;

int main() {
    Core::MachineConfig config;
    config.controllerType = static_cast<int>(ControllerType::Hurco_WinMax);
    
    QList<Core::ToolDefinition> tools;
    tools.append({1, "T1", Core::ToolType::EndMill, 6.0});
    
    Core::BoundingBox stock{{0,0,-20}, {80,60,0}};
    FacingParams fp;
    fp.startZ = 0;
    fp.targetZ = -1;
    fp.stepDown = 1;
    fp.stepOver = 3;
    fp.clearanceZ = 5;
    fp.extension = 0;
    
    Toolpath tp = ToolpathGenerator::generateFacing(stock, tools.first(), fp);
    
    auto pp = PostProcessorFactory::create(ControllerType::Hurco_WinMax);
    PostProcessorContext ctx;
    ctx.toolNumber = 1;
    ctx.toolRadius = 3.0;
    
    QString out = pp->process(tp, config, tools, ctx);
    std::cout << out.toStdString();
    return 0;
}
