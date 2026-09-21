#ifndef GEMINI_CNC_AUTOPROGRAMGENERATOR_H
#define GEMINI_CNC_AUTOPROGRAMGENERATOR_H

#include "FeatureRecognizer.h"
#include "ConversationalProgram.h"
#include "geometry/Mesh.h"
#include "core/BoundingBox.h"
#include "core/ToolDefinition.h"
#include <QList>

namespace GeminiCNC::CAM {

struct AutoProgramSettings {
    int materialId{1};
    double clearanceZ{5.0};
    double finishAllowance{0.1};
    bool generateFinishPasses{true};
    bool generateFacing{true};
};

class AutoProgramGenerator {
public:
    using Settings = AutoProgramSettings;

    [[nodiscard]] static ConversationalProgram generate(
        const RecognitionResult& recognitionResult,
        const QList<Core::ToolDefinition>& toolLibrary,
        const Core::BoundingBox& stockBounds,
        const Geometry::Mesh& partMesh,
        const Settings& settings = Settings()
    );
};

} // namespace GeminiCNC::CAM

#endif // GEMINI_CNC_AUTOPROGRAMGENERATOR_H
