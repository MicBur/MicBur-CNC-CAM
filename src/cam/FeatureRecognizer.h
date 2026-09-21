#ifndef GEMINI_CNC_FEATURERECOGNIZER_H
#define GEMINI_CNC_FEATURERECOGNIZER_H

#include <vector>
#include <QString>
#include <QList>
#include "geometry/Mesh.h"
#include "geometry/Contour.h"
#include "core/BoundingBox.h"
#include "core/ToolDefinition.h"

namespace GeminiCNC::CAM {

enum class FeatureType {
    ExternalContour,
    Pocket,
    ThroughPocket,
    Slot,
    CircularHole,
    Facing,
    FreeformSurface,
    Step
};

struct RecognizedFeature {
    FeatureType type;
    QString name;
    Geometry::Contour boundary;
    std::vector<Geometry::Contour> islands;
    double topZ{0.0};
    double bottomZ{0.0};
    double estimatedVolume{0.0};
    double aspectRatio{1.0};
    double circularity{0.0};
    double diameter{0.0}; // Für Löcher relevant
    int suggestedToolId{-1};
    int priority{50};
};

struct RecognitionResult {
    std::vector<RecognizedFeature> features;
    bool hasFreeformSurfaces{false};
    double totalRemovalVolume{0.0};
    QString summary;
};

struct FeatureRecognitionSettings {
    double sliceZInterval{0.5};
    double mergeTolerance{0.5}; // Z-Stacking centroid tolerance
    double areaTolerance{0.20}; // Z-Stacking area difference tolerance
    double minCircularityHole{0.85};
    double maxHoleDiameter{30.0};
    double minSlotAspectRatio{3.0};
    double stockMarginTolerance{0.5}; // Touching stock edge
};

class FeatureRecognizer {
public:
    using Settings = FeatureRecognitionSettings;

    [[nodiscard]] static RecognitionResult analyze(
        const Geometry::Mesh& partMesh,
        const Geometry::Mesh& stockMesh,
        const QList<Core::ToolDefinition>& toolLibrary,
        const Settings& settings = Settings()
    );
};

} // namespace GeminiCNC::CAM

#endif // GEMINI_CNC_FEATURERECOGNIZER_H
