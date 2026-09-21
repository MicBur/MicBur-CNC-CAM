#ifndef GEMINI_CNC_MESHSLICER_H
#define GEMINI_CNC_MESHSLICER_H

#include "geometry/Mesh.h"
#include "geometry/Contour.h"
#include "core/BoundingBox.h"
#include <vector>

namespace GeminiCNC::CAM {

struct SliceContour {
    Geometry::Contour contour;  // Closed 2D contour
    double area;                 // Signed area (+ = CCW outer, - = CW hole)
    bool isHole;                 // True = inner contour / hole
};

struct SliceResult {
    double z;                               // Z-height of this slice
    std::vector<SliceContour> contours;     // All closed contours at this Z
};

class MeshSlicer {
public:
    // Slice mesh at a single Z-plane
    [[nodiscard]] static SliceResult sliceAtZ(const Geometry::Mesh& mesh, double z);
    
    // Slice mesh at uniform Z intervals
    [[nodiscard]] static std::vector<SliceResult> sliceUniform(
        const Geometry::Mesh& mesh, double zMin, double zMax, double stepZ);

    // Adaptive slicing with extra cuts where contour changes significantly  
    [[nodiscard]] static std::vector<SliceResult> sliceAdaptive(
        const Geometry::Mesh& mesh, double zMin, double zMax,
        double minStepZ, double maxStepZ, double changeTolerance);
};

}

#endif // GEMINI_CNC_MESHSLICER_H
