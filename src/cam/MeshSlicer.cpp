#include "MeshSlicer.h"
#include <cmath>
#include <unordered_map>
#include <vector>
#include <optional>
#include <algorithm>

namespace GeminiCNC::CAM {

namespace {

struct Point2D {
    double x, y;
};

struct Segment {
    Point2D p1, p2;
};

// Quantize point to 0.001mm precision for robust endpoint matching
struct PointKey {
    int64_t x, y;
    
    bool operator==(const PointKey& other) const {
        return x == other.x && y == other.y;
    }
};

struct PointKeyHash {
    std::size_t operator()(const PointKey& k) const {
        // Simple hash combining x and y
        return std::hash<int64_t>{}(k.x) ^ (std::hash<int64_t>{}(k.y) << 1);
    }
};

PointKey quantizePoint(const Point2D& p) {
    return {
        static_cast<int64_t>(std::round(p.x * 1000.0)),
        static_cast<int64_t>(std::round(p.y * 1000.0))
    };
}

std::optional<Point2D> intersectEdgeWithZPlane(const Geometry::Vertex& a, const Geometry::Vertex& b, double z) {
    if (a.z == b.z) return std::nullopt; // Edge is parallel to Z plane
    
    double t = (z - a.z) / (b.z - a.z);
    if (t >= 0.0 && t <= 1.0) {
        return Point2D{
            a.x + t * (b.x - a.x),
            a.y + t * (b.y - a.y)
        };
    }
    return std::nullopt;
}

} // anonymous namespace

SliceResult MeshSlicer::sliceAtZ(const Geometry::Mesh& mesh, double z) {
    SliceResult result;
    result.z = z;
    
    std::vector<Segment> segments;
    const double eps = 1e-6;

    // 1-7. Extract line segments
    for (const auto& tri : mesh.triangles) {
        if (tri.i0 >= mesh.vertices.size() || tri.i1 >= mesh.vertices.size() || tri.i2 >= mesh.vertices.size()) {
            continue; // Skip invalid triangles (robustness)
        }

        const auto& v0 = mesh.vertices[tri.i0];
        const auto& v1 = mesh.vertices[tri.i1];
        const auto& v2 = mesh.vertices[tri.i2];

        // Classify vertices: if exactly on plane, treat as above (c = 1)
        int c0 = (v0.z > z + eps) ? 1 : ((v0.z < z - eps) ? -1 : 1);
        int c1 = (v1.z > z + eps) ? 1 : ((v1.z < z - eps) ? -1 : 1);
        int c2 = (v2.z > z + eps) ? 1 : ((v2.z < z - eps) ? -1 : 1);

        // If all above or all below, skip
        if ((c0 == 1 && c1 == 1 && c2 == 1) || (c0 == -1 && c1 == -1 && c2 == -1)) {
            continue; 
        }

        std::vector<Point2D> intersectionPts;
        
        // Find intersections on edges
        auto p01 = intersectEdgeWithZPlane(v0, v1, z); if (p01) intersectionPts.push_back(*p01);
        auto p12 = intersectEdgeWithZPlane(v1, v2, z); if (p12) intersectionPts.push_back(*p12);
        auto p20 = intersectEdgeWithZPlane(v2, v0, z); if (p20) intersectionPts.push_back(*p20);

        if (intersectionPts.size() >= 2) {
            // Note: If 3 points intersect (e.g., vertex lies on plane), just take first two
            // to form the line segment since triangle is flat against plane.
            segments.push_back({intersectionPts[0], intersectionPts[1]});
        }
    }

    // 8. Chain segments into closed polylines
    std::unordered_map<PointKey, std::vector<size_t>, PointKeyHash> startMap;
    std::unordered_map<PointKey, std::vector<size_t>, PointKeyHash> endMap;
    std::vector<bool> used(segments.size(), false);

    for (size_t i = 0; i < segments.size(); ++i) {
        startMap[quantizePoint(segments[i].p1)].push_back(i);
        endMap[quantizePoint(segments[i].p2)].push_back(i);
    }

    for (size_t i = 0; i < segments.size(); ++i) {
        if (used[i]) continue;

        Geometry::Contour currentContour;
        currentContour.addPoint(segments[i].p1.x, segments[i].p1.y);
        
        size_t currentSeg = i;
        used[currentSeg] = true;
        
        PointKey targetStart = quantizePoint(segments[i].p1);
        PointKey currentEnd = quantizePoint(segments[i].p2);

        bool closed = false;

        while (true) {
            currentContour.addPoint(segments[currentSeg].p2.x, segments[currentSeg].p2.y);
            if (currentEnd == targetStart) {
                closed = true;
                break;
            }

            // Find next segment
            auto it = startMap.find(currentEnd);
            bool foundNext = false;
            if (it != startMap.end()) {
                for (size_t nextIdx : it->second) {
                    if (!used[nextIdx]) {
                        currentSeg = nextIdx;
                        used[currentSeg] = true;
                        currentEnd = quantizePoint(segments[currentSeg].p2);
                        foundNext = true;
                        break;
                    }
                }
            }
            
            if (!foundNext) {
                // Try reverse match if needed (sometimes segments have opposite direction)
                auto itRev = endMap.find(currentEnd);
                if (itRev != endMap.end()) {
                    for (size_t nextIdx : itRev->second) {
                        if (!used[nextIdx]) {
                            currentSeg = nextIdx;
                            used[currentSeg] = true;
                            // Reverse the segment logically
                            Point2D temp = segments[currentSeg].p1;
                            segments[currentSeg].p1 = segments[currentSeg].p2;
                            segments[currentSeg].p2 = temp;
                            
                            currentEnd = quantizePoint(segments[currentSeg].p2);
                            foundNext = true;
                            break;
                        }
                    }
                }
            }

            if (!foundNext) {
                break; // Open contour, cannot close.
            }
        }

        // 9-11. For each closed polyline, create Contour and SliceContour
        if (closed) {
            currentContour.isClosed = true;
            SliceContour sc;
            sc.contour = currentContour;
            sc.area = currentContour.signedArea();
            sc.isHole = (sc.area < 0.0);
            result.contours.push_back(sc);
        }
    }

    return result;
}

std::vector<SliceResult> MeshSlicer::sliceUniform(
    const Geometry::Mesh& mesh, double zMin, double zMax, double stepZ) 
{
    std::vector<SliceResult> results;
    if (stepZ <= 0.0 || zMax <= zMin) return results;

    for (double z = zMin + stepZ / 2.0; z <= zMax; z += stepZ) {
        results.push_back(sliceAtZ(mesh, z));
    }
    return results;
}

static void sliceAdaptiveRecursive(
    const Geometry::Mesh& mesh, double zStart, double zEnd, 
    const SliceResult& startRes, const SliceResult& endRes,
    double minStepZ, double changeTolerance, 
    std::vector<SliceResult>& outResults)
{
    if (std::abs(zEnd - zStart) <= minStepZ * 1.001) {
        outResults.push_back(startRes);
        return;
    }

    double startArea = 0.0;
    for (const auto& c : startRes.contours) startArea += std::abs(c.area);
    
    double endArea = 0.0;
    for (const auto& c : endRes.contours) endArea += std::abs(c.area);

    bool needSubdivision = false;
    if (startRes.contours.size() != endRes.contours.size()) {
        needSubdivision = true;
    } else {
        double maxArea = std::max(startArea, endArea);
        if (maxArea > 0.0) {
            double diffPercentage = (std::abs(startArea - endArea) / maxArea) * 100.0;
            if (diffPercentage > changeTolerance) {
                needSubdivision = true;
            }
        }
    }

    if (needSubdivision) {
        double midZ = (zStart + zEnd) / 2.0;
        SliceResult midRes = MeshSlicer::sliceAtZ(mesh, midZ);
        sliceAdaptiveRecursive(mesh, zStart, midZ, startRes, midRes, minStepZ, changeTolerance, outResults);
        sliceAdaptiveRecursive(mesh, midZ, zEnd, midRes, endRes, minStepZ, changeTolerance, outResults);
    } else {
        outResults.push_back(startRes);
    }
}

std::vector<SliceResult> MeshSlicer::sliceAdaptive(
    const Geometry::Mesh& mesh, double zMin, double zMax,
    double minStepZ, double maxStepZ, double changeTolerance)
{
    std::vector<SliceResult> results;
    if (maxStepZ <= 0.0 || minStepZ <= 0.0 || zMax <= zMin) return results;

    std::vector<double> baseZLevels;
    for (double z = zMin; z <= zMax; z += maxStepZ) {
        baseZLevels.push_back(z);
    }
    if (baseZLevels.back() < zMax - 1e-5) {
        baseZLevels.push_back(zMax);
    }

    std::vector<SliceResult> baseSlices;
    for (double z : baseZLevels) {
        baseSlices.push_back(sliceAtZ(mesh, z));
    }

    for (size_t i = 0; i < baseSlices.size() - 1; ++i) {
        sliceAdaptiveRecursive(mesh, baseZLevels[i], baseZLevels[i+1], 
                               baseSlices[i], baseSlices[i+1], 
                               minStepZ, changeTolerance, results);
    }
    
    if (!baseSlices.empty()) {
        results.push_back(baseSlices.back());
    }

    return results;
}

} // namespace GeminiCNC::CAM
