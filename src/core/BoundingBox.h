#ifndef GEMINI_CNC_BOUNDINGBOX_H
#define GEMINI_CNC_BOUNDINGBOX_H

#include "Vector3D.h"
#include <limits>

namespace GeminiCNC::Core {

/**
 * @brief Achsenparallele 3D-Bounding-Box (AABB) für Bauteil- und Rohteilbegrenzung.
 */
class BoundingBox {
public:
    Vector3D minPoint;
    Vector3D maxPoint;

    BoundingBox();
    BoundingBox(const Vector3D& minPt, const Vector3D& maxPt);

    void reset();
    void expand(const Vector3D& point);
    void expand(const BoundingBox& other);

    [[nodiscard]] bool isValid() const;
    [[nodiscard]] bool contains(const Vector3D& point) const;
    [[nodiscard]] bool intersects(const BoundingBox& other) const;

    [[nodiscard]] Vector3D center() const;
    [[nodiscard]] Vector3D size() const;
    [[nodiscard]] double widthX() const;
    [[nodiscard]] double depthY() const;
    [[nodiscard]] double heightZ() const;
    [[nodiscard]] double volume() const;

    [[nodiscard]] QString toString(int precision = 2) const;
};

} // namespace GeminiCNC::Core

#endif // GEMINI_CNC_BOUNDINGBOX_H
