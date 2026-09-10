#include "BoundingBox.h"
#include <algorithm>

namespace GeminiCNC::Core {

BoundingBox::BoundingBox() {
    reset();
}

BoundingBox::BoundingBox(const Vector3D& minPt, const Vector3D& maxPt)
    : minPoint(minPt), maxPoint(maxPt) {}

void BoundingBox::reset() {
    constexpr double inf = std::numeric_limits<double>::infinity();
    minPoint = Vector3D(inf, inf, inf, inf);
    maxPoint = Vector3D(-inf, -inf, -inf, -inf);
}

void BoundingBox::expand(const Vector3D& point) {
    minPoint.x = std::min(minPoint.x, point.x);
    minPoint.y = std::min(minPoint.y, point.y);
    minPoint.z = std::min(minPoint.z, point.z);
    minPoint.a = std::min(minPoint.a, point.a);

    maxPoint.x = std::max(maxPoint.x, point.x);
    maxPoint.y = std::max(maxPoint.y, point.y);
    maxPoint.z = std::max(maxPoint.z, point.z);
    maxPoint.a = std::max(maxPoint.a, point.a);
}

void BoundingBox::expand(const BoundingBox& other) {
    if (!other.isValid()) {
        return;
    }
    expand(other.minPoint);
    expand(other.maxPoint);
}

bool BoundingBox::isValid() const {
    return minPoint.x <= maxPoint.x &&
           minPoint.y <= maxPoint.y &&
           minPoint.z <= maxPoint.z;
}

bool BoundingBox::contains(const Vector3D& point) const {
    if (!isValid()) return false;
    return point.x >= minPoint.x && point.x <= maxPoint.x &&
           point.y >= minPoint.y && point.y <= maxPoint.y &&
           point.z >= minPoint.z && point.z <= maxPoint.z;
}

bool BoundingBox::intersects(const BoundingBox& other) const {
    if (!isValid() || !other.isValid()) return false;
    return (minPoint.x <= other.maxPoint.x && maxPoint.x >= other.minPoint.x) &&
           (minPoint.y <= other.maxPoint.y && maxPoint.y >= other.minPoint.y) &&
           (minPoint.z <= other.maxPoint.z && maxPoint.z >= other.minPoint.z);
}

Vector3D BoundingBox::center() const {
    if (!isValid()) return {0.0, 0.0, 0.0, 0.0};
    return (minPoint + maxPoint) * 0.5;
}

Vector3D BoundingBox::size() const {
    if (!isValid()) return {0.0, 0.0, 0.0, 0.0};
    return maxPoint - minPoint;
}

double BoundingBox::widthX() const {
    return isValid() ? (maxPoint.x - minPoint.x) : 0.0;
}

double BoundingBox::depthY() const {
    return isValid() ? (maxPoint.y - minPoint.y) : 0.0;
}

double BoundingBox::heightZ() const {
    return isValid() ? (maxPoint.z - minPoint.z) : 0.0;
}

double BoundingBox::volume() const {
    return widthX() * depthY() * heightZ();
}

QString BoundingBox::toString(int precision) const {
    if (!isValid()) {
        return QString("BoundingBox[Invalid]");
    }
    return QString("BoundingBox[Min=(%1, %2, %3), Max=(%4, %5, %6), Dim=%7x%8x%9mm]")
        .arg(minPoint.x, 0, 'f', precision)
        .arg(minPoint.y, 0, 'f', precision)
        .arg(minPoint.z, 0, 'f', precision)
        .arg(maxPoint.x, 0, 'f', precision)
        .arg(maxPoint.y, 0, 'f', precision)
        .arg(maxPoint.z, 0, 'f', precision)
        .arg(widthX(), 0, 'f', precision)
        .arg(depthY(), 0, 'f', precision)
        .arg(heightZ(), 0, 'f', precision);
}

} // namespace GeminiCNC::Core
