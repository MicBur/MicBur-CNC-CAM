#include "Vector3D.h"
#include <limits>

namespace GeminiCNC::Core {

static constexpr double EPSILON = 1e-7;

Vector3D Vector3D::operator+(const Vector3D& other) const {
    return {x + other.x, y + other.y, z + other.z, a + other.a};
}

Vector3D Vector3D::operator-(const Vector3D& other) const {
    return {x - other.x, y - other.y, z - other.z, a - other.a};
}

Vector3D Vector3D::operator*(double scalar) const {
    return {x * scalar, y * scalar, z * scalar, a * scalar};
}

Vector3D Vector3D::operator/(double scalar) const {
    if (std::abs(scalar) < EPSILON) {
        return *this;
    }
    const double inv = 1.0 / scalar;
    return {x * inv, y * inv, z * inv, a * inv};
}

Vector3D& Vector3D::operator+=(const Vector3D& other) {
    x += other.x;
    y += other.y;
    z += other.z;
    a += other.a;
    return *this;
}

Vector3D& Vector3D::operator-=(const Vector3D& other) {
    x -= other.x;
    y -= other.y;
    z -= other.z;
    a -= other.a;
    return *this;
}

Vector3D& Vector3D::operator*=(double scalar) {
    x *= scalar;
    y *= scalar;
    z *= scalar;
    a *= scalar;
    return *this;
}

Vector3D& Vector3D::operator/=(double scalar) {
    if (std::abs(scalar) >= EPSILON) {
        const double inv = 1.0 / scalar;
        x *= inv;
        y *= inv;
        z *= inv;
        a *= inv;
    }
    return *this;
}

bool Vector3D::operator==(const Vector3D& other) const {
    return std::abs(x - other.x) < EPSILON &&
           std::abs(y - other.y) < EPSILON &&
           std::abs(z - other.z) < EPSILON &&
           std::abs(a - other.a) < EPSILON;
}

bool Vector3D::operator!=(const Vector3D& other) const {
    return !(*this == other);
}

double Vector3D::length3D() const {
    return std::sqrt(lengthSquared3D());
}

double Vector3D::lengthSquared3D() const {
    return x * x + y * y + z * z;
}

Vector3D Vector3D::normalized3D() const {
    const double len = length3D();
    if (len < EPSILON) {
        return {0.0, 0.0, 0.0, a};
    }
    const double inv = 1.0 / len;
    return {x * inv, y * inv, z * inv, a};
}

double Vector3D::dot3D(const Vector3D& other) const {
    return x * other.x + y * other.y + z * other.z;
}

Vector3D Vector3D::cross3D(const Vector3D& other) const {
    return {
        y * other.z - z * other.y,
        z * other.x - x * other.z,
        x * other.y - y * other.x,
        0.0
    };
}

double Vector3D::distanceTo3D(const Vector3D& other) const {
    const double dx = x - other.x;
    const double dy = y - other.y;
    const double dz = z - other.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double Vector3D::distanceTo4D(const Vector3D& other) const {
    const double dx = x - other.x;
    const double dy = y - other.y;
    const double dz = z - other.z;
    const double da = a - other.a;
    return std::sqrt(dx * dx + dy * dy + dz * dz + da * da);
}

Vector3D Vector3D::lerp(const Vector3D& start, const Vector3D& end, double t) {
    return start + (end - start) * t;
}

QVector3D Vector3D::toQVector3D() const {
    return QVector3D(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
}

QString Vector3D::toString(int precision) const {
    return QString("X:%1 Y:%2 Z:%3 A:%4")
        .arg(x, 0, 'f', precision)
        .arg(y, 0, 'f', precision)
        .arg(z, 0, 'f', precision)
        .arg(a, 0, 'f', precision);
}

} // namespace GeminiCNC::Core
