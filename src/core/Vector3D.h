#ifndef GEMINI_CNC_VECTOR3D_H
#define GEMINI_CNC_VECTOR3D_H

#include <QString>
#include <cmath>
#include <QVector3D>

namespace GeminiCNC::Core {

/**
 * @brief 4D Kinematik-Vektor für CNC-Steuerungen (X, Y, Z in mm, A in Grad).
 */
class Vector3D {
public:
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double a{0.0}; // 4. Drehachse (Winkel in Grad)

    constexpr Vector3D() = default;
    constexpr Vector3D(double xVal, double yVal, double zVal, double aVal = 0.0)
        : x(xVal), y(yVal), z(zVal), a(aVal) {}

    // Vektor-Arithmetik
    Vector3D operator+(const Vector3D& other) const;
    Vector3D operator-(const Vector3D& other) const;
    Vector3D operator*(double scalar) const;
    Vector3D operator/(double scalar) const;
    Vector3D& operator+=(const Vector3D& other);
    Vector3D& operator-=(const Vector3D& other);
    Vector3D& operator*=(double scalar);
    Vector3D& operator/=(double scalar);

    bool operator==(const Vector3D& other) const;
    bool operator!=(const Vector3D& other) const;

    // Mathematische Hilfsfunktionen (3D Raumanteil)
    [[nodiscard]] double length3D() const;
    [[nodiscard]] double lengthSquared3D() const;
    [[nodiscard]] Vector3D normalized3D() const;
    [[nodiscard]] double dot3D(const Vector3D& other) const;
    [[nodiscard]] Vector3D cross3D(const Vector3D& other) const;
    [[nodiscard]] double distanceTo3D(const Vector3D& other) const;

    // Gesamtdistanz inkl. A-Achsen-Wichtung falls erforderlich
    [[nodiscard]] double distanceTo4D(const Vector3D& other) const;

    // Lineare Interpolation
    [[nodiscard]] static Vector3D lerp(const Vector3D& start, const Vector3D& end, double t);

    // Konvertierung
    [[nodiscard]] QVector3D toQVector3D() const;
    [[nodiscard]] QString toString(int precision = 3) const;
};

inline Vector3D operator*(double scalar, const Vector3D& vec) {
    return vec * scalar;
}

} // namespace GeminiCNC::Core

#endif // GEMINI_CNC_VECTOR3D_H
