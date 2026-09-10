#ifndef GEMINI_CNC_CONTOUR_H
#define GEMINI_CNC_CONTOUR_H

#include "core/Vector3D.h"
#include "core/BoundingBox.h"
#include <vector>
#include <QPointF>

namespace GeminiCNC::Geometry {

struct ContourPoint {
    double x{0.0};
    double y{0.0};
    double bulge{0.0}; // DXF-Bulge für Kreisbögen zwischen Punkten (tan(angle/4))

    ContourPoint() = default;
    ContourPoint(double px, double py, double b = 0.0) : x(px), y(py), bulge(b) {}

    [[nodiscard]] QPointF toQPointF() const { return QPointF(x, y); }
};

enum class ContourSegmentType {
    StartPoint, // X0, Y0
    Line,       // Gerade zu X, Y (oder polar)
    ArcCW,      // Kreisbogen CW mit Radius
    ArcCCW,     // Kreisbogen CCW mit Radius
    Chamfer,    // Fase
    Fillet,     // Rundungsradius
    Helix       // 3D Helix / Gewinde
};

struct ContourSegment {
    ContourSegmentType type{ContourSegmentType::Line};
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double radius{0.0};
    double pitch{0.0};    // Für Helix / Gewinde
    double angleDeg{0.0}; // Für Polar / Slot
    double length{0.0};
};

/**
 * @brief 2D-Kontur für Fräsbahnen, Taschen und DXF-Import.
 */
class Contour {
public:
    std::vector<ContourPoint> points;
    bool isClosed{true};
    QString layerName{"0"};

    Contour() = default;

    void addPoint(double x, double y, double bulge = 0.0);
    void clear();

    [[nodiscard]] size_t size() const { return points.size(); }
    [[nodiscard]] bool empty() const { return points.empty(); }

    [[nodiscard]] double perimeter() const;
    [[nodiscard]] double signedArea() const;
    [[nodiscard]] bool isClockwise() const;
    void reverse();

    [[nodiscard]] bool containsPoint(double x, double y) const;
    [[nodiscard]] Core::BoundingBox getBoundingBox(double zMin = 0.0, double zMax = 0.0) const;

    /**
     * @brief Berechnet eine parallele Offset-Kontur (Fräserradiuskorrektur innen/außen).
     */
    [[nodiscard]] Contour createOffset(double offsetDistance) const;

    // Erzeugt eine geschlossene Rechteck-Kontur
    [[nodiscard]] static Contour createRectangle(double minX, double minY, double widthX, double heightY);

    // Erzeugt ein abgerundetes Rechteck (Mill Frame mit Eckenradius)
    [[nodiscard]] static Contour createRoundedRectangle(double cx, double cy, double widthX, double heightY, double radius);

    // Erzeugt eine angenäherte Kreis-Kontur
    [[nodiscard]] static Contour createCircle(double centerX, double centerY, double radius, int segments = 64);

    // Erzeugt ein Langloch / Nut (Mill Slot) mit Drehwinkel
    [[nodiscard]] static Contour createSlot(double centerX, double centerY, double length, double width, double angleDeg = 0.0);

    // Baut eine Kontur aus Einzelsegmenten (Linien, Bögen, Fasen)
    [[nodiscard]] static Contour createFromSegments(const std::vector<ContourSegment>& segments, bool closeContour = true);

    // Generiert 3D-Helix-Punkte für zirkulares Eintauchen oder Gewindefräsen
    [[nodiscard]] static std::vector<Core::Vector3D> createHelixPoints(double cx, double cy, double startZ, double targetZ, double radius, double pitch, int stepsPerTurn = 32);
};

} // namespace GeminiCNC::Geometry

#endif // GEMINI_CNC_CONTOUR_H
