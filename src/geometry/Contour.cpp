#include "Contour.h"
#include <cmath>
#include <algorithm>

namespace GeminiCNC::Geometry {

void Contour::addPoint(double x, double y, double bulge) {
    points.emplace_back(x, y, bulge);
}

void Contour::clear() {
    points.clear();
}

double Contour::perimeter() const {
    if (points.size() < 2) return 0.0;
    double len = 0.0;
    const size_t count = isClosed ? points.size() : points.size() - 1;
    for (size_t i = 0; i < count; ++i) {
        const auto& p1 = points[i];
        const auto& p2 = points[(i + 1) % points.size()];
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        len += std::sqrt(dx * dx + dy * dy);
    }
    return len;
}

double Contour::signedArea() const {
    if (points.size() < 3) return 0.0;
    double area = 0.0;
    for (size_t i = 0; i < points.size(); ++i) {
        const auto& p1 = points[i];
        const auto& p2 = points[(i + 1) % points.size()];
        area += (p1.x * p2.y - p2.x * p1.y);
    }
    return area * 0.5;
}

bool Contour::isClockwise() const {
    return signedArea() < 0.0;
}

void Contour::reverse() {
    std::reverse(points.begin(), points.end());
}

bool Contour::containsPoint(double x, double y) const {
    if (points.size() < 3) return false;
    bool inside = false;
    size_t j = points.size() - 1;
    for (size_t i = 0; i < points.size(); ++i) {
        double xi = points[i].x, yi = points[i].y;
        double xj = points[j].x, yj = points[j].y;

        if (((yi > y) != (yj > y)) &&
            (x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi)) {
            inside = !inside;
        }
        j = i;
    }
    return inside;
}

Core::BoundingBox Contour::getBoundingBox(double zMin, double zMax) const {
    Core::BoundingBox box;
    for (const auto& pt : points) {
        box.expand(Core::Vector3D(pt.x, pt.y, zMin));
        box.expand(Core::Vector3D(pt.x, pt.y, zMax));
    }
    return box;
}

Contour Contour::createOffset(double offsetDistance) const {
    Contour out;
    out.isClosed = this->isClosed;
    out.layerName = this->layerName;

    if (points.size() < 2) {
        return *this;
    }

    const size_t n = points.size();
    out.points.reserve(n);

    // Kanten-Normalen berechnen
    std::vector<std::pair<double, double>> edgeNormals(n);
    for (size_t i = 0; i < n; ++i) {
        size_t nextIdx = (i + 1) % n;
        double dx = points[nextIdx].x - points[i].x;
        double dy = points[nextIdx].y - points[i].y;
        double len = std::sqrt(dx * dx + dy * dy);
        if (len > 1e-7) {
            // Normalvektor senkrecht nach rechts (bei CCW nach außen)
            edgeNormals[i] = {dy / len, -dx / len};
        } else {
            edgeNormals[i] = {0.0, 0.0};
        }
    }

    // Für jeden Scheitelpunkt den gemittelten Normalenvektor berechnen
    for (size_t i = 0; i < n; ++i) {
        size_t prevIdx = (i + n - 1) % n;
        double nx = edgeNormals[prevIdx].first + edgeNormals[i].first;
        double ny = edgeNormals[prevIdx].second + edgeNormals[i].second;
        double len = std::sqrt(nx * nx + ny * ny);

        if (len > 1e-7) {
            nx /= len;
            ny /= len;
        }

        // Skalierung für Miter-Ecken begrenzen
        double dot = edgeNormals[i].first * nx + edgeNormals[i].second * ny;
        double miterLimit = (dot > 0.3) ? (1.0 / dot) : 2.0;
        if (miterLimit > 3.0) miterLimit = 3.0;

        double ox = points[i].x + nx * offsetDistance * miterLimit;
        double oy = points[i].y + ny * offsetDistance * miterLimit;
        out.addPoint(ox, oy);
    }

    return out;
}

Contour Contour::createRectangle(double minX, double minY, double widthX, double heightY) {
    Contour c;
    c.isClosed = true;
    c.addPoint(minX, minY);
    c.addPoint(minX + widthX, minY);
    c.addPoint(minX + widthX, minY + heightY);
    c.addPoint(minX, minY + heightY);
    return c;
}

Contour Contour::createCircle(double centerX, double centerY, double radius, int segments) {
    Contour c;
    c.isClosed = true;
    c.points.reserve(segments);
    const double step = 2.0 * M_PI / segments;
    for (int i = 0; i < segments; ++i) {
        double angle = i * step;
        c.addPoint(centerX + radius * std::cos(angle),
                   centerY + radius * std::sin(angle));
    }
    return c;
}

Contour Contour::createRoundedRectangle(double cx, double cy, double widthX, double heightY, double radius) {
    Contour c;
    const double hw = widthX * 0.5;
    const double hh = heightY * 0.5;
    const double r = std::min({radius, hw, hh});

    if (r <= 1e-4) {
        return createRectangle(cx - hw, cy - hh, widthX, heightY);
    }

    const int arcSegs = 8;
    auto addCornerArc = [&](double cornerX, double cornerY, double startAngle) {
        for (int i = 0; i <= arcSegs; ++i) {
            double a = startAngle + (M_PI * 0.5) * (static_cast<double>(i) / arcSegs);
            c.addPoint(cornerX + r * std::cos(a), cornerY + r * std::sin(a));
        }
    };

    // 4 Ecken mit Verrundung
    addCornerArc(cx + hw - r, cy - hh + r, -M_PI * 0.5); // Rechts-Unten
    addCornerArc(cx + hw - r, cy + hh - r, 0.0);         // Rechts-Oben
    addCornerArc(cx - hw + r, cy + hh - r, M_PI * 0.5);  // Links-Oben
    addCornerArc(cx - hw + r, cy - hh + r, M_PI);        // Links-Unten
    return c;
}

Contour Contour::createSlot(double cx, double cy, double length, double width, double angleDeg) {
    Contour c;
    const double r = width * 0.5;
    const double halfDist = std::max(0.0, (length - width) * 0.5);
    const double rad = angleDeg * M_PI / 180.0;
    const double cosA = std::cos(rad);
    const double sinA = std::sin(rad);

    auto transform = [&](double lx, double ly) {
        double gx = cx + lx * cosA - ly * sinA;
        double gy = cy + lx * sinA + ly * cosA;
        c.addPoint(gx, gy);
    };

    const int arcSegs = 16;
    // Rechte Halbkreis-Kappe
    for (int i = 0; i <= arcSegs; ++i) {
        double a = -M_PI * 0.5 + (M_PI * static_cast<double>(i) / arcSegs);
        transform(halfDist + r * std::cos(a), r * std::sin(a));
    }
    // Linke Halbkreis-Kappe
    for (int i = 0; i <= arcSegs; ++i) {
        double a = M_PI * 0.5 + (M_PI * static_cast<double>(i) / arcSegs);
        transform(-halfDist + r * std::cos(a), r * std::sin(a));
    }

    return c;
}

void Contour::applyStartDepth(std::vector<ContourSegment>& segments, double zStart, double zBottom) {
    if (segments.empty() || segments.front().type != ContourSegmentType::StartPoint) return;

    const double oldBottom = segments.front().z;
    segments.front().zStart = zStart;
    segments.front().z = zBottom;
    for (size_t i = 1; i < segments.size(); ++i) {
        if (std::abs(segments[i].z - oldBottom) < 1e-6) {
            segments[i].z = zBottom;
        }
    }
}

Contour Contour::createFromSegments(const std::vector<ContourSegment>& segments, bool closeContour) {
    return createFromSegments(segments, closeContour, nullptr);
}

Contour Contour::createFromSegments(const std::vector<ContourSegment>& segments, bool closeContour,
                                    std::vector<double>* pointZ) {
    Contour c;
    c.isClosed = closeContour;
    if (pointZ) pointZ->clear();
    if (segments.empty()) return c;

    double curX = 0.0;
    double curY = 0.0;
    double curZ = 0.0;
    auto addPt = [&c, pointZ](double x, double y, double z) {
        c.addPoint(x, y);
        if (pointZ) pointZ->push_back(z);
    };

    for (const auto& seg : segments) {
        switch (seg.type) {
            case ContourSegmentType::StartPoint:
            case ContourSegmentType::Line:
                curX = seg.x;
                curY = seg.y;
                curZ = seg.z;
                addPt(curX, curY, curZ);
                break;
            case ContourSegmentType::ArcCW:
            case ContourSegmentType::ArcCCW: {
                const bool cw = (seg.type == ContourSegmentType::ArcCW);
                const double targetX = seg.x;
                const double targetY = seg.y;
                double centerX = 0.0;
                double centerY = 0.0;
                double r = 0.0;

                if (seg.hasCenter) {
                    // Mittelpunkt vorgegeben (Hurco: X/Y MITTELPUNKT)
                    centerX = seg.centerX;
                    centerY = seg.centerY;
                    r = std::hypot(curX - centerX, curY - centerY);
                    if (r < 1e-6) {
                        curX = targetX;
                        curY = targetY;
                        curZ = seg.z;
                        addPt(curX, curY, curZ);
                        break;
                    }
                } else {
                    // Nur Radius: kürzerer Bogen auf der Seite der Drehrichtung
                    r = std::max(0.1, std::abs(seg.radius));
                    const double dx = targetX - curX;
                    const double dy = targetY - curY;
                    const double d = std::hypot(dx, dy);
                    if (d < 1e-4) break;
                    if (d > 2.0 * r) r = d * 0.5; // Absicherung
                    const double h = std::sqrt(std::max(0.0, r * r - 0.25 * d * d));
                    double normX = -dy / d;
                    double normY = dx / d;
                    if (cw) {
                        normX = -normX;
                        normY = -normY;
                    }
                    centerX = 0.5 * (curX + targetX) + h * normX;
                    centerY = 0.5 * (curY + targetY) + h * normY;
                }

                const double startAngle = std::atan2(curY - centerY, curX - centerX);
                double endAngle = std::atan2(targetY - centerY, targetX - centerX);
                if (cw) {
                    if (endAngle >= startAngle - 1e-12) endAngle -= 2.0 * M_PI;
                } else {
                    if (endAngle <= startAngle + 1e-12) endAngle += 2.0 * M_PI;
                }

                // Höchstens 10° je Teilstück; Tiefe linear über den Bogen
                const double sweep = endAngle - startAngle;
                const int steps = std::max(4, static_cast<int>(std::ceil(std::abs(sweep) / (M_PI / 18.0))));
                const double zFrom = curZ;
                for (int i = 1; i <= steps; ++i) {
                    const double t = static_cast<double>(i) / steps;
                    const double a = startAngle + sweep * t;
                    const double z = zFrom + (seg.z - zFrom) * t;
                    if (i == steps) {
                        addPt(targetX, targetY, z);
                    } else {
                        addPt(centerX + r * std::cos(a), centerY + r * std::sin(a), z);
                    }
                }
                curX = targetX;
                curY = targetY;
                curZ = seg.z;
                break;
            }
            case ContourSegmentType::Chamfer:
            case ContourSegmentType::Fillet:
            case ContourSegmentType::Helix:
                // Werden im Pfad-Compiler interpoliert
                break;
        }
    }

    // Geschlossene Kontur: doppelten Schlusspunkt entfernen
    // Der Benutzer definiert z.B. Start(5,5) → ... → Line(5,5) zum Schließen.
    // Der identische Endpunkt erzeugt eine Null-Kante, die die Normalen in
    // createOffset() korrumpiert und die Offset-Kontur schief versetzt.
    if (closeContour && c.points.size() >= 3) {
        const auto& first = c.points.front();
        const auto& last = c.points.back();
        if (std::abs(first.x - last.x) < 1e-4 && std::abs(first.y - last.y) < 1e-4) {
            c.points.pop_back();
            if (pointZ && !pointZ->empty()) pointZ->pop_back();
        }
    }

    return c;
}

std::vector<Core::Vector3D> Contour::createHelixPoints(double cx, double cy, double startZ, double targetZ, double radius, double pitch, int stepsPerTurn) {
    std::vector<Core::Vector3D> points;
    const double totalZ = std::abs(targetZ - startZ);
    const double p = std::max(0.1, pitch);
    const double turns = totalZ / p;
    const int totalSteps = static_cast<int>(std::ceil(turns * stepsPerTurn));
    const double zDir = (targetZ < startZ) ? -1.0 : 1.0;

    for (int i = 0; i <= totalSteps; ++i) {
        double progress = static_cast<double>(i) / totalSteps;
        double currentAngle = progress * turns * 2.0 * M_PI;
        double z = startZ + zDir * progress * totalZ;
        double x = cx + radius * std::cos(currentAngle);
        double y = cy + radius * std::sin(currentAngle);
        points.push_back({x, y, z});
    }

    // Volle 360°-Bodenrunde auf Zielhöhe zum Schlichten
    for (int i = 1; i <= stepsPerTurn; ++i) {
        double a = (static_cast<double>(i) / stepsPerTurn) * 2.0 * M_PI;
        points.push_back({cx + radius * std::cos(a), cy + radius * std::sin(a), targetZ});
    }

    return points;
}

} // namespace GeminiCNC::Geometry
