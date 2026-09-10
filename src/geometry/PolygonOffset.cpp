#include "PolygonOffset.h"
#include <cmath>
#include <algorithm>
#include <QPolygonF>
#include <QDebug>

namespace GeminiCNC::Geometry {

// ═══════════════════════════════════════════════════════════════════════════════
// Konvertierung: Contour ↔ QPainterPath
// ═══════════════════════════════════════════════════════════════════════════════

QPainterPath PolygonOffset::contourToPath(const Contour& contour) {
    QPainterPath path;
    if (contour.points.size() < 3) return path;

    QPolygonF poly;
    poly.reserve(static_cast<int>(contour.points.size()));
    for (const auto& pt : contour.points) {
        poly.append(QPointF(pt.x, pt.y));
    }

    // QPainterPath::addPolygon schließt das Polygon automatisch
    path.addPolygon(poly);
    path.closeSubpath();
    return path;
}

std::vector<Contour> PolygonOffset::pathToContours(const QPainterPath& path) {
    std::vector<Contour> result;

    // toSubpathPolygons() extrahiert alle geschlossenen Teil-Polygone
    const QList<QPolygonF> subPolygons = path.toSubpathPolygons();

    for (const auto& poly : subPolygons) {
        if (poly.size() < 3) continue;

        Contour c;
        c.isClosed = true;

        for (int i = 0; i < poly.size(); ++i) {
            c.addPoint(poly[i].x(), poly[i].y());
        }

        // Duplikat-Schlusspunkt entfernen (QPainterPath fügt ihn manchmal hinzu)
        if (c.points.size() >= 2) {
            const auto& first = c.points.front();
            const auto& last = c.points.back();
            if (std::abs(first.x - last.x) < 1e-4 && std::abs(first.y - last.y) < 1e-4) {
                c.points.pop_back();
            }
        }

        // Nur Konturen mit ausreichender Fläche behalten
        if (c.points.size() >= 3 && std::abs(c.signedArea()) > kMinAreaThreshold) {
            result.push_back(std::move(c));
        }
    }

    return result;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Boolean-Differenz: Boundary minus Islands
// ═══════════════════════════════════════════════════════════════════════════════

QPainterPath PolygonOffset::subtractIslands(const Contour& boundary,
                                            const std::vector<Contour>& islands) {
    QPainterPath boundaryPath = contourToPath(boundary);

    if (islands.empty()) {
        return boundaryPath.simplified();
    }

    // Jede Insel vom Boundary subtrahieren
    QPainterPath islandPaths;
    for (const auto& island : islands) {
        if (island.points.size() >= 3) {
            islandPaths.addPath(contourToPath(island));
        }
    }

    return boundaryPath.subtracted(islandPaths).simplified();
}

// ═══════════════════════════════════════════════════════════════════════════════
// Einzelner bereinigter Offset
// ═══════════════════════════════════════════════════════════════════════════════

std::vector<Contour> PolygonOffset::cleanOffset(const Contour& contour, double distance) {
    if (contour.points.size() < 3) return {};
    if (std::abs(distance) < 1e-6) return {contour};

    // 1. Raw Miter-Offset über die existierende Methode
    Contour rawOffset = contour.createOffset(distance);
    if (rawOffset.points.size() < 3) return {};

    // 2. Prüfe ob die Fläche das gleiche Vorzeichen hat (Inset darf nicht invertieren)
    double originalArea = contour.signedArea();
    double offsetArea = rawOffset.signedArea();
    if (originalArea * offsetArea <= 0) {
        // Inset hat die Kontur invertiert → komplett degeneriert
        return {};
    }

    // 3. Miter-Spike Schutz: Ein Inset (negativer Offset) darf die ursprüngliche BoundingBox nicht überschreiten
    if (distance < 0.0) {
        Core::BoundingBox origBox = contour.getBoundingBox();
        Core::BoundingBox offsetBox = rawOffset.getBoundingBox();
        if (offsetBox.widthX() > origBox.widthX() * 1.5 || offsetBox.depthY() > origBox.depthY() * 1.5) {
            return {}; // Miter-Spike detektiert (Offset explodiert ins Unendliche)
        }
    }

    // 4. QPainterPath-Bereinigung: Selbstüberschneidungen entfernen
    QPainterPath rawPath = contourToPath(rawOffset);
    QPainterPath cleanPath = rawPath.simplified();

    // 4. Extrahiere bereinigte Teil-Konturen
    std::vector<Contour> cleaned = pathToContours(cleanPath);

    // 5. Nur Konturen behalten, deren Wicklungsrichtung zur Original-Kontur passt
    bool originalCW = contour.isClockwise();
    std::vector<Contour> valid;
    for (auto& c : cleaned) {
        if (c.isClockwise() == originalCW) {
            valid.push_back(std::move(c));
        }
    }

    return valid;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Konzentrische Pocket-Konturen
// ═══════════════════════════════════════════════════════════════════════════════

std::vector<std::vector<Contour>> PolygonOffset::generatePocketContours(
    const Contour& boundary,
    const std::vector<Contour>& islands,
    double toolRadius,
    double stepOver,
    double finishAllowance)
{
    std::vector<std::vector<Contour>> rings;

    if (boundary.points.size() < 3) return rings;
    if (toolRadius <= 0 || stepOver <= 0) return rings;

    // Early Abort: Wenn die Tasche kleiner ist als der Fräser, direkt abbrechen
    Core::BoundingBox bbox = boundary.getBoundingBox();
    if (bbox.isValid() && (bbox.widthX() < toolRadius * 2.0 || bbox.depthY() < toolRadius * 2.0)) {
        return rings;
    }

    // ── Schritt 1: Insel-Expansion ──
    // Jede Insel muss um toolRadius expandiert werden,
    // damit der Werkzeugmittelpunkt genug Abstand hält.
    std::vector<Contour> expandedIslands;
    for (const auto& island : islands) {
        if (island.points.size() < 3) continue;

        // Inseln sind typisch CW (Löcher) → Expansion nach außen = positiver Offset
        // Aber createOffset interpretiert die Richtung automatisch basierend auf Wicklung
        std::vector<Contour> expanded = cleanOffset(island, toolRadius);
        for (auto& e : expanded) {
            expandedIslands.push_back(std::move(e));
        }
    }

    // ── Schritt 2: Konzentrische Insets berechnen ──
    // Erster Inset: toolRadius + finishAllowance (Werkzeugmitte + Schlichtaufmaß)
    double currentInset = -(toolRadius + finishAllowance);

    for (int ringIdx = 0; ringIdx < kMaxRings; ++ringIdx) {
        // Raw-Inset der Boundary
        std::vector<Contour> insetContours = cleanOffset(boundary, currentInset);

        if (insetContours.empty()) break;

        // Boolean-Subtraktion der expandierten Inseln
        if (!expandedIslands.empty()) {
            std::vector<Contour> validContours;

            for (const auto& insetC : insetContours) {
                QPainterPath insetPath = contourToPath(insetC);

                // Alle expandierten Inseln subtrahieren
                QPainterPath islandPaths;
                for (const auto& ei : expandedIslands) {
                    islandPaths.addPath(contourToPath(ei));
                }

                QPainterPath resultPath = insetPath.subtracted(islandPaths).simplified();
                std::vector<Contour> resultContours = pathToContours(resultPath);

                for (auto& rc : resultContours) {
                    validContours.push_back(std::move(rc));
                }
            }

            if (validContours.empty()) break;
            rings.push_back(std::move(validContours));
        } else {
            rings.push_back(std::move(insetContours));
        }

        // Nächster Ring: um stepOver weiter nach innen
        currentInset -= stepOver;
    }

    return rings;
}

} // namespace GeminiCNC::Geometry
