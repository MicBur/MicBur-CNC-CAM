#ifndef GEMINI_CNC_POLYGON_OFFSET_H
#define GEMINI_CNC_POLYGON_OFFSET_H

#include "Contour.h"
#include <vector>
#include <QPainterPath>

namespace GeminiCNC::Geometry {

/**
 * @brief Robuste Polygon-Offset-Engine für CNC-Taschenfräsen.
 *
 * Nutzt QPainterPath für:
 * - Selbstüberschneidungs-Bereinigung (simplified())
 * - Boolean-Differenz für Insel-Aussparung (subtracted())
 * - Extraktion bereinigter Sub-Konturen (toSubpathPolygons())
 *
 * Die Raw-Offsets werden vom existierenden Contour::createOffset() (Miter-Join)
 * berechnet und anschließend durch QPainterPath bereinigt.
 */
class PolygonOffset {
public:
    /**
     * @brief Generiert konzentrische Inset-Konturen für Taschenfräsen.
     *
     * Algorithmus:
     * 1. Erster Inset: -(toolRadius + finishAllowance)
     * 2. Folge-Insets: jeweils um -stepOver
     * 3. Bei jedem Schritt: Inseln expandieren und subtrahieren
     * 4. Stopp wenn Inset leer/degeneriert
     *
     * @param boundary     Äußere Taschenkontur (geschlossen, CCW)
     * @param islands      Innere Inseln die ausgespart werden (CW)
     * @param toolRadius   Werkzeugradius (mm)
     * @param stepOver     Seitliche Zustellung pro Ring (mm)
     * @param finishAllowance Schlichtaufmaß das stehen bleibt (mm), 0 = kein Aufmaß
     * @return Vektor von Ringen (außen→innen). Jeder Ring = Vektor von Konturen
     *         (bei Aufspaltung durch Inseln oder enge Stellen können es mehrere sein).
     */
    [[nodiscard]] static std::vector<std::vector<Contour>> generatePocketContours(
        const Contour& boundary,
        const std::vector<Contour>& islands,
        double toolRadius,
        double stepOver,
        double finishAllowance = 0.0);

    /**
     * @brief Berechnet einen einzelnen sauberen Polygon-Inset/Offset.
     *
     * Erzeugt Raw-Offset via Contour::createOffset(), bereinigt
     * Selbstüberschneidungen via QPainterPath::simplified(), und
     * extrahiert die resultierenden Teil-Konturen.
     *
     * @param contour  Eingabekontur
     * @param distance Offset-Distanz (negativ = Inset, positiv = Expansion)
     * @return Bereinigte Konturen (können bei Aufspaltung mehrere sein, oder leer)
     */
    [[nodiscard]] static std::vector<Contour> cleanOffset(
        const Contour& contour,
        double distance);

    /**
     * @brief Erzeugt die Taschen-Region als QPainterPath (Boundary minus Inseln).
     */
    [[nodiscard]] static QPainterPath subtractIslands(
        const Contour& boundary,
        const std::vector<Contour>& islands);

    // ── Konvertierungshelfer ──

    /** @brief Konvertiert eine Contour in einen QPainterPath. */
    [[nodiscard]] static QPainterPath contourToPath(const Contour& contour);

    /** @brief Extrahiert geschlossene Konturen aus einem QPainterPath. */
    [[nodiscard]] static std::vector<Contour> pathToContours(const QPainterPath& path);

private:
    /// Minimale Fläche (mm²) ab der eine Kontur als gültig gilt
    static constexpr double kMinAreaThreshold = 0.5;
    /// Maximale Anzahl konzentrischer Rings (Sicherheit gegen Endlosschleife)
    static constexpr int kMaxRings = 200;
};

} // namespace GeminiCNC::Geometry

#endif // GEMINI_CNC_POLYGON_OFFSET_H
