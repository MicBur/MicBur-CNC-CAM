#ifndef GEMINI_CNC_CONTOURSOLVER_H
#define GEMINI_CNC_CONTOURSOLVER_H

#include <optional>
#include <cmath>
#include <vector>
#include <QPointF>

namespace GeminiCNC::Geometry {

/**
 * @brief Eingabeparameter für eine Kontur-Gerade (LINE).
 *
 * Jedes Feld (außer Startpunkt) ist optional. Der Solver ermittelt
 * die fehlenden Werte anhand trigonometrischer Zusammenhänge.
 */
struct LineSolveInput {
    double startX{0.0};
    double startY{0.0};
    std::optional<double> endX;
    std::optional<double> endY;
    std::optional<double> length;
    std::optional<double> angleDeg; // 0° = +X (Osten), 90° = +Y (Norden)
};

struct LineSolveResult {
    bool success{false};
    double endX{0.0};
    double endY{0.0};
    double length{0.0};
    double angleDeg{0.0};
};

/**
 * @brief Eingabeparameter für einen Kontur-Bogen (ARC).
 */
struct ArcSolveInput {
    double startX{0.0};
    double startY{0.0};
    bool isCW{true};                // true = CW (Uhrzeigersinn), false = CCW
    std::optional<double> endX;
    std::optional<double> endY;
    std::optional<double> centerX;
    std::optional<double> centerY;
    std::optional<double> radius;
    std::optional<double> sweepAngleDeg;
};

struct ArcSolveResult {
    bool success{false};
    double endX{0.0};
    double endY{0.0};
    double centerX{0.0};
    double centerY{0.0};
    double radius{0.0};
    double sweepAngleDeg{0.0};
};

/**
 * @brief Hurco WinMax Kontur-Solver.
 *
 * Ermöglicht die conversational Eingabe: Bediener gibt nur bekannte Maße ein,
 * die Steuerung berechnet die restlichen Maße automatisch.
 */
class ContourSolver {
public:
    /**
     * @brief Berechnet eine Gerade aus den gegebenen Teilwerten.
     */
    static LineSolveResult solveLine(const LineSolveInput& in);

    /**
     * @brief Berechnet einen Kreisbogen aus den gegebenen Teilwerten.
     * Kann mehrere Lösungen liefern (z. B. 2 mögliche Mittelpunkte bei gegebenem Radius & Endpunkt).
     */
    static std::vector<ArcSolveResult> solveArc(const ArcSolveInput& in);

private:
    static constexpr double PI = 3.14159265358979323846;
    static double degToRad(double deg) { return deg * (PI / 180.0); }
    static double radToDeg(double rad) { return rad * (180.0 / PI); }
    static double normalizeAngle(double deg);
};

} // namespace GeminiCNC::Geometry

#endif // GEMINI_CNC_CONTOURSOLVER_H
