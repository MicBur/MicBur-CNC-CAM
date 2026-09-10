#include "ContourSolver.h"
#include <cmath>

namespace GeminiCNC::Geometry {

double ContourSolver::normalizeAngle(double deg) {
    while (deg < 0.0) deg += 360.0;
    while (deg >= 360.0) deg -= 360.0;
    return deg;
}

LineSolveResult ContourSolver::solveLine(const LineSolveInput& in) {
    LineSolveResult res;

    // Fall 1: Endpunkt (X, Y) komplett gegeben
    if (in.endX.has_value() && in.endY.has_value()) {
        res.endX = *in.endX;
        res.endY = *in.endY;
        double dx = res.endX - in.startX;
        double dy = res.endY - in.startY;
        res.length = std::hypot(dx, dy);
        res.angleDeg = normalizeAngle(radToDeg(std::atan2(dy, dx)));
        res.success = true;
        return res;
    }

    // Fall 2: Länge und Winkel gegeben
    if (in.length.has_value() && in.angleDeg.has_value()) {
        res.length = *in.length;
        res.angleDeg = normalizeAngle(*in.angleDeg);
        double rad = degToRad(res.angleDeg);
        res.endX = in.startX + res.length * std::cos(rad);
        res.endY = in.startY + res.length * std::sin(rad);
        res.success = true;
        return res;
    }

    // Fall 3: Endpunkt X und Winkel gegeben
    if (in.endX.has_value() && in.angleDeg.has_value()) {
        res.endX = *in.endX;
        res.angleDeg = normalizeAngle(*in.angleDeg);
        double rad = degToRad(res.angleDeg);
        double cosA = std::cos(rad);
        if (std::abs(cosA) > 1e-6) {
            double dx = res.endX - in.startX;
            res.length = std::abs(dx / cosA);
            res.endY = in.startY + (dx / cosA) * std::sin(rad);
            res.success = true;
            return res;
        }
    }

    // Fall 4: Endpunkt Y und Winkel gegeben
    if (in.endY.has_value() && in.angleDeg.has_value()) {
        res.endY = *in.endY;
        res.angleDeg = normalizeAngle(*in.angleDeg);
        double rad = degToRad(res.angleDeg);
        double sinA = std::sin(rad);
        if (std::abs(sinA) > 1e-6) {
            double dy = res.endY - in.startY;
            res.length = std::abs(dy / sinA);
            res.endX = in.startX + (dy / sinA) * std::cos(rad);
            res.success = true;
            return res;
        }
    }

    // Fall 5: Endpunkt X und Länge gegeben
    if (in.endX.has_value() && in.length.has_value()) {
        res.endX = *in.endX;
        res.length = *in.length;
        double dx = res.endX - in.startX;
        if (res.length >= std::abs(dx)) {
            double dy = std::sqrt(std::max(0.0, res.length * res.length - dx * dx));
            res.endY = in.startY + dy; // Standard: positive Richtung
            res.angleDeg = normalizeAngle(radToDeg(std::atan2(dy, dx)));
            res.success = true;
            return res;
        }
    }

    // Fall 6: Endpunkt Y und Länge gegeben
    if (in.endY.has_value() && in.length.has_value()) {
        res.endY = *in.endY;
        res.length = *in.length;
        double dy = res.endY - in.startY;
        if (res.length >= std::abs(dy)) {
            double dx = std::sqrt(std::max(0.0, res.length * res.length - dy * dy));
            res.endX = in.startX + dx;
            res.angleDeg = normalizeAngle(radToDeg(std::atan2(dy, dx)));
            res.success = true;
            return res;
        }
    }

    // Fall 7: Nur Endpunkt X gegeben (Horizontale Bewegung angenommen)
    if (in.endX.has_value()) {
        res.endX = *in.endX;
        res.endY = in.startY;
        res.length = std::abs(res.endX - in.startX);
        res.angleDeg = (res.endX >= in.startX) ? 0.0 : 180.0;
        res.success = true;
        return res;
    }

    // Fall 8: Nur Endpunkt Y gegeben (Vertikale Bewegung angenommen)
    if (in.endY.has_value()) {
        res.endX = in.startX;
        res.endY = *in.endY;
        res.length = std::abs(res.endY - in.startY);
        res.angleDeg = (res.endY >= in.startY) ? 90.0 : 270.0;
        res.success = true;
        return res;
    }

    return res; // Nicht auflösbar
}

std::vector<ArcSolveResult> ContourSolver::solveArc(const ArcSolveInput& in) {
    std::vector<ArcSolveResult> results;

    // Fall 1: Mittelpunkt und Endpunkt gegeben
    if (in.centerX.has_value() && in.centerY.has_value() && in.endX.has_value() && in.endY.has_value()) {
        ArcSolveResult res;
        res.centerX = *in.centerX;
        res.centerY = *in.centerY;
        res.endX = *in.endX;
        res.endY = *in.endY;
        res.radius = std::hypot(in.startX - res.centerX, in.startY - res.centerY);

        double aStart = std::atan2(in.startY - res.centerY, in.startX - res.centerX);
        double aEnd = std::atan2(res.endY - res.centerY, res.endX - res.centerX);
        double diff = aEnd - aStart;
        if (in.isCW) {
            if (diff > 0) diff -= 2 * PI;
        } else {
            if (diff < 0) diff += 2 * PI;
        }
        res.sweepAngleDeg = std::abs(radToDeg(diff));
        res.success = true;
        results.push_back(res);
        return results;
    }

    // Fall 2: Mittelpunkt und Sweep-Winkel gegeben
    if (in.centerX.has_value() && in.centerY.has_value() && in.sweepAngleDeg.has_value()) {
        ArcSolveResult res;
        res.centerX = *in.centerX;
        res.centerY = *in.centerY;
        res.sweepAngleDeg = *in.sweepAngleDeg;
        res.radius = std::hypot(in.startX - res.centerX, in.startY - res.centerY);

        double aStart = std::atan2(in.startY - res.centerY, in.startX - res.centerX);
        double dAngle = degToRad(res.sweepAngleDeg);
        double aEnd = in.isCW ? (aStart - dAngle) : (aStart + dAngle);
        res.endX = res.centerX + res.radius * std::cos(aEnd);
        res.endY = res.centerY + res.radius * std::sin(aEnd);
        res.success = true;
        results.push_back(res);
        return results;
    }

    // Fall 3: Endpunkt und Radius gegeben -> 2 mögliche Mittelpunkte!
    if (in.endX.has_value() && in.endY.has_value() && in.radius.has_value()) {
        double r = std::abs(*in.radius);
        double dx = *in.endX - in.startX;
        double dy = *in.endY - in.startY;
        double chordLen = std::hypot(dx, dy);

        if (chordLen > 1e-6 && chordLen <= 2.0 * r + 1e-6) {
            double midX = (in.startX + *in.endX) / 2.0;
            double midY = (in.startY + *in.endY) / 2.0;
            double h = std::sqrt(std::max(0.0, r * r - (chordLen / 2.0) * (chordLen / 2.0)));
            double nx = -dy / chordLen;
            double ny = dx / chordLen;

            // Lösung A
            ArcSolveResult solA;
            solA.endX = *in.endX;
            solA.endY = *in.endY;
            solA.radius = r;
            solA.centerX = midX + h * nx;
            solA.centerY = midY + h * ny;

            double aStartA = std::atan2(in.startY - solA.centerY, in.startX - solA.centerX);
            double aEndA = std::atan2(solA.endY - solA.centerY, solA.endX - solA.centerX);
            double diffA = aEndA - aStartA;
            if (in.isCW && diffA > 0) diffA -= 2 * PI;
            if (!in.isCW && diffA < 0) diffA += 2 * PI;
            solA.sweepAngleDeg = std::abs(radToDeg(diffA));
            solA.success = true;
            results.push_back(solA);

            // Lösung B
            ArcSolveResult solB;
            solB.endX = *in.endX;
            solB.endY = *in.endY;
            solB.radius = r;
            solB.centerX = midX - h * nx;
            solB.centerY = midY - h * ny;

            double aStartB = std::atan2(in.startY - solB.centerY, in.startX - solB.centerX);
            double aEndB = std::atan2(solB.endY - solB.centerY, solB.endX - solB.centerX);
            double diffB = aEndB - aStartB;
            if (in.isCW && diffB > 0) diffB -= 2 * PI;
            if (!in.isCW && diffB < 0) diffB += 2 * PI;
            solB.sweepAngleDeg = std::abs(radToDeg(diffB));
            solB.success = true;
            results.push_back(solB);

            return results;
        }
    }

    return results;
}

} // namespace GeminiCNC::Geometry
