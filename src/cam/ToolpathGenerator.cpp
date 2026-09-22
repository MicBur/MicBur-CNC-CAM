#include "ToolpathGenerator.h"
#include <cmath>
#include <algorithm>
#include <limits>
#include "geometry/PolygonOffset.h"

namespace GeminiCNC::CAM {

Toolpath ToolpathGenerator::generateFacing(const Core::BoundingBox& stockBounds,
                                          const Core::ToolDefinition& tool,
                                          const FacingParams& params) {
    Toolpath tp(QStringLiteral("PlanfrÃ¤sen (Facing)"));
    if (!stockBounds.isValid()) return tp;

    const double toolRadius = tool.diameter * 0.5;
    const double minX = stockBounds.minPoint.x - toolRadius - params.extension;
    const double maxX = stockBounds.maxPoint.x + toolRadius + params.extension;
    // Y-Grenzen: Fräser-Mittelpunkt fährt von minY bis maxY,
    // dabei überragt der Fräserradius die Stock-Kante → volle Abdeckung
    const double minY = stockBounds.minPoint.y - toolRadius;
    const double maxY = stockBounds.maxPoint.y + toolRadius;

    double stepOver = (params.stepOver > 0.1) ? params.stepOver : tool.effectiveStepOver();
    if (stepOver < 0.5) stepOver = tool.diameter * 0.5;

    double currentZ = params.startZ;
    const double targetZ = params.targetZ;
    const double stepDown = std::max(0.1, params.stepDown);

    Core::Vector3D currentPos(minX, minY, params.clearanceZ);

    while (currentZ > targetZ - 1e-5) {
        currentZ -= stepDown;
        if (currentZ < targetZ) currentZ = targetZ;

        // Anfahrt auf Eilgang Ã¼ber Startpunkt
        tp.addSegment({MotionType::Rapid, currentPos, {minX, minY, params.clearanceZ}, {}, 0, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
        currentPos = {minX, minY, params.clearanceZ};

        // Eintauchen auf Z-Tiefe
        tp.addSegment({MotionType::LinearFeed, currentPos, {minX, minY, currentZ}, {}, tool.plungeFeedRate, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
        currentPos = {minX, minY, currentZ};

        // MÃ¤ander-Zickzack-Bahnen in Y
        double y = minY;
        bool leftToRight = true;

        while (y <= maxY + 1e-4) {
            double targetX = leftToRight ? maxX : minX;
            tp.addSegment({MotionType::LinearFeed, currentPos, {targetX, y, currentZ}, {}, tool.defaultFeedRate, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
            currentPos = {targetX, y, currentZ};

            y += stepOver;
            if (y <= maxY + 1e-4) {
                // Parallel-Versatz
                tp.addSegment({MotionType::LinearFeed, currentPos, {targetX, y, currentZ}, {}, tool.defaultFeedRate, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
                currentPos = {targetX, y, currentZ};
            }
            leftToRight = !leftToRight;
        }

        // RÃ¼ckzug auf SicherheitshÃ¶he
        tp.addSegment({MotionType::Rapid, currentPos, {currentPos.x, currentPos.y, params.clearanceZ}, {}, 0, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
        currentPos = {currentPos.x, currentPos.y, params.clearanceZ};

        if (std::abs(currentZ - targetZ) < 1e-5) break;
    }

    return tp;
}

namespace {

constexpr double kPi = 3.14159265358979323846;

struct P2 {
    double x{0.0};
    double y{0.0};
};

P2 add(P2 a, P2 b) { return {a.x + b.x, a.y + b.y}; }
P2 sub(P2 a, P2 b) { return {a.x - b.x, a.y - b.y}; }
P2 mul(P2 a, double s) { return {a.x * s, a.y * s}; }
double len(P2 a) { return std::hypot(a.x, a.y); }
P2 normalized(P2 a) {
    const double l = len(a);
    return l > 1e-12 ? P2{a.x / l, a.y / l} : P2{1.0, 0.0};
}
P2 leftNormal(P2 d) { return {-d.y, d.x}; }

double polygonArea(const std::vector<P2>& pts) {
    double area = 0.0;
    for (size_t i = 0; i < pts.size(); ++i) {
        const P2 a = pts[i];
        const P2 b = pts[(i + 1) % pts.size()];
        area += a.x * b.y - b.x * a.y;
    }
    return area * 0.5;
}

bool pointInPolygon(const std::vector<P2>& poly, P2 p) {
    bool inside = false;
    if (poly.size() < 3) return false;
    for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
        const P2 a = poly[i];
        const P2 b = poly[j];
        if (((a.y > p.y) != (b.y > p.y)) && (p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x)) {
            inside = !inside;
        }
    }
    return inside;
}

// Echte Kreuzung zweier Strecken (Berühren an Endpunkten oder kollineares Anliegen zählt nicht)
bool segmentsCross(P2 a, P2 b, P2 c, P2 d) {
    auto orient = [](P2 o, P2 p, P2 q) { return (p.x - o.x) * (q.y - o.y) - (p.y - o.y) * (q.x - o.x); };
    const double d1 = orient(a, b, c), d2 = orient(a, b, d);
    const double d3 = orient(c, d, a), d4 = orient(c, d, b);
    constexpr double eps = 1e-9;
    return ((d1 > eps && d2 < -eps) || (d1 < -eps && d2 > eps))
        && ((d3 > eps && d4 < -eps) || (d3 < -eps && d4 > eps));
}

// Konturpunkte ohne Doppelpunkte; geschlossene Konturen ohne wiederholten Startpunkt
std::vector<P2> toPoints(const Geometry::Contour& c) {
    std::vector<P2> pts;
    pts.reserve(c.points.size());
    for (const auto& p : c.points) {
        const P2 q{p.x, p.y};
        if (pts.empty() || len(sub(q, pts.back())) > 1e-7) pts.push_back(q);
    }
    if (c.isClosed && pts.size() > 1 && len(sub(pts.front(), pts.back())) < 1e-7) pts.pop_back();
    return pts;
}

std::vector<double> zLevels(double startZ, double targetZ, double stepDown) {
    std::vector<double> levels;
    const double step = std::max(0.1, stepDown);
    double z = startZ;
    while (z > targetZ + 1e-5) {
        z = std::max(targetZ, z - step);
        levels.push_back(z);
    }
    if (levels.empty()) levels.push_back(targetZ);
    return levels;
}

// Schreibt Fahrbewegungen und merkt sich die aktuelle Werkzeugposition
struct PathWriter {
    Toolpath& tp;
    const Core::ToolDefinition& tool;
    Core::Vector3D pos;

    void move(MotionType motion, double x, double y, double z, double feed) {
        if (std::abs(x - pos.x) < 1e-9 && std::abs(y - pos.y) < 1e-9 && std::abs(z - pos.z) < 1e-9) return;
        PathSegment seg;
        seg.motion = motion;
        seg.startPos = pos;
        seg.endPos = Core::Vector3D(x, y, z);
        seg.feedRate = (motion == MotionType::Rapid) ? 0.0 : feed;
        seg.spindleRpm = tool.spindleSpeed;
        seg.toolId = tool.id;
        seg.toolDiameter = tool.diameter;
        tp.addSegment(seg);
        pos = seg.endPos;
    }
    void rapid(double x, double y, double z) { move(MotionType::Rapid, x, y, z, 0.0); }
    void feed(double x, double y, double z, double f) { move(MotionType::LinearFeed, x, y, z, f); }
};

// Werkzeugbahn als Polylinie mit Bogenlängen-Parametrisierung
struct Polyline {
    std::vector<P2> pts;
    std::vector<double> z;   // Tiefe je Punkt (Tiefenprofil)
    bool closed{false};
    std::vector<double> cum; // cum[i] = Bogenlänge bis Punkt i, cum.back() = Gesamtlänge

    void build() {
        cum.assign(1, 0.0);
        const size_t edges = closed ? pts.size() : pts.size() - 1;
        for (size_t i = 0; i < edges; ++i) {
            cum.push_back(cum.back() + len(sub(pts[(i + 1) % pts.size()], pts[i])));
        }
    }
    [[nodiscard]] double length() const { return cum.back(); }
    [[nodiscard]] size_t edgeAt(double local) const {
        const auto it = std::upper_bound(cum.begin(), cum.end(), local);
        const size_t e = (it == cum.begin()) ? 0 : static_cast<size_t>(it - cum.begin()) - 1;
        return std::min(e, cum.size() - 2);
    }
    [[nodiscard]] P2 pointOnEdge(size_t e, double local) const {
        const P2 a = pts[e];
        const P2 b = pts[(e + 1) % pts.size()];
        const double el = cum[e + 1] - cum[e];
        const double t = el > 1e-12 ? std::clamp((local - cum[e]) / el, 0.0, 1.0) : 0.0;
        return add(a, mul(sub(b, a), t));
    }
    [[nodiscard]] P2 pointAt(double s) const {
        const double total = length();
        const double local = closed ? s - std::floor(s / total) * total : std::clamp(s, 0.0, total);
        return pointOnEdge(edgeAt(local), local);
    }
    // Tiefenprofil an Bogenlänge s (linear zwischen den Punkten)
    [[nodiscard]] double zAt(double s) const {
        const double total = length();
        const double local = closed ? s - std::floor(s / total) * total : std::clamp(s, 0.0, total);
        const size_t e = edgeAt(local);
        const double el = cum[e + 1] - cum[e];
        const double t = el > 1e-12 ? std::clamp((local - cum[e]) / el, 0.0, 1.0) : 0.0;
        return z[e] + (z[(e + 1) % pts.size()] - z[e]) * t;
    }
};

// Haltestege entlang einer geschlossenen Bahn (Mitte und halbe Breite als Bogenlänge)
struct TabLayout {
    bool active{false};
    double topZ{0.0};
    double halfWidth{0.0};
    double total{0.0};
    std::vector<double> centers;

    [[nodiscard]] bool inTab(double s) const {
        if (!active || total <= 0.0) return false;
        const double local = s - std::floor(s / total) * total;
        for (double c : centers) {
            double d = std::abs(local - c);
            d = std::min(d, total - d);
            if (d < halfWidth) return true;
        }
        return false;
    }
    [[nodiscard]] std::vector<double> breaks() const {
        std::vector<double> b;
        if (!active) return b;
        for (double c : centers) {
            for (double e : {c - halfWidth, c + halfWidth}) b.push_back(e - std::floor(e / total) * total);
        }
        return b;
    }
};

// Fährt die Polylinie von Bogenlänge a bis b (geschlossen auch über das Ende hinaus) mit Höhe zAt(s).
// An Unstetigkeiten (Haltestegkanten) wird senkrecht verfahren.
template <typename ZFunc>
void followPath(PathWriter& w, const Polyline& path, double a, double b,
                const std::vector<double>& breaks, ZFunc zAt, double feed, double plunge) {
    const double total = path.length();
    if (total < 1e-9 || b - a < 1e-9) return;

    std::vector<double> stops{a, b};
    const int kFrom = path.closed ? static_cast<int>(std::floor(a / total)) - 1 : 0;
    const int kTo = path.closed ? static_cast<int>(std::floor(b / total)) + 1 : 0;
    for (int k = kFrom; k <= kTo; ++k) {
        for (double c : path.cum) {
            const double s = c + k * total;
            if (s > a + 1e-9 && s < b - 1e-9) stops.push_back(s);
        }
        for (double br : breaks) {
            const double s = br + k * total;
            if (s > a + 1e-9 && s < b - 1e-9) stops.push_back(s);
        }
    }
    std::sort(stops.begin(), stops.end());

    for (size_t i = 0; i + 1 < stops.size(); ++i) {
        const double u = stops[i];
        const double v = stops[i + 1];
        if (v - u < 1e-9) continue;
        const double mid = 0.5 * (u + v);
        const double shift = path.closed ? std::floor(mid / total) * total : 0.0;
        const size_t e = path.edgeAt(mid - shift);
        const P2 pu = path.pointOnEdge(e, u - shift);
        const P2 pv = path.pointOnEdge(e, v - shift);
        const double eps = std::min(1e-6, 0.25 * (v - u));
        const double zu = zAt(u + eps);
        const double zv = zAt(v - eps);
        if (std::abs(w.pos.z - zu) > 1e-6 || std::abs(w.pos.x - pu.x) > 1e-6 || std::abs(w.pos.y - pu.y) > 1e-6) {
            w.feed(pu.x, pu.y, zu, zu < w.pos.z ? plunge : feed);
        }
        w.feed(pv.x, pv.y, zv, feed);
    }
}

// Rampe hin und her entlang der ersten Schnittrichtung bis auf Tiefe
void rampDown(PathWriter& w, P2 a, P2 toward, double zFrom, double zTo, double slope, double maxLen, double feed, double plunge) {
    const double l = std::min(len(sub(toward, a)), maxLen);
    if (l < 0.05 || zFrom <= zTo + 1e-6) {
        w.feed(a.x, a.y, zTo, plunge);
        return;
    }
    const P2 b = add(a, mul(normalized(sub(toward, a)), l));
    double z = zFrom;
    bool atA = true;
    while (z > zTo + 1e-6) {
        z = std::max(zTo, z - l * slope);
        const P2 t = atA ? b : a;
        w.feed(t.x, t.y, z, feed);
        atA = !atA;
    }
    if (!atA) w.feed(a.x, a.y, zTo, feed);
}

} // namespace

Toolpath ToolpathGenerator::generateContourMilling(const Geometry::Contour& contour,
                                                  const Core::ToolDefinition& tool,
                                                  const ContourParams& params) {
    Toolpath tp(QStringLiteral("Konturfräsen"));
    if (contour.points.size() < 2) return tp;

    // Offset-Kontur berechnen (Fräserradius + Schlichtaufmaß)
    double offsetDist = 0.0;
    const double r = tool.diameter * 0.5 + params.finishAllowance;
    if (params.side == ContourSide::Outside) {
        offsetDist = contour.isClockwise() ? -r : r;
    } else if (params.side == ContourSide::Inside) {
        offsetDist = contour.isClockwise() ? r : -r;
    }
    std::vector<size_t> sourceIndex;
    const Geometry::Contour pathContour = (params.side != ContourSide::OnLine)
                                          ? contour.createOffset(offsetDist, &sourceIndex)
                                          : contour;
    if (params.side == ContourSide::OnLine) {
        for (size_t k = 0; k < contour.points.size(); ++k) sourceIndex.push_back(k);
    }

    // Punkte mit Tiefenprofil (Tiefe des zugehörigen Konturpunkts, sonst überall targetZ)
    const bool hasProfile = !params.vertexZ.empty() && params.vertexZ.size() == contour.points.size()
                         && sourceIndex.size() == pathContour.points.size();
    Polyline path;
    for (size_t k = 0; k < pathContour.points.size(); ++k) {
        const P2 q{pathContour.points[k].x, pathContour.points[k].y};
        const double qz = hasProfile ? params.vertexZ[sourceIndex[k]] : params.targetZ;
        if (path.pts.empty() || len(sub(q, path.pts.back())) > 1e-7) {
            path.pts.push_back(q);
            path.z.push_back(qz);
        } else {
            path.z.back() = std::min(path.z.back(), qz);
        }
    }
    if (pathContour.isClosed && path.pts.size() > 1 && len(sub(path.pts.front(), path.pts.back())) < 1e-7) {
        path.z.front() = std::min(path.z.front(), path.z.back());
        path.pts.pop_back();
        path.z.pop_back();
    }
    path.closed = pathContour.isClosed && path.pts.size() >= 3;
    if (path.pts.size() < 2) return tp;

    // Fräsrichtung (Spindel M3): Gleichlauf = Material rechts der Vorschubrichtung,
    // also außen im Uhrzeigersinn und innen gegen den Uhrzeigersinn. Der Startpunkt bleibt.
    const bool oriented = path.closed && params.side != ContourSide::OnLine;
    if (oriented) {
        const bool isCW = polygonArea(path.pts) < 0.0;
        const bool wantCW = (params.side == ContourSide::Outside) == params.climbMilling;
        if (isCW != wantCW) {
            std::reverse(path.pts.begin() + 1, path.pts.end());
            std::reverse(path.z.begin() + 1, path.z.end());
        }
    }
    path.build();
    const double total = path.length();
    if (total < 1e-6) return tp;

    const double feed = tool.defaultFeedRate;
    const double plunge = tool.plungeFeedRate > 0.0 ? tool.plungeFeedRate : feed;
    const double clearanceZ = params.clearanceZ;
    const double retractZ = std::max(params.startZ + 1.0, std::min(clearanceZ, params.startZ + 2.0));
    const P2 p0 = path.pts.front();

    // An-/Abfahrt auf der materialabgewandten Seite (bei Gleichlauf links der Vorschubrichtung)
    std::vector<P2> leadIn;
    std::vector<P2> leadOut;
    if (oriented && params.leadType > 0 && params.leadRadius > 1e-3) {
        const double R = params.leadRadius;
        auto awayFrom = [&params](P2 dir) {
            const P2 l = leftNormal(dir);
            return params.climbMilling ? l : mul(l, -1.0);
        };
        const P2 dIn = normalized(sub(path.pts[1], p0));
        const P2 dOut = normalized(sub(p0, path.pts.back()));
        const P2 nIn = awayFrom(dIn);
        const P2 nOut = awayFrom(dOut);
        if (params.leadType == 1) {
            constexpr int arcSteps = 8;
            const P2 cIn = add(p0, mul(nIn, R));
            for (int i = 0; i < arcSteps; ++i) {
                const double t = 0.5 * kPi * i / arcSteps;
                leadIn.push_back(add(cIn, add(mul(dIn, -R * std::cos(t)), mul(nIn, -R * std::sin(t)))));
            }
            const P2 cOut = add(p0, mul(nOut, R));
            for (int i = 1; i <= arcSteps; ++i) {
                const double t = 0.5 * kPi * i / arcSteps;
                leadOut.push_back(add(cOut, add(mul(nOut, -R * std::cos(t)), mul(dOut, R * std::sin(t)))));
            }
        } else {
            leadIn.push_back(add(p0, mul(nIn, R)));
            leadOut.push_back(add(p0, mul(nOut, R)));
        }
    }

    // Haltestege: in den Ebenen unterhalb der Steghöhe bleibt Material stehen
    TabLayout tabs;
    tabs.total = total;
    if (params.useTabs && path.closed && params.tabCount > 0 && params.tabHeight > 1e-3) {
        tabs.active = true;
        tabs.topZ = params.targetZ + params.tabHeight;
        tabs.halfWidth = std::min(0.5 * (params.tabWidth + tool.diameter), 0.45 * total / params.tabCount);
        for (int i = 0; i < params.tabCount; ++i) {
            tabs.centers.push_back((i + 0.5) * total / params.tabCount);
        }
    }
    const std::vector<double> tabBreaks = tabs.breaks();
    const double tabHeight = params.tabHeight;
    auto tabZ = [&tabs, &path, tabHeight](double s, double z) {
        return (tabs.active && tabs.inTab(s)) ? std::max(z, path.zAt(s) + tabHeight) : z;
    };

    const bool useRamp = leadIn.empty() && path.closed && (params.entryType > 0 || params.useRampEntry);
    const double rampSlope = std::tan(std::clamp(params.rampAngleDeg, 0.5, 45.0) * kPi / 180.0);

    const P2 entry = leadIn.empty() ? p0 : leadIn.front();
    PathWriter w{tp, tool, Core::Vector3D(entry.x, entry.y, clearanceZ)};

    double zPrev = params.startZ;
    double sPos = 0.0;    // Bogenlänge, an der das Werkzeug auf der Bahn steht
    bool onPath = false;  // Werkzeug steht nach einer Umrundung auf Tiefe an sPos

    const double deepestZ = *std::min_element(path.z.begin(), path.z.end());
    for (double z : zLevels(params.startZ, deepestZ, params.stepDown)) {
        // Zustellebene z, aber nie tiefer als das Tiefenprofil der Kontur
        auto cutZ = [&tabZ, &path, z](double s) { return tabZ(s, std::max(z, path.zAt(s))); };
        const double zAtStart = cutZ(0.0);

        if (useRamp) {
            // Rampe entlang der Bahn von der vorigen Ebene auf die neue Tiefe, danach volle Umrundung
            const P2 at = path.pointAt(sPos);
            if (!onPath) {
                w.rapid(at.x, at.y, std::max(w.pos.z, retractZ));
                w.feed(at.x, at.y, zPrev, plunge);
            }
            const double s0 = sPos;
            const double rampLen = std::max(0.0, (zPrev - z) / rampSlope);
            const double zTop = zPrev;
            followPath(w, path, s0, s0 + rampLen, tabBreaks,
                       [&tabZ, &path, z, zTop, s0, rampSlope](double s) {
                           return tabZ(s, std::max({z, path.zAt(s), zTop - (s - s0) * rampSlope}));
                       },
                       feed, plunge);
            followPath(w, path, s0 + rampLen, s0 + rampLen + total, tabBreaks, cutZ, feed, plunge);
            sPos = std::fmod(s0 + rampLen, total);
            onPath = true;
        } else if (!leadIn.empty()) {
            // Eintauchen neben der Kontur, An- und Abfahrt auf jeder Ebene
            if (w.pos.z < retractZ - 1e-9) w.rapid(w.pos.x, w.pos.y, retractZ);
            w.rapid(entry.x, entry.y, w.pos.z);
            w.feed(entry.x, entry.y, zAtStart, plunge);
            for (size_t i = 1; i < leadIn.size(); ++i) w.feed(leadIn[i].x, leadIn[i].y, zAtStart, feed);
            w.feed(p0.x, p0.y, zAtStart, feed);
            followPath(w, path, 0.0, total, tabBreaks, cutZ, feed, plunge);
            for (const P2& q : leadOut) w.feed(q.x, q.y, w.pos.z, feed);
        } else {
            // Senkrecht eintauchen am Startpunkt
            if (!onPath) {
                if (w.pos.z < retractZ - 1e-9) w.rapid(w.pos.x, w.pos.y, retractZ);
                w.rapid(p0.x, p0.y, w.pos.z);
            }
            w.feed(p0.x, p0.y, zAtStart, plunge);
            followPath(w, path, 0.0, total, tabBreaks, cutZ, feed, plunge);
            onPath = path.closed;
        }
        zPrev = z;
    }

    // Rückzug
    w.rapid(w.pos.x, w.pos.y, clearanceZ);
    return tp;
}

Toolpath ToolpathGenerator::generatePocketMilling(const Geometry::Contour& boundary,
                                                 const Core::ToolDefinition& tool,
                                                 const PocketParams& params) {
    // Wrapper für Taschen ohne Inseln
    return generatePocketMilling(boundary, {}, tool, params);
}

Toolpath ToolpathGenerator::generatePocketMilling(const Geometry::Contour& boundary,
                                                 const std::vector<Geometry::Contour>& islands,
                                                 const Core::ToolDefinition& tool,
                                                 const PocketParams& params) {
    Toolpath tp(QStringLiteral("Taschenfräsen (Pocketing)"));
    if (boundary.points.size() < 3) return tp;

    const double toolRadius = tool.diameter * 0.5;
    const double stepOver = tool.diameter * std::clamp(params.stepOverRatio, 0.1, 0.9);
    const double clearanceZ = params.clearanceZ;
    const double retractZ = std::max(params.startZ + 1.0, std::min(clearanceZ, params.startZ + 2.0));
    const double feed = tool.defaultFeedRate;
    const double plunge = tool.plungeFeedRate > 0.0 ? tool.plungeFeedRate : feed;
    const double rampSlope = std::tan(std::clamp(params.rampAngleDeg, 0.5, 45.0) * kPi / 180.0);

    // 1. Konzentrische Ringe (außen → innen) über PolygonOffset
    const auto rings = Geometry::PolygonOffset::generatePocketContours(boundary, islands, toolRadius, stepOver, params.finishAllowance);

    // 2. Ringkonturen als Polylinien; Umlaufsinn nach Fräsrichtung
    //    (Gleichlauf: Außenrand gegen, Inselränder im Uhrzeigersinn)
    using Loop = std::vector<P2>;
    std::vector<std::vector<Loop>> ringLoops;
    for (const auto& ring : rings) {
        std::vector<Loop> loops;
        for (const auto& c : ring) {
            Loop pts = toPoints(c);
            if (pts.size() >= 3) loops.push_back(std::move(pts));
        }
        for (size_t i = 0; i < loops.size(); ++i) {
            bool isHole = false;
            for (size_t j = 0; j < loops.size(); ++j) {
                if (i != j && pointInPolygon(loops[j], loops[i].front())) isHole = !isHole;
            }
            const bool wantCCW = (params.climbMilling != isHole);
            if ((polygonArea(loops[i]) > 0.0) != wantCCW) std::reverse(loops[i].begin() + 1, loops[i].end());
        }
        if (!loops.empty()) ringLoops.push_back(std::move(loops));
    }
    if (ringLoops.empty()) return tp; // Tasche zu klein für diesen Fräser oder ungültig

    // Äußerster Ring = zulässiger Bereich der Fräsermitte
    const std::vector<Loop>& region = ringLoops.front();
    auto insideRegion = [&region](P2 p) {
        bool in = false;
        for (const auto& loop : region) {
            if (pointInPolygon(loop, p)) in = !in;
        }
        return in;
    };
    auto crossesRegion = [&region](P2 a, P2 b) {
        for (const auto& loop : region) {
            for (size_t i = 0; i < loop.size(); ++i) {
                if (segmentsCross(a, b, loop[i], loop[(i + 1) % loop.size()])) return true;
            }
        }
        return false;
    };

    // 3. Bearbeitungsfolge je Ebene
    struct Cut {
        std::vector<P2> pts;
        bool closed{false};
    };
    std::vector<Cut> cuts;
    if (params.strategy == 0) {
        // Zickzack: Rasterzeilen in X, abwechselnd hin und zurück; danach Randbahnen
        double yMin = std::numeric_limits<double>::max();
        double yMax = std::numeric_limits<double>::lowest();
        for (const auto& loop : region) {
            for (const P2& p : loop) {
                yMin = std::min(yMin, p.y);
                yMax = std::max(yMax, p.y);
            }
        }
        const double height = yMax - yMin - 2e-3;
        if (height > 0.0) {
            const int lineCount = std::max(1, static_cast<int>(std::ceil(height / stepOver))) + 1;
            bool reverseRow = false;
            for (int i = 0; i < lineCount; ++i) {
                const double y = yMin + 1e-3 + height * i / (lineCount - 1);
                std::vector<double> xs;
                for (const auto& loop : region) {
                    for (size_t k = 0; k < loop.size(); ++k) {
                        const P2 a = loop[k];
                        const P2 b = loop[(k + 1) % loop.size()];
                        if ((a.y <= y && b.y > y) || (b.y <= y && a.y > y)) {
                            xs.push_back(a.x + (y - a.y) * (b.x - a.x) / (b.y - a.y));
                        }
                    }
                }
                std::sort(xs.begin(), xs.end());
                std::vector<Cut> row;
                for (size_t k = 0; k + 1 < xs.size(); k += 2) {
                    if (xs[k + 1] - xs[k] > 1e-3) row.push_back({{P2{xs[k], y}, P2{xs[k + 1], y}}, false});
                }
                if (row.empty()) continue;
                if (reverseRow) {
                    std::reverse(row.begin(), row.end());
                    for (auto& cut : row) std::reverse(cut.pts.begin(), cut.pts.end());
                }
                reverseRow = !reverseRow;
                cuts.insert(cuts.end(), row.begin(), row.end());
            }
        }
        for (const auto& loop : region) cuts.push_back({loop, true});
    } else {
        auto ordered = ringLoops;
        if (params.strategy == 1) std::reverse(ordered.begin(), ordered.end()); // Spiral: innen → außen
        for (const auto& ring : ordered) {
            for (const auto& loop : ring) cuts.push_back({loop, true});
        }
    }
    if (cuts.empty()) return tp;

    const double maxDirectLink = (params.strategy == 0 ? 3.0 : 2.0) * stepOver;
    const P2 entry = cuts.front().pts[0];
    const P2 toward = cuts.front().pts[1];

    // Helix-Eintauchen nur, wenn der Helixkreis vollständig im zulässigen Bereich liegt
    P2 helixCenter;
    double helixR = 0.0;
    if (params.entryType == 1) {
        P2 c;
        for (const P2& p : cuts.front().pts) c = add(c, p);
        c = mul(c, 1.0 / static_cast<double>(cuts.front().pts.size()));
        double minDist = std::numeric_limits<double>::max();
        for (const P2& p : cuts.front().pts) minDist = std::min(minDist, len(sub(p, c)));
        const double hr = std::min(0.25 * tool.diameter, 0.5 * minDist);
        bool fits = hr >= 0.1 && insideRegion(c) && !crossesRegion(c, entry);
        for (int i = 0; fits && i < 16; ++i) {
            const double a = 2.0 * kPi * i / 16.0;
            const P2 q = add(c, P2{hr * std::cos(a), hr * std::sin(a)});
            fits = insideRegion(q) && !crossesRegion(c, q);
        }
        if (fits) {
            helixCenter = c;
            helixR = hr;
        }
    }

    PathWriter w{tp, tool, Core::Vector3D(entry.x, entry.y, clearanceZ)};

    // Verbindung zur nächsten Bahn: kurz und im zulässigen Bereich → im Vorschub,
    // sonst über die Sicherheitshöhe (kein Eilgang und keine Überfahrt durch Material/Inseln)
    auto linkTo = [&](P2 target, P2 next, double z) {
        const P2 cur{w.pos.x, w.pos.y};
        const double dist = len(sub(target, cur));
        if (dist < 1e-6 && std::abs(w.pos.z - z) < 1e-6) return;
        bool direct = dist <= maxDirectLink && std::abs(w.pos.z - z) < 1e-6;
        if (direct) {
            P2 probe = mul(add(cur, target), 0.5);
            if (len(sub(next, target)) > 1e-6) probe = add(probe, mul(normalized(sub(next, target)), 1e-3));
            direct = !crossesRegion(cur, target) && insideRegion(probe);
        }
        if (direct) {
            w.feed(target.x, target.y, z, feed);
        } else {
            w.rapid(cur.x, cur.y, std::max(w.pos.z, retractZ));
            w.rapid(target.x, target.y, w.pos.z);
            w.feed(target.x, target.y, z, plunge);
        }
    };

    double zPrev = params.startZ;
    for (double z : zLevels(params.startZ, params.targetZ, params.stepDown)) {
        // Einstieg
        if (helixR > 0.0) {
            const P2 s0 = add(helixCenter, P2{helixR, 0.0});
            w.rapid(s0.x, s0.y, clearanceZ);
            w.feed(s0.x, s0.y, zPrev, plunge);
            const double pitch = std::max(0.2, 2.0 * kPi * helixR * rampSlope);
            const double depth = zPrev - z;
            const double turns = depth / pitch;
            const int steps = std::max(8, static_cast<int>(std::ceil(turns * 24.0)));
            for (int i = 1; i <= steps; ++i) {
                const double t = static_cast<double>(i) / steps;
                const double a = 2.0 * kPi * turns * t;
                w.feed(helixCenter.x + helixR * std::cos(a), helixCenter.y + helixR * std::sin(a), zPrev - depth * t, feed);
            }
            const double aEnd = 2.0 * kPi * turns;
            for (int i = 1; i <= 24; ++i) {
                const double a = aEnd + 2.0 * kPi * i / 24.0;
                w.feed(helixCenter.x + helixR * std::cos(a), helixCenter.y + helixR * std::sin(a), z, feed);
            }
            w.feed(entry.x, entry.y, z, feed);
        } else if (params.entryType > 0) {
            w.rapid(entry.x, entry.y, clearanceZ);
            w.feed(entry.x, entry.y, zPrev, plunge);
            rampDown(w, entry, toward, zPrev, z, rampSlope, 2.0 * tool.diameter, feed, plunge);
        } else {
            w.rapid(entry.x, entry.y, clearanceZ);
            w.feed(entry.x, entry.y, z, plunge);
        }

        // Bahnen abfahren
        for (const auto& cut : cuts) {
            linkTo(cut.pts[0], cut.pts.size() > 1 ? cut.pts[1] : cut.pts[0], z);
            for (size_t k = 1; k < cut.pts.size(); ++k) w.feed(cut.pts[k].x, cut.pts[k].y, z, feed);
            if (cut.closed) w.feed(cut.pts[0].x, cut.pts[0].y, z, feed);
        }

        // Rückzug am Ende jeder Ebene
        w.rapid(w.pos.x, w.pos.y, clearanceZ);
        zPrev = z;
    }

    return tp;
}

Toolpath ToolpathGenerator::generateStockRoughing(const Geometry::Mesh& stockMesh,
                                                 const Geometry::Mesh& targetPartMesh,
                                                 const Core::ToolDefinition& tool,
                                                 const StockRoughingParams& params) {
    Toolpath tp(QStringLiteral("3D Rohteilschruppen (Stock Roughing)"));

    const auto& stockBox = stockMesh.boundingBox;
    const auto& partBox = targetPartMesh.boundingBox;

    if (!stockBox.isValid()) return tp;

    const double startZ = stockBox.maxPoint.z;
    const double targetZ = partBox.isValid() ? partBox.minPoint.z : stockBox.minPoint.z;
    const double stepDown = std::max(0.2, params.stepDown);
    const double stepOver = tool.diameter * std::clamp(params.stepOverRatio, 0.2, 0.8);

    double currentZ = startZ;
    const double toolRadius = tool.diameter * 0.5 + params.finishAllowance;

    const double minX = stockBox.minPoint.x + toolRadius;
    const double maxX = stockBox.maxPoint.x - toolRadius;
    const double minY = stockBox.minPoint.y + toolRadius;
    const double maxY = stockBox.maxPoint.y - toolRadius;

    Core::Vector3D currentPos(minX, minY, params.clearanceZ);

    while (currentZ > targetZ - 1e-5) {
        currentZ -= stepDown;
        if (currentZ < targetZ) currentZ = targetZ;

        // Anfahrt auf Eilgang Ã¼ber Startpunkt der Schicht
        tp.addSegment({MotionType::Rapid, currentPos, {minX, minY, params.clearanceZ}, {}, 0, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
        tp.addSegment({MotionType::LinearFeed, {minX, minY, params.clearanceZ}, {minX, minY, currentZ}, {}, tool.plungeFeedRate, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
        currentPos = {minX, minY, currentZ};

        double y = minY;
        bool leftToRight = true;

        while (y <= maxY + 1e-4) {
            double targetX = leftToRight ? maxX : minX;

            // In dieser Schicht abtragen
            tp.addSegment({MotionType::LinearFeed, currentPos, {targetX, y, currentZ}, {}, tool.defaultFeedRate, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
            currentPos = {targetX, y, currentZ};

            y += stepOver;
            if (y <= maxY + 1e-4) {
                tp.addSegment({MotionType::LinearFeed, currentPos, {targetX, y, currentZ}, {}, tool.defaultFeedRate, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
                currentPos = {targetX, y, currentZ};
            }
            leftToRight = !leftToRight;
        }

        // RÃ¼ckzug
        tp.addSegment({MotionType::Rapid, currentPos, {currentPos.x, currentPos.y, params.clearanceZ}, {}, 0, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
        currentPos = {currentPos.x, currentPos.y, params.clearanceZ};

        if (std::abs(currentZ - targetZ) < 1e-5) break;
    }

    return tp;
}

Toolpath ToolpathGenerator::generate3DSurfaceFinishing(const Geometry::Mesh& targetPartMesh,
                                                      const Core::ToolDefinition& tool,
                                                      const SurfaceFinishingParams& params) {
    Toolpath tp(QStringLiteral("3D-FreiformflÃ¤chen-Schlichten"));
    if (targetPartMesh.isEmpty()) return tp;

    const auto& bbox = targetPartMesh.boundingBox;
    if (!bbox.isValid()) return tp;

    const double toolRadius = tool.diameter * 0.5;
    const double stepOver = std::max(0.05, params.stepOver);
    const double sampleDx = std::max(0.1, params.sampleStep);

    const double minX = bbox.minPoint.x - toolRadius;
    const double maxX = bbox.maxPoint.x + toolRadius;
    const double minY = bbox.minPoint.y - toolRadius;
    const double maxY = bbox.maxPoint.y + toolRadius;

    // Lambda zur Bestimmung der 3D-Z-HÃ¶he auf dem Mesh fÃ¼r (x, y)
    auto queryMeshZ = [&](double qx, double qy) -> double {
        double maxZ = bbox.minPoint.z;
        bool found = false;

        for (const auto& tri : targetPartMesh.triangles) {
            const auto& v0 = targetPartMesh.vertices[tri.i0];
            const auto& v1 = targetPartMesh.vertices[tri.i1];
            const auto& v2 = targetPartMesh.vertices[tri.i2];

            // Bounding Box Check 2D
            float triMinX = std::min({v0.x, v1.x, v2.x});
            float triMaxX = std::max({v0.x, v1.x, v2.x});
            float triMinY = std::min({v0.y, v1.y, v2.y});
            float triMaxY = std::max({v0.y, v1.y, v2.y});

            if (qx < triMinX || qx > triMaxX || qy < triMinY || qy > triMaxY) continue;

            // Baryzentrische Koordinaten in 2D
            double det = (v1.y - v2.y) * (v0.x - v2.x) + (v2.x - v1.x) * (v0.y - v2.y);
            if (std::abs(det) < 1e-9) continue;

            double w0 = ((v1.y - v2.y) * (qx - v2.x) + (v2.x - v1.x) * (qy - v2.y)) / det;
            double w1 = ((v2.y - v0.y) * (qx - v2.x) + (v0.x - v2.x) * (qy - v2.y)) / det;
            double w2 = 1.0 - w0 - w1;

            if (w0 >= -1e-4 && w1 >= -1e-4 && w2 >= -1e-4) {
                double z = w0 * v0.z + w1 * v1.z + w2 * v2.z;
                if (!found || z > maxZ) {
                    maxZ = z;
                    found = true;
                }
            }
        }
        return found ? maxZ : bbox.minPoint.z;
    };

    Core::Vector3D currentPos(minX, minY, params.clearanceZ);
    tp.addSegment({MotionType::Rapid, currentPos, {minX, minY, params.clearanceZ}, {}, 0, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});

    double y = minY;
    bool leftToRight = true;

    while (y <= maxY + 1e-4) {
        double startLineX = leftToRight ? minX : maxX;
        double endLineX = leftToRight ? maxX : minX;
        double dir = leftToRight ? 1.0 : -1.0;

        // Anfahrt Ã¼ber ersten Punkt der Zeile
        double firstZ = queryMeshZ(startLineX, y);
        tp.addSegment({MotionType::Rapid, currentPos, {startLineX, y, params.clearanceZ}, {}, 0, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
        tp.addSegment({MotionType::LinearFeed, {startLineX, y, params.clearanceZ}, {startLineX, y, firstZ}, {}, tool.plungeFeedRate, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
        currentPos = {startLineX, y, firstZ};

        double curX = startLineX;
        while ((dir > 0 && curX < endLineX) || (dir < 0 && curX > endLineX)) {
            curX += dir * sampleDx;
            if ((dir > 0 && curX > endLineX) || (dir < 0 && curX < endLineX)) curX = endLineX;

            double surfaceZ = queryMeshZ(curX, y);
            Core::Vector3D nextPos(curX, y, surfaceZ);
            tp.addSegment({MotionType::LinearFeed, currentPos, nextPos, {}, tool.defaultFeedRate, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
            currentPos = nextPos;
        }

        // RÃ¼ckzug am Zeilenende
        tp.addSegment({MotionType::Rapid, currentPos, {currentPos.x, currentPos.y, params.clearanceZ}, {}, 0, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
        currentPos = {currentPos.x, currentPos.y, params.clearanceZ};

        y += stepOver;
        leftToRight = !leftToRight;
    }

    return tp;
}

Toolpath ToolpathGenerator::generateStlMilling(const Geometry::Mesh& mesh,
                                              const Core::BoundingBox& stockBounds,
                                              const Core::ToolDefinition& tool,
                                              const StlMillingParams& params) {
    Toolpath tp(QStringLiteral("3D-STL FrÃ¤sen"));
    if (mesh.isEmpty() || mesh.triangles.empty()) return tp;

    Core::BoundingBox areaBox = stockBounds.isValid() ? stockBounds : mesh.boundingBox;
    if (!areaBox.isValid()) return tp;

    const double toolRadius = std::max(0.5, tool.diameter * 0.5);
    const double sampleDx = std::max(0.1, params.sampleStep);
    const double clearanceZ = std::max(areaBox.maxPoint.z + 1.0, params.clearanceZ);

    // 1. Spatial Acceleration Grid fÃ¼r blitzschnelle Triangeldurchsuchung
    const double gridMinX = mesh.boundingBox.minPoint.x;
    const double gridMinY = mesh.boundingBox.minPoint.y;
    const double gridWidth = std::max(1.0, mesh.boundingBox.widthX());
    const double gridDepth = std::max(1.0, mesh.boundingBox.depthY());

    const int gridCols = std::clamp(static_cast<int>(gridWidth / 4.0), 16, 128);
    const int gridRows = std::clamp(static_cast<int>(gridDepth / 4.0), 16, 128);
    const double cellW = gridWidth / gridCols;
    const double cellH = gridDepth / gridRows;

    std::vector<std::vector<uint32_t>> spatialGrid(gridCols * gridRows);
    for (size_t tIdx = 0; tIdx < mesh.triangles.size(); ++tIdx) {
        const auto& tri = mesh.triangles[tIdx];
        const auto& v0 = mesh.vertices[tri.i0];
        const auto& v1 = mesh.vertices[tri.i1];
        const auto& v2 = mesh.vertices[tri.i2];

        float minTx = std::min({v0.x, v1.x, v2.x});
        float maxTx = std::max({v0.x, v1.x, v2.x});
        float minTy = std::min({v0.y, v1.y, v2.y});
        float maxTy = std::max({v0.y, v1.y, v2.y});

        int c0 = std::clamp(static_cast<int>((minTx - gridMinX) / cellW), 0, gridCols - 1);
        int c1 = std::clamp(static_cast<int>((maxTx - gridMinX) / cellW), 0, gridCols - 1);
        int r0 = std::clamp(static_cast<int>((minTy - gridMinY) / cellH), 0, gridRows - 1);
        int r1 = std::clamp(static_cast<int>((maxTy - gridMinY) / cellH), 0, gridRows - 1);

        for (int r = r0; r <= r1; ++r) {
            for (int c = c0; c <= c1; ++c) {
                spatialGrid[r * gridCols + c].push_back(static_cast<uint32_t>(tIdx));
            }
        }
    }

    // Punkt-Strahlschnitt mit STL-OberflÃ¤che
    auto queryPointZ = [&](double qx, double qy) -> double {
        int c = static_cast<int>((qx - gridMinX) / cellW);
        int r = static_cast<int>((qy - gridMinY) / cellH);
        if (c < 0 || c >= gridCols || r < 0 || r >= gridRows) {
            return areaBox.minPoint.z;
        }

        const auto& cellTris = spatialGrid[r * gridCols + c];
        double highestZ = areaBox.minPoint.z;
        bool hit = false;

        for (uint32_t tIdx : cellTris) {
            const auto& tri = mesh.triangles[tIdx];
            const auto& v0 = mesh.vertices[tri.i0];
            const auto& v1 = mesh.vertices[tri.i1];
            const auto& v2 = mesh.vertices[tri.i2];

            double det = (v1.y - v2.y) * (v0.x - v2.x) + (v2.x - v1.x) * (v0.y - v2.y);
            if (std::abs(det) < 1e-9) continue;

            double w0 = ((v1.y - v2.y) * (qx - v2.x) + (v2.x - v1.x) * (qy - v2.y)) / det;
            double w1 = ((v2.y - v0.y) * (qx - v2.x) + (v0.x - v2.x) * (qy - v2.y)) / det;
            double w2 = 1.0 - w0 - w1;

            if (w0 >= -1e-4 && w1 >= -1e-4 && w2 >= -1e-4) {
                double z = w0 * v0.z + w1 * v1.z + w2 * v2.z;
                if (!hit || z > highestZ) {
                    highestZ = z;
                    hit = true;
                }
            }
        }
        return hit ? std::max(highestZ, areaBox.minPoint.z) : areaBox.minPoint.z;
    };

    // Werkzeugradius-Kompensation (untere StirnflÃ¤che bzw. Kugelradius)
    auto queryToolZ = [&](double cx, double cy) -> double {
        bool isBallEnd = (tool.type == Core::ToolType::BallMill);
        double maxZ = queryPointZ(cx, cy);

        // Mehrere Abtastpunkte auf dem Werkzeugradius
        constexpr int numRings = 2;
        constexpr int ptsPerRing = 6;
        for (int ring = 1; ring <= numRings; ++ring) {
            double rFrac = static_cast<double>(ring) / numRings;
            double curR = toolRadius * rFrac;
            double zOffset = isBallEnd ? (toolRadius - std::sqrt(std::max(0.0, toolRadius * toolRadius - curR * curR))) : 0.0;

            for (int p = 0; p < ptsPerRing; ++p) {
                double ang = p * (2.0 * M_PI / ptsPerRing);
                double sx = cx + curR * std::cos(ang);
                double sy = cy + curR * std::sin(ang);
                double sZ = queryPointZ(sx, sy) - zOffset;
                if (sZ > maxZ) maxZ = sZ;
            }
        }

        return std::clamp(maxZ + params.finishAllowance, areaBox.minPoint.z, areaBox.maxPoint.z + 10.0);
    };

    const double minX = areaBox.minPoint.x;
    const double maxX = areaBox.maxPoint.x;
    const double minY = areaBox.minPoint.y;
    const double maxY = areaBox.maxPoint.y;

    Core::Vector3D currentPos(minX, minY, clearanceZ);
    tp.addSegment({MotionType::Rapid, currentPos, {minX, minY, clearanceZ}, {}, 0, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});

    bool doRoughing = (params.mode == StlMillingMode::RoughAndFinish || params.mode == StlMillingMode::RoughOnly);
    bool doFinishing = (params.mode == StlMillingMode::RoughAndFinish || params.mode == StlMillingMode::FinishOnly ||
                        params.mode == StlMillingMode::FinishRasterXY || params.mode == StlMillingMode::WaterlineFinish);

    // â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• 
    // PHASE 1: Intelligentes 3D Z-Ebenen Schruppen (Materialabtrag)
    // â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• 
    if (doRoughing) {
        double curZ = areaBox.maxPoint.z;
        const double roughStepDown = std::max(0.2, params.roughStepDown);

        if (params.useTrochoidal && params.trochoidalEngagement > 0.01) {
            // ── Trochoidales 3D-Schruppen ──
            // Trochoidale Kreisbögen pro Z-Ebene mit materialabhängigem ae
            const double trochoidAe = std::max(0.3, tool.diameter * params.trochoidalEngagement);
            const double trochoidFeed = tool.defaultFeedRate * std::max(1.0, params.trochoidalFeedFactor);
            const double trochoidR = trochoidAe * 0.5;  // Radius der Kreisbögen
            constexpr int arcSteps = 12;  // Punkte pro Halbkreis
            constexpr double kPi = 3.14159265358979323846;

            while (curZ > areaBox.minPoint.z - 1e-4) {
                curZ -= roughStepDown;
                if (curZ < areaBox.minPoint.z) curZ = areaBox.minPoint.z;

                double y = minY;
                bool leftToRight = true;

                while (y <= maxY + 1e-4) {
                    const double startX = leftToRight ? minX : maxX;
                    const double endX = leftToRight ? maxX : minX;
                    const double dir = leftToRight ? 1.0 : -1.0;

                    // Eintauchen
                    double surfaceZ = queryToolZ(startX, y);
                    double targetCutZ = std::max(curZ, surfaceZ);
                    tp.addSegment({MotionType::Rapid, currentPos, {startX, y, clearanceZ}, {}, 0, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
                    tp.addSegment({MotionType::LinearFeed, {startX, y, clearanceZ}, {startX, y, targetCutZ}, {}, tool.plungeFeedRate, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
                    currentPos = Core::Vector3D(startX, y, targetCutZ);

                    // Trochoidale Bögen entlang X
                    double cx = startX;
                    bool halfUp = true;  // true = Bogen nach +Y, false = nach -Y
                    while ((dir > 0 && cx < endX - 1e-4) || (dir < 0 && cx > endX + 1e-4)) {
                        cx += dir * trochoidAe;
                        if ((dir > 0 && cx > endX) || (dir < 0 && cx < endX)) cx = endX;

                        double localSurfZ = queryToolZ(cx, y);
                        double localCutZ = std::max(curZ, localSurfZ);

                        // Linearisierter Halbkreis (12 Schritte)
                        const double cxCenter = cx - dir * trochoidAe * 0.5;
                        for (int i = 1; i <= arcSteps; ++i) {
                            double t = static_cast<double>(i) / arcSteps;
                            double angle = halfUp ? (kPi * t) : (-kPi * t);
                            double ax = cxCenter + trochoidR * std::cos(angle) * dir;
                            double ay = y + trochoidR * std::sin(angle);
                            double az = std::max(localCutZ, queryToolZ(ax, ay));
                            Core::Vector3D arcPt(ax, ay, az);
                            tp.addSegment({MotionType::LinearFeed, currentPos, arcPt, {}, trochoidFeed, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
                            currentPos = arcPt;
                        }

                        // Gerade zurück zur Zeilenmitte
                        Core::Vector3D rowPt(cx, y, localCutZ);
                        tp.addSegment({MotionType::LinearFeed, currentPos, rowPt, {}, trochoidFeed, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
                        currentPos = rowPt;
                        halfUp = !halfUp;
                    }

                    // Rückzug
                    tp.addSegment({MotionType::Rapid, currentPos, {currentPos.x, currentPos.y, clearanceZ}, {}, 0, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
                    currentPos = Core::Vector3D(currentPos.x, currentPos.y, clearanceZ);

                    y += trochoidAe;  // Trochoidaler Zeilenabstand = ae
                    leftToRight = !leftToRight;
                }

                if (std::abs(curZ - areaBox.minPoint.z) < 1e-4) break;
            }
        } else {
            // ── Konventionelles Zickzack-Schruppen ──
            const double roughStepOver = std::max(0.5, tool.diameter * params.roughStepOverRatio);

            while (curZ > areaBox.minPoint.z - 1e-4) {
                curZ -= roughStepDown;
                if (curZ < areaBox.minPoint.z) curZ = areaBox.minPoint.z;

                double y = minY;
                bool leftToRight = true;

                while (y <= maxY + 1e-4) {
                    double startX = leftToRight ? minX : maxX;
                    double endX = leftToRight ? maxX : minX;
                    double dir = leftToRight ? 1.0 : -1.0;

                    double surfaceZ = queryToolZ(startX, y);
                    double targetCutZ = std::max(curZ, surfaceZ);

                    tp.addSegment({MotionType::Rapid, currentPos, {startX, y, clearanceZ}, {}, 0, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
                    tp.addSegment({MotionType::LinearFeed, {startX, y, clearanceZ}, {startX, y, targetCutZ}, {}, tool.plungeFeedRate, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
                    currentPos = {startX, y, targetCutZ};

                    double curX = startX;
                    while ((dir > 0 && curX < endX - 1e-4) || (dir < 0 && curX > endX + 1e-4)) {
                        curX += dir * sampleDx;
                        if ((dir > 0 && curX > endX) || (dir < 0 && curX < endX)) curX = endX;

                        double sZ = queryToolZ(curX, y);
                        double cutZ = std::max(curZ, sZ);
                        Core::Vector3D nextPt(curX, y, cutZ);
                        tp.addSegment({MotionType::LinearFeed, currentPos, nextPt, {}, tool.defaultFeedRate, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
                        currentPos = nextPt;
                    }

                    tp.addSegment({MotionType::Rapid, currentPos, {currentPos.x, currentPos.y, clearanceZ}, {}, 0, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
                    currentPos = {currentPos.x, currentPos.y, clearanceZ};

                    y += roughStepOver;
                    leftToRight = !leftToRight;
                }

                if (std::abs(curZ - areaBox.minPoint.z) < 1e-4) break;
            }
        }
    }

    // â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• 
    // PHASE 2: Intelligentes 3D FreiformflÃ¤chen-Schlichten (Endkontur)
    // â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• â• 
    if (doFinishing) {
        const double finishStepOver = std::max(0.05, params.finishStepOver);

        // Werkzeug-Z-Abfrage ohne AufmaÃŸ fÃ¼r exakte BauteiloberflÃ¤che
        auto queryFinishZ = [&](double cx, double cy) -> double {
            bool isBallEnd = (tool.type == Core::ToolType::BallMill);
            double maxZ = queryPointZ(cx, cy);

            constexpr int numRings = 2;
            constexpr int ptsPerRing = 6;
            for (int ring = 1; ring <= numRings; ++ring) {
                double rFrac = static_cast<double>(ring) / numRings;
                double curR = toolRadius * rFrac;
                double zOffset = isBallEnd ? (toolRadius - std::sqrt(std::max(0.0, toolRadius * toolRadius - curR * curR))) : 0.0;

                for (int p = 0; p < ptsPerRing; ++p) {
                    double ang = p * (2.0 * M_PI / ptsPerRing);
                    double sx = cx + curR * std::cos(ang);
                    double sy = cy + curR * std::sin(ang);
                    double sZ = queryPointZ(sx, sy) - zOffset;
                    if (sZ > maxZ) maxZ = sZ;
                }
            }
            return std::clamp(maxZ, areaBox.minPoint.z, areaBox.maxPoint.z + 5.0);
        };

        auto rasterX = [&]() {
            double y = minY;
            bool leftToRight = true;

            while (y <= maxY + 1e-4) {
                double startX = leftToRight ? minX : maxX;
                double endX = leftToRight ? maxX : minX;
                double dir = leftToRight ? 1.0 : -1.0;

                double firstZ = queryFinishZ(startX, y);
                tp.addSegment({MotionType::Rapid, currentPos, {startX, y, clearanceZ}, {}, 0, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
                tp.addSegment({MotionType::LinearFeed, {startX, y, clearanceZ}, {startX, y, firstZ}, {}, tool.plungeFeedRate, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
                currentPos = {startX, y, firstZ};

                double curX = startX;
                while ((dir > 0 && curX < endX - 1e-4) || (dir < 0 && curX > endX + 1e-4)) {
                    curX += dir * sampleDx;
                    if ((dir > 0 && curX > endX) || (dir < 0 && curX < endX)) curX = endX;

                    double nextZ = queryFinishZ(curX, y);
                    Core::Vector3D nextPt(curX, y, nextZ);
                    tp.addSegment({MotionType::LinearFeed, currentPos, nextPt, {}, tool.defaultFeedRate, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
                    currentPos = nextPt;
                }

                tp.addSegment({MotionType::Rapid, currentPos, {currentPos.x, currentPos.y, clearanceZ}, {}, 0, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
                currentPos = {currentPos.x, currentPos.y, clearanceZ};

                y += finishStepOver;
                leftToRight = !leftToRight;
            }
        };

        auto rasterY = [&]() {
            double x = minX;
            bool bottomToTop = true;

            while (x <= maxX + 1e-4) {
                double startY = bottomToTop ? minY : maxY;
                double endY = bottomToTop ? maxY : minY;
                double dir = bottomToTop ? 1.0 : -1.0;

                double firstZ = queryFinishZ(x, startY);
                tp.addSegment({MotionType::Rapid, currentPos, {x, startY, clearanceZ}, {}, 0, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
                tp.addSegment({MotionType::LinearFeed, {x, startY, clearanceZ}, {x, startY, firstZ}, {}, tool.plungeFeedRate, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
                currentPos = {x, startY, firstZ};

                double curY = startY;
                while ((dir > 0 && curY < endY - 1e-4) || (dir < 0 && curY > endY + 1e-4)) {
                    curY += dir * sampleDx;
                    if ((dir > 0 && curY > endY) || (dir < 0 && curY < endY)) curY = endY;

                    double nextZ = queryFinishZ(x, curY);
                    Core::Vector3D nextPt(x, curY, nextZ);
                    tp.addSegment({MotionType::LinearFeed, currentPos, nextPt, {}, tool.defaultFeedRate, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
                    currentPos = nextPt;
                }

                tp.addSegment({MotionType::Rapid, currentPos, {currentPos.x, currentPos.y, clearanceZ}, {}, 0, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});
                currentPos = {currentPos.x, currentPos.y, clearanceZ};

                x += finishStepOver;
                bottomToTop = !bottomToTop;
            }
        };

        if (params.mode == StlMillingMode::FinishRasterXY) {
            rasterX();
            rasterY();
        } else if (params.mode == StlMillingMode::WaterlineFinish) {
            // Einfacher Fallback: Für Waterline nehmen wir RasterXY, um eine gute Oberfläche zu kriegen,
            // da echte Kontur-Verfolgung (Marching Squares) hier nicht implementiert ist.
            rasterX();
            rasterY();
        } else if (params.finishDirection == 0) {
            rasterX();
        } else {
            rasterY();
        }
    }

    // Finaler RÃ¼ckzug auf Sicherheitsebene
    tp.addSegment({MotionType::Rapid, currentPos, {currentPos.x, currentPos.y, clearanceZ}, {}, 0, tool.spindleSpeed, false, tool.id, tool.diameter, 0, false, {}});

    return tp;
}

// ═══════════════════════════════════════════════════════════════════════
// Trochoidales Fräsen
// ═══════════════════════════════════════════════════════════════════════

Toolpath ToolpathGenerator::generateTrochoidalSlot(
    double startX, double startY,
    double endX, double endY,
    double slotWidth,
    const Core::ToolDefinition& tool,
    double startZ, double targetZ, double clearanceZ,
    double engagement, double feedFactor,
    bool climbMilling) {

    Toolpath tp(QStringLiteral("Trochoidale Nut"));

    constexpr double kPi = 3.14159265358979323846;
    const double toolDia = std::max(0.5, tool.diameter);
    const double toolRadius = toolDia * 0.5;

    // Trochoid-Radius: halbe Differenz Nut - Fräser
    const double trochoidR = std::max(0.05, (slotWidth - toolDia) * 0.5);

    // Trochoidaler Vorschub-Schritt pro Bogen (ae = d * engagement)
    const double ae = std::max(0.05, toolDia * std::clamp(engagement, 0.02, 0.25));

    // Richtungsvektor entlang der Nut
    const double dx = endX - startX;
    const double dy = endY - startY;
    const double slotLen = std::hypot(dx, dy);
    if (slotLen < 0.1) return tp;

    // Einheitsvektor Vorschubrichtung und Senkrechte
    const double ux = dx / slotLen;
    const double uy = dy / slotLen;
    const double nx = -uy;  // Senkrecht links
    const double ny = ux;

    // Vorschub: Basis-Feed * Faktor
    const double feed = tool.defaultFeedRate * std::clamp(feedFactor, 1.0, 4.0);
    const double plunge = tool.plungeFeedRate > 0.0 ? tool.plungeFeedRate : feed * 0.3;
    const int arcSteps = 12;  // Punkte pro Halbkreis (linearisiert)

    PathWriter w{tp, tool, Core::Vector3D(startX, startY, clearanceZ)};

    // Z-Ebenen: volle Tiefe bevorzugt, aber stepDown begrenzt
    const double maxAp = tool.fluteLength > 0.5 ? tool.fluteLength * 0.8 : toolDia * 2.0;
    std::vector<double> levels;
    {
        double z = startZ;
        while (z > targetZ + 1e-5) {
            z = std::max(targetZ, z - maxAp);
            levels.push_back(z);
        }
        if (levels.empty()) levels.push_back(targetZ);
    }

    for (double zLevel : levels) {
        // Anfahren über Nutstart
        w.rapid(startX, startY, clearanceZ);

        // Helix-Eintauchen am Startpunkt (Kreisbogen spiralförmig nach unten)
        if (trochoidR > 0.3) {
            const int helixSteps = 24;
            const double helixDz = (w.pos.z - zLevel) / helixSteps;
            double hz = w.pos.z;
            for (int i = 0; i < helixSteps; ++i) {
                hz -= helixDz;
                const double a = 2.0 * kPi * i / helixSteps;
                const double hx = startX + trochoidR * std::cos(a) * 0.5;
                const double hy = startY + trochoidR * std::sin(a) * 0.5;
                w.feed(hx, hy, hz, plunge);
            }
            w.feed(startX, startY, zLevel, plunge);
        } else {
            // Direkt eintauchen bei sehr kleinem Radius
            w.feed(startX, startY, zLevel, plunge);
        }

        // Trochoidale Bahnen entlang der Nutmittellinie
        double pos = 0.0;  // Position entlang der Nut
        while (pos < slotLen - 1e-3) {
            const double cx = startX + ux * pos;
            const double cy = startY + uy * pos;

            // Schnittbogen: Halbkreis zur einen Seite (Gleichlauf)
            for (int i = 0; i <= arcSteps; ++i) {
                double a;
                if (climbMilling) {
                    a = kPi * 0.5 + kPi * i / arcSteps;  // links herum
                } else {
                    a = kPi * 0.5 - kPi * i / arcSteps;  // rechts herum
                }
                const double px = cx + trochoidR * (nx * std::cos(a) + ux * std::sin(a));
                const double py = cy + trochoidR * (ny * std::cos(a) + uy * std::sin(a));
                w.feed(px, py, zLevel, feed);
            }

            // Rückkehrbogen: Halbkreis zur anderen Seite (Rücklauf)
            for (int i = 0; i <= arcSteps; ++i) {
                double a;
                if (climbMilling) {
                    a = -kPi * 0.5 - kPi * i / arcSteps;
                } else {
                    a = -kPi * 0.5 + kPi * i / arcSteps;
                }
                const double px = cx + trochoidR * (nx * std::cos(a) + ux * std::sin(a));
                const double py = cy + trochoidR * (ny * std::cos(a) + uy * std::sin(a));
                w.feed(px, py, zLevel, feed);
            }

            // Vorschub entlang der Nut um ae
            pos += ae;
            const double nextX = startX + ux * std::min(pos, slotLen);
            const double nextY = startY + uy * std::min(pos, slotLen);
            w.feed(nextX, nextY, zLevel, feed);
        }

        // Schlichtbahn: Einmal die Nutwand entlang (volle Kontur)
        // Rechte Wand
        w.feed(startX + nx * trochoidR, startY + ny * trochoidR, zLevel, feed);
        w.feed(endX + nx * trochoidR, endY + ny * trochoidR, zLevel, feed);
        // Endkappe (Halbkreis)
        for (int i = 1; i <= arcSteps; ++i) {
            const double a = kPi * 0.5 - kPi * i / arcSteps;
            w.feed(endX + trochoidR * (nx * std::cos(a) - ux * std::sin(a)),
                   endY + trochoidR * (ny * std::cos(a) - uy * std::sin(a)),
                   zLevel, feed);
        }
        // Linke Wand zurück
        w.feed(startX - nx * trochoidR, startY - ny * trochoidR, zLevel, feed);
        // Startkappe (Halbkreis)
        for (int i = 1; i <= arcSteps; ++i) {
            const double a = -kPi * 0.5 - kPi * i / arcSteps;
            w.feed(startX + trochoidR * (nx * std::cos(a) + ux * std::sin(a)),
                   startY + trochoidR * (ny * std::cos(a) + uy * std::sin(a)),
                   zLevel, feed);
        }
    }

    // Rückzug
    w.rapid(w.pos.x, w.pos.y, clearanceZ);

    return tp;
}

Toolpath ToolpathGenerator::generateTrochoidalPocket(
    const Geometry::Contour& boundary,
    const Core::ToolDefinition& tool,
    double startZ, double targetZ, double clearanceZ,
    double engagement, double feedFactor,
    bool climbMilling) {

    Toolpath tp(QStringLiteral("Trochoidale Tasche"));
    if (boundary.points.size() < 3) return tp;

    constexpr double kPi = 3.14159265358979323846;
    const double toolDia = std::max(0.5, tool.diameter);
    const double toolRadius = toolDia * 0.5;
    const double ae = std::max(0.05, toolDia * std::clamp(engagement, 0.02, 0.25));
    const double feed = tool.defaultFeedRate * std::clamp(feedFactor, 1.0, 4.0);
    const double plunge = tool.plungeFeedRate > 0.0 ? tool.plungeFeedRate : feed * 0.3;

    // Bounding Box der Tasche
    double bMinX = 1e20, bMinY = 1e20, bMaxX = -1e20, bMaxY = -1e20;
    for (const auto& p : boundary.points) {
        bMinX = std::min(bMinX, p.x);
        bMinY = std::min(bMinY, p.y);
        bMaxX = std::max(bMaxX, p.x);
        bMaxY = std::max(bMaxY, p.y);
    }
    const double cx = (bMinX + bMaxX) * 0.5;
    const double cy = (bMinY + bMaxY) * 0.5;
    const double pocketW = bMaxX - bMinX;
    const double pocketH = bMaxY - bMinY;
    const double maxDist = std::hypot(pocketW, pocketH) * 0.5 + toolRadius;

    // Polygon-Punkt-Test
    auto inBoundary = [&boundary, toolRadius](double x, double y) -> bool {
        const auto& pts = boundary.points;
        if (pts.size() < 3) return false;
        bool inside = false;
        for (size_t i = 0, j = pts.size() - 1; i < pts.size(); j = i++) {
            if (((pts[i].y > y) != (pts[j].y > y)) &&
                (x < (pts[j].x - pts[i].x) * (y - pts[i].y) / (pts[j].y - pts[i].y) + pts[i].x)) {
                inside = !inside;
            }
        }
        return inside;
    };

    // Z-Ebenen
    const double maxAp = tool.fluteLength > 0.5 ? tool.fluteLength * 0.8 : toolDia * 2.0;
    std::vector<double> levels;
    {
        double z = startZ;
        while (z > targetZ + 1e-5) {
            z = std::max(targetZ, z - maxAp);
            levels.push_back(z);
        }
        if (levels.empty()) levels.push_back(targetZ);
    }

    const int arcSteps = 12;
    PathWriter w{tp, tool, Core::Vector3D(cx, cy, clearanceZ)};

    for (double zLevel : levels) {
        // Anfahren über Taschenmitte
        w.rapid(cx, cy, clearanceZ);

        // Helix-Eintauchen in der Mitte
        {
            const double helixR = std::min(ae * 2.0, toolRadius * 0.8);
            const int helixSteps = 24;
            const double dz = (w.pos.z - zLevel) / helixSteps;
            double hz = w.pos.z;
            for (int i = 0; i < helixSteps; ++i) {
                hz -= dz;
                const double a = 2.0 * kPi * i / helixSteps;
                w.feed(cx + helixR * std::cos(a), cy + helixR * std::sin(a), hz, plunge);
            }
            w.feed(cx, cy, zLevel, plunge);
        }

        // Trochoidale Spirale: vom Zentrum nach außen
        double spiralR = ae;
        while (spiralR < maxDist) {
            // Trochoidal-Bogen bei aktuellem Radius
            const int steps = std::max(8, static_cast<int>(2.0 * kPi * spiralR / ae));
            bool anyInside = false;

            for (int i = 0; i < steps; ++i) {
                const double a = 2.0 * kPi * i / steps;
                const double bx = cx + spiralR * std::cos(a);
                const double by = cy + spiralR * std::sin(a);

                // Prüfe ob Punkt innerhalb der Tasche (mit Fräserradius Abstand zur Wand)
                if (!inBoundary(bx, by)) continue;
                anyInside = true;

                // Trochoid-Kreisbogen an dieser Position
                const double trochR = ae * 0.5;
                for (int j = 0; j <= arcSteps; ++j) {
                    const double ta = a + kPi * j / arcSteps;
                    const double tx = bx + trochR * std::cos(ta);
                    const double ty = by + trochR * std::sin(ta);
                    if (inBoundary(tx, ty)) {
                        w.feed(tx, ty, zLevel, feed);
                    }
                }

                // Vorschub zum nächsten Punkt
                const double nextA = 2.0 * kPi * (i + 1) / steps;
                const double nx2 = cx + spiralR * std::cos(nextA);
                const double ny2 = cy + spiralR * std::sin(nextA);
                if (inBoundary(nx2, ny2)) {
                    w.feed(nx2, ny2, zLevel, feed);
                }
            }

            if (!anyInside && spiralR > toolDia) break;  // Tasche vollständig geräumt
            spiralR += ae;
        }

        // Schlicht-Konturfahrt: einmal die Taschenwand entlang
        {
            auto innerContour = boundary.createOffset(
                boundary.isClockwise() ? toolRadius : -toolRadius);
            if (innerContour.points.size() >= 3) {
                const auto& ip = innerContour.points;
                w.feed(ip[0].x, ip[0].y, zLevel, feed);
                for (size_t i = 1; i < ip.size(); ++i) {
                    w.feed(ip[i].x, ip[i].y, zLevel, feed);
                }
                w.feed(ip[0].x, ip[0].y, zLevel, feed);  // schließen
            }
        }
    }

    // Rückzug
    w.rapid(w.pos.x, w.pos.y, clearanceZ);

    return tp;
}

} // namespace GeminiCNC::CAM
