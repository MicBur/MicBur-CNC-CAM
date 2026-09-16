#include "StockModel.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace GeminiCNC::Simulation {

namespace {
constexpr float kMinLayerThickness = 1e-3f;
constexpr uint32_t kNoVertex = std::numeric_limits<uint32_t>::max();

// Farbkonfiguration nach Bedienerwunsch:
// Ungefrästes Material = BLAU (CNC-Standard), gefrästes Material = Werkzeugfarbe
constexpr float uncutR = 0.16f, uncutG = 0.42f, uncutB = 0.88f, uncutA = 0.92f;
constexpr float sideR  = 0.12f, sideG  = 0.32f, sideB  = 0.72f, sideA  = 0.96f;
constexpr float botR   = 0.10f, botG   = 0.28f, botB   = 0.65f, botA   = 0.98f;
} // namespace

StockModel::StockModel(const Core::BoundingBox& stockBounds, int resolution)
    : bounds(stockBounds) {
    if (stockBounds.isValid()) {
        double maxDim = std::max(stockBounds.widthX(), stockBounds.depthY());
        if (maxDim > 300.0) {
            resX = 250;
            resY = 250;
        } else if (maxDim > 150.0) {
            resX = 200;
            resY = 200;
        } else {
            resX = 160;
            resY = 160;
        }
    } else {
        resX = resolution;
        resY = resolution;
    }
    initialTopZ = static_cast<float>(stockBounds.isValid() ? stockBounds.maxPoint.z : 0.0);

    // Ausgangszustand: voller Quader (eine Schicht von Unter- bis Oberkante)
    const size_t n = static_cast<size_t>(resX) * resY;
    m_initialCount.assign(n, stockBounds.isValid() ? 1 : 0);
    m_initialLo.assign(n * kMaxLayers, floorZ());
    m_initialHi.assign(n * kMaxLayers, initialTopZ);
    reset();
}

float StockModel::floorZ() const {
    return static_cast<float>(bounds.isValid() ? bounds.minPoint.z : 0.0);
}

void StockModel::updateTop(size_t idx) {
    const uint8_t c = layerCount[idx];
    heightField[idx] = c > 0 ? layerHi[idx * kMaxLayers + c - 1] : floorZ();
}

void StockModel::reset() {
    const size_t n = static_cast<size_t>(resX) * resY;
    if (m_initialCount.size() != n) {
        m_initialCount.assign(n, 0);
        m_initialLo.assign(n * kMaxLayers, 0.0f);
        m_initialHi.assign(n * kMaxLayers, 0.0f);
    }
    layerCount = m_initialCount;
    layerLo = m_initialLo;
    layerHi = m_initialHi;
    cutColors.assign(n, 0);
    marks.assign(n, ToolMark{});
    edgeHints.assign(n, EdgeHint{});
    heightField.assign(n, floorZ());
    for (size_t i = 0; i < n; ++i) {
        updateTop(i);
    }
}

void StockModel::maskCylinder(double radius) {
    isCylinder = true;
    cylinderRadius = radius;
    if (!bounds.isValid() || m_initialCount.empty()) return;

    const double cx = (bounds.minPoint.x + bounds.maxPoint.x) * 0.5;
    const double cy = (bounds.minPoint.y + bounds.maxPoint.y) * 0.5;
    const double dx = bounds.widthX() / (resX - 1);
    const double dy = bounds.depthY() / (resY - 1);
    const double r2 = cylinderRadius * cylinderRadius;

    for (int j = 0; j < resY; ++j) {
        double py = bounds.minPoint.y + j * dy;
        for (int i = 0; i < resX; ++i) {
            double px = bounds.minPoint.x + i * dx;
            double distSq = (px - cx) * (px - cx) + (py - cy) * (py - cy);
            if (distSq > r2) {
                m_initialCount[j * resX + i] = 0;
            }
        }
    }
    reset();
}

void StockModel::initFromMesh(const Geometry::Mesh& mesh) {
    *this = StockModel(mesh.boundingBox);
    if (!bounds.isValid() || mesh.triangles.empty()) return;

    const double dx = bounds.widthX() / (resX - 1);
    const double dy = bounds.depthY() / (resY - 1);
    if (dx <= 1e-6 || dy <= 1e-6) return;

    const size_t n = static_cast<size_t>(resX) * resY;
    const size_t vCount = mesh.vertices.size();

    // Orientierung über das Volumen bestimmen (nach innen gedrehte STL-Dateien abfangen)
    double signedVolume = 0.0;
    for (const auto& tri : mesh.triangles) {
        if (tri.i0 >= vCount || tri.i1 >= vCount || tri.i2 >= vCount) continue;
        const auto& a = mesh.vertices[tri.i0];
        const auto& b = mesh.vertices[tri.i1];
        const auto& c = mesh.vertices[tri.i2];
        signedVolume += static_cast<double>(a.x) * (static_cast<double>(b.y) * c.z - static_cast<double>(b.z) * c.y)
                      - static_cast<double>(a.y) * (static_cast<double>(b.x) * c.z - static_cast<double>(b.z) * c.x)
                      + static_cast<double>(a.z) * (static_cast<double>(b.x) * c.y - static_cast<double>(b.y) * c.x);
    }
    const double orientation = signedVolume < 0.0 ? -1.0 : 1.0;

    // Senkrechte Strahlen je Gitterpunkt: +1 = Eintritt (Unterseite), -1 = Austritt (Oberseite)
    struct Hit { float z; int dir; };
    std::vector<std::vector<Hit>> hits(n);

    for (const auto& tri : mesh.triangles) {
        if (tri.i0 >= vCount || tri.i1 >= vCount || tri.i2 >= vCount) continue;
        const auto& v0 = mesh.vertices[tri.i0];
        const auto& v1 = mesh.vertices[tri.i1];
        const auto& v2 = mesh.vertices[tri.i2];

        const double ax = v1.x - v0.x, ay = v1.y - v0.y;
        const double bx = v2.x - v0.x, by = v2.y - v0.y;
        const double det = ax * by - bx * ay;
        if (std::abs(det) < 1e-9) continue; // senkrechte Wand – wird von Z-Strahlen nicht getroffen
        const int dir = (det * orientation > 0.0) ? -1 : 1; // von oben gegen den Uhrzeigersinn = Oberseite

        const double minX = std::min({v0.x, v1.x, v2.x}), maxX = std::max({v0.x, v1.x, v2.x});
        const double minY = std::min({v0.y, v1.y, v2.y}), maxY = std::max({v0.y, v1.y, v2.y});
        int i0 = std::clamp(static_cast<int>(std::ceil((minX - bounds.minPoint.x) / dx - 1e-6)), 0, resX - 1);
        int i1 = std::clamp(static_cast<int>(std::floor((maxX - bounds.minPoint.x) / dx + 1e-6)), 0, resX - 1);
        int j0 = std::clamp(static_cast<int>(std::ceil((minY - bounds.minPoint.y) / dy - 1e-6)), 0, resY - 1);
        int j1 = std::clamp(static_cast<int>(std::floor((maxY - bounds.minPoint.y) / dy + 1e-6)), 0, resY - 1);

        for (int j = j0; j <= j1; ++j) {
            const double py = bounds.minPoint.y + j * dy - v0.y;
            for (int i = i0; i <= i1; ++i) {
                const double px = bounds.minPoint.x + i * dx - v0.x;
                const double w1 = (px * by - bx * py) / det;
                const double w2 = (ax * py - px * ay) / det;
                const double w0 = 1.0 - w1 - w2;
                const double eps = -1e-6;
                if (w0 < eps || w1 < eps || w2 < eps) continue;
                hits[j * resX + i].push_back({static_cast<float>(w0 * v0.z + w1 * v1.z + w2 * v2.z), dir});
            }
        }
    }

    std::vector<std::pair<float, float>> spans;
    std::vector<std::pair<float, float>> merged;
    std::vector<float> zs;

    for (size_t p = 0; p < n; ++p) {
        auto& h = hits[p];
        m_initialCount[p] = 0;
        if (h.empty()) continue;

        // Bei gleicher Höhe Eintritt vor Austritt → aufeinanderliegende Körper verschmelzen
        std::sort(h.begin(), h.end(), [](const Hit& a, const Hit& b) {
            return a.z < b.z || (a.z == b.z && a.dir > b.dir);
        });

        spans.clear();
        int wind = 0;
        float start = 0.0f;
        float lastZ[3] = {-1e30f, -1e30f, -1e30f}; // je Richtung, gegen Doppeltreffer an gemeinsamen Kanten
        for (const auto& hit : h) {
            float& last = lastZ[hit.dir + 1];
            if (std::abs(hit.z - last) < 1e-4f) continue;
            last = hit.z;
            const int before = wind;
            wind += hit.dir;
            if (before <= 0 && wind > 0) {
                start = hit.z;
            } else if (before > 0 && wind <= 0) {
                spans.emplace_back(start, hit.z);
            }
        }

        if (wind != 0) {
            // Offenes oder uneinheitlich orientiertes Mesh → Paritätsregel
            spans.clear();
            zs.clear();
            for (const auto& hit : h) {
                if (zs.empty() || hit.z - zs.back() > 1e-4f) zs.push_back(hit.z);
            }
            for (size_t k = 0; k + 1 < zs.size(); k += 2) {
                spans.emplace_back(zs[k], zs[k + 1]);
            }
        }

        // Dünne Abschnitte verwerfen, fast berührende zusammenfügen
        merged.clear();
        for (const auto& s : spans) {
            if (s.second - s.first < kMinLayerThickness) continue;
            if (!merged.empty() && s.first - merged.back().second < kMinLayerThickness) {
                merged.back().second = std::max(merged.back().second, s.second);
            } else {
                merged.push_back(s);
            }
        }
        if (merged.empty()) continue;

        // Nur die obersten kMaxLayers behalten; darunterliegende Abschnitte in die unterste Schicht einrechnen
        size_t first = merged.size() > static_cast<size_t>(kMaxLayers) ? merged.size() - kMaxLayers : 0;
        if (first > 0) merged[first].first = merged.front().first;

        uint8_t count = 0;
        for (size_t k = first; k < merged.size(); ++k) {
            m_initialLo[p * kMaxLayers + count] = merged[k].first;
            m_initialHi[p * kMaxLayers + count] = merged[k].second;
            ++count;
        }
        m_initialCount[p] = count;
    }

    reset();
}

void StockModel::carveCylinder(const Core::Vector3D& toolCenter, double radius, double cutZ, const QColor& toolColor) {
    carveSegment(toolCenter, toolCenter, radius, toolColor);
}

void StockModel::carveSegment(const Core::Vector3D& p0, const Core::Vector3D& p1, double radius,
                              const QColor& toolColor, int markKind, double markPitch) {
    const size_t n = static_cast<size_t>(resX) * resY;
    if (!bounds.isValid() || layerCount.size() != n) return;
    if (cutColors.size() != n) {
        cutColors.assign(n, 0);
    }
    if (marks.size() != n) {
        marks.assign(n, ToolMark{});
    }
    if (edgeHints.size() != n) {
        edgeHints.assign(n, EdgeHint{});
    }

    const double dx = bounds.widthX() / (resX - 1);
    const double dy = bounds.depthY() / (resY - 1);
    if (dx <= 1e-6 || dy <= 1e-6) return;

    // Bounding Box des abgefahrenen Zylinder-Segments (Kapsel) plus Randstreifen für die Kantenlage
    const double edgeBand = 1.5 * std::max(dx, dy);
    const double reach = radius + edgeBand;
    double bbMinX = std::min(p0.x, p1.x) - reach;
    double bbMaxX = std::max(p0.x, p1.x) + reach;
    double bbMinY = std::min(p0.y, p1.y) - reach;
    double bbMaxY = std::max(p0.y, p1.y) + reach;

    int minI = std::clamp(static_cast<int>((bbMinX - bounds.minPoint.x) / dx), 0, resX - 1);
    int maxI = std::clamp(static_cast<int>((bbMaxX - bounds.minPoint.x) / dx) + 1, 0, resX - 1);
    int minJ = std::clamp(static_cast<int>((bbMinY - bounds.minPoint.y) / dy), 0, resY - 1);
    int maxJ = std::clamp(static_cast<int>((bbMaxY - bounds.minPoint.y) / dy) + 1, 0, resY - 1);

    const double rSq = radius * radius;
    const double segDx = p1.x - p0.x;
    const double segDy = p1.y - p0.y;
    const double segLenSq = segDx * segDx + segDy * segDy;
    const uint32_t colorRgba = toolColor.isValid() ? toolColor.rgba() : qRgba(255, 230, 20, 255);

    for (int j = minJ; j <= maxJ; ++j) {
        double py = bounds.minPoint.y + j * dy;

        for (int i = minI; i <= maxI; ++i) {
            double px = bounds.minPoint.x + i * dx;

            double t = 0.0;
            if (segLenSq > 1e-8) {
                t = ((px - p0.x) * segDx + (py - p0.y) * segDy) / segLenSq;
                t = std::clamp(t, 0.0, 1.0);
            }

            double nearestX = p0.x + t * segDx;
            double nearestY = p0.y + t * segDy;
            double dX = px - nearestX;
            double dY = py - nearestY;
            const double distSq = dX * dX + dY * dY;
            const float cutZf = static_cast<float>(p0.z + t * (p1.z - p0.z));
            const size_t idx = static_cast<size_t>(j) * resX + i;
            EdgeHint& hint = edgeHints[idx];

            if (distSq > rSq) {
                // Stehengebliebener Punkt neben tieferem Schnitt: Abstand zur Schneidenbahn merken
                const double dist = std::sqrt(distSq);
                const uint8_t cnt = layerCount[idx];
                if (dist <= reach && cnt > 0 && cutZf < layerHi[idx * kMaxLayers + cnt - 1] - 0.05f) {
                    const float clear = static_cast<float>(dist - radius);
                    if (clear < hint.outClear) {
                        hint.outClear = clear;
                        hint.outX = static_cast<float>(-dX / dist) * clear;
                        hint.outY = static_cast<float>(-dY / dist) * clear;
                    }
                }
                continue;
            }

            // Senkrechter Fräser: alles oberhalb der Werkzeugspitze wird entfernt
            uint8_t& count = layerCount[idx];
            bool changed = false;
            while (count > 0) {
                const size_t k = idx * kMaxLayers + count - 1;
                if (layerHi[k] <= cutZf) break;
                if (cutZf <= layerLo[k] + kMinLayerThickness) {
                    --count; // Schicht vollständig abgetragen (Durchbruch)
                    changed = true;
                    continue;
                }
                layerHi[k] = cutZf;
                changed = true;
                break;
            }
            if (changed) {
                updateTop(idx);
                cutColors[idx] = colorRgba;
            }

            // Abgetragener Punkt: Abstand zum Fräserrand (Wandfuß) merken
            {
                const bool touching = count > 0 && layerHi[idx * kMaxLayers + count - 1] >= cutZf - 0.02f;
                const double dist = std::sqrt(distSq);
                const float depth = static_cast<float>(radius - dist);
                if (changed) hint.outClear = 1e9f; // neue Oberkante: frühere Nachbarschnitte gelten nicht mehr
                if (changed || (touching && depth > hint.inDepth)) {
                    hint.inDepth = depth;
                    hint.inX = dist > 1e-9 ? static_cast<float>(dX / dist) * depth : 0.0f;
                    hint.inY = dist > 1e-9 ? static_cast<float>(dY / dist) * depth : 0.0f;
                }
            }

            // Bearbeitungsspur merken: bei Abtrag oder wenn der Fräser die Fläche nur überstreicht (Schlichten)
            const bool touches = count > 0 && layerHi[idx * kMaxLayers + count - 1] >= cutZf - 0.02f;
            if (changed || touches) {
                ToolMark& mark = marks[idx];
                if (segLenSq > 1e-8) {
                    const double segLen = std::sqrt(segLenSq);
                    const double dirX = segDx / segLen;
                    const double dirY = segDy / segLen;
                    mark.dirX = static_cast<float>(dirX);
                    mark.dirY = static_cast<float>(dirY);
                    mark.u = static_cast<float>(px * dirX + py * dirY);
                    mark.d = static_cast<float>(-(px - p0.x) * dirY + (py - p0.y) * dirX);
                    mark.kind = static_cast<uint8_t>(std::clamp(markKind, 1, 3));
                } else {
                    // Senkrechtes Eintauchen: konzentrische Ringe um die Werkzeugmitte
                    mark.dirX = 1.0f;
                    mark.dirY = 0.0f;
                    mark.u = static_cast<float>(std::hypot(px - p0.x, py - p0.y));
                    mark.d = 0.0f;
                    mark.kind = 3;
                }
                mark.radius = static_cast<float>(radius);
                mark.pitch = static_cast<float>(markPitch);
            }
        }
    }
}

StockModel::Surface StockModel::buildSurface() const {
    Surface s;
    const size_t n = static_cast<size_t>(resX) * resY;
    if (!bounds.isValid() || resX < 2 || resY < 2 || layerCount.size() != n) return s;

    const double dx = bounds.widthX() / (resX - 1);
    const double dy = bounds.depthY() / (resY - 1);
    const int L = kMaxLayers;

    auto has = [&](int i, int j, int k) {
        return i >= 0 && j >= 0 && i < resX && j < resY && layerCount[static_cast<size_t>(j) * resX + i] > k;
    };
    auto lo = [&](int i, int j, int k) { return layerLo[(static_cast<size_t>(j) * resX + i) * L + k]; };
    auto hi = [&](int i, int j, int k) { return layerHi[(static_cast<size_t>(j) * resX + i) * L + k]; };
    auto cellDrawn = [&](int i, int j, int k) {
        return i >= 0 && j >= 0 && i < resX - 1 && j < resY - 1
            && has(i, j, k) && has(i + 1, j, k) && has(i, j + 1, k) && has(i + 1, j + 1, k);
    };
    auto px = [&](int i) { return static_cast<float>(bounds.minPoint.x + i * dx); };
    auto py = [&](int j) { return static_cast<float>(bounds.minPoint.y + j * dy); };

    // Höhensprung ab dem zwei Nachbarpunkte als Wand (nicht als Schräge) gelten
    const float wallStep = std::max(0.3f, 1.2f * static_cast<float>(std::max(dx, dy)));

    // Glatte Normale aus den Nachbarhöhen derselben Schicht
    auto surfaceNormal = [&](int i, int j, int k, bool top, float& nx, float& ny, float& nz) {
        auto zAt = [&](int ii, int jj) { return top ? hi(ii, jj, k) : lo(ii, jj, k); };
        const float zc = zAt(i, j);
        auto usable = [&](int ii, int jj) { return has(ii, jj, k) && std::abs(zAt(ii, jj) - zc) <= wallStep; };
        const int il = usable(i - 1, j) ? i - 1 : i, ir = usable(i + 1, j) ? i + 1 : i;
        const int jd = usable(i, j - 1) ? j - 1 : j, ju = usable(i, j + 1) ? j + 1 : j;
        const float dzdx = (ir != il) ? (zAt(ir, j) - zAt(il, j)) / static_cast<float>((ir - il) * dx) : 0.0f;
        const float dzdy = (ju != jd) ? (zAt(i, ju) - zAt(i, jd)) / static_cast<float>((ju - jd) * dy) : 0.0f;
        nx = top ? -dzdx : dzdx;
        ny = top ? -dzdy : dzdy;
        nz = top ? 1.0f : -1.0f;
        const float len = std::sqrt(nx * nx + ny * ny + nz * nz);
        nx /= len; ny /= len; nz /= len;
    };

    // Umgebungsverdeckung (Taschen, Ecken, Wandfüße) aus dem Höhenfeld und Bearbeitungsspur je Punkt
    const float cell = static_cast<float>(std::min(dx, dy));
    auto shadingAt = [&](int i, int j, int k) {
        VertexShading sh;
        const size_t p = static_cast<size_t>(j) * resX + i;
        if (k != layerCount[p] - 1) return sh; // untere Schichten: ungefräst, unverdeckt

        static constexpr int dirs[8][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
        static constexpr int steps[4] = {1, 2, 4, 8};
        const float z = hi(i, j, k);
        float occlusion = 0.0f;
        for (const auto& dir : dirs) {
            const float stepLen = cell * ((dir[0] != 0 && dir[1] != 0) ? 1.4142f : 1.0f);
            float maxSlope = 0.0f;
            for (int st : steps) {
                const int ii = i + dir[0] * st;
                const int jj = j + dir[1] * st;
                if (ii < 0 || jj < 0 || ii >= resX || jj >= resY) break;
                maxSlope = std::max(maxSlope, (heightField[static_cast<size_t>(jj) * resX + ii] - z) / (stepLen * st));
            }
            occlusion += maxSlope / (1.0f + maxSlope);
        }
        sh.ao = std::clamp(1.0f - 0.9f * occlusion / 8.0f, 0.3f, 1.0f);

        if (p < marks.size() && marks[p].kind > 0) {
            const ToolMark& m = marks[p];
            sh.markU = m.u;
            sh.markD = m.d;
            sh.markRadius = m.radius;
            sh.markPitch = m.pitch;
            sh.dirX = m.dirX;
            sh.dirY = m.dirY;
            sh.kind = static_cast<float>(m.kind);
        }
        return sh;
    };

    // Lage eines Randpunkts auf der tatsächlichen Schneidenbahn (statt auf dem Raster)
    const float maxShift = 0.95f * static_cast<float>(std::min(dx, dy));
    const bool haveHints = edgeHints.size() == n;
    auto vertexXY = [&](int i, int j, int k, bool top, float& x, float& y) {
        x = px(i);
        y = py(j);
        if (!haveHints) return;
        const size_t p = static_cast<size_t>(j) * resX + i;
        if (k != layerCount[p] - 1) return;

        float maxDrop = 0.0f;
        float maxRise = 0.0f;
        const float z = top ? hi(i, j, k) : lo(i, j, k);
        for (int dj = -1; dj <= 1; ++dj) {
            for (int di = -1; di <= 1; ++di) {
                if (di == 0 && dj == 0) continue;
                const int ii = i + di, jj = j + dj;
                if (ii < 0 || jj < 0 || ii >= resX || jj >= resY) continue;
                if (!has(ii, jj, k)) {
                    maxDrop = 1e9f; // Durchbruch neben diesem Punkt
                    continue;
                }
                if (!top) continue;
                const float nz = hi(ii, jj, k);
                maxDrop = std::max(maxDrop, z - nz);
                maxRise = std::max(maxRise, nz - z);
            }
        }

        const EdgeHint& h = edgeHints[p];
        float sx = 0.0f, sy = 0.0f;
        if (maxDrop > wallStep && maxDrop >= maxRise && h.outClear < 1e8f) {
            sx = h.outX;
            sy = h.outY;
        } else if (top && maxRise > wallStep && h.inDepth >= 0.0f) {
            sx = h.inX;
            sy = h.inY;
        } else {
            return;
        }
        const float len = std::sqrt(sx * sx + sy * sy);
        if (len > maxShift) {
            sx *= maxShift / len;
            sy *= maxShift / len;
        }
        x += sx;
        y += sy;
    };

    std::vector<uint32_t> topIdx(n * L, kNoVertex);
    std::vector<uint32_t> botIdx(n * L, kNoVertex);

    auto topColor = [&](int i, int j, int k, float& r, float& g, float& b, float& a) {
        const size_t p = static_cast<size_t>(j) * resX + i;
        r = uncutR; g = uncutG; b = uncutB; a = uncutA;
        if (k == layerCount[p] - 1 && p < cutColors.size() && cutColors[p] != 0) {
            // Gefräste Stelle: Werkzeugfarbe (Standard: Signalgelb #FFE614)
            const QColor c = QColor::fromRgba(cutColors[p]);
            r = static_cast<float>(c.redF());
            g = static_cast<float>(c.greenF());
            b = static_cast<float>(c.blueF());
            a = 1.0f;
        }
    };
    auto topVertex = [&](int i, int j, int k) -> uint32_t {
        const size_t p = static_cast<size_t>(j) * resX + i;
        uint32_t& slot = topIdx[p * L + k];
        if (slot == kNoVertex) {
            float nx, ny, nz, r, g, b, a, x, y;
            surfaceNormal(i, j, k, true, nx, ny, nz);
            topColor(i, j, k, r, g, b, a);
            vertexXY(i, j, k, true, x, y);
            slot = static_cast<uint32_t>(s.vertices.size());
            s.vertices.push_back({x, y, hi(i, j, k), nx, ny, nz, r, g, b, a});
            s.shading.push_back(shadingAt(i, j, k));
        }
        return slot;
    };
    auto bottomVertex = [&](int i, int j, int k) -> uint32_t {
        const size_t p = static_cast<size_t>(j) * resX + i;
        uint32_t& slot = botIdx[p * L + k];
        if (slot == kNoVertex) {
            float nx, ny, nz, x, y;
            surfaceNormal(i, j, k, false, nx, ny, nz);
            vertexXY(i, j, k, false, x, y);
            slot = static_cast<uint32_t>(s.vertices.size());
            s.vertices.push_back({x, y, lo(i, j, k), nx, ny, nz, botR, botG, botB, botA});
            s.shading.push_back(VertexShading{});
        }
        return slot;
    };
    auto addTri = [&](uint32_t a, uint32_t b, uint32_t c) {
        s.indices.push_back(a);
        s.indices.push_back(b);
        s.indices.push_back(c);
    };

    // Senkrechte Wand entlang der Kante a → b mit Außennormale (nx, ny)
    auto addWall = [&](int ia, int ja, int ib, int jb, int k, float nx, float ny) {
        const float ta = hi(ia, ja, k), tb = hi(ib, jb, k);
        const float ba = lo(ia, ja, k), bb = lo(ib, jb, k);
        if (ta - ba < kMinLayerThickness && tb - bb < kMinLayerThickness) return;

        float xa, ya, xb, yb, xab, yab, xbb, ybb;
        vertexXY(ia, ja, k, true, xa, ya);
        vertexXY(ib, jb, k, true, xb, yb);
        vertexXY(ia, ja, k, false, xab, yab);
        vertexXY(ib, jb, k, false, xbb, ybb);

        // Normale senkrecht zur (verschobenen) Wand, gleiche Seite wie die Rasternormale
        float ex = xb - xa, ey = yb - ya;
        float wnx = ey, wny = -ex;
        const float wlen = std::sqrt(wnx * wnx + wny * wny);
        if (wlen > 1e-6f) {
            wnx /= wlen;
            wny /= wlen;
            if (wnx * nx + wny * ny < 0.0f) {
                wnx = -wnx;
                wny = -wny;
            }
        } else {
            wnx = nx;
            wny = ny;
        }

        const uint32_t base = static_cast<uint32_t>(s.vertices.size());
        s.vertices.push_back({xab, yab, ba, wnx, wny, 0.0f, sideR, sideG, sideB, sideA});
        s.vertices.push_back({xbb, ybb, bb, wnx, wny, 0.0f, sideR, sideG, sideB, sideA});
        s.vertices.push_back({xb, yb, tb, wnx, wny, 0.0f, sideR, sideG, sideB, sideA});
        s.vertices.push_back({xa, ya, ta, wnx, wny, 0.0f, sideR, sideG, sideB, sideA});

        // Wandfuß stärker verdeckt als die Oberkante; Spuren wie am angrenzenden Punkt
        const VertexShading shA = shadingAt(ia, ja, k);
        const VertexShading shB = shadingAt(ib, jb, k);
        VertexShading shABottom = shA;
        VertexShading shBBottom = shB;
        shABottom.ao *= 0.45f;
        shBBottom.ao *= 0.45f;
        s.shading.push_back(shABottom);
        s.shading.push_back(shBBottom);
        s.shading.push_back(shB);
        s.shading.push_back(shA);

        // Umlaufsinn so wählen, dass die Dreiecksnormale nach außen zeigt
        ex = px(ib) - px(ia);
        ey = py(jb) - py(ja);
        if (ey * nx - ex * ny > 0.0f) {
            addTri(base, base + 1, base + 2);
            addTri(base, base + 2, base + 3);
        } else {
            addTri(base, base + 2, base + 1);
            addTri(base, base + 3, base + 2);
        }
    };

    // Steile Zelle (Fräswand zwischen Oberkante und Boden): eigene Eckpunkte mit Wandnormale,
    // damit Oberseite und Boden scharfkantig bleiben
    auto addSteepCell = [&](int i, int j, int k) {
        const int ci[4] = {i, i + 1, i + 1, i};
        const int cj[4] = {j, j, j + 1, j + 1};
        float X[4], Y[4], Z[4];
        for (int c = 0; c < 4; ++c) {
            vertexXY(ci[c], cj[c], k, true, X[c], Y[c]);
            Z[c] = hi(ci[c], cj[c], k);
        }
        // Flächennormale aus den Diagonalen (0→2, 1→3)
        const float ax = X[2] - X[0], ay = Y[2] - Y[0], az = Z[2] - Z[0];
        const float bx = X[3] - X[1], by = Y[3] - Y[1], bz = Z[3] - Z[1];
        float nx = ay * bz - az * by;
        float ny = az * bx - ax * bz;
        float nz = ax * by - ay * bx;
        float len = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (len < 1e-9f) { nx = 0.0f; ny = 0.0f; nz = 1.0f; len = 1.0f; }
        // Richtung: vom hohen zum tiefen Rand (in die Tasche hinein)
        const float gx = -((Z[1] + Z[2]) - (Z[0] + Z[3])) / static_cast<float>(2.0 * dx);
        const float gy = -((Z[2] + Z[3]) - (Z[0] + Z[1])) / static_cast<float>(2.0 * dy);
        if (nx * gx + ny * gy + nz < 0.0f) len = -len;
        nx /= len; ny /= len; nz /= len;

        const uint32_t base = static_cast<uint32_t>(s.vertices.size());
        for (int c = 0; c < 4; ++c) {
            float r, g, b, a;
            topColor(ci[c], cj[c], k, r, g, b, a);
            s.vertices.push_back({X[c], Y[c], Z[c], nx, ny, nz, r, g, b, a});
            s.shading.push_back(shadingAt(ci[c], cj[c], k));
        }
        // Diagonale mit dem kleineren Höhenunterschied: Wand folgt der Kontur statt Zickzack
        if (std::abs(Z[0] - Z[2]) <= std::abs(Z[1] - Z[3])) {
            addTri(base, base + 1, base + 2);
            addTri(base, base + 2, base + 3);
        } else {
            addTri(base, base + 1, base + 3);
            addTri(base + 1, base + 2, base + 3);
        }
    };

    for (int k = 0; k < L; ++k) {
        for (int j = 0; j < resY - 1; ++j) {
            for (int i = 0; i < resX - 1; ++i) {
                if (!cellDrawn(i, j, k)) continue;

                const float z00 = hi(i, j, k), z10 = hi(i + 1, j, k);
                const float z01 = hi(i, j + 1, k), z11 = hi(i + 1, j + 1, k);
                const float zMin = std::min({z00, z10, z01, z11});
                const float zMax = std::max({z00, z10, z01, z11});
                if (zMax - zMin > wallStep) {
                    addSteepCell(i, j, k);
                } else {
                    const uint32_t t00 = topVertex(i, j, k), t10 = topVertex(i + 1, j, k);
                    const uint32_t t01 = topVertex(i, j + 1, k), t11 = topVertex(i + 1, j + 1, k);
                    // Diagonale entlang der geringeren Höhenänderung (glattere Schrägen)
                    if (std::abs(z00 - z11) <= std::abs(z10 - z01)) {
                        addTri(t00, t10, t11);
                        addTri(t00, t11, t01);
                    } else {
                        addTri(t00, t10, t01);
                        addTri(t10, t11, t01);
                    }
                }

                const uint32_t b00 = bottomVertex(i, j, k), b10 = bottomVertex(i + 1, j, k);
                const uint32_t b01 = bottomVertex(i, j + 1, k), b11 = bottomVertex(i + 1, j + 1, k);
                addTri(b00, b11, b10);
                addTri(b00, b01, b11);

                // Wände dort, wo die Nachbarzelle dieser Schicht fehlt (Außenrand, Durchbruch, Überhang)
                if (!cellDrawn(i, j - 1, k)) addWall(i, j, i + 1, j, k, 0.0f, -1.0f);
                if (!cellDrawn(i, j + 1, k)) addWall(i, j + 1, i + 1, j + 1, k, 0.0f, 1.0f);
                if (!cellDrawn(i - 1, j, k)) addWall(i, j, i, j + 1, k, -1.0f, 0.0f);
                if (!cellDrawn(i + 1, j, k)) addWall(i + 1, j, i + 1, j + 1, k, 1.0f, 0.0f);
            }
        }
    }

    return s;
}

Geometry::Mesh StockModel::toMesh() const {
    Geometry::Mesh mesh(Geometry::MeshRole::Stock, QStringLiteral("Dynamisches Rohteil"));
    Surface s = buildSurface();
    if (s.indices.empty()) return mesh;

    mesh.vertices = std::move(s.vertices);
    mesh.triangles.reserve(s.indices.size() / 3);
    for (size_t t = 0; t + 2 < s.indices.size(); t += 3) {
        mesh.triangles.push_back({s.indices[t], s.indices[t + 1], s.indices[t + 2]});
    }
    mesh.computeBoundingBox();
    return mesh;
}

double StockModel::materialVolume() const {
    const size_t n = static_cast<size_t>(resX) * resY;
    if (!bounds.isValid() || resX < 2 || resY < 2 || layerCount.size() != n) return 0.0;

    const double dx = bounds.widthX() / (resX - 1);
    const double dy = bounds.depthY() / (resY - 1);
    double sum = 0.0;
    for (int j = 0; j < resY; ++j) {
        const double wy = (j == 0 || j == resY - 1) ? 0.5 : 1.0;
        for (int i = 0; i < resX; ++i) {
            const double wx = (i == 0 || i == resX - 1) ? 0.5 : 1.0;
            const size_t p = static_cast<size_t>(j) * resX + i;
            double thickness = 0.0;
            for (int k = 0; k < layerCount[p]; ++k) {
                thickness += layerHi[p * kMaxLayers + k] - layerLo[p * kMaxLayers + k];
            }
            sum += wx * wy * thickness;
        }
    }
    return sum * dx * dy;
}

} // namespace GeminiCNC::Simulation
