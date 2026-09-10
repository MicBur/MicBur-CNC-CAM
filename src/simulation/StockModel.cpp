#include "StockModel.h"
#include <algorithm>
#include <cmath>

namespace GeminiCNC::Simulation {

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
    reset();
}

void StockModel::reset() {
    heightField.assign(resX * resY, initialTopZ);
    cutColors.assign(resX * resY, 0);
}

void StockModel::carveCylinder(const Core::Vector3D& toolCenter, double radius, double cutZ, const QColor& toolColor) {
    carveSegment(toolCenter, toolCenter, radius, toolColor);
}

void StockModel::carveSegment(const Core::Vector3D& p0, const Core::Vector3D& p1, double radius, const QColor& toolColor) {
    if (!bounds.isValid() || heightField.empty()) return;
    if (cutColors.size() != heightField.size()) {
        cutColors.assign(resX * resY, 0);
    }

    const double dx = bounds.widthX() / (resX - 1);
    const double dy = bounds.depthY() / (resY - 1);
    if (dx <= 1e-6 || dy <= 1e-6) return;

    // Bounding Box des abgefahrenen Zylinder-Segments (Kapsel)
    double bbMinX = std::min(p0.x, p1.x) - radius;
    double bbMaxX = std::max(p0.x, p1.x) + radius;
    double bbMinY = std::min(p0.y, p1.y) - radius;
    double bbMaxY = std::max(p0.y, p1.y) + radius;

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

            if (dX * dX + dY * dY <= rSq) {
                float cutZf = static_cast<float>(p0.z + t * (p1.z - p0.z));
                int idx = j * resX + i;
                if (heightField[idx] > cutZf) {
                    heightField[idx] = cutZf;
                    cutColors[idx] = colorRgba;
                }
            }
        }
    }
}

Geometry::Mesh StockModel::toMesh() const {
    Geometry::Mesh mesh(Geometry::MeshRole::Stock, QStringLiteral("Dynamisches Rohteil"));
    if (!bounds.isValid() || heightField.size() != static_cast<size_t>(resX * resY)) {
        return mesh;
    }

    const double dx = bounds.widthX() / (resX - 1);
    const double dy = bounds.depthY() / (resY - 1);
    const float bottomZ = static_cast<float>(bounds.minPoint.z);

    // Farbkonfiguration nach Bedienerwunsch:
    // Ungefrästes Material = BLAU (CNC-Standard)
    // Gefrästes Material = GELB (oder spezifische Werkzeugfarbe)
    constexpr float uncutR = 0.16f, uncutG = 0.42f, uncutB = 0.88f, uncutA = 0.92f;
    constexpr float sideR  = 0.12f, sideG  = 0.32f, sideB  = 0.72f, sideA  = 0.96f;
    constexpr float botR   = 0.10f, botG   = 0.28f, botB   = 0.65f, botA   = 0.98f;

    // 1. Oberflächen-Gitter (bearbeitete Z-Höhen mit Farbunterscheidung)
    mesh.vertices.reserve(resX * resY + 2 * (resX + resY) + 4);
    for (int j = 0; j < resY; ++j) {
        float y = static_cast<float>(bounds.minPoint.y + j * dy);
        for (int i = 0; i < resX; ++i) {
            int idx = j * resX + i;
            float x = static_cast<float>(bounds.minPoint.x + i * dx);
            float z = heightField[idx];

            float vr = uncutR, vg = uncutG, vb = uncutB, va = uncutA;
            if (idx < static_cast<int>(cutColors.size()) && cutColors[idx] != 0) {
                // Gefräste Stelle: Werkzeugfarbe (Standard: Signalgelb #FFE614)
                QColor c = QColor::fromRgba(cutColors[idx]);
                vr = static_cast<float>(c.redF());
                vg = static_cast<float>(c.greenF());
                vb = static_cast<float>(c.blueF());
                va = 1.0f;
            }
            mesh.vertices.push_back({x, y, z, 0.0f, 0.0f, 1.0f, vr, vg, vb, va});
        }
    }

    mesh.triangles.reserve((resX - 1) * (resY - 1) * 2 + (resX + resY) * 4 + 2);
    for (int j = 0; j < resY - 1; ++j) {
        for (int i = 0; i < resX - 1; ++i) {
            uint32_t i00 = j * resX + i;
            uint32_t i10 = i00 + 1;
            uint32_t i01 = (j + 1) * resX + i;
            uint32_t i11 = i01 + 1;

            mesh.triangles.push_back({i00, i10, i11});
            mesh.triangles.push_back({i00, i11, i01});
        }
    }

    // 2. Solide blaue Seitenwände nach unten ziehen
    // Vorne (j = 0)
    for (int i = 0; i < resX - 1; ++i) {
        float x1 = static_cast<float>(bounds.minPoint.x + i * dx);
        float x2 = static_cast<float>(bounds.minPoint.x + (i + 1) * dx);
        float y = static_cast<float>(bounds.minPoint.y);

        uint32_t top1 = i;
        uint32_t top2 = i + 1;
        uint32_t bot1 = static_cast<uint32_t>(mesh.vertices.size());
        mesh.vertices.push_back({x1, y, bottomZ, 0.0f, -1.0f, 0.0f, sideR, sideG, sideB, sideA});
        uint32_t bot2 = static_cast<uint32_t>(mesh.vertices.size());
        mesh.vertices.push_back({x2, y, bottomZ, 0.0f, -1.0f, 0.0f, sideR, sideG, sideB, sideA});

        mesh.triangles.push_back({top1, bot1, bot2});
        mesh.triangles.push_back({top1, bot2, top2});
    }

    // Hinten (j = resY - 1)
    int lastRow = (resY - 1) * resX;
    for (int i = 0; i < resX - 1; ++i) {
        float x1 = static_cast<float>(bounds.minPoint.x + i * dx);
        float x2 = static_cast<float>(bounds.minPoint.x + (i + 1) * dx);
        float y = static_cast<float>(bounds.maxPoint.y);

        uint32_t top1 = lastRow + i;
        uint32_t top2 = lastRow + i + 1;
        uint32_t bot1 = static_cast<uint32_t>(mesh.vertices.size());
        mesh.vertices.push_back({x1, y, bottomZ, 0.0f, 1.0f, 0.0f, sideR, sideG, sideB, sideA});
        uint32_t bot2 = static_cast<uint32_t>(mesh.vertices.size());
        mesh.vertices.push_back({x2, y, bottomZ, 0.0f, 1.0f, 0.0f, sideR, sideG, sideB, sideA});

        mesh.triangles.push_back({top1, top2, bot2});
        mesh.triangles.push_back({top1, bot2, bot1});
    }

    // Links (i = 0)
    for (int j = 0; j < resY - 1; ++j) {
        float y1 = static_cast<float>(bounds.minPoint.y + j * dy);
        float y2 = static_cast<float>(bounds.minPoint.y + (j + 1) * dy);
        float x = static_cast<float>(bounds.minPoint.x);

        uint32_t top1 = j * resX;
        uint32_t top2 = (j + 1) * resX;
        uint32_t bot1 = static_cast<uint32_t>(mesh.vertices.size());
        mesh.vertices.push_back({x, y1, bottomZ, -1.0f, 0.0f, 0.0f, sideR, sideG, sideB, sideA});
        uint32_t bot2 = static_cast<uint32_t>(mesh.vertices.size());
        mesh.vertices.push_back({x, y2, bottomZ, -1.0f, 0.0f, 0.0f, sideR, sideG, sideB, sideA});

        mesh.triangles.push_back({top1, bot2, top2});
        mesh.triangles.push_back({top1, bot1, bot2});
    }

    // Rechts (i = resX - 1)
    for (int j = 0; j < resY - 1; ++j) {
        float y1 = static_cast<float>(bounds.minPoint.y + j * dy);
        float y2 = static_cast<float>(bounds.minPoint.y + (j + 1) * dy);
        float x = static_cast<float>(bounds.maxPoint.x);

        uint32_t top1 = j * resX + (resX - 1);
        uint32_t top2 = (j + 1) * resX + (resX - 1);
        uint32_t bot1 = static_cast<uint32_t>(mesh.vertices.size());
        mesh.vertices.push_back({x, y1, bottomZ, 1.0f, 0.0f, 0.0f, sideR, sideG, sideB, sideA});
        uint32_t bot2 = static_cast<uint32_t>(mesh.vertices.size());
        mesh.vertices.push_back({x, y2, bottomZ, 1.0f, 0.0f, 0.0f, sideR, sideG, sideB, sideA});

        mesh.triangles.push_back({top1, top2, bot2});
        mesh.triangles.push_back({top1, bot2, bot1});
    }

    // 3. Blaue Bodenfläche (2 Dreiecke)
    float minX = static_cast<float>(bounds.minPoint.x);
    float maxX = static_cast<float>(bounds.maxPoint.x);
    float minY = static_cast<float>(bounds.minPoint.y);
    float maxY = static_cast<float>(bounds.maxPoint.y);

    uint32_t b00 = static_cast<uint32_t>(mesh.vertices.size());
    mesh.vertices.push_back({minX, minY, bottomZ, 0.0f, 0.0f, -1.0f, botR, botG, botB, botA});
    uint32_t b10 = static_cast<uint32_t>(mesh.vertices.size());
    mesh.vertices.push_back({maxX, minY, bottomZ, 0.0f, 0.0f, -1.0f, botR, botG, botB, botA});
    uint32_t b11 = static_cast<uint32_t>(mesh.vertices.size());
    mesh.vertices.push_back({maxX, maxY, bottomZ, 0.0f, 0.0f, -1.0f, botR, botG, botB, botA});
    uint32_t b01 = static_cast<uint32_t>(mesh.vertices.size());
    mesh.vertices.push_back({minX, maxY, bottomZ, 0.0f, 0.0f, -1.0f, botR, botG, botB, botA});

    mesh.triangles.push_back({b00, b10, b11});
    mesh.triangles.push_back({b00, b11, b01});

    mesh.computeBoundingBox();
    mesh.computeNormals();
    return mesh;
}

} // namespace GeminiCNC::Simulation
