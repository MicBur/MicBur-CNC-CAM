#include "Mesh.h"
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <cmath>

namespace GeminiCNC::Geometry {

Mesh::Mesh(MeshRole meshRole, const QString& meshName)
    : role(meshRole), name(meshName) {}

void Mesh::clear() {
    vertices.clear();
    triangles.clear();
    boundingBox.reset();
}

void Mesh::computeBoundingBox() {
    boundingBox.reset();
    for (const auto& v : vertices) {
        boundingBox.expand(Core::Vector3D(v.x, v.y, v.z));
    }
}

void Mesh::computeNormals() {
    // Normale zurücksetzen
    for (auto& v : vertices) {
        v.nx = 0.0f;
        v.ny = 0.0f;
        v.nz = 0.0f;
    }

    // Flächennormalen akkumulieren
    for (const auto& tri : triangles) {
        if (tri.i0 >= vertices.size() || tri.i1 >= vertices.size() || tri.i2 >= vertices.size()) {
            continue;
        }

        auto& v0 = vertices[tri.i0];
        auto& v1 = vertices[tri.i1];
        auto& v2 = vertices[tri.i2];

        float e1x = v1.x - v0.x;
        float e1y = v1.y - v0.y;
        float e1z = v1.z - v0.z;

        float e2x = v2.x - v0.x;
        float e2y = v2.y - v0.y;
        float e2z = v2.z - v0.z;

        float nx = e1y * e2z - e1z * e2y;
        float ny = e1z * e2x - e1x * e2z;
        float nz = e1x * e2y - e1y * e2x;

        v0.nx += nx; v0.ny += ny; v0.nz += nz;
        v1.nx += nx; v1.ny += ny; v1.nz += nz;
        v2.nx += nx; v2.ny += ny; v2.nz += nz;
    }

    // Normalisieren
    for (auto& v : vertices) {
        float len = std::sqrt(v.nx * v.nx + v.ny * v.ny + v.nz * v.nz);
        if (len > 1e-6f) {
            float inv = 1.0f / len;
            v.nx *= inv;
            v.ny *= inv;
            v.nz *= inv;
        } else {
            v.nz = 1.0f;
        }
    }
}

void Mesh::translate(const Core::Vector3D& delta) {
    for (auto& v : vertices) {
        v.x += static_cast<float>(delta.x);
        v.y += static_cast<float>(delta.y);
        v.z += static_cast<float>(delta.z);
    }
    computeBoundingBox();
}

void Mesh::rotateX(double angleDeg, bool aroundCenter) {
    if (vertices.empty()) return;
    computeBoundingBox();
    double rad = angleDeg * M_PI / 180.0;
    double cosA = std::cos(rad);
    double sinA = std::sin(rad);
    Core::Vector3D c = aroundCenter ? boundingBox.center() : Core::Vector3D{0.0, 0.0, 0.0};

    for (auto& v : vertices) {
        double y = v.y - c.y;
        double z = v.z - c.z;
        v.y = static_cast<float>(c.y + (y * cosA - z * sinA));
        v.z = static_cast<float>(c.z + (y * sinA + z * cosA));

        double ny = v.ny;
        double nz = v.nz;
        v.ny = static_cast<float>(ny * cosA - nz * sinA);
        v.nz = static_cast<float>(ny * sinA + nz * cosA);
    }
    computeBoundingBox();
}

void Mesh::rotateY(double angleDeg, bool aroundCenter) {
    if (vertices.empty()) return;
    computeBoundingBox();
    double rad = angleDeg * M_PI / 180.0;
    double cosA = std::cos(rad);
    double sinA = std::sin(rad);
    Core::Vector3D c = aroundCenter ? boundingBox.center() : Core::Vector3D{0.0, 0.0, 0.0};

    for (auto& v : vertices) {
        double x = v.x - c.x;
        double z = v.z - c.z;
        v.x = static_cast<float>(c.x + (x * cosA + z * sinA));
        v.z = static_cast<float>(c.z + (-x * sinA + z * cosA));

        double nx = v.nx;
        double nz = v.nz;
        v.nx = static_cast<float>(nx * cosA + nz * sinA);
        v.nz = static_cast<float>(-nx * sinA + nz * cosA);
    }
    computeBoundingBox();
}

void Mesh::rotateZ(double angleDeg, bool aroundCenter) {
    if (vertices.empty()) return;
    computeBoundingBox();
    double rad = angleDeg * M_PI / 180.0;
    double cosA = std::cos(rad);
    double sinA = std::sin(rad);
    Core::Vector3D c = aroundCenter ? boundingBox.center() : Core::Vector3D{0.0, 0.0, 0.0};

    for (auto& v : vertices) {
        double x = v.x - c.x;
        double y = v.y - c.y;
        v.x = static_cast<float>(c.x + (x * cosA - y * sinA));
        v.y = static_cast<float>(c.y + (x * sinA + y * cosA));

        double nx = v.nx;
        double ny = v.ny;
        v.nx = static_cast<float>(nx * cosA - ny * sinA);
        v.ny = static_cast<float>(nx * sinA + ny * cosA);
    }
    computeBoundingBox();
}

void Mesh::rotate(double degX, double degY, double degZ, bool aroundCenter) {
    if (std::abs(degX) > 1e-4) rotateX(degX, aroundCenter);
    if (std::abs(degY) > 1e-4) rotateY(degY, aroundCenter);
    if (std::abs(degZ) > 1e-4) rotateZ(degZ, aroundCenter);
}

void Mesh::alignToOrigin(bool centerXY, bool zeroTopZ) {
    computeBoundingBox();
    if (!boundingBox.isValid()) return;

    Core::Vector3D shift{0.0, 0.0, 0.0};
    if (centerXY) {
        Core::Vector3D center = boundingBox.center();
        shift.x = -center.x;
        shift.y = -center.y;
    } else {
        shift.x = -boundingBox.minPoint.x;
        shift.y = -boundingBox.minPoint.y;
    }

    if (zeroTopZ) {
        // Z=0 an Bauteiloberseite
        shift.z = -boundingBox.maxPoint.z;
    } else {
        // Z=0 an Unterseite
        shift.z = -boundingBox.minPoint.z;
    }

    translate(shift);
}

Mesh Mesh::createBoxStock(double widthX, double depthY, double heightZ, const Core::Vector3D& minOrigin) {
    Mesh mesh(MeshRole::Stock, QStringLiteral("Standard-Rohteil"));

    float x0 = static_cast<float>(minOrigin.x);
    float y0 = static_cast<float>(minOrigin.y);
    float z0 = static_cast<float>(minOrigin.z);

    float x1 = x0 + static_cast<float>(widthX);
    float y1 = y0 + static_cast<float>(depthY);
    float z1 = z0 + static_cast<float>(heightZ);

    // 8 Eckpunkte
    // Unten: 0=(x0,y0,z0), 1=(x1,y0,z0), 2=(x1,y1,z0), 3=(x0,y1,z0)
    // Oben:  4=(x0,y0,z1), 5=(x1,y0,z1), 6=(x1,y1,z1), 7=(x0,y1,z1)
    mesh.vertices = {
        // Unten (z0)
        {x0, y0, z0, 0, 0, -1}, {x1, y0, z0, 0, 0, -1}, {x1, y1, z0, 0, 0, -1}, {x0, y1, z0, 0, 0, -1},
        // Oben (z1)
        {x0, y0, z1, 0, 0, 1},  {x1, y0, z1, 0, 0, 1},  {x1, y1, z1, 0, 0, 1},  {x0, y1, z1, 0, 0, 1},
        // Vorne (y0)
        {x0, y0, z0, 0, -1, 0}, {x1, y0, z0, 0, -1, 0}, {x1, y0, z1, 0, -1, 0}, {x0, y0, z1, 0, -1, 0},
        // Hinten (y1)
        {x1, y1, z0, 0, 1, 0},  {x0, y1, z0, 0, 1, 0},  {x0, y1, z1, 0, 1, 0},  {x1, y1, z1, 0, 1, 0},
        // Links (x0)
        {x0, y1, z0, -1, 0, 0}, {x0, y0, z0, -1, 0, 0}, {x0, y0, z1, -1, 0, 0}, {x0, y1, z1, -1, 0, 0},
        // Rechts (x1)
        {x1, y0, z0, 1, 0, 0},  {x1, y1, z0, 1, 0, 0},  {x1, y1, z1, 1, 0, 0},  {x1, y0, z1, 1, 0, 0}
    };

    mesh.triangles.reserve(12);
    for (uint32_t face = 0; face < 6; ++face) {
        uint32_t base = face * 4;
        mesh.triangles.push_back({base + 0, base + 1, base + 2});
        mesh.triangles.push_back({base + 0, base + 2, base + 3});
    }

    mesh.computeBoundingBox();
    return mesh;
}

void Mesh::scale(double sx, double sy, double sz) {
    if (vertices.empty()) return;
    computeBoundingBox();
    auto center = boundingBox.center();
    for (auto& v : vertices) {
        v.x = static_cast<float>(center.x + (static_cast<double>(v.x) - center.x) * sx);
        v.y = static_cast<float>(center.y + (static_cast<double>(v.y) - center.y) * sy);
        v.z = static_cast<float>(center.z + (static_cast<double>(v.z) - center.z) * sz);
    }
    computeBoundingBox();
    computeNormals();
}

void Mesh::scaleUniform(double factor) {
    scale(factor, factor, factor);
}

Mesh Mesh::createCylinderStock(double radius, double height, int segments, int axis, bool positive) {
    Mesh mesh(MeshRole::Stock, QStringLiteral("Zylinder-Rohteil"));

    if (segments < 8) segments = 8;
    if (radius <= 0.0) radius = 1.0;
    if (height <= 0.0) height = 1.0;

    const int N = segments;
    const auto r = static_cast<float>(radius);
    const auto h = static_cast<float>(height);

    // Generate along Z axis: base at z0, top at z1
    const float z0 = positive ? 0.0f : -h;
    const float z1 = positive ? h : 0.0f;

    mesh.vertices.reserve(static_cast<size_t>(4 * N + 2));
    mesh.triangles.reserve(static_cast<size_t>(4 * N));

    // === Bottom cap (z0, normal pointing down) ===
    const uint32_t bottomCenter = 0;
    mesh.vertices.push_back({0, 0, z0, 0, 0, -1});
    for (int i = 0; i < N; ++i) {
        float angle = 2.0f * static_cast<float>(M_PI) * static_cast<float>(i) / static_cast<float>(N);
        mesh.vertices.push_back({r * std::cos(angle), r * std::sin(angle), z0, 0, 0, -1});
    }

    // === Top cap (z1, normal pointing up) ===
    const auto topCenter = static_cast<uint32_t>(N + 1);
    mesh.vertices.push_back({0, 0, z1, 0, 0, 1});
    for (int i = 0; i < N; ++i) {
        float angle = 2.0f * static_cast<float>(M_PI) * static_cast<float>(i) / static_cast<float>(N);
        mesh.vertices.push_back({r * std::cos(angle), r * std::sin(angle), z1, 0, 0, 1});
    }

    // === Side wall (radial normals) ===
    const auto sideBase = static_cast<uint32_t>(2 * N + 2);
    for (int i = 0; i < N; ++i) {
        float angle = 2.0f * static_cast<float>(M_PI) * static_cast<float>(i) / static_cast<float>(N);
        float cx = r * std::cos(angle);
        float cy = r * std::sin(angle);
        float nx = std::cos(angle);
        float ny = std::sin(angle);
        mesh.vertices.push_back({cx, cy, z0, nx, ny, 0}); // bottom
        mesh.vertices.push_back({cx, cy, z1, nx, ny, 0}); // top
    }

    // === Bottom cap triangles ===
    for (int i = 0; i < N; ++i) {
        auto curr = bottomCenter + 1 + static_cast<uint32_t>(i);
        auto next = bottomCenter + 1 + static_cast<uint32_t>((i + 1) % N);
        mesh.triangles.push_back({bottomCenter, next, curr});
    }

    // === Top cap triangles ===
    for (int i = 0; i < N; ++i) {
        auto curr = topCenter + 1 + static_cast<uint32_t>(i);
        auto next = topCenter + 1 + static_cast<uint32_t>((i + 1) % N);
        mesh.triangles.push_back({topCenter, curr, next});
    }

    // === Side wall triangles ===
    for (int i = 0; i < N; ++i) {
        auto bl = sideBase + static_cast<uint32_t>(i) * 2;
        auto tl = bl + 1;
        auto br = sideBase + static_cast<uint32_t>((i + 1) % N) * 2;
        auto tr = br + 1;
        mesh.triangles.push_back({bl, br, tr});
        mesh.triangles.push_back({bl, tr, tl});
    }

    // Rotate to target axis if not Z
    if (axis == 1) {
        // X axis: rotate 90° around Y so Z→X
        mesh.rotateY(90.0, true);
    } else if (axis == 2) {
        // Y axis: rotate -90° around X so Z→Y
        mesh.rotateX(-90.0, true);
    }

    mesh.computeBoundingBox();
    return mesh;
}

} // namespace GeminiCNC::Geometry
