#include "StlLoader.h"
#include <QFile>
#include <QFileInfo>
#include <QTextStream>
#include <cstring>

namespace GeminiCNC::Geometry {

StlLoader::LoadResult StlLoader::loadFromFile(const QString& filePath, MeshRole role) {
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly)) {
        return {false, QString("Konnte Datei nicht öffnen: %1").arg(file.errorString()), {}};
    }

    QByteArray data = file.readAll();
    file.close();

    if (data.size() < 84) {
        return {false, QStringLiteral("Datei ist zu klein für ein gültiges STL-Format."), {}};
    }

    // Heuristik zur Erkennung Binär vs. ASCII:
    // Im Binär-STL steht ab Byte 80 die Anzahl der Dreiecke (uint32).
    // Die erwartete Dateigröße ist exakt 80 + 4 + (N * 50).
    uint32_t numTriangles = 0;
    std::memcpy(&numTriangles, data.constData() + 80, sizeof(uint32_t));
    const quint64 expectedBinarySize = 84ULL + static_cast<quint64>(numTriangles) * 50ULL;

    if (static_cast<quint64>(data.size()) == expectedBinarySize) {
        return loadBinary(filePath, role, data);
    }

    // Prüfen, ob die Datei mit 'solid' beginnt und Text enthält
    if (data.startsWith("solid")) {
        auto asciiRes = loadAscii(filePath, role, data);
        if (asciiRes.success) {
            return asciiRes;
        }
    }

    // Fallback: Als Binärdatei versuchen
    return loadBinary(filePath, role, data);
}

StlLoader::LoadResult StlLoader::loadBinary(const QString& filePath, MeshRole role, const QByteArray& data) {
    if (data.size() < 84) {
        return {false, QStringLiteral("Ungültige Binär-STL-Größe."), {}};
    }

    uint32_t numTriangles = 0;
    std::memcpy(&numTriangles, data.constData() + 80, sizeof(uint32_t));

    LoadResult result;
    result.mesh.role = role;
    result.mesh.name = QFileInfo(filePath).baseName();

    result.mesh.vertices.reserve(numTriangles * 3);
    result.mesh.triangles.reserve(numTriangles);

    const char* ptr = data.constData() + 84;
    const char* endPtr = data.constData() + data.size();

    uint32_t vertexIdx = 0;
    for (uint32_t i = 0; i < numTriangles; ++i) {
        if (ptr + 50 > endPtr) break;

        float nx, ny, nz;
        float x1, y1, z1;
        float x2, y2, z2;
        float x3, y3, z3;

        std::memcpy(&nx, ptr, 4); ptr += 4;
        std::memcpy(&ny, ptr, 4); ptr += 4;
        std::memcpy(&nz, ptr, 4); ptr += 4;

        std::memcpy(&x1, ptr, 4); ptr += 4;
        std::memcpy(&y1, ptr, 4); ptr += 4;
        std::memcpy(&z1, ptr, 4); ptr += 4;

        std::memcpy(&x2, ptr, 4); ptr += 4;
        std::memcpy(&y2, ptr, 4); ptr += 4;
        std::memcpy(&z2, ptr, 4); ptr += 4;

        std::memcpy(&x3, ptr, 4); ptr += 4;
        std::memcpy(&y3, ptr, 4); ptr += 4;
        std::memcpy(&z3, ptr, 4); ptr += 4;

        ptr += 2; // 2 Attribute Bytes überspringen

        result.mesh.vertices.push_back({x1, y1, z1, nx, ny, nz});
        result.mesh.vertices.push_back({x2, y2, z2, nx, ny, nz});
        result.mesh.vertices.push_back({x3, y3, z3, nx, ny, nz});

        result.mesh.triangles.push_back({vertexIdx, vertexIdx + 1, vertexIdx + 2});
        vertexIdx += 3;
    }

    result.mesh.computeBoundingBox();
    result.mesh.computeNormals();
    result.success = true;
    return result;
}

StlLoader::LoadResult StlLoader::loadAscii(const QString& filePath, MeshRole role, const QByteArray& data) {
    LoadResult result;
    result.mesh.role = role;
    result.mesh.name = QFileInfo(filePath).baseName();

    QTextStream stream(data);
    float nx = 0.0f, ny = 0.0f, nz = 1.0f;
    std::vector<Vertex> facetVertices;
    uint32_t vertexIdx = 0;

    while (!stream.atEnd()) {
        QString line = stream.readLine().trimmed();
        if (line.startsWith(QStringLiteral("facet normal"), Qt::CaseInsensitive)) {
            QStringList parts = line.split(QLatin1Char(' '), Qt::SkipEmptyParts);
            if (parts.size() >= 5) {
                nx = parts[2].toFloat();
                ny = parts[3].toFloat();
                nz = parts[4].toFloat();
            }
            facetVertices.clear();
        } else if (line.startsWith(QStringLiteral("vertex"), Qt::CaseInsensitive)) {
            QStringList parts = line.split(QLatin1Char(' '), Qt::SkipEmptyParts);
            if (parts.size() >= 4) {
                facetVertices.push_back({parts[1].toFloat(), parts[2].toFloat(), parts[3].toFloat(), nx, ny, nz});
            }
        } else if (line.startsWith(QStringLiteral("endfacet"), Qt::CaseInsensitive)) {
            if (facetVertices.size() == 3) {
                result.mesh.vertices.push_back(facetVertices[0]);
                result.mesh.vertices.push_back(facetVertices[1]);
                result.mesh.vertices.push_back(facetVertices[2]);
                result.mesh.triangles.push_back({vertexIdx, vertexIdx + 1, vertexIdx + 2});
                vertexIdx += 3;
            }
        }
    }

    if (result.mesh.vertices.empty()) {
        return {false, QStringLiteral("Keine Dreiecke im ASCII-STL gefunden."), {}};
    }

    result.mesh.computeBoundingBox();
    result.mesh.computeNormals();
    result.success = true;
    return result;
}

bool StlLoader::saveBinary(const QString& filePath, const Mesh& mesh) {
    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly)) {
        return false;
    }

    // 80 Bytes Header
    char header[80] = {0};
    const char* title = "Gemini CNC Binary STL Export";
    std::strncpy(header, title, 79);
    file.write(header, 80);

    // 4 Bytes Dreiecksanzahl
    const uint32_t numTriangles = static_cast<uint32_t>(mesh.triangles.size());
    file.write(reinterpret_cast<const char*>(&numTriangles), sizeof(uint32_t));

    // Dreiecke schreiben
    const uint16_t attributeByteCount = 0;
    for (const auto& tri : mesh.triangles) {
        if (tri.i0 >= mesh.vertices.size() || tri.i1 >= mesh.vertices.size() || tri.i2 >= mesh.vertices.size()) {
            continue;
        }
        const auto& v0 = mesh.vertices[tri.i0];
        const auto& v1 = mesh.vertices[tri.i1];
        const auto& v2 = mesh.vertices[tri.i2];

        // Normale (v0.nx, v0.ny, v0.nz)
        file.write(reinterpret_cast<const char*>(&v0.nx), sizeof(float));
        file.write(reinterpret_cast<const char*>(&v0.ny), sizeof(float));
        file.write(reinterpret_cast<const char*>(&v0.nz), sizeof(float));

        // Vertex 1
        file.write(reinterpret_cast<const char*>(&v0.x), sizeof(float));
        file.write(reinterpret_cast<const char*>(&v0.y), sizeof(float));
        file.write(reinterpret_cast<const char*>(&v0.z), sizeof(float));

        // Vertex 2
        file.write(reinterpret_cast<const char*>(&v1.x), sizeof(float));
        file.write(reinterpret_cast<const char*>(&v1.y), sizeof(float));
        file.write(reinterpret_cast<const char*>(&v1.z), sizeof(float));

        // Vertex 3
        file.write(reinterpret_cast<const char*>(&v2.x), sizeof(float));
        file.write(reinterpret_cast<const char*>(&v2.y), sizeof(float));
        file.write(reinterpret_cast<const char*>(&v2.z), sizeof(float));

        file.write(reinterpret_cast<const char*>(&attributeByteCount), sizeof(uint16_t));
    }

    file.close();
    return true;
}

} // namespace GeminiCNC::Geometry
