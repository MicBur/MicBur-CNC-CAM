#include "ObjLoader.h"
#include <QFile>
#include <QFileInfo>
#include <QTextStream>
#include <vector>

namespace GeminiCNC::Geometry {

ObjLoader::LoadResult ObjLoader::loadFromFile(const QString& filePath, MeshRole role) {
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        return {false, QString("Konnte OBJ-Datei nicht öffnen: %1").arg(file.errorString()), {}};
    }

    QTextStream in(&file);
    LoadResult result;
    result.mesh.role = role;
    result.mesh.name = QFileInfo(filePath).baseName();

    struct TempVertex { float x, y, z; };
    struct TempNormal { float nx, ny, nz; };

    std::vector<TempVertex> tempPositions;
    std::vector<TempNormal> tempNormals;

    bool hasAnyFace = false;

    while (!in.atEnd()) {
        QString line = in.readLine().trimmed();
        if (line.isEmpty() || line.startsWith('#')) continue;

        QStringList tokens = line.split(QLatin1Char(' '), Qt::SkipEmptyParts);
        if (tokens.isEmpty()) continue;

        const QString& prefix = tokens[0];

        if (prefix == QStringLiteral("v") && tokens.size() >= 4) {
            tempPositions.push_back({tokens[1].toFloat(), tokens[2].toFloat(), tokens[3].toFloat()});
        } else if (prefix == QStringLiteral("vn") && tokens.size() >= 4) {
            tempNormals.push_back({tokens[1].toFloat(), tokens[2].toFloat(), tokens[3].toFloat()});
        } else if (prefix == QStringLiteral("f") && tokens.size() >= 4) {
            hasAnyFace = true;
            // Face-Elemente parsen: 'v', 'v/vt', 'v/vt/vn', 'v//vn'
            std::vector<uint32_t> faceVertexIndices;

            for (int i = 1; i < tokens.size(); ++i) {
                QStringList parts = tokens[i].split('/');
                int vIdx = parts[0].toInt();
                // 1-based oder negativ
                if (vIdx < 0) {
                    vIdx = static_cast<int>(tempPositions.size()) + vIdx + 1;
                }
                if (vIdx > 0 && vIdx <= static_cast<int>(tempPositions.size())) {
                    float nx = 0.0f, ny = 0.0f, nz = 1.0f;
                    if (parts.size() >= 3 && !parts[2].isEmpty()) {
                        int vnIdx = parts[2].toInt();
                        if (vnIdx < 0) vnIdx = static_cast<int>(tempNormals.size()) + vnIdx + 1;
                        if (vnIdx > 0 && vnIdx <= static_cast<int>(tempNormals.size())) {
                            nx = tempNormals[vnIdx - 1].nx;
                            ny = tempNormals[vnIdx - 1].ny;
                            nz = tempNormals[vnIdx - 1].nz;
                        }
                    }

                    uint32_t currentMeshIndex = static_cast<uint32_t>(result.mesh.vertices.size());
                    const auto& pos = tempPositions[vIdx - 1];
                    result.mesh.vertices.push_back({pos.x, pos.y, pos.z, nx, ny, nz});
                    faceVertexIndices.push_back(currentMeshIndex);
                }
            }

            // Fächer-Triangulierung (Fan Triangulation) für Polygone/Quads
            if (faceVertexIndices.size() >= 3) {
                for (size_t i = 1; i + 1 < faceVertexIndices.size(); ++i) {
                    result.mesh.triangles.push_back({
                        faceVertexIndices[0],
                        faceVertexIndices[i],
                        faceVertexIndices[i + 1]
                    });
                }
            }
        }
    }

    file.close();

    if (!hasAnyFace || result.mesh.vertices.empty()) {
        return {false, QStringLiteral("Keine gültigen 3D-Flächen (Faces) in der OBJ-Datei gefunden."), {}};
    }

    result.mesh.computeBoundingBox();
    if (tempNormals.empty()) {
        result.mesh.computeNormals();
    }

    result.success = true;
    return result;
}

bool ObjLoader::saveToFile(const QString& filePath, const Mesh& mesh) {
    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) return false;

    QTextStream out(&file);
    out << "# Wavefront OBJ exportiert von Gemini CNC Control\n";
    out << "o " << (mesh.name.isEmpty() ? QStringLiteral("Mesh") : mesh.name) << "\n\n";

    for (const auto& v : mesh.vertices) {
        out << "v " << v.x << " " << v.y << " " << v.z << "\n";
    }
    out << "\n";

    for (const auto& v : mesh.vertices) {
        out << "vn " << v.nx << " " << v.ny << " " << v.nz << "\n";
    }
    out << "\n";

    for (const auto& tri : mesh.triangles) {
        out << "f " << (tri.i0 + 1) << "//" << (tri.i0 + 1) << " "
                    << (tri.i1 + 1) << "//" << (tri.i1 + 1) << " "
                    << (tri.i2 + 1) << "//" << (tri.i2 + 1) << "\n";
    }

    file.close();
    return true;
}

} // namespace GeminiCNC::Geometry
