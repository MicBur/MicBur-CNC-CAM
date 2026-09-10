#ifndef GEMINI_CNC_OBJLOADER_H
#define GEMINI_CNC_OBJLOADER_H

#include "Mesh.h"
#include <QString>

namespace GeminiCNC::Geometry {

/**
 * @brief Parser für 3D Wavefront .obj Dateien mit Unterstützung für Polygontriangulierung und Normalen.
 */
class ObjLoader {
public:
    struct LoadResult {
        bool success{false};
        QString errorMessage;
        Mesh mesh;
    };

    /**
     * @brief Lädt ein 3D-Mesh aus einer Wavefront OBJ-Datei.
     */
    [[nodiscard]] static LoadResult loadFromFile(const QString& filePath, MeshRole role = MeshRole::TargetPart);

    /**
     * @brief Exportiert ein 3D-Mesh als Wavefront OBJ.
     */
    [[nodiscard]] static bool saveToFile(const QString& filePath, const Mesh& mesh);
};

} // namespace GeminiCNC::Geometry

#endif // GEMINI_CNC_OBJLOADER_H
