#ifndef GEMINI_CNC_STLLOADER_H
#define GEMINI_CNC_STLLOADER_H

#include "Mesh.h"
#include <QString>

namespace GeminiCNC::Geometry {

/**
 * @brief High-Performance STL-Loader für binäre und ASCII-STL-Dateien.
 */
class StlLoader {
public:
    struct LoadResult {
        bool success{false};
        QString errorMessage;
        Mesh mesh;
    };

    /**
     * @brief Lädt eine STL-Datei und erzeugt ein Mesh mit der angegebenen Rolle.
     */
    [[nodiscard]] static LoadResult loadFromFile(const QString& filePath, MeshRole role = MeshRole::TargetPart);

    /**
     * @brief Speichert ein Mesh als kompaktes Binär-STL.
     */
    [[nodiscard]] static bool saveBinary(const QString& filePath, const Mesh& mesh);

private:
    [[nodiscard]] static LoadResult loadBinary(const QString& filePath, MeshRole role, const QByteArray& data);
    [[nodiscard]] static LoadResult loadAscii(const QString& filePath, MeshRole role, const QByteArray& data);
};

} // namespace GeminiCNC::Geometry

#endif // GEMINI_CNC_STLLOADER_H
