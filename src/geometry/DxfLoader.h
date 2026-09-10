#ifndef GEMINI_CNC_DXFLOADER_H
#define GEMINI_CNC_DXFLOADER_H

#include "Contour.h"
#include "Mesh.h"
#include <QString>
#include <vector>

namespace GeminiCNC::Geometry {

/**
 * @brief DXF-Parser für 2D-Zeichnungen (Linien, Kreisbögen, Leichtgewicht-Polylinien).
 */
class DxfLoader {
public:
    struct LoadResult {
        bool success{false};
        QString errorMessage;
        std::vector<Contour> contours;
    };

    /**
     * @brief Parst eine DXF-Datei und extrahiert 2D-Konturen.
     */
    [[nodiscard]] static LoadResult loadFromFile(const QString& filePath);

    /**
     * @brief Wandelt 2D-Konturen durch Extrusion in ein 3D-Mesh (z.B. für Rohteil oder Bauteil) um.
     */
    [[nodiscard]] static Mesh extrudeContours(const std::vector<Contour>& contours,
                                              double depthZ,
                                              MeshRole role = MeshRole::TargetPart);
};

} // namespace GeminiCNC::Geometry

#endif // GEMINI_CNC_DXFLOADER_H
