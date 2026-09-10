#ifndef GEMINI_CNC_STOCKMODEL_H
#define GEMINI_CNC_STOCKMODEL_H

#include "core/Vector3D.h"
#include "core/BoundingBox.h"
#include "geometry/Mesh.h"
#include <vector>
#include <QColor>

namespace GeminiCNC::Simulation {

/**
 * @brief Extrem speichereffizientes Höhenfeld-Rohteilmodell für Echtzeit-Materialabtrag.
 * Ideal für integrierte Grafikkarten (Intel HD/UHD) und mobile i5 CPUs ohne Framedrops.
 */
class StockModel {
public:
    int resX{200};
    int resY{200};

    Core::BoundingBox bounds;
    std::vector<float> heightField;   // Z-Höhen je Gitterpunkt (resX * resY)
    std::vector<uint32_t> cutColors; // RGBA-Farbe je Gitterpunkt (0 = ungefräst)
    float initialTopZ{0.0f};

    StockModel() = default;
    explicit StockModel(const Core::BoundingBox& stockBounds, int resolution = 200);

    void reset();
    void carveCylinder(const Core::Vector3D& toolCenter, double radius, double cutZ, const QColor& toolColor = QColor(255, 230, 20));
    void carveSegment(const Core::Vector3D& p0, const Core::Vector3D& p1, double radius, const QColor& toolColor = QColor(255, 230, 20));

    [[nodiscard]] Geometry::Mesh toMesh() const;
};

} // namespace GeminiCNC::Simulation

#endif // GEMINI_CNC_STOCKMODEL_H
