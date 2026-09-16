#ifndef GEMINI_CNC_STOCKMODEL_H
#define GEMINI_CNC_STOCKMODEL_H

#include "core/Vector3D.h"
#include "core/BoundingBox.h"
#include "geometry/Mesh.h"
#include <vector>
#include <cstdint>
#include <QColor>

namespace GeminiCNC::Simulation {

/**
 * @brief Mehrschichtiges Höhenfeld-Rohteilmodell (Dexel entlang Z) für Echtzeit-Materialabtrag.
 *
 * Je Gitterpunkt werden bis zu kMaxLayers Materialabschnitte [unten, oben] gespeichert.
 * Dadurch bleibt unter Überhängen (gekippte oder vorgefräste Rohteile) Luft, statt dass
 * bis zum Boden aufgefüllt wird. Ein senkrechter Fräser trägt von oben ab.
 */
class StockModel {
public:
    static constexpr int kMaxLayers = 4;

    int resX{200};
    int resY{200};

    Core::BoundingBox bounds;
    std::vector<float> heightField;   // Oberste Materialkante je Gitterpunkt (Boden, wenn kein Material)
    std::vector<uint32_t> cutColors;  // RGBA-Farbe je Gitterpunkt (0 = ungefräst)
    float initialTopZ{0.0f};

    // Materialschichten je Gitterpunkt, von unten nach oben sortiert
    std::vector<uint8_t> layerCount;  // resX * resY
    std::vector<float> layerLo;       // resX * resY * kMaxLayers
    std::vector<float> layerHi;       // resX * resY * kMaxLayers

    // Cylinder shape tracking
    bool isCylinder{false};
    double cylinderRadius{0.0};

    /** Dreiecksoberfläche des Rohteils (gemeinsam für GPU-Darstellung und STL-Export). */
    struct Surface {
        std::vector<Geometry::Vertex> vertices;
        std::vector<uint32_t> indices;
    };

    StockModel() = default;
    explicit StockModel(const Core::BoundingBox& stockBounds, int resolution = 200);

    void reset();
    /** Rohteil aus beliebigem geschlossenem Mesh (z. B. gekipptes oder vorgefrästes STL). */
    void initFromMesh(const Geometry::Mesh& mesh);
    void maskCylinder(double radius);  // Carve away everything outside the cylinder radius
    void carveCylinder(const Core::Vector3D& toolCenter, double radius, double cutZ, const QColor& toolColor = QColor(255, 230, 20));
    void carveSegment(const Core::Vector3D& p0, const Core::Vector3D& p1, double radius, const QColor& toolColor = QColor(255, 230, 20));

    [[nodiscard]] Surface buildSurface() const;
    [[nodiscard]] Geometry::Mesh toMesh() const;
    /** Materialvolumen in mm³ (Trapezregel über das Gitter). */
    [[nodiscard]] double materialVolume() const;

private:
    void updateTop(size_t idx);
    [[nodiscard]] float floorZ() const;

    // Ausgangszustand für reset()
    std::vector<uint8_t> m_initialCount;
    std::vector<float> m_initialLo;
    std::vector<float> m_initialHi;
};

} // namespace GeminiCNC::Simulation

#endif // GEMINI_CNC_STOCKMODEL_H
