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

    /** Bearbeitungsspur je Gitterpunkt (letzter Fräserkontakt) für die Darstellung von Fräserspuren. */
    struct ToolMark {
        float u{0.0f};       // Koordinate entlang der Vorschubrichtung (mm); beim Eintauchen: Abstand zur Mitte
        float d{0.0f};       // Abstand quer zur Bahn (mm, links positiv)
        float radius{0.0f};  // Werkzeugradius (mm)
        float pitch{0.0f};   // Vorschub je Umdrehung (mm)
        float dirX{1.0f};    // Vorschubrichtung
        float dirY{0.0f};
        uint8_t kind{0};     // 0 = ungefräst, 1 = Schaft-/Planfräser, 2 = Kugelfräser, 3 = Bohren/Eintauchen
    };
    std::vector<ToolMark> marks; // resX * resY

    /** Darstellungsdaten je Oberflächenpunkt (parallel zu Surface::vertices). */
    struct VertexShading {
        float ao{1.0f};          // Umgebungsverdeckung 0..1
        float markU{0.0f};
        float markD{0.0f};
        float markRadius{0.0f};
        float markPitch{0.0f};
        float dirX{1.0f};
        float dirY{0.0f};
        float kind{0.0f};        // 0 = ungefräst
    };

    /** Dreiecksoberfläche des Rohteils (gemeinsam für GPU-Darstellung und STL-Export). */
    struct Surface {
        std::vector<Geometry::Vertex> vertices;
        std::vector<VertexShading> shading; // parallel zu vertices
        std::vector<uint32_t> indices;
    };

    StockModel() = default;
    explicit StockModel(const Core::BoundingBox& stockBounds, int resolution = 200);

    void reset();
    /** Rohteil aus beliebigem geschlossenem Mesh (z. B. gekipptes oder vorgefrästes STL). */
    void initFromMesh(const Geometry::Mesh& mesh);
    void maskCylinder(double radius);  // Carve away everything outside the cylinder radius
    void carveCylinder(const Core::Vector3D& toolCenter, double radius, double cutZ, const QColor& toolColor = QColor(255, 230, 20));
    // markKind: 1 = Schaft-/Planfräser, 2 = Kugelfräser, 3 = Bohren; markPitch = Vorschub je Umdrehung (mm)
    void carveSegment(const Core::Vector3D& p0, const Core::Vector3D& p1, double radius,
                      const QColor& toolColor = QColor(255, 230, 20), int markKind = 1, double markPitch = 0.0);

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
