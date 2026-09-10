#ifndef GEMINI_CNC_MESH_H
#define GEMINI_CNC_MESH_H

#include "core/Vector3D.h"
#include "core/BoundingBox.h"
#include <vector>
#include <QString>

namespace GeminiCNC::Geometry {

enum class MeshRole {
    Stock,      // Rohteil
    TargetPart, // Fertigteil / Ziel-Bauteil
    Fixture     // Spannmittel / Schraubstock
};

struct Vertex {
    float x{0.0f}, y{0.0f}, z{0.0f};
    float nx{0.0f}, ny{0.0f}, nz{1.0f};
    float r{1.0f}, g{1.0f}, b{1.0f}, a{1.0f}; // RGBA Vertex-Farbe
};

struct Triangle {
    uint32_t i0{0}, i1{0}, i2{0};
};

/**
 * @brief Dreiecksnetz für Rohteile und Ziel-Bauteile mit GPU-freundlicher Pufferstruktur.
 */
class Mesh {
public:
    MeshRole role{MeshRole::TargetPart};
    QString name;

    std::vector<Vertex> vertices;
    std::vector<Triangle> triangles;
    Core::BoundingBox boundingBox;

    Mesh() = default;
    explicit Mesh(MeshRole meshRole, const QString& meshName = QString());

    void clear();
    void computeBoundingBox();
    void computeNormals();

    // Transformationen
    void translate(const Core::Vector3D& delta);
    void rotateX(double angleDeg, bool aroundCenter = true);
    void rotateY(double angleDeg, bool aroundCenter = true);
    void rotateZ(double angleDeg, bool aroundCenter = true);
    void rotate(double degX, double degY, double degZ, bool aroundCenter = true);
    void alignToOrigin(bool centerXY = true, bool zeroTopZ = true);

    // Skalierung
    void scale(double sx, double sy, double sz);
    void scaleUniform(double factor);

    [[nodiscard]] size_t vertexCount() const { return vertices.size(); }
    [[nodiscard]] size_t triangleCount() const { return triangles.size(); }
    [[nodiscard]] bool isEmpty() const { return vertices.empty(); }

    // Rohteil-Generatoren
    [[nodiscard]] static Mesh createBoxStock(double widthX, double depthY, double heightZ,
                                            const Core::Vector3D& minOrigin = {0.0, 0.0, 0.0});
    /// @brief Erzeugt einen Zylinder-Rohteilquerschnitt.
    /// @param axis 0=Z (vertikal), 1=X (horizontal), 2=Y (horizontal)
    /// @param positive true = Richtung positiv, false = negativ (Standard CNC: false)
    [[nodiscard]] static Mesh createCylinderStock(double radius, double height,
                                                  int segments = 48, int axis = 0,
                                                  bool positive = false);
};

} // namespace GeminiCNC::Geometry

#endif // GEMINI_CNC_MESH_H
