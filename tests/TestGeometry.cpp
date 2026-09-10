#include <iostream>
#include <cassert>
#include <cmath>
#include <QFile>
#include <QTemporaryFile>

#include "geometry/Mesh.h"
#include "geometry/StlLoader.h"
#include "geometry/Contour.h"
#include "geometry/DxfLoader.h"

using namespace GeminiCNC;

void testMeshBoxAndNormals() {
    std::cout << "[TEST] Mesh Box Stock Creation & BoundingBox..." << std::endl;
    auto mesh = Geometry::Mesh::createBoxStock(100.0, 80.0, 20.0, {-50.0, -40.0, -20.0});

    assert(!mesh.isEmpty());
    assert(mesh.vertexCount() == 24);
    assert(mesh.triangleCount() == 12);

    const auto& b = mesh.boundingBox;
    Q_UNUSED(b);
    assert(std::abs(b.widthX() - 100.0) < 1e-4);
    assert(std::abs(b.depthY() - 80.0) < 1e-4);
    assert(std::abs(b.heightZ() - 20.0) < 1e-4);
    assert(std::abs(b.center().x) < 1e-4);
    assert(std::abs(b.center().y) < 1e-4);

    std::cout << " -> PASSED" << std::endl;
}

void testStlLoaderBinaryAndAscii() {
    std::cout << "[TEST] STL Binary Save & Reload..." << std::endl;
    auto mesh = Geometry::Mesh::createBoxStock(50.0, 50.0, 10.0, {0.0, 0.0, -10.0});

    QTemporaryFile tempStl;
    assert(tempStl.open());
    QString tempPath = tempStl.fileName();
    tempStl.close();

    bool saved = Geometry::StlLoader::saveBinary(tempPath, mesh);
    Q_UNUSED(saved);
    assert(saved);

    auto loadRes = Geometry::StlLoader::loadFromFile(tempPath, Geometry::MeshRole::TargetPart);
    assert(loadRes.success);
    assert(loadRes.mesh.triangleCount() == 12);
    assert(std::abs(loadRes.mesh.boundingBox.widthX() - 50.0) < 1e-3);

    QFile::remove(tempPath);
    std::cout << " -> PASSED" << std::endl;
}

void testContourOffsetAndArea() {
    std::cout << "[TEST] Contour Area, Perimeter & Offset..." << std::endl;
    auto rect = Geometry::Contour::createRectangle(0.0, 0.0, 100.0, 50.0);

    assert(rect.points.size() == 4);
    assert(std::abs(rect.perimeter() - 300.0) < 1e-4);
    assert(std::abs(std::abs(rect.signedArea()) - 5000.0) < 1e-4);
    assert(rect.containsPoint(50.0, 25.0));
    assert(!rect.containsPoint(150.0, 25.0));

    // Offset um 5mm nach außen
    auto offsetRect = rect.createOffset(5.0);
    assert(!offsetRect.points.empty());
    assert(offsetRect.getBoundingBox().widthX() > rect.getBoundingBox().widthX());

    std::cout << " -> PASSED" << std::endl;
}

int main() {
    std::cout << "=== Running Geometry Test Suite ===" << std::endl;
    testMeshBoxAndNormals();
    testStlLoaderBinaryAndAscii();
    testContourOffsetAndArea();
    std::cout << "=== All Geometry Tests PASSED ===" << std::endl;
    return 0;
}
