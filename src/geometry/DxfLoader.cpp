#include "DxfLoader.h"
#include <QFile>
#include <QTextStream>
#include <cmath>

namespace GeminiCNC::Geometry {

DxfLoader::LoadResult DxfLoader::loadFromFile(const QString& filePath) {
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        return {false, QString("Konnte DXF-Datei nicht öffnen: %1").arg(file.errorString()), {}};
    }

    QTextStream in(&file);
    LoadResult result;

    bool inEntities = false;
    QString currentEntity;
    Contour currentContour;
    QString currentLayer = QStringLiteral("0");

    double lineStartX = 0, lineStartY = 0;
    double lineEndX = 0, lineEndY = 0;
    double circleX = 0, circleY = 0, circleR = 0;
    double arcStartX = 0, arcStartY = 0, arcR = 0, arcStartAngle = 0, arcEndAngle = 0;

    auto finishEntity = [&]() {
        if (currentEntity == QStringLiteral("LINE")) {
            Contour c;
            c.isClosed = false;
            c.layerName = currentLayer;
            c.addPoint(lineStartX, lineStartY);
            c.addPoint(lineEndX, lineEndY);
            result.contours.push_back(c);
        } else if (currentEntity == QStringLiteral("CIRCLE")) {
            if (circleR > 1e-4) {
                Contour c = Contour::createCircle(circleX, circleY, circleR);
                c.layerName = currentLayer;
                result.contours.push_back(c);
            }
        } else if (currentEntity == QStringLiteral("ARC")) {
            if (arcR > 1e-4) {
                Contour c;
                c.isClosed = false;
                c.layerName = currentLayer;
                double a1 = arcStartAngle * M_PI / 180.0;
                double a2 = arcEndAngle * M_PI / 180.0;
                if (a2 < a1) a2 += 2.0 * M_PI;
                const int steps = std::max(8, static_cast<int>((a2 - a1) * 16.0 / M_PI));
                for (int s = 0; s <= steps; ++s) {
                    double ang = a1 + (a2 - a1) * s / steps;
                    c.addPoint(arcStartX + arcR * std::cos(ang),
                               arcStartY + arcR * std::sin(ang));
                }
                result.contours.push_back(c);
            }
        } else if (currentEntity == QStringLiteral("LWPOLYLINE")) {
            if (!currentContour.points.empty()) {
                currentContour.layerName = currentLayer;
                result.contours.push_back(currentContour);
                currentContour.clear();
            }
        }
        currentEntity.clear();
    };

    while (!in.atEnd()) {
        QString codeLine = in.readLine().trimmed();
        if (in.atEnd()) break;
        QString valLine = in.readLine().trimmed();

        int code = codeLine.toInt();

        if (code == 0) {
            if (valLine == QStringLiteral("SECTION")) {
                // Warte auf ENTITIES
            } else if (valLine == QStringLiteral("ENDSEC")) {
                finishEntity();
                inEntities = false;
            } else if (valLine == QStringLiteral("EOF")) {
                finishEntity();
                break;
            } else if (inEntities) {
                finishEntity();
                currentEntity = valLine;
                if (currentEntity == QStringLiteral("LWPOLYLINE")) {
                    currentContour.clear();
                    currentContour.isClosed = false;
                }
            }
        } else if (code == 2 && valLine == QStringLiteral("ENTITIES")) {
            inEntities = true;
        } else if (inEntities) {
            if (code == 8) {
                currentLayer = valLine;
            } else if (currentEntity == QStringLiteral("LINE")) {
                if (code == 10) lineStartX = valLine.toDouble();
                else if (code == 20) lineStartY = valLine.toDouble();
                else if (code == 11) lineEndX = valLine.toDouble();
                else if (code == 21) lineEndY = valLine.toDouble();
            } else if (currentEntity == QStringLiteral("CIRCLE")) {
                if (code == 10) circleX = valLine.toDouble();
                else if (code == 20) circleY = valLine.toDouble();
                else if (code == 40) circleR = valLine.toDouble();
            } else if (currentEntity == QStringLiteral("ARC")) {
                if (code == 10) arcStartX = valLine.toDouble();
                else if (code == 20) arcStartY = valLine.toDouble();
                else if (code == 40) arcR = valLine.toDouble();
                else if (code == 50) arcStartAngle = valLine.toDouble();
                else if (code == 51) arcEndAngle = valLine.toDouble();
            } else if (currentEntity == QStringLiteral("LWPOLYLINE")) {
                static double polyX = 0.0;
                if (code == 70) {
                    int flags = valLine.toInt();
                    currentContour.isClosed = (flags & 1) != 0;
                } else if (code == 10) {
                    polyX = valLine.toDouble();
                } else if (code == 20) {
                    double polyY = valLine.toDouble();
                    currentContour.addPoint(polyX, polyY);
                } else if (code == 42 && !currentContour.points.empty()) {
                    currentContour.points.back().bulge = valLine.toDouble();
                }
            }
        }
    }

    file.close();

    if (result.contours.empty()) {
        return {false, QStringLiteral("Keine geometrischen Konturen (Linien, Polylinien, Bögen) in der DXF gefunden."), {}};
    }

    result.success = true;
    return result;
}

Mesh DxfLoader::extrudeContours(const std::vector<Contour>& contours, double depthZ, MeshRole role) {
    Mesh mesh(role, QStringLiteral("DXF-Extrusion"));
    if (depthZ <= 0.0) depthZ = 10.0;

    const float zTop = 0.0f;
    const float zBottom = -static_cast<float>(depthZ);

    for (const auto& contour : contours) {
        if (contour.points.size() < 2) continue;

        const size_t ptCount = contour.points.size();
        const size_t segCount = contour.isClosed ? ptCount : ptCount - 1;

        // Seitenflächen triangulieren
        for (size_t i = 0; i < segCount; ++i) {
            size_t nextIdx = (i + 1) % ptCount;
            float x0 = static_cast<float>(contour.points[i].x);
            float y0 = static_cast<float>(contour.points[i].y);
            float x1 = static_cast<float>(contour.points[nextIdx].x);
            float y1 = static_cast<float>(contour.points[nextIdx].y);

            uint32_t base = static_cast<uint32_t>(mesh.vertices.size());

            // 4 Vertices für das Seitenrechteck
            mesh.vertices.push_back({x0, y0, zTop, 0, 0, 0});
            mesh.vertices.push_back({x1, y1, zTop, 0, 0, 0});
            mesh.vertices.push_back({x1, y1, zBottom, 0, 0, 0});
            mesh.vertices.push_back({x0, y0, zBottom, 0, 0, 0});

            mesh.triangles.push_back({base + 0, base + 1, base + 2});
            mesh.triangles.push_back({base + 0, base + 2, base + 3});
        }

        // Deck- und Bodenfläche (einfacher Fächer für konvexe Polygone)
        if (contour.isClosed && ptCount >= 3) {
            uint32_t topBase = static_cast<uint32_t>(mesh.vertices.size());
            for (size_t i = 0; i < ptCount; ++i) {
                mesh.vertices.push_back({static_cast<float>(contour.points[i].x),
                                         static_cast<float>(contour.points[i].y),
                                         zTop, 0, 0, 1});
            }
            for (size_t i = 1; i + 1 < ptCount; ++i) {
                mesh.triangles.push_back({topBase, topBase + static_cast<uint32_t>(i), topBase + static_cast<uint32_t>(i + 1)});
            }

            uint32_t botBase = static_cast<uint32_t>(mesh.vertices.size());
            for (size_t i = 0; i < ptCount; ++i) {
                mesh.vertices.push_back({static_cast<float>(contour.points[i].x),
                                         static_cast<float>(contour.points[i].y),
                                         zBottom, 0, 0, -1});
            }
            for (size_t i = 1; i + 1 < ptCount; ++i) {
                mesh.triangles.push_back({botBase, botBase + static_cast<uint32_t>(i + 1), botBase + static_cast<uint32_t>(i)});
            }
        }
    }

    mesh.computeBoundingBox();
    mesh.computeNormals();
    return mesh;
}

} // namespace GeminiCNC::Geometry
