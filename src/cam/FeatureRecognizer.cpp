#include "FeatureRecognizer.h"
#include "MeshSlicer.h"
#include <QPainterPath>
#include <QPolygonF>
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace GeminiCNC::CAM {

namespace {

QPainterPath contourToPath(const Geometry::Contour& contour) {
    QPainterPath path;
    if (contour.points.empty()) return path;
    
    QPolygonF poly;
    for (const auto& pt : contour.points) {
        poly << QPointF(pt.x, pt.y);
    }
    
    if (!poly.isClosed() && !poly.empty()) {
        poly << poly.first();
    }
    
    path.addPolygon(poly);
    return path;
}

Geometry::Contour pathToContour(const QPolygonF& poly) {
    Geometry::Contour contour;
    for (int i = 0; i < poly.size(); ++i) {
        contour.addPoint(poly[i].x(), poly[i].y());
    }
    contour.isClosed = true;
    return contour;
}

double getPolygonArea(const QPolygonF& poly) {
    double area = 0.0;
    int n = poly.size();
    for (int i = 0; i < n; i++) {
        int j = (i + 1) % n;
        area += poly[i].x() * poly[j].y() - poly[j].x() * poly[i].y();
    }
    return std::abs(area) / 2.0;
}

double getPolygonPerimeter(const QPolygonF& poly) {
    double perimeter = 0.0;
    int n = poly.size();
    for (int i = 0; i < n; i++) {
        int j = (i + 1) % n;
        double dx = poly[j].x() - poly[i].x();
        double dy = poly[j].y() - poly[i].y();
        perimeter += std::sqrt(dx*dx + dy*dy);
    }
    return perimeter;
}

QPointF getPolygonCentroid(const QPolygonF& poly) {
    double cx = 0.0, cy = 0.0;
    double area = 0.0;
    int n = poly.size();
    for (int i = 0; i < n; i++) {
        int j = (i + 1) % n;
        double factor = poly[i].x() * poly[j].y() - poly[j].x() * poly[i].y();
        cx += (poly[i].x() + poly[j].x()) * factor;
        cy += (poly[i].y() + poly[j].y()) * factor;
        area += factor;
    }
    area /= 2.0;
    if (std::abs(area) < 1e-9) {
        if (n > 0) return poly.first();
        return QPointF(0,0);
    }
    cx /= (6.0 * area);
    cy /= (6.0 * area);
    return QPointF(cx, cy);
}

struct TempContourData {
    Geometry::Contour contour;
    double z;
    double area;
    double perimeter;
    QPointF centroid;
    double minX, maxX, minY, maxY;
    bool touchesStockEdge;
    bool used = false;
};

} // anonymous namespace

RecognitionResult FeatureRecognizer::analyze(
    const Geometry::Mesh& partMesh,
    const Geometry::Mesh& stockMesh,
    const QList<Core::ToolDefinition>& toolLibrary,
    const Settings& settings
) {
    RecognitionResult result;
    
    // BoundingBox-Zugriff über minPoint/maxPoint
    const auto& partBB = partMesh.boundingBox;
    const auto& stockBB = stockMesh.boundingBox;
    
    const double stockMinX = stockBB.minPoint.x;
    const double stockMaxX = stockBB.maxPoint.x;
    const double stockMinY = stockBB.minPoint.y;
    const double stockMaxY = stockBB.maxPoint.y;
    const double stockMinZ = stockBB.minPoint.z;
    const double stockMaxZ = stockBB.maxPoint.z;
    const double partMaxZ  = partBB.maxPoint.z;
    
    // Planfräsen nötig? (Stock-Oberkante > Part-Oberkante)
    if (stockMaxZ > partMaxZ + 0.1) {
        RecognizedFeature facingFeature;
        facingFeature.type = FeatureType::Facing;
        facingFeature.name = QStringLiteral("Planfräsen");
        facingFeature.topZ = stockMaxZ;
        facingFeature.bottomZ = partMaxZ;
        facingFeature.priority = 10;
        
        // FaceMill bevorzugen
        for (const auto& tool : toolLibrary) {
            if (tool.type == Core::ToolType::FaceMill) {
                facingFeature.suggestedToolId = tool.id;
                break;
            }
        }
        result.features.push_back(facingFeature);
    }
    
    // Phase 1: Z-Slicing beider Meshes
    auto stockSlices = MeshSlicer::sliceUniform(stockMesh, stockMinZ, stockMaxZ, settings.sliceZInterval);
    auto partSlices = MeshSlicer::sliceUniform(partMesh, stockMinZ, stockMaxZ, settings.sliceZInterval);
    
    std::vector<TempContourData> allRemovalContours;
    
    // Phase 2: Boolean-Differenz (Stock \ Part) pro Z-Ebene
    size_t numSlices = std::min(stockSlices.size(), partSlices.size());
    for (size_t i = 0; i < numSlices; ++i) {
        double currentZ = stockSlices[i].z;
        if (currentZ > partMaxZ) continue; // Facing regelt das
        
        QPainterPath stockPath;
        for (const auto& sc : stockSlices[i].contours) {
            stockPath.addPath(contourToPath(sc.contour));
        }
        
        QPainterPath partPath;
        for (const auto& pc : partSlices[i].contours) {
            partPath.addPath(contourToPath(pc.contour));
        }
        
        QPainterPath removalPath = stockPath.subtracted(partPath).simplified();
        QList<QPolygonF> polys = removalPath.toSubpathPolygons();
        
        for (const auto& poly : polys) {
            if (poly.size() < 3) continue;
            
            TempContourData td;
            td.contour = pathToContour(poly);
            td.z = currentZ;
            td.area = getPolygonArea(poly);
            if (td.area < 1.0) continue; // Kleinartefakte ignorieren
            
            td.perimeter = getPolygonPerimeter(poly);
            td.centroid = getPolygonCentroid(poly);
            
            td.minX = td.maxX = poly[0].x();
            td.minY = td.maxY = poly[0].y();
            for (const auto& pt : poly) {
                td.minX = std::min(td.minX, pt.x());
                td.maxX = std::max(td.maxX, pt.x());
                td.minY = std::min(td.minY, pt.y());
                td.maxY = std::max(td.maxY, pt.y());
            }
            
            // Berührt die Kontur den Stock-Rand?
            td.touchesStockEdge = false;
            for (const auto& pt : poly) {
                if (std::abs(pt.x() - stockMinX) < settings.stockMarginTolerance ||
                    std::abs(pt.x() - stockMaxX) < settings.stockMarginTolerance ||
                    std::abs(pt.y() - stockMinY) < settings.stockMarginTolerance ||
                    std::abs(pt.y() - stockMaxY) < settings.stockMarginTolerance) {
                    td.touchesStockEdge = true;
                    break;
                }
            }
            
            allRemovalContours.push_back(td);
        }
    }
    
    // Nach Z sortieren (oben zuerst)
    std::sort(allRemovalContours.begin(), allRemovalContours.end(), [](const TempContourData& a, const TempContourData& b) {
        return a.z > b.z;
    });
    
    // Phase 3+4: Z-Stacking und Klassifikation
    std::vector<std::vector<TempContourData*>> stackedFeatures;
    
    for (auto& c : allRemovalContours) {
        if (c.used) continue;
        
        std::vector<TempContourData*> currentFeatureGroup;
        currentFeatureGroup.push_back(&c);
        c.used = true;
        
        TempContourData* lastAdded = &c;
        
        // Tiefere Schichten suchen
        for (auto& cand : allRemovalContours) {
            if (cand.used || cand.z >= lastAdded->z) continue;
            
            // Benachbarte Schicht?
            if (std::abs(cand.z - lastAdded->z) > settings.sliceZInterval * 1.5) continue;
            
            // Schwerpunkt- und Flächen-Toleranzprüfung
            double dist = std::sqrt(std::pow(cand.centroid.x() - lastAdded->centroid.x(), 2) + 
                                    std::pow(cand.centroid.y() - lastAdded->centroid.y(), 2));
                                    
            double areaDiff = std::abs(cand.area - lastAdded->area) / std::max(cand.area, lastAdded->area);
            
            if (dist < settings.mergeTolerance && areaDiff < settings.areaTolerance) {
                currentFeatureGroup.push_back(&cand);
                cand.used = true;
                lastAdded = &cand;
            }
        }
        
        stackedFeatures.push_back(currentFeatureGroup);
    }
    
    // RecognizedFeatures erzeugen
    int featureCounter = 1;
    for (const auto& group : stackedFeatures) {
        if (group.empty()) continue;
        
        RecognizedFeature f;
        const auto* topContour = group.front();
        f.boundary = topContour->contour;
        f.topZ = topContour->z;
        f.bottomZ = group.back()->z;
        
        double avgArea = 0;
        for (const auto* c : group) avgArea += c->area;
        avgArea /= static_cast<double>(group.size());
        
        f.estimatedVolume = avgArea * (f.topZ - f.bottomZ + settings.sliceZInterval);
        result.totalRemovalVolume += f.estimatedVolume;
        
        f.circularity = 4.0 * M_PI * topContour->area / (topContour->perimeter * topContour->perimeter);
        
        double width = topContour->maxX - topContour->minX;
        double height = topContour->maxY - topContour->minY;
        f.aspectRatio = std::max(width, height) / std::max(1e-6, std::min(width, height));
        f.diameter = std::max(width, height);
        
        // Phase 3: Klassifikation
        if (f.circularity > settings.minCircularityHole && f.diameter < settings.maxHoleDiameter) {
            f.type = FeatureType::CircularHole;
            f.name = QStringLiteral("Bohrung Ø%1").arg(f.diameter, 0, 'f', 1);
            f.priority = 50;
        } else if (f.aspectRatio > settings.minSlotAspectRatio) {
            f.type = FeatureType::Slot;
            f.name = QStringLiteral("Nut %1").arg(featureCounter);
            f.priority = 40;
        } else if (topContour->touchesStockEdge) {
            f.type = FeatureType::ExternalContour;
            f.name = QStringLiteral("Außenkontur %1").arg(featureCounter);
            f.priority = 20;
        } else {
            if (f.bottomZ <= stockMinZ + 0.1) {
                f.type = FeatureType::ThroughPocket;
                f.name = QStringLiteral("Durchgangstasche %1").arg(featureCounter);
            } else {
                f.type = FeatureType::Pocket;
                f.name = QStringLiteral("Tasche %1").arg(featureCounter);
            }
            f.priority = 35;
        }
        ++featureCounter;
        
        // Phase 5: Werkzeug-Empfehlung
        f.suggestedToolId = -1;
        double bestDia = 0.0;
        if (f.type == FeatureType::CircularHole) {
            // Bohrer suchen: größter der passt
            for (const auto& tool : toolLibrary) {
                if (tool.type == Core::ToolType::Drill && tool.diameter <= f.diameter) {
                    if (tool.diameter > bestDia) {
                        bestDia = tool.diameter;
                        f.suggestedToolId = tool.id;
                    }
                }
            }
            // Fallback: EndMill
            if (f.suggestedToolId < 0) {
                for (const auto& tool : toolLibrary) {
                    if (tool.type == Core::ToolType::EndMill && tool.diameter < f.diameter * 0.9) {
                        if (tool.diameter > bestDia) {
                            bestDia = tool.diameter;
                            f.suggestedToolId = tool.id;
                        }
                    }
                }
            }
        } else {
            // Größten Schaftfräser der passt (< 90% der Feature-Mindestbreite)
            double minDim = std::min(width, height);
            for (const auto& tool : toolLibrary) {
                if (tool.type == Core::ToolType::EndMill && tool.diameter < minDim * 0.9) {
                    if (tool.diameter > bestDia) {
                        bestDia = tool.diameter;
                        f.suggestedToolId = tool.id;
                    }
                }
            }
        }
        
        // Fallback: erstes Werkzeug in der Bibliothek
        if (f.suggestedToolId < 0 && !toolLibrary.isEmpty()) {
            f.suggestedToolId = toolLibrary.first().id;
        }
        
        result.features.push_back(f);
    }
    
    // Phase 6: Prioritäten feinjustieren — Taschen nach Volumen (größere zuerst)
    for (auto& f : result.features) {
        if (f.type == FeatureType::Pocket || f.type == FeatureType::ThroughPocket) {
            f.priority = 39 - std::min(9, static_cast<int>(f.estimatedVolume / 10000.0));
        }
    }
    
    // Sortieren: niedrigere Priorität = zuerst bearbeiten
    std::sort(result.features.begin(), result.features.end(), [](const RecognizedFeature& a, const RecognizedFeature& b) {
        if (a.priority != b.priority) return a.priority < b.priority;
        return a.topZ > b.topZ; // Gleiche Priorität: von oben nach unten
    });
    
    // Zusammenfassung
    int holeCount = 0, pocketCount = 0, slotCount = 0, contourCount = 0, facingCount = 0;
    for (const auto& f : result.features) {
        switch (f.type) {
            case FeatureType::CircularHole: holeCount++; break;
            case FeatureType::Pocket:
            case FeatureType::ThroughPocket: pocketCount++; break;
            case FeatureType::Slot: slotCount++; break;
            case FeatureType::ExternalContour: contourCount++; break;
            case FeatureType::Facing: facingCount++; break;
            default: break;
        }
    }
    
    result.summary = QStringLiteral("Analyse abgeschlossen.\n"
        "Gefundene Features: %1 Planfräsen, %2 Außenkonturen, %3 Taschen, %4 Nuten, %5 Bohrungen.\n"
        "Geschätztes Zerspanvolumen: %6 mm³.")
        .arg(facingCount).arg(contourCount).arg(pocketCount).arg(slotCount).arg(holeCount)
        .arg(result.totalRemovalVolume, 0, 'f', 1);
        
    return result;
}

} // namespace GeminiCNC::CAM
