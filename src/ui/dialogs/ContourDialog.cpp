#include "ContourDialog.h"
#include "cam/ToolpathGenerator.h"
#include "cam/CollisionDetector.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QGroupBox>

namespace GeminiCNC::UI {

ContourDialog::ContourDialog(QWidget* parent) : QWidget(parent) {
    m_activeTool = Core::ToolDefinition(1, QStringLiteral("6mm Schaftfräser"), Core::ToolType::EndMill, 6.0);

    auto* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(6, 6, 6, 6);
    mainLayout->setSpacing(8);

    // Werkzeug-Zusammenfassung
    m_lblToolSummary = new QLabel(this);
    m_lblToolSummary->setStyleSheet("background-color: #2D3748; padding: 6px; border-radius: 4px; font-weight: bold; color: #63B3ED;");
    mainLayout->addWidget(m_lblToolSummary);

    // Operations-Parameter
    auto* opGroup = new QGroupBox(QStringLiteral("Fräsoperation"), this);
    auto* formLayout = new QFormLayout(opGroup);

    m_cmbOperation = new QComboBox(this);
    m_cmbOperation->addItems({
        QStringLiteral("Planfräsen (Facing)"),
        QStringLiteral("2D Konturfräsen (Contour)"),
        QStringLiteral("Taschenfräsen (Pocketing)"),
        QStringLiteral("3D Rohteilschruppen (Stock Roughing)")
    });
    formLayout->addRow(QStringLiteral("Strategie:"), m_cmbOperation);

    m_cmbContourSide = new QComboBox(this);
    m_cmbContourSide->addItems({
        QStringLiteral("Außen (Outside)"),
        QStringLiteral("Innen (Inside)"),
        QStringLiteral("Auf Kontur (On Line)")
    });
    formLayout->addRow(QStringLiteral("Bahnkorrektur:"), m_cmbContourSide);

    m_spinStartZ = new QDoubleSpinBox(this);
    m_spinStartZ->setRange(-500.0, 500.0);
    m_spinStartZ->setValue(0.0);
    m_spinStartZ->setSuffix(" mm");
    formLayout->addRow(QStringLiteral("Start Z (Rohteil Oben):"), m_spinStartZ);

    m_spinTargetZ = new QDoubleSpinBox(this);
    m_spinTargetZ->setRange(-500.0, 500.0);
    m_spinTargetZ->setValue(-5.0);
    m_spinTargetZ->setSuffix(" mm");
    formLayout->addRow(QStringLiteral("Ziel Z (Frästiefe):"), m_spinTargetZ);

    m_spinStepDown = new QDoubleSpinBox(this);
    m_spinStepDown->setRange(0.1, 50.0);
    m_spinStepDown->setValue(1.5);
    m_spinStepDown->setSingleStep(0.5);
    m_spinStepDown->setSuffix(" mm");
    formLayout->addRow(QStringLiteral("Zustellung ap:"), m_spinStepDown);

    m_spinStepOver = new QDoubleSpinBox(this);
    m_spinStepOver->setRange(5.0, 95.0);
    m_spinStepOver->setValue(50.0);
    m_spinStepOver->setSuffix(" %");
    formLayout->addRow(QStringLiteral("Überlappung ae:"), m_spinStepOver);

    m_spinAllowance = new QDoubleSpinBox(this);
    m_spinAllowance->setRange(0.0, 10.0);
    m_spinAllowance->setValue(0.2);
    m_spinAllowance->setSingleStep(0.1);
    m_spinAllowance->setSuffix(" mm");
    formLayout->addRow(QStringLiteral("Schlichtaufmaß:"), m_spinAllowance);

    m_spinClearanceZ = new QDoubleSpinBox(this);
    m_spinClearanceZ->setRange(1.0, 100.0);
    m_spinClearanceZ->setValue(5.0);
    m_spinClearanceZ->setSuffix(" mm");
    formLayout->addRow(QStringLiteral("Sicherheitshöhe Z:"), m_spinClearanceZ);

    mainLayout->addWidget(opGroup);

    // Berechnen-Button
    m_btnCalculate = new QPushButton(QStringLiteral("⚡ Werkzeugwege berechnen"), this);
    m_btnCalculate->setStyleSheet("padding: 10px; font-weight: bold; background-color: #DD6B20; color: white; border-radius: 4px; font-size: 13px;");
    connect(m_btnCalculate, &QPushButton::clicked, this, &ContourDialog::onCalculateClicked);
    mainLayout->addWidget(m_btnCalculate);

    // Ergebnis-Anzeige
    m_lblResultSummary = new QLabel(QStringLiteral("Noch keine Fräsbahn berechnet."), this);
    m_lblResultSummary->setStyleSheet("background-color: #1A202C; padding: 8px; border-radius: 4px; color: #CBD5E0; font-family: Consolas, monospace;");
    m_lblResultSummary->setWordWrap(true);
    mainLayout->addWidget(m_lblResultSummary);

    mainLayout->addStretch(1);
    updateToolInfo();
}

void ContourDialog::updateToolInfo() {
    m_lblToolSummary->setText(QString("Aktives Werkzeug: T%1 - %2 (Ø %3 mm)")
        .arg(m_activeTool.id)
        .arg(m_activeTool.name)
        .arg(m_activeTool.diameter, 0, 'f', 1));
}

void ContourDialog::onCalculateClicked() {
    int opIndex = m_cmbOperation->currentIndex();
    CAM::Toolpath tp;

    double startZ = m_spinStartZ->value();
    double targetZ = m_spinTargetZ->value();
    double stepDown = m_spinStepDown->value();
    double clearanceZ = m_spinClearanceZ->value();

    Core::BoundingBox stockBounds = m_stockMesh.boundingBox.isValid()
        ? m_stockMesh.boundingBox
        : Core::BoundingBox({-50, -40, -10}, {50, 40, 0});

    if (opIndex == 0) {
        // Planfräsen
        CAM::FacingParams p;
        p.startZ = startZ;
        p.targetZ = targetZ;
        p.stepDown = stepDown;
        p.stepOver = m_activeTool.diameter * (m_spinStepOver->value() / 100.0);
        p.clearanceZ = clearanceZ;
        tp = CAM::ToolpathGenerator::generateFacing(stockBounds, m_activeTool, p);
    } else if (opIndex == 1) {
        // 2D Konturfräsen
        CAM::ContourParams p;
        p.startZ = startZ;
        p.targetZ = targetZ;
        p.stepDown = stepDown;
        p.clearanceZ = clearanceZ;
        p.finishAllowance = m_spinAllowance->value();

        int sideIdx = m_cmbContourSide->currentIndex();
        p.side = (sideIdx == 0) ? CAM::ContourSide::Outside : ((sideIdx == 1) ? CAM::ContourSide::Inside : CAM::ContourSide::OnLine);

        // Falls DXF-Konturen geladen sind, die erste verwenden, sonst Rohteilkontur als Test
        Geometry::Contour c;
        if (!m_contours.empty()) {
            c = m_contours.front();
        } else {
            c = Geometry::Contour::createRectangle(stockBounds.minPoint.x + 10, stockBounds.minPoint.y + 10,
                                                   stockBounds.widthX() - 20, stockBounds.depthY() - 20);
        }
        tp = CAM::ToolpathGenerator::generateContourMilling(c, m_activeTool, p);
    } else if (opIndex == 2) {
        // Taschenfräsen
        CAM::PocketParams p;
        p.startZ = startZ;
        p.targetZ = targetZ;
        p.stepDown = stepDown;
        p.clearanceZ = clearanceZ;
        p.stepOverRatio = m_spinStepOver->value() / 100.0;
        p.finishAllowance = m_spinAllowance->value();

        Geometry::Contour c;
        if (!m_contours.empty()) {
            c = m_contours.front();
        } else {
            c = Geometry::Contour::createRectangle(stockBounds.minPoint.x + 15, stockBounds.minPoint.y + 15,
                                                   stockBounds.widthX() - 30, stockBounds.depthY() - 30);
        }
        tp = CAM::ToolpathGenerator::generatePocketMilling(c, m_activeTool, p);
    } else if (opIndex == 3) {
        // 3D Rohteilschruppen
        CAM::StockRoughingParams p;
        p.stepDown = stepDown;
        p.stepOverRatio = m_spinStepOver->value() / 100.0;
        p.finishAllowance = m_spinAllowance->value();
        p.clearanceZ = clearanceZ;
        tp = CAM::ToolpathGenerator::generateStockRoughing(m_stockMesh, m_partMesh, m_activeTool, p);
    }

    // Automatische Kollisionsprüfung
    auto collisionRep = CAM::CollisionDetector::verifyToolpath(tp, m_machineConfig, m_activeTool, stockBounds);

    m_currentToolpath = tp;

    QString statusText = QString("Segmente: %1 | Weg: %2 mm\nGeschätzte Fräszeit: %3 min\nKollisionsstatus: %4")
        .arg(tp.size())
        .arg(tp.totalLength(), 0, 'f', 1)
        .arg(tp.estimatedTotalTimeSeconds() / 60.0, 0, 'f', 1)
        .arg(collisionRep.hasErrors
             ? QString("⚠ %1 KRITISCHE FEHLER GEFUNDEN!").arg(collisionRep.totalViolations)
             : QStringLiteral("✔ KEINE KOLLISIONEN"));

    if (collisionRep.hasErrors) {
        m_lblResultSummary->setStyleSheet("background-color: #742A2A; padding: 8px; border-radius: 4px; color: #FED7D7; font-family: Consolas, monospace; font-weight: bold;");
    } else {
        m_lblResultSummary->setStyleSheet("background-color: #22543D; padding: 8px; border-radius: 4px; color: #C6F6D5; font-family: Consolas, monospace; font-weight: bold;");
    }
    m_lblResultSummary->setText(statusText);

    emit toolpathGenerated(m_currentToolpath);
}

} // namespace GeminiCNC::UI
