#ifndef GEMINI_CNC_CONTOURDIALOG_H
#define GEMINI_CNC_CONTOURDIALOG_H

#include <QWidget>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QLabel>
#include "cam/Toolpath.h"
#include "core/ToolDefinition.h"
#include "core/MachineConfig.h"
#include "geometry/Mesh.h"
#include "geometry/Contour.h"

namespace GeminiCNC::UI {

/**
 * @brief Dialog für Schritt 3: CAM-Bearbeitungsdefinition & Konturprogrammierung.
 */
class ContourDialog : public QWidget {
    Q_OBJECT
public:
    explicit ContourDialog(QWidget* parent = nullptr);

    void setStockMesh(const Geometry::Mesh& mesh) { m_stockMesh = mesh; }
    void setPartMesh(const Geometry::Mesh& mesh) { m_partMesh = mesh; }
    void setContours(const std::vector<Geometry::Contour>& contours) { m_contours = contours; }
    void setActiveTool(const Core::ToolDefinition& tool) { m_activeTool = tool; updateToolInfo(); }
    void setMachineConfig(const Core::MachineConfig& config) { m_machineConfig = config; }

    [[nodiscard]] const CAM::Toolpath& currentToolpath() const { return m_currentToolpath; }

signals:
    void toolpathGenerated(const CAM::Toolpath& toolpath);

private slots:
    void onCalculateClicked();

private:
    void updateToolInfo();

    Geometry::Mesh m_stockMesh;
    Geometry::Mesh m_partMesh;
    std::vector<Geometry::Contour> m_contours;
    Core::ToolDefinition m_activeTool;
    Core::MachineConfig m_machineConfig;
    CAM::Toolpath m_currentToolpath;

    QComboBox* m_cmbOperation{nullptr};
    QComboBox* m_cmbContourSide{nullptr};

    QDoubleSpinBox* m_spinStartZ{nullptr};
    QDoubleSpinBox* m_spinTargetZ{nullptr};
    QDoubleSpinBox* m_spinStepDown{nullptr};
    QDoubleSpinBox* m_spinStepOver{nullptr};
    QDoubleSpinBox* m_spinAllowance{nullptr};
    QDoubleSpinBox* m_spinClearanceZ{nullptr};

    QLabel* m_lblToolSummary{nullptr};
    QLabel* m_lblResultSummary{nullptr};
    QPushButton* m_btnCalculate{nullptr};
};

} // namespace GeminiCNC::UI

#endif // GEMINI_CNC_CONTOURDIALOG_H
