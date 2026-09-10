#ifndef GEMINI_CNC_PROBEDIALOG_H
#define GEMINI_CNC_PROBEDIALOG_H

#include <QWidget>
#include <QTabWidget>
#include <QPushButton>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QLabel>
#include <QTextEdit>
#include "hardware/ProbeController.h"
#include "core/ToolDefinition.h"

namespace GeminiCNC::UI {

/**
 * @brief Dialog für Werkzeug- und Werkstück-Einmessen im Hurco WinMax-Stil.
 */
class ProbeDialog : public QWidget {
    Q_OBJECT
public:
    explicit ProbeDialog(Hardware::ProbeController* probeCtrl, QWidget* parent = nullptr);

    void setToolLibrary(const QList<Core::ToolDefinition>& tools);
    void setActiveTool(const Core::ToolDefinition& tool);

private slots:
    void onMeasureToolClicked();
    void onCheckBreakageClicked();
    void onProbeZClicked();
    void onProbeTouchPlateZClicked();
    void onProbeCornerTouchPlateClicked();
    void onProbeEdgeClicked(Hardware::JogAxis axis, int dir);
    void onProbeCornerClicked(Hardware::CornerPosition corner);
    void onProbeBoreClicked();
    void onProbeSkewClicked();

    void onProbingStarted(const QString& cycleName);
    void onProbingFinished(const QString& result);
    void onProbeError(const QString& error);

private:
    void setupUi();

    Hardware::ProbeController* m_probeCtrl{nullptr};
    Core::ToolDefinition m_activeTool;
    QList<Core::ToolDefinition> m_toolLibrary;

    QLabel* m_lblActiveToolInfo{nullptr};
    QLabel* m_lblProbeStatus{nullptr};
    QTextEdit* m_txtLog{nullptr};

    QDoubleSpinBox* m_spinSetterX{nullptr};
    QDoubleSpinBox* m_spinSetterY{nullptr};
    QDoubleSpinBox* m_spinSetterHeight{nullptr};

    QDoubleSpinBox* m_spinPlateThickness{nullptr};
    QDoubleSpinBox* m_spinPlateLipX{nullptr};
    QDoubleSpinBox* m_spinPlateLipY{nullptr};

    QDoubleSpinBox* m_spinBoreDia{nullptr};
    QDoubleSpinBox* m_spinSkewDistance{nullptr};
};

} // namespace GeminiCNC::UI

#endif // GEMINI_CNC_PROBEDIALOG_H
