#ifndef GEMINI_CNC_SETUPDIALOG_H
#define GEMINI_CNC_SETUPDIALOG_H

#include <QWidget>
#include <QLabel>
#include <QPushButton>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QComboBox>
#include <QStackedWidget>
#include "geometry/Mesh.h"
#include "geometry/Contour.h"
#include "core/MachineConfig.h"

namespace GeminiCNC::UI {

/**
 * @brief Dialog für Schritt 1: Geometrie-Import (Rohteil & Bauteil via STL/DXF), Nullpunktlage und Maschinenparameter.
 */
class SetupDialog : public QWidget {
    Q_OBJECT
public:
    explicit SetupDialog(QWidget* parent = nullptr);

    [[nodiscard]] const Geometry::Mesh& stockMesh() const { return m_stockMesh; }
    [[nodiscard]] const Geometry::Mesh& partMesh() const { return m_partMesh; }
    [[nodiscard]] const std::vector<Geometry::Contour>& contours() const { return m_contours; }
    [[nodiscard]] const Core::MachineConfig& machineConfig() const { return m_machineConfig; }

signals:
    void stockMeshChanged(const Geometry::Mesh& mesh);
    void partMeshChanged(const Geometry::Mesh& mesh);
    void contoursChanged(const std::vector<Geometry::Contour>& contours);
    void machineConfigChanged(const Core::MachineConfig& config);
    void materialPresetChanged(int presetId);

private slots:
    void onImportPartClicked();
    void onImportStockClicked();
    void onGenerateStockClicked();
    void onAlignOriginClicked();
    void onMachinePresetChanged(int index);
    void onMaterialPresetChanged(int index);

private:
    void updateInfoLabels();
    void setupUi();
    void loadMachinePresets();
    void applyMachineUiToConfig();

    Geometry::Mesh m_partMesh;
    Geometry::Mesh m_stockMesh;
    std::vector<Geometry::Contour> m_contours;
    Core::MachineConfig m_machineConfig;
    bool m_isUpdatingMachineUi{false};

    // ─── Bauteil ───
    QLabel* m_lblPartInfo{nullptr};

    // ─── Rohteil ───
    QLabel* m_lblStockInfo{nullptr};
    QComboBox* m_cmbStockType{nullptr};
    QComboBox* m_cmbMaterialPreset{nullptr};
    QStackedWidget* m_stockParamStack{nullptr};
    QPushButton* m_btnGenStock{nullptr};

    // Box-Parameter
    QDoubleSpinBox* m_spinStockX{nullptr};
    QDoubleSpinBox* m_spinStockY{nullptr};
    QDoubleSpinBox* m_spinStockZ{nullptr};

    // Zylinder-Parameter
    QDoubleSpinBox* m_spinStockRadius{nullptr};
    QDoubleSpinBox* m_spinStockCylHeight{nullptr};
    QComboBox* m_cmbCylAxis{nullptr};
    QComboBox* m_cmbCylDir{nullptr};

    // ─── Nullpunkt ───
    QComboBox* m_cmbRefPoint{nullptr};
    QCheckBox* m_chkCenterXY{nullptr};
    QCheckBox* m_chkZeroTopZ{nullptr};
    QDoubleSpinBox* m_spinOriginX{nullptr};
    QDoubleSpinBox* m_spinOriginY{nullptr};
    QDoubleSpinBox* m_spinOriginZ{nullptr};

    // ─── Maschinenparameter ───
    QComboBox* m_cmbMachinePreset{nullptr};
    QDoubleSpinBox* m_spinTravelX{nullptr};
    QDoubleSpinBox* m_spinTravelY{nullptr};
    QDoubleSpinBox* m_spinTravelZ{nullptr};
    QDoubleSpinBox* m_spinRapidXY{nullptr};
    QDoubleSpinBox* m_spinRapidZ{nullptr};
    QDoubleSpinBox* m_spinSpindleMin{nullptr};
    QDoubleSpinBox* m_spinSpindleMax{nullptr};
    QDoubleSpinBox* m_spinAccel{nullptr};
};

} // namespace GeminiCNC::UI

#endif // GEMINI_CNC_SETUPDIALOG_H
