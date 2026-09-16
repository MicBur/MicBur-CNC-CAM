#ifndef GEMINI_CNC_TOOLMANAGERDIALOG_H
#define GEMINI_CNC_TOOLMANAGERDIALOG_H

#include <QWidget>
#include <QListWidget>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QComboBox>
#include <QPushButton>
#include <QLabel>
#include <QGroupBox>
#include <QLineEdit>
#include "core/ToolDefinition.h"
#include "core/MaterialDatabase.h"
#include "ToolGraphicsWidget.h"

namespace GeminiCNC::UI {

/**
 * @brief Grafischer Werkzeug-Einrichtungsdialog im Hurco WinMax-Stil.
 *
 * Split-Screen: Links Werkzeugliste + Parameter, rechts grafische
 * Werkzeug-Silhouette + automatisch berechnete Schnittdaten.
 */
class ToolManagerDialog : public QWidget {
    Q_OBJECT
public:
    explicit ToolManagerDialog(QWidget* parent = nullptr);

    [[nodiscard]] Core::ToolDefinition activeTool() const;
    [[nodiscard]] const QList<Core::ToolDefinition>& toolList() const { return m_tools; }

signals:
    void activeToolChanged(const Core::ToolDefinition& tool);
    void toolLibraryChanged(const QList<Core::ToolDefinition>& tools);

private slots:
    void onToolListSelectionChanged();
    void onAddToolClicked();
    void onDeleteToolClicked();
    void onActivateClicked();
    void onToolTypeChanged(int index);
    void onParameterChanged();
    void onMaterialChanged(int index);
    void onColorClicked();

private:
    void setupUi();
    void populateToolList();
    void loadToolToEditor(int index);
    void saveEditorToTool(int index);
    void recalculateCuttingData();
    void updateGraphics();
    void persistLibrary(); // speichert die Bibliothek und meldet Änderungen weiter

    bool m_saveErrorShown{false};

    // Werkzeugbibliothek
    QList<Core::ToolDefinition> m_tools;
    int m_activeToolIndex{0};
    int m_editingIndex{-1};
    bool m_isLoadingEditor{false};

    // Materialdatenbank
    Core::MaterialDatabase m_materialDb;

    // === Linke Spalte ===
    QListWidget* m_toolList{nullptr};
    QPushButton* m_btnActivate{nullptr};
    QPushButton* m_btnAdd{nullptr};
    QPushButton* m_btnDelete{nullptr};

    // Typ-Auswahl
    QComboBox* m_cmbType{nullptr};

    // Parameter-Eingaben
    QLineEdit*      m_editName{nullptr};
    QDoubleSpinBox* m_spinDiameter{nullptr};
    QDoubleSpinBox* m_spinFluteLength{nullptr};
    QDoubleSpinBox* m_spinShaftDia{nullptr};
    QDoubleSpinBox* m_spinStickOut{nullptr};
    QDoubleSpinBox* m_spinOverallLength{nullptr};
    QDoubleSpinBox* m_spinHolderDia{nullptr};
    QSpinBox*       m_spinFlutes{nullptr};
    QPushButton*    m_btnColor{nullptr};

    // === Rechte Spalte ===
    ToolGraphicsWidget* m_toolGraphic{nullptr};

    // Schnittdaten (berechnet)
    QComboBox* m_cmbMaterial{nullptr};
    QLabel* m_lblRpm{nullptr};
    QLabel* m_lblFeed{nullptr};
    QLabel* m_lblFz{nullptr};
    QLabel* m_lblPlunge{nullptr};
    QLabel* m_lblAp{nullptr};
    QLabel* m_lblAe{nullptr};
    QLabel* m_lblVc{nullptr};
};

} // namespace GeminiCNC::UI

#endif // GEMINI_CNC_TOOLMANAGERDIALOG_H
