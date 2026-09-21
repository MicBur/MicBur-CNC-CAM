#ifndef GEMINI_CNC_FEATUREPREVIEWDIALOG_H
#define GEMINI_CNC_FEATUREPREVIEWDIALOG_H

#include <QDialog>
#include <QTableWidget>
#include <QPushButton>
#include <QLabel>
#include <QCheckBox>
#include <QComboBox>
#include <QVBoxLayout>

#include "cam/FeatureRecognizer.h"
#include "cam/AutoProgramGenerator.h"
#include "cam/ConversationalProgram.h"
#include "core/ToolDefinition.h"
#include "geometry/Mesh.h"

namespace GeminiCNC::UI {

/**
 * @brief Vorschau-Dialog für automatisch erkannte STL-Features.
 *
 * Zeigt eine Tabelle aller erkannten Features mit Typ, Z-Bereich,
 * Werkzeug-Vorschlag und geschätztem Volumen. Der Benutzer kann
 * Features an-/abwählen und Werkzeuge überschreiben.
 *
 * Bei "Übernehmen" wird ein vollständiges ConversationalProgram erzeugt.
 */
class FeaturePreviewDialog : public QDialog {
    Q_OBJECT
public:
    explicit FeaturePreviewDialog(QWidget* parent = nullptr);

    /// Setzt Stock- und Part-Mesh sowie die Werkzeugbibliothek, startet die Analyse.
    void runAnalysis(
        const Geometry::Mesh& stockMesh,
        const Geometry::Mesh& partMesh,
        const QList<Core::ToolDefinition>& toolLibrary
    );

    /// Gibt das generierte Programm zurück (nur gültig nach accept()).
    [[nodiscard]] CAM::ConversationalProgram generatedProgram() const { return m_program; }

signals:
    /// Wird gesendet wenn der Benutzer "Übernehmen" klickt.
    void programGenerated(const CAM::ConversationalProgram& program);

private slots:
    void onAcceptClicked();

private:
    void setupUi();
    void populateTable();
    QString featureTypeName(CAM::FeatureType type) const;

    CAM::RecognitionResult m_result;
    CAM::ConversationalProgram m_program;
    Geometry::Mesh m_stockMesh;
    Geometry::Mesh m_partMesh;
    QList<Core::ToolDefinition> m_toolLibrary;

    // UI
    QLabel* m_lblSummary{nullptr};
    QTableWidget* m_table{nullptr};
    QPushButton* m_btnAccept{nullptr};
    QPushButton* m_btnCancel{nullptr};
};

} // namespace GeminiCNC::UI

#endif // GEMINI_CNC_FEATUREPREVIEWDIALOG_H
