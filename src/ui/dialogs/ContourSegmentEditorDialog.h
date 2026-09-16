#ifndef GEMINI_CNC_CONTOURSEGMENTEDITORDIALOG_H
#define GEMINI_CNC_CONTOURSEGMENTEDITORDIALOG_H

#include <QWidget>
#include <QLabel>
#include <QLineEdit>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QTabWidget>
#include <QPushButton>
#include <QFormLayout>
#include <vector>
#include "geometry/Contour.h"
#include "geometry/ContourSolver.h"
#include "core/ToolDefinition.h"
#include <QList>
#include <QSet>

namespace GeminiCNC::UI {

/**
 * @brief Hurco WinMax Datensatz-Editor (MILL CONTOUR):
 *
 * Exakte Nachbildung von Screenshot 191921 (LINE) und 192011 (ARC).
 * - Spalten-Aufbau: Links Eingaben (Endpunkte/Maße), Rechts Kontext (Startpunkte)
 * - Auto-Berechnung: Sobald Teilmaße eingegeben werden, löst der ContourSolver
 *   die restlichen Felder live auf!
 * - F4 "Werte speichern" übernimmt die berechneten Werte.
 * - F5 "Nächste Lösung" toggelt alternative Kreismittelpunkte.
 * - Untere Tabs: [SCHRUPPEN], [SCHLICHTEN], [KÜHLMITTEL].
 */
class ContourSegmentEditorDialog : public QWidget {
    Q_OBJECT
public:
    explicit ContourSegmentEditorDialog(QWidget* parent = nullptr);

    void setSegments(const std::vector<Geometry::ContourSegment>& segments);
    [[nodiscard]] const std::vector<Geometry::ContourSegment>& segments() const { return m_segments; }
    [[nodiscard]] Geometry::Contour compiledContour() const { return m_compiledContour; }

    // Werkzeugauswahl aus der Bibliothek und Technologie des Blocks (Werkzeug, Fräsart, Schnittwerte)
    void setToolLibrary(const QList<Core::ToolDefinition>& tools);
    void setTechnology(int toolId, int contourSide, double feed, double plunge, double rpm, double stepDown);
    // Konturart (0 = Kontur, 1 = Tasche, 2 = Insel) und ob Z UNTEN von Segment 0 für alle Segmente gilt
    void setContourOptions(int role, bool zForAll);

    // Navigation & Softkey-Aktionen (F1..F8)
    void navigateNext();
    void navigatePrevious();
    void addSegment(Geometry::ContourSegmentType type);
    void deleteCurrentSegment();
    void storeCalculatedValue();
    void findAlternativeSolution();

    [[nodiscard]] int currentStep() const { return m_currentIndex; }
    [[nodiscard]] int totalSteps() const { return static_cast<int>(m_segments.size()); }

signals:
    void contourUpdated(const Geometry::Contour& contour);
    void promptChanged(const QString& promptText);
    void accepted();
    // contourSide: 0 = Außen, 1 = Innen, 2 = Auf Kontur (CAM::ContourSide)
    void technologyChanged(int toolId, int contourSide, double feed, double plunge, double rpm, double stepDown);
    void contourOptionsChanged(int role, bool zForAll);

private slots:
    void onInputEdited();
    void onDepthEdited();
    void onArcInputEdited();
    void onTechnologyEdited();
    void onMillingTypeChanged(int idx);

private:
    void setupUi();
    void loadStep(int index);
    void saveCurrentStep();
    void recompileContour();
    void updateContextPrompt();
    [[nodiscard]] bool isArcStep(int index) const;
    void applyArcSolution();
    void applyContourOptions();          // Bedienereingabe Konturart / Z für alle übernehmen
    void updateDepthFieldVisibility();   // nur die nötigen Tiefenfelder abfragen
    [[nodiscard]] bool perSegmentDepth() const { return m_contourRole == 0 && !m_zForAll; }

    int m_contourRole{0};
    bool m_zForAll{false};
    QComboBox* m_cmbContourRole{nullptr};
    QComboBox* m_cmbZForAll{nullptr};
    QFormLayout* m_roughForm{nullptr};

    bool m_isSyncingTechnology{false};
    QFormLayout* m_arcForm{nullptr};
    QSet<QLineEdit*> m_arcKnownFields; // vom Bediener vorgegebene Bogenwerte (Rest wird berechnet)

    std::vector<Geometry::ContourSegment> m_segments;
    Geometry::Contour m_compiledContour;
    int m_currentIndex{0};
    bool m_isLoading{false};
    int m_currentSolutionIndex{0};
    std::vector<Geometry::ArcSolveResult> m_cachedArcSolutions;

    // Kopfzeile (Hurco: BLOCK 2 MILL CONTOUR / SEGMENT 1 LINE)
    QLabel* m_lblBlockHeader{nullptr};
    QLabel* m_lblSegmentHeader{nullptr};

    // ─── Eingabefelder (Links) ───
    QWidget* m_lineInputsWidget{nullptr};
    QWidget* m_arcInputsWidget{nullptr};

    // LINE Eingaben (Screenshot 191921)
    QFormLayout* m_leftForm{nullptr};
    QLabel* m_lblXCaption{nullptr};    // X END bzw. X START (Segment 0)
    QLabel* m_lblYCaption{nullptr};
    QLabel* m_lblZEndCaption{nullptr}; // Z END bzw. Z UNTEN (Segment 0)
    QLineEdit* m_editZStart{nullptr};  // Nur Segment 0
    QLineEdit* m_editLineEndX{nullptr};
    QLineEdit* m_editLineEndY{nullptr};
    QLineEdit* m_editLineZEnd{nullptr};
    QLineEdit* m_editLineLength{nullptr};
    QLineEdit* m_editLineAngle{nullptr};

    // ARC Eingaben (Screenshot 192011)
    QComboBox* m_cmbArcDirection{nullptr}; // CW / CCW
    QLineEdit* m_editArcEndX{nullptr};
    QLineEdit* m_editArcEndY{nullptr};
    QLineEdit* m_editArcZEnd{nullptr};
    QLineEdit* m_editArcCenterX{nullptr};
    QLineEdit* m_editArcCenterY{nullptr};
    QLineEdit* m_editArcRadius{nullptr};
    QLineEdit* m_editArcSweepAngle{nullptr};

    // ─── Kontextfelder (Rechts - schreibgeschützt) ───
    QLabel* m_lblStartX{nullptr};
    QLabel* m_lblStartY{nullptr};
    QLabel* m_lblStartZ{nullptr};

    // Status / Berechnungsanzeige
    QLabel* m_lblCalcStatus{nullptr};

    // ─── Untere Tabs [SCHRUPPEN] [SCHLICHTEN] [KÜHLMITTEL] ───
    QTabWidget* m_techTabs{nullptr};
    QComboBox* m_cmbTool{nullptr};
    QComboBox* m_cmbMillingType{nullptr}; // ON, INSIDE, OUTSIDE, POCKET
    QDoubleSpinBox* m_spinFeed{nullptr};
    QDoubleSpinBox* m_spinPlunge{nullptr};
    QDoubleSpinBox* m_spinRpm{nullptr};
    QDoubleSpinBox* m_spinPeckDepth{nullptr};
};

} // namespace GeminiCNC::UI

#endif // GEMINI_CNC_CONTOURSEGMENTEDITORDIALOG_H
