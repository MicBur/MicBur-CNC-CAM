#ifndef GEMINI_CNC_CONVERSATIONALEDITORDIALOG_H
#define GEMINI_CNC_CONVERSATIONALEDITORDIALOG_H

#include <QWidget>
#include <QListWidget>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QGroupBox>
#include <QLineEdit>
#include <QPushButton>
#include <QLabel>
#include <QStackedWidget>
#include <QTextEdit>
#include <QCheckBox>
#include <QTableWidget>
#include <QHeaderView>
#include <QFormLayout>

#include "cam/ConversationalProgram.h"
#include "core/ToolDefinition.h"
#include "core/MaterialDatabase.h"
#include "core/MachineConfig.h"
#include "geometry/Mesh.h"
#include "geometry/Contour.h"

namespace GeminiCNC::UI {

class ContourSegmentEditorDialog; // Forward-Deklaration

/**
 * @brief Hurco WinMax-artiger Arbeitsplan- und Programm-Editor.
 */
class ConversationalEditorDialog : public QWidget {
    Q_OBJECT
public:
    explicit ConversationalEditorDialog(QWidget* parent = nullptr);

    void setStockMesh(const Geometry::Mesh& mesh) { m_stockMesh = mesh; }
    void setPartMesh(const Geometry::Mesh& mesh);
    void setToolLibrary(const QList<Core::ToolDefinition>& tools);
    void setContours(const std::vector<Geometry::Contour>& contours) { m_contours = contours; }
    void setMachineConfig(const Core::MachineConfig& config) { m_machineConfig = config; }

    void applyPickedContour(int index, const Geometry::Contour& contour);

    [[nodiscard]] const CAM::Toolpath& currentToolpath() const { return m_currentToolpath; }
    [[nodiscard]] const CAM::ConversationalProgram& program() const { return m_program; }

    // Datensatz-Editor: Direkte Segment-Aktionen (aufrufbar von WinMax Softkeys)
    void showSegmentEditor();
    void hideSegmentEditor();
    void addContourSegment(Geometry::ContourSegmentType type);
    void navigateSegmentNext();
    void navigateSegmentPrev();
    void deleteCurrentSegment();
    void storeSegmentCalculatedValue();
    void findSegmentAlternativeSolution();
    [[nodiscard]] bool isSegmentEditorVisible() const;

signals:
    void toolpathGenerated(const CAM::Toolpath& toolpath);   // Anzeige/Vorschau
    void programCalculated(const CAM::Toolpath& toolpath);   // Gesamtprogramm neu berechnet → Simulation
    void pickingModeRequested(bool enabled);
    void promptChanged(const QString& promptText);
    void partMeshChanged(const Geometry::Mesh& mesh);
    void activeToolChanged(const Core::ToolDefinition& tool);
    void segmentEditorVisibilityChanged(bool visible);

public slots:
    void onAddBlockClicked(CAM::BlockType type);
    void onRemoveBlockClicked();
    void onMoveUpClicked();
    void onMoveDownClicked();
    void onDuplicateClicked();
    void onBlockSelectionChanged(int row);
    void onBlockItemChanged(QListWidgetItem* item);

    void onManageIslandsClicked();

    void onMaterialChanged(int index);
    void onToolChanged(int index);
    void onCalculateTechnology();

    void onPickContourClicked();
    void onEditContourSegmentsClicked();
    void onCalculateProgramClicked();
    void onSaveProgramClicked();
    void onLoadProgramClicked();

    void saveCurrentBlockFromUi();

private:
    void setupUi();
    void refreshBlockList();
    void loadBlockToUi(int index);
    [[nodiscard]] QString blockListLabel(int index) const; // mit Einrückung innerhalb von Mustern
    void updatePatternFieldVisibility();
    void syncBlockDepthFromSegments(CAM::ConversationalBlock& b); // Z START / Z UNTEN von Segment 0 → Block

    CAM::ConversationalProgram m_program;
    int m_selectedBlockIndex{0};


    Geometry::Mesh m_stockMesh;
    std::vector<Geometry::Contour> m_contours;
    QList<Core::ToolDefinition> m_toolLibrary;
    Core::MaterialDatabase m_materialDb;
    Core::MachineConfig m_machineConfig;
    CAM::Toolpath m_currentToolpath;

    bool m_isUpdatingUi{false};

    enum class SegmentEditorMode { BlockContour, PocketIsland };
    SegmentEditorMode m_segmentEditorMode{SegmentEditorMode::BlockContour};
    int m_editingIslandIndex{-1};

    // UI Widgets
    QListWidget* m_blockList{nullptr};

    QLineEdit* m_editName{nullptr};
    QComboBox* m_cmbTool{nullptr};
    QComboBox* m_cmbFinishTool{nullptr};

    QComboBox* m_cmbMaterial{nullptr};
    QDoubleSpinBox* m_spinRpm{nullptr};
    QDoubleSpinBox* m_spinFeed{nullptr};
    QDoubleSpinBox* m_spinPlunge{nullptr};

    QDoubleSpinBox* m_spinStartZ{nullptr};
    QDoubleSpinBox* m_spinTargetZ{nullptr};
    QDoubleSpinBox* m_spinStepDown{nullptr};
    QDoubleSpinBox* m_spinStepOver{nullptr};
    QDoubleSpinBox* m_spinClearanceZ{nullptr};
    QCheckBox* m_chkVisible{nullptr};

    // Kontextuelle Seiten
    QStackedWidget* m_stackParams{nullptr};

    // ─── Globale Optionen (unter Z-Ebenen) ───
    QDoubleSpinBox* m_spinPosX{nullptr};
    QDoubleSpinBox* m_spinPosY{nullptr};
    QComboBox* m_cmbMillDirection{nullptr};
    QCheckBox* m_chkCoolant{nullptr};
    QCheckBox* m_chkFinishPass{nullptr};
    QDoubleSpinBox* m_spinFinishStep{nullptr};
    QComboBox* m_cmbApproach{nullptr};

    // ─── Planfräsen Seite ───
    QDoubleSpinBox* m_spinFaceWidth{nullptr};
    QDoubleSpinBox* m_spinFaceDepth{nullptr};
    QCheckBox* m_chkUseStockDims{nullptr};

    // ─── Kontur Seite ───
    QComboBox* m_cmbContourSide{nullptr};
    QDoubleSpinBox* m_spinAllowance{nullptr};
    QDoubleSpinBox* m_spinLeadRadius{nullptr};
    QComboBox* m_cmbLeadType{nullptr};
    QCheckBox* m_chkUseTabs{nullptr};
    QDoubleSpinBox* m_spinTabWidth{nullptr};
    QDoubleSpinBox* m_spinTabHeight{nullptr};
    QSpinBox* m_spinTabCount{nullptr};
    QPushButton* m_btnPickContour{nullptr};
    QPushButton* m_btnEditSegments{nullptr};
    QLabel* m_lblContourStatus{nullptr};

    // ─── Hurco Block Header ───
    QLabel* m_lblBlockBigHeader{nullptr};

    // ─── Untere Technologie-Tabs (Hurco Style) ───
    QTabWidget* m_techTabWidget{nullptr};
    QComboBox* m_cmbMillingType{nullptr}; // ON, INSIDE, OUTSIDE, POCKET
    QLabel* m_lblMillingType{nullptr};    // Label für FRÄSART (ausblendbar)
    QComboBox* m_cmbStartSide{nullptr};   // BOTTOM, TOP, LEFT, RIGHT

    // ─── Tasche / Rahmen Seite (Hurco Frame) ───
    QComboBox* m_cmbPocketShape{nullptr};
    QDoubleSpinBox* m_spinPocketWidthX{nullptr};
    QDoubleSpinBox* m_spinPocketDepthY{nullptr};
    QDoubleSpinBox* m_spinPocketRadius{nullptr};
    QDoubleSpinBox* m_spinPocketCornerR{nullptr};
    QComboBox* m_cmbPocketStrategy{nullptr};

    // Schlichten-Optionen (Tasche)
    QGroupBox* m_grpFinishing{nullptr};
    QDoubleSpinBox* m_spinFinishAllowanceXY{nullptr};
    QDoubleSpinBox* m_spinFinishAllowanceZ{nullptr};
    QDoubleSpinBox* m_spinFinishFeed{nullptr};
    QDoubleSpinBox* m_spinFinishSpindle{nullptr};


    // Insel-Management (Tasche)
    QPushButton* m_btnManageIslands{nullptr};
    QLabel* m_lblIslandsCount{nullptr};

    // ─── Langloch Seite ───
    QDoubleSpinBox* m_spinSlotLength{nullptr};
    QDoubleSpinBox* m_spinSlotWidth{nullptr};
    QDoubleSpinBox* m_spinSlotAngle{nullptr};
    QDoubleSpinBox* m_spinSlotCornerR{nullptr};
    QSpinBox* m_spinSlotCount{nullptr};
    QDoubleSpinBox* m_spinSlotSpacing{nullptr};

    // ─── Helix / Gewinde Seite ───
    QDoubleSpinBox* m_spinHelixDia{nullptr};
    QDoubleSpinBox* m_spinHelixPitch{nullptr};
    QComboBox* m_cmbHelixType{nullptr};
    QComboBox* m_cmbHelixDir{nullptr};
    QSpinBox* m_spinHelixStarts{nullptr};

    // ─── Bohren Seite ───
    QComboBox* m_cmbDrillCycle{nullptr};
    QComboBox* m_cmbDrillPattern{nullptr};
    QStackedWidget* m_stackDrillPattern{nullptr};
    QDoubleSpinBox* m_spinPeckDepth{nullptr};
    QDoubleSpinBox* m_spinDwellTime{nullptr};
    QDoubleSpinBox* m_spinBoltRadius{nullptr};
    QSpinBox* m_spinBoltCount{nullptr};
    QDoubleSpinBox* m_spinBoltStartAngle{nullptr};
    QSpinBox* m_spinGridCols{nullptr};
    QSpinBox* m_spinGridRows{nullptr};
    QDoubleSpinBox* m_spinGridPitchX{nullptr};
    QDoubleSpinBox* m_spinGridPitchY{nullptr};
    // Lochreihe
    QSpinBox* m_spinLineCount{nullptr};
    QDoubleSpinBox* m_spinLineSpacing{nullptr};
    QDoubleSpinBox* m_spinLineAngle{nullptr};
    // Bogenreihe
    QDoubleSpinBox* m_spinArcRadius{nullptr};
    QSpinBox* m_spinArcCount{nullptr};
    QDoubleSpinBox* m_spinArcStartAngle{nullptr};
    QDoubleSpinBox* m_spinArcEndAngle{nullptr};
    // Rahmen
    QDoubleSpinBox* m_spinFrameWidth{nullptr};
    QDoubleSpinBox* m_spinFrameHeight{nullptr};
    QSpinBox* m_spinFrameCountX{nullptr};
    QSpinBox* m_spinFrameCountY{nullptr};
    // Manuell
    QTableWidget* m_tblManualPositions{nullptr};

    // ─── NC Seite ───
    QTextEdit* m_txtRawGCode{nullptr};

    // ─── Muster Seite (Muster Start) ───
    QFormLayout* m_patternForm{nullptr};
    QComboBox* m_cmbPatternType{nullptr};
    QSpinBox* m_spinPatternCountX{nullptr};
    QSpinBox* m_spinPatternCountY{nullptr};
    QDoubleSpinBox* m_spinPatternSpacingX{nullptr};
    QDoubleSpinBox* m_spinPatternSpacingY{nullptr};
    QDoubleSpinBox* m_spinPatternAngle{nullptr};
    QDoubleSpinBox* m_spinPatternStepAngle{nullptr};
    QDoubleSpinBox* m_spinPatternCenterX{nullptr};
    QDoubleSpinBox* m_spinPatternCenterY{nullptr};
    QCheckBox* m_chkPatternMirrorX{nullptr};
    QCheckBox* m_chkPatternMirrorY{nullptr};
    QLabel* m_lblPatternInfo{nullptr};

    // ─── 3D-STL Fräsen Seite ───
    QLabel* m_lblStlInfo{nullptr};
    QPushButton* m_btnLoadStlFile{nullptr};
    QPushButton* m_btnRotX{nullptr};
    QPushButton* m_btnRotY{nullptr};
    QPushButton* m_btnRotZ{nullptr};
    QPushButton* m_btnCenterOrigin{nullptr};
    QComboBox* m_cmbStlMode{nullptr};
    QComboBox* m_cmbStlDirection{nullptr};
    QDoubleSpinBox* m_spinStlRoughStepDown{nullptr};
    QDoubleSpinBox* m_spinStlFinishStepOver{nullptr};
    QDoubleSpinBox* m_spinStlAllowance{nullptr};
    QDoubleSpinBox* m_spinStlSampleStep{nullptr};
    QCheckBox* m_chkStlUseStockDims{nullptr};
    QPushButton* m_btnOptimizeStlParams{nullptr};
    QDoubleSpinBox* m_spinStlScale{nullptr};
    QPushButton* m_btnApplyStlScale{nullptr};

    void updateStlInfoLabel();
    void updateTargetZForCurrentMesh();

    QLabel* m_lblProgramSummary{nullptr};
    Geometry::Mesh m_partMesh;

    // Hurco WinMax Datensatz-Editor (eingebettet, Wechsel mit Master-Stack)
    QStackedWidget* m_masterStack{nullptr};   // Index 0 = Block-Editor, Index 1 = Segment-Datensatz-Editor
    QWidget* m_blockEditorPage{nullptr};       // Alle bisherigen Block-Widgets
    ContourSegmentEditorDialog* m_segmentEditor{nullptr};
};

} // namespace GeminiCNC::UI

#endif // GEMINI_CNC_CONVERSATIONALEDITORDIALOG_H
