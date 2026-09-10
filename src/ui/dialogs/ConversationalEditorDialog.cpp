#include "ConversationalEditorDialog.h"
#include "ContourSegmentEditorDialog.h"
#include "cam/CollisionDetector.h"
#include "geometry/StlLoader.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <QFileDialog>
#include <QMessageBox>

namespace GeminiCNC::UI {

ConversationalEditorDialog::ConversationalEditorDialog(QWidget* parent) : QWidget(parent) {
    m_program = CAM::ConversationalProgram::createSampleProgram();
    m_toolLibrary = Core::ToolDefinition::createDefaultLibrary();
    setupUi();
    refreshBlockList();
    loadBlockToUi(0);
}

void ConversationalEditorDialog::setToolLibrary(const QList<Core::ToolDefinition>& tools) {
    m_toolLibrary = tools;
    m_cmbTool->clear();
    if (m_cmbFinishTool) m_cmbFinishTool->clear();
    if (m_cmbFinishTool) m_cmbFinishTool->addItem("Wie Schruppwerkzeug", -1);
    
    for (const auto& t : m_toolLibrary) {
        QString text = QString("T%1: %2 (Ø %3mm)").arg(t.id).arg(t.name).arg(t.diameter, 0, 'f', 1);
        m_cmbTool->addItem(text, t.id);
        if (m_cmbFinishTool) m_cmbFinishTool->addItem(text, t.id);
    }
    loadBlockToUi(m_selectedBlockIndex);
}

void ConversationalEditorDialog::setPartMesh(const Geometry::Mesh& mesh) {
    m_partMesh = mesh;
    updateStlInfoLabel();
    updateTargetZForCurrentMesh();
}

void ConversationalEditorDialog::updateTargetZForCurrentMesh() {
    if (m_partMesh.isEmpty() || !m_spinTargetZ) return;
    double realDepth = m_partMesh.boundingBox.minPoint.z;
    if (m_stockMesh.boundingBox.isValid()) {
        realDepth = std::max(realDepth, m_stockMesh.boundingBox.minPoint.z);
    }
    m_spinTargetZ->setValue(realDepth);
    if (m_selectedBlockIndex >= 0 && m_selectedBlockIndex < static_cast<int>(m_program.size())) {
        if (m_program[m_selectedBlockIndex].type == CAM::BlockType::Stl3D) {
            m_program[m_selectedBlockIndex].targetZ = realDepth;
        }
    }
}

void ConversationalEditorDialog::updateStlInfoLabel() {
    if (!m_lblStlInfo) return;
    if (m_partMesh.isEmpty()) {
        m_lblStlInfo->setText(QStringLiteral("Kein 3D-Modell geladen (📂 STL laden)"));
        m_lblStlInfo->setStyleSheet("color: #E2E8F0; background-color: #4A5568; padding: 4px; border-radius: 3px; font-weight: normal;");
    } else {
        const auto& bbox = m_partMesh.boundingBox;
        QString text = QString("✔ STL: %1 Dreiecke | %2 x %3 x %4 mm")
            .arg(m_partMesh.triangleCount())
            .arg(bbox.widthX(), 0, 'f', 1)
            .arg(bbox.depthY(), 0, 'f', 1)
            .arg(bbox.heightZ(), 0, 'f', 1);

        if (bbox.heightZ() > 100.0 && (bbox.heightZ() > bbox.widthX() * 1.5 || bbox.heightZ() > bbox.depthY() * 1.5)) {
            text += QString("\n💡 TIPP: Modell steht aufrecht (%1 mm hoch)! Klicke auf '↻ +90° X' oder '↻ +90° Y', um es flach hinzulegen.").arg(bbox.heightZ(), 0, 'f', 1);
            m_lblStlInfo->setStyleSheet("color: #F6E05E; font-weight: bold; background-color: #2A2312; padding: 6px; border-radius: 3px; border: 1px solid #D69E2E;");
        } else {
            m_lblStlInfo->setStyleSheet("color: #48BB78; font-weight: bold; background-color: #1A202C; padding: 4px; border-radius: 3px; border: 1px solid #38A169;");
        }
        m_lblStlInfo->setText(text);
    }
}

void ConversationalEditorDialog::setupUi() {
    auto* outerLayout = new QVBoxLayout(this);
    outerLayout->setContentsMargins(0, 0, 0, 0);
    outerLayout->setSpacing(0);

    // Master-Stack: Seite 0 = Block-Editor, Seite 1 = Datensatz-Editor
    m_masterStack = new QStackedWidget(this);
    outerLayout->addWidget(m_masterStack);

    // ──── SEITE 0: Block-Editor (alle bisherigen Widgets) ────
    m_blockEditorPage = new QWidget(this);
    auto* mainLayout = new QVBoxLayout(m_blockEditorPage);
    mainLayout->setContentsMargins(4, 4, 4, 4);
    mainLayout->setSpacing(6);

    // 1. Obere WinMax-Aktionsleiste: Blöcke hinzufügen
    auto* topBtnLayout = new QHBoxLayout();
    auto makeAddBtn = [this, topBtnLayout](const QString& text, CAM::BlockType type, const QString& color) {
        auto* btn = new QPushButton(text, this);
        btn->setStyleSheet(QString("background-color: %1; color: white; font-weight: bold; font-size: 11px; padding: 5px; border-radius: 3px;").arg(color));
        connect(btn, &QPushButton::clicked, this, [this, type]() { onAddBlockClicked(type); });
        topBtnLayout->addWidget(btn);
    };

    makeAddBtn("+ Planen", CAM::BlockType::Facing, "#2B6CB0");
    makeAddBtn("+ Tasche", CAM::BlockType::Pocket, "#319795");
    makeAddBtn("+ Kontur", CAM::BlockType::Contour, "#D69E2E");
    makeAddBtn("+ Langloch", CAM::BlockType::Slot, "#DD6B20");
    makeAddBtn("+ Helix", CAM::BlockType::HelixThread, "#38A169");
    makeAddBtn("+ Bohren", CAM::BlockType::Drill, "#6B46C1");
    makeAddBtn("+ 3D-STL", CAM::BlockType::Stl3D, "#9F7AEA");
    makeAddBtn("+ NC", CAM::BlockType::RawNC, "#4A5568");
    mainLayout->addLayout(topBtnLayout);

    // 2. Arbeitsplan Block-Liste & Steuerknöpfe
    auto* listGroup = new QGroupBox(QStringLiteral("Arbeitsplan (Conversational Blocks)"), this);
    auto* listLayout = new QVBoxLayout(listGroup);

    m_blockList = new QListWidget(this);
    m_blockList->setStyleSheet("background-color: #1A202C; color: #EDF2F7; font-size: 12px; border-radius: 4px;");
    m_blockList->setFixedHeight(120);
    connect(m_blockList, &QListWidget::currentRowChanged, this, &ConversationalEditorDialog::onBlockSelectionChanged);
    connect(m_blockList, &QListWidget::itemChanged, this, &ConversationalEditorDialog::onBlockItemChanged);
    listLayout->addWidget(m_blockList);

    auto* manageLayout = new QHBoxLayout();
    auto* btnUp = new QPushButton(QStringLiteral("▲ Hoch"), this);
    auto* btnDown = new QPushButton(QStringLiteral("▼ Runter"), this);
    auto* btnDup = new QPushButton(QStringLiteral("Kopieren"), this);
    auto* btnDel = new QPushButton(QStringLiteral("Löschen"), this);

    btnDel->setStyleSheet("background-color: #E53E3E; color: white; border-radius: 3px;");
    connect(btnUp, &QPushButton::clicked, this, &ConversationalEditorDialog::onMoveUpClicked);
    connect(btnDown, &QPushButton::clicked, this, &ConversationalEditorDialog::onMoveDownClicked);
    connect(btnDup, &QPushButton::clicked, this, &ConversationalEditorDialog::onDuplicateClicked);
    connect(btnDel, &QPushButton::clicked, this, &ConversationalEditorDialog::onRemoveBlockClicked);

    manageLayout->addWidget(btnUp);
    manageLayout->addWidget(btnDown);
    manageLayout->addWidget(btnDup);
    manageLayout->addWidget(btnDel);
    listLayout->addLayout(manageLayout);

    mainLayout->addWidget(listGroup);

    // 3. Detail-Parameter des ausgewählten Blocks (Hurco WinMax Architektur)
    auto* detailGroup = new QGroupBox(QStringLiteral("Block-Parameter"), this);
    auto* detailLayout = new QVBoxLayout(detailGroup);
    detailLayout->setSpacing(8);

    // Hurco Block-Kopfzeile (z. B. BLOCK 1    MILL FRAME)
    m_lblBlockBigHeader = new QLabel(QStringLiteral("BLOCK 1    MILL FRAME"), this);
    m_lblBlockBigHeader->setStyleSheet(QStringLiteral(
        "font-size: 15px; font-weight: 900; color: #00D2FF; letter-spacing: 1px; padding: 4px 8px; background-color: #0E1620; border: 1px solid #1E2D3D; border-radius: 4px;"));
    detailLayout->addWidget(m_lblBlockBigHeader);

    m_editName = new QLineEdit(this);
    connect(m_editName, &QLineEdit::textChanged, this, &ConversationalEditorDialog::saveCurrentBlockFromUi);
    
    m_chkVisible = new QCheckBox(QStringLiteral("👁 3D-Ansicht"), this);
    connect(m_chkVisible, &QCheckBox::toggled, this, [this](bool visible) {
        saveCurrentBlockFromUi();
        if (m_selectedBlockIndex >= 0 && m_selectedBlockIndex < static_cast<int>(m_program.size())) {
            int currentId = m_program[m_selectedBlockIndex].id;
            for (auto& seg : m_currentToolpath.segments) {
                if (seg.blockId == currentId) {
                    seg.visible = visible;
                }
            }
            emit toolpathGenerated(m_currentToolpath);
        }
    });
    m_chkVisible->setToolTip(QStringLiteral("Werkzeugweg für diesen Block in der 3D-Ansicht ein-/ausblenden"));

    auto* nameRow = new QHBoxLayout();
    nameRow->addWidget(new QLabel(QStringLiteral("Name / Kommentar:")));
    nameRow->addWidget(m_editName, 1);
    nameRow->addWidget(m_chkVisible);
    detailLayout->addLayout(nameRow);

    // Z-Ebenen und Position (gemeinsam für Geometrie)
    auto* geomCommonLayout = new QHBoxLayout();
    m_spinPosX = new QDoubleSpinBox(this); m_spinPosX->setRange(-2000, 2000); m_spinPosX->setSuffix(" mm"); m_spinPosX->setDecimals(3);
    m_spinPosY = new QDoubleSpinBox(this); m_spinPosY->setRange(-2000, 2000); m_spinPosY->setSuffix(" mm"); m_spinPosY->setDecimals(3);
    connect(m_spinPosX, &QDoubleSpinBox::valueChanged, this, &ConversationalEditorDialog::saveCurrentBlockFromUi);
    connect(m_spinPosY, &QDoubleSpinBox::valueChanged, this, &ConversationalEditorDialog::saveCurrentBlockFromUi);

    m_spinStartZ = new QDoubleSpinBox(this); m_spinStartZ->setRange(-500, 500); m_spinStartZ->setSuffix(" mm"); m_spinStartZ->setDecimals(3);
    m_spinTargetZ = new QDoubleSpinBox(this); m_spinTargetZ->setRange(-500, 500); m_spinTargetZ->setSuffix(" mm"); m_spinTargetZ->setDecimals(3); m_spinTargetZ->setValue(-2.0);
    connect(m_spinStartZ, &QDoubleSpinBox::valueChanged, this, &ConversationalEditorDialog::saveCurrentBlockFromUi);
    connect(m_spinTargetZ, &QDoubleSpinBox::valueChanged, this, &ConversationalEditorDialog::saveCurrentBlockFromUi);

    geomCommonLayout->addWidget(new QLabel(QStringLiteral("X:"))); geomCommonLayout->addWidget(m_spinPosX);
    geomCommonLayout->addWidget(new QLabel(QStringLiteral("Y:"))); geomCommonLayout->addWidget(m_spinPosY);
    geomCommonLayout->addWidget(new QLabel(QStringLiteral("Z-Start:"))); geomCommonLayout->addWidget(m_spinStartZ);
    geomCommonLayout->addWidget(new QLabel(QStringLiteral("Z-Tiefe:"))); geomCommonLayout->addWidget(m_spinTargetZ);
    detailLayout->addLayout(geomCommonLayout);

    // 4. Kontext-spezifischer Stack für Block-Typen (Geometrie)
    m_stackParams = new QStackedWidget(this);

    auto makeSpinMM = [this](double min, double max, double val) {
        auto* sp = new QDoubleSpinBox(this);
        sp->setRange(min, max); sp->setValue(val); sp->setSuffix(" mm"); sp->setDecimals(3);
        connect(sp, &QDoubleSpinBox::valueChanged, this, &ConversationalEditorDialog::saveCurrentBlockFromUi);
        return sp;
    };

    // ════════════════════════════════════════════
    // Seite 0: Planfräsen (Facing)
    // ════════════════════════════════════════════
    auto* pageFacing = new QWidget(this);
    auto* lFacing = new QFormLayout(pageFacing);
    lFacing->setSpacing(6);

    auto* faceDimRow = new QHBoxLayout();
    m_spinFaceWidth = makeSpinMM(1, 2000, 100);
    m_spinFaceDepth = makeSpinMM(1, 2000, 80);
    faceDimRow->addWidget(new QLabel("Breite:")); faceDimRow->addWidget(m_spinFaceWidth);
    faceDimRow->addWidget(new QLabel("Tiefe:")); faceDimRow->addWidget(m_spinFaceDepth);
    lFacing->addRow(QStringLiteral("Bearbeitungsfläche:"), faceDimRow);

    m_chkUseStockDims = new QCheckBox(QStringLiteral("Rohteildimensionen verwenden"), this);
    m_chkUseStockDims->setChecked(true);
    connect(m_chkUseStockDims, &QCheckBox::toggled, this, [this](bool checked) {
        m_spinPosX->setEnabled(!checked);
        m_spinPosY->setEnabled(!checked);
        m_spinFaceWidth->setEnabled(!checked);
        m_spinFaceDepth->setEnabled(!checked);
        saveCurrentBlockFromUi();
    });
    lFacing->addRow(QString(), m_chkUseStockDims);

    m_spinStepOver = makeSpinMM(0.5, 100, 3);
    lFacing->addRow(QStringLiteral("Überlappung ae:"), m_spinStepOver);

    m_stackParams->addWidget(pageFacing); // Index 0

    // ════════════════════════════════════════════
    // Seite 1: Konturfräsen
    // ════════════════════════════════════════════
    auto* pageContour = new QWidget(this);
    auto* lContour = new QFormLayout(pageContour);
    lContour->setSpacing(6);

    m_cmbContourSide = new QComboBox(this);
    m_cmbContourSide->addItems({QStringLiteral("Außen"), QStringLiteral("Innen"), QStringLiteral("Auf Kontur")});
    connect(m_cmbContourSide, &QComboBox::currentIndexChanged, this, &ConversationalEditorDialog::saveCurrentBlockFromUi);
    lContour->addRow(QStringLiteral("Bahnkorrektur:"), m_cmbContourSide);

    m_spinAllowance = makeSpinMM(0, 10, 0.2);

    // Anfahrt
    auto* leadRow = new QHBoxLayout();
    m_cmbLeadType = new QComboBox(this);
    m_cmbLeadType->addItems({QStringLiteral("Direkt"), QStringLiteral("Tangentialbogen"), QStringLiteral("Senkrecht")});
    m_cmbLeadType->setCurrentIndex(1);
    connect(m_cmbLeadType, &QComboBox::currentIndexChanged, this, &ConversationalEditorDialog::saveCurrentBlockFromUi);
    m_spinLeadRadius = makeSpinMM(0.5, 50, 2);
    leadRow->addWidget(new QLabel("Typ:")); leadRow->addWidget(m_cmbLeadType);
    leadRow->addWidget(new QLabel("R:")); leadRow->addWidget(m_spinLeadRadius);
    lContour->addRow(QStringLiteral("An-/Abfahrt:"), leadRow);

    // Haltestege
    auto* tabRow = new QHBoxLayout();
    m_chkUseTabs = new QCheckBox(QStringLiteral("Stege"), this);
    connect(m_chkUseTabs, &QCheckBox::toggled, this, &ConversationalEditorDialog::saveCurrentBlockFromUi);
    m_spinTabCount = new QSpinBox(this); m_spinTabCount->setRange(1, 20); m_spinTabCount->setValue(4);
    connect(m_spinTabCount, &QSpinBox::valueChanged, this, &ConversationalEditorDialog::saveCurrentBlockFromUi);
    m_spinTabWidth = makeSpinMM(1, 30, 5);
    m_spinTabHeight = makeSpinMM(0.3, 5, 1);
    tabRow->addWidget(m_chkUseTabs);
    tabRow->addWidget(new QLabel("Anz:")); tabRow->addWidget(m_spinTabCount);
    tabRow->addWidget(new QLabel("B:")); tabRow->addWidget(m_spinTabWidth);
    tabRow->addWidget(new QLabel("H:")); tabRow->addWidget(m_spinTabHeight);
    lContour->addRow(QStringLiteral("Haltestege:"), tabRow);

    auto* pickLayout = new QHBoxLayout();
    m_btnPickContour = new QPushButton(QStringLiteral("⌖ Kontur anklicken"), this);
    m_btnPickContour->setStyleSheet("background-color: #D69E2E; color: white; font-weight: bold; padding: 4px; border-radius: 3px;");
    connect(m_btnPickContour, &QPushButton::clicked, this, &ConversationalEditorDialog::onPickContourClicked);

    auto* btnEditSegments = new QPushButton(QStringLiteral("✏ Datensätze bearbeiten"), this);
    btnEditSegments->setStyleSheet("background-color: #2B6CB0; color: white; font-weight: bold; padding: 4px; border-radius: 3px;");
    connect(btnEditSegments, &QPushButton::clicked, this, &ConversationalEditorDialog::onEditContourSegmentsClicked);

    m_lblContourStatus = new QLabel(QStringLiteral("Standard-Rechteck"), this);
    pickLayout->addWidget(m_btnPickContour);
    pickLayout->addWidget(btnEditSegments);
    pickLayout->addWidget(m_lblContourStatus);
    lContour->addRow(QStringLiteral("Kontur:"), pickLayout);

    m_stackParams->addWidget(pageContour); // Index 1

    // ════════════════════════════════════════════
    // Seite 2: Taschenfräsen
    // ════════════════════════════════════════════
    auto* pagePocket = new QWidget(this);
    auto* lPocket = new QFormLayout(pagePocket);
    lPocket->setSpacing(6);

    m_cmbPocketShape = new QComboBox(this);
    m_cmbPocketShape->addItems({QStringLiteral("Rechteck"), QStringLiteral("Kreis"), QStringLiteral("DXF-Kontur")});
    connect(m_cmbPocketShape, &QComboBox::currentIndexChanged, this, &ConversationalEditorDialog::saveCurrentBlockFromUi);
    lPocket->addRow(QStringLiteral("Geometrie:"), m_cmbPocketShape);

    auto* pDimLayout = new QHBoxLayout();
    m_spinPocketWidthX = makeSpinMM(1, 1000, 50);
    m_spinPocketDepthY = makeSpinMM(1, 1000, 30);
    m_spinPocketRadius = makeSpinMM(0, 500, 15);
    pDimLayout->addWidget(new QLabel("B:")); pDimLayout->addWidget(m_spinPocketWidthX);
    pDimLayout->addWidget(new QLabel("H:")); pDimLayout->addWidget(m_spinPocketDepthY);
    pDimLayout->addWidget(new QLabel("R:")); pDimLayout->addWidget(m_spinPocketRadius);
    lPocket->addRow(QStringLiteral("Abmessungen:"), pDimLayout);

    m_spinPocketCornerR = makeSpinMM(0, 50, 3);
    lPocket->addRow(QStringLiteral("Eckenradius:"), m_spinPocketCornerR);

    m_cmbStartSide = new QComboBox(this);
    m_cmbStartSide->addItems({QStringLiteral("UNTEN (BOTTOM)"), QStringLiteral("OBEN (TOP)"), QStringLiteral("LINKS (LEFT)"), QStringLiteral("RECHTS (RIGHT)")});
    connect(m_cmbStartSide, &QComboBox::currentIndexChanged, this, &ConversationalEditorDialog::saveCurrentBlockFromUi);
    lPocket->addRow(QStringLiteral("Startkante:"), m_cmbStartSide);

    m_cmbPocketStrategy = new QComboBox(this);
    m_cmbPocketStrategy->addItems({QStringLiteral("Zickzack"), QStringLiteral("Spiral (innen→außen)"), QStringLiteral("Konturparallel")});
    connect(m_cmbPocketStrategy, &QComboBox::currentIndexChanged, this, &ConversationalEditorDialog::saveCurrentBlockFromUi);
    lPocket->addRow(QStringLiteral("Räumstrategie:"), m_cmbPocketStrategy);

    // --- Insel-Management ---
    auto* islandLayout = new QHBoxLayout();
    m_lblIslandsCount = new QLabel("Keine Inseln definiert");
    m_btnManageIslands = new QPushButton("Inseln bearbeiten...");
    connect(m_btnManageIslands, &QPushButton::clicked, this, &ConversationalEditorDialog::onManageIslandsClicked);
    islandLayout->addWidget(m_lblIslandsCount);
    islandLayout->addWidget(m_btnManageIslands);
    lPocket->addRow(QStringLiteral("Inseln:"), islandLayout);

    // --- Schlicht-Optionen ---
    m_grpFinishing = new QGroupBox("Schlicht-Durchgang aktivieren", this);
    m_grpFinishing->setCheckable(true);
    m_grpFinishing->setChecked(false);
    connect(m_grpFinishing, &QGroupBox::toggled, this, &ConversationalEditorDialog::saveCurrentBlockFromUi);
    
    auto* lPocketFinish = new QFormLayout(m_grpFinishing);
    lPocketFinish->setSpacing(4);
    m_spinFinishAllowanceXY = makeSpinMM(0, 10, 0.5);
    m_spinFinishAllowanceZ = makeSpinMM(0, 10, 0.0);
    m_spinFinishFeed = makeSpinMM(10, 10000, 1000);
    m_spinFinishSpindle = makeSpinMM(100, 50000, 20000);
    
    m_cmbFinishTool = new QComboBox(this);
    // Werkzeuge werden in reloadToolBoxen oder so ähnlich aktualisiert? Wir können erstmal die gleichen reinladen.
    connect(m_cmbFinishTool, &QComboBox::currentIndexChanged, this, &ConversationalEditorDialog::saveCurrentBlockFromUi);

    lPocketFinish->addRow("Schlicht-Werkzeug:", m_cmbFinishTool);
    lPocketFinish->addRow("Aufmaß Seite (XY):", m_spinFinishAllowanceXY);
    lPocketFinish->addRow("Aufmaß Tiefe (Z):", m_spinFinishAllowanceZ);
    lPocketFinish->addRow("Schlicht-Vorschub:", m_spinFinishFeed);
    lPocketFinish->addRow("Schlicht-Drehzahl:", m_spinFinishSpindle);
    
    lPocket->addRow(m_grpFinishing);

    m_stackParams->addWidget(pagePocket); // Index 2

    // ════════════════════════════════════════════
    // Seite 3: Bohrbild
    // ════════════════════════════════════════════
    auto* pageDrill = new QWidget(this);
    auto* lDrill = new QFormLayout(pageDrill);
    lDrill->setSpacing(6);

    m_cmbDrillCycle = new QComboBox(this);
    m_cmbDrillCycle->addItems({
        QStringLiteral("Einfach (G81)"), QStringLiteral("Spanbruch (G83)"),
        QStringLiteral("Tiefloch (G73)"), QStringLiteral("Gewinde (G84)"),
        QStringLiteral("Ausbohren (G85)"), QStringLiteral("Ausspindeln (G86)")
    });
    connect(m_cmbDrillCycle, &QComboBox::currentIndexChanged, this, &ConversationalEditorDialog::saveCurrentBlockFromUi);
    lDrill->addRow(QStringLiteral("Bohrzyklus:"), m_cmbDrillCycle);

    m_cmbDrillPattern = new QComboBox(this);
    m_cmbDrillPattern->addItems({
        QStringLiteral("Einzelbohrung"),
        QStringLiteral("Lochkreis"),
        QStringLiteral("Lochraster"),
        QStringLiteral("Lochreihe"),
        QStringLiteral("Bogenreihe"),
        QStringLiteral("Rahmen"),
        QStringLiteral("Manuell")
    });
    connect(m_cmbDrillPattern, &QComboBox::currentIndexChanged, this, &ConversationalEditorDialog::saveCurrentBlockFromUi);
    lDrill->addRow(QStringLiteral("Muster:"), m_cmbDrillPattern);

    // ── Muster-Parameter als StackedWidget (zeigt nur aktives Muster) ──
    m_stackDrillPattern = new QStackedWidget(this);

    // Index 0: Einzelbohrung — keine Extra-Parameter
    m_stackDrillPattern->addWidget(new QWidget(this));

    // Index 1: Lochkreis
    auto* pageBoltCircle = new QWidget(this);
    auto* bCircleLayout = new QHBoxLayout(pageBoltCircle);
    bCircleLayout->setContentsMargins(0, 0, 0, 0);
    m_spinBoltRadius = makeSpinMM(1, 500, 25);
    m_spinBoltCount = new QSpinBox(this); m_spinBoltCount->setRange(1, 64); m_spinBoltCount->setValue(6);
    connect(m_spinBoltCount, &QSpinBox::valueChanged, this, &ConversationalEditorDialog::saveCurrentBlockFromUi);
    m_spinBoltStartAngle = makeSpinMM(0, 360, 0); m_spinBoltStartAngle->setSuffix("°"); m_spinBoltStartAngle->setDecimals(1);
    bCircleLayout->addWidget(new QLabel("R:")); bCircleLayout->addWidget(m_spinBoltRadius);
    bCircleLayout->addWidget(new QLabel("Anz:")); bCircleLayout->addWidget(m_spinBoltCount);
    bCircleLayout->addWidget(new QLabel("Start°:")); bCircleLayout->addWidget(m_spinBoltStartAngle);
    m_stackDrillPattern->addWidget(pageBoltCircle);

    // Index 2: Lochraster
    auto* pageGrid = new QWidget(this);
    auto* gridLayout = new QHBoxLayout(pageGrid);
    gridLayout->setContentsMargins(0, 0, 0, 0);
    m_spinGridCols = new QSpinBox(this); m_spinGridCols->setRange(1, 50); m_spinGridCols->setValue(3);
    m_spinGridRows = new QSpinBox(this); m_spinGridRows->setRange(1, 50); m_spinGridRows->setValue(2);
    m_spinGridPitchX = makeSpinMM(1, 500, 20);
    m_spinGridPitchY = makeSpinMM(1, 500, 20);
    connect(m_spinGridCols, &QSpinBox::valueChanged, this, &ConversationalEditorDialog::saveCurrentBlockFromUi);
    connect(m_spinGridRows, &QSpinBox::valueChanged, this, &ConversationalEditorDialog::saveCurrentBlockFromUi);
    gridLayout->addWidget(new QLabel("Sp:")); gridLayout->addWidget(m_spinGridCols);
    gridLayout->addWidget(new QLabel("Ze:")); gridLayout->addWidget(m_spinGridRows);
    gridLayout->addWidget(new QLabel("aX:")); gridLayout->addWidget(m_spinGridPitchX);
    gridLayout->addWidget(new QLabel("aY:")); gridLayout->addWidget(m_spinGridPitchY);
    m_stackDrillPattern->addWidget(pageGrid);

    // Index 3: Lochreihe
    auto* pageLine = new QWidget(this);
    auto* lineLayout = new QHBoxLayout(pageLine);
    lineLayout->setContentsMargins(0, 0, 0, 0);
    m_spinLineCount = new QSpinBox(this); m_spinLineCount->setRange(1, 100); m_spinLineCount->setValue(5);
    connect(m_spinLineCount, &QSpinBox::valueChanged, this, &ConversationalEditorDialog::saveCurrentBlockFromUi);
    m_spinLineSpacing = makeSpinMM(1, 500, 15);
    m_spinLineAngle = makeSpinMM(0, 360, 0); m_spinLineAngle->setSuffix("°"); m_spinLineAngle->setDecimals(1);
    lineLayout->addWidget(new QLabel("Anz:")); lineLayout->addWidget(m_spinLineCount);
    lineLayout->addWidget(new QLabel("Abst:")); lineLayout->addWidget(m_spinLineSpacing);
    lineLayout->addWidget(new QLabel("Winkel:")); lineLayout->addWidget(m_spinLineAngle);
    m_stackDrillPattern->addWidget(pageLine);

    // Index 4: Bogenreihe
    auto* pageArc = new QWidget(this);
    auto* arcLayout = new QHBoxLayout(pageArc);
    arcLayout->setContentsMargins(0, 0, 0, 0);
    m_spinArcRadius = makeSpinMM(1, 500, 30);
    m_spinArcCount = new QSpinBox(this); m_spinArcCount->setRange(2, 100); m_spinArcCount->setValue(5);
    connect(m_spinArcCount, &QSpinBox::valueChanged, this, &ConversationalEditorDialog::saveCurrentBlockFromUi);
    m_spinArcStartAngle = makeSpinMM(0, 360, 0); m_spinArcStartAngle->setSuffix("°"); m_spinArcStartAngle->setDecimals(1);
    m_spinArcEndAngle = makeSpinMM(0, 360, 180); m_spinArcEndAngle->setSuffix("°"); m_spinArcEndAngle->setDecimals(1);
    arcLayout->addWidget(new QLabel("R:")); arcLayout->addWidget(m_spinArcRadius);
    arcLayout->addWidget(new QLabel("Anz:")); arcLayout->addWidget(m_spinArcCount);
    arcLayout->addWidget(new QLabel("Von°:")); arcLayout->addWidget(m_spinArcStartAngle);
    arcLayout->addWidget(new QLabel("Bis°:")); arcLayout->addWidget(m_spinArcEndAngle);
    m_stackDrillPattern->addWidget(pageArc);

    // Index 5: Rahmen
    auto* pageFrame = new QWidget(this);
    auto* frameLayout = new QHBoxLayout(pageFrame);
    frameLayout->setContentsMargins(0, 0, 0, 0);
    m_spinFrameWidth = makeSpinMM(5, 2000, 60);
    m_spinFrameHeight = makeSpinMM(5, 2000, 40);
    m_spinFrameCountX = new QSpinBox(this); m_spinFrameCountX->setRange(2, 50); m_spinFrameCountX->setValue(3);
    m_spinFrameCountY = new QSpinBox(this); m_spinFrameCountY->setRange(0, 50); m_spinFrameCountY->setValue(2);
    connect(m_spinFrameCountX, &QSpinBox::valueChanged, this, &ConversationalEditorDialog::saveCurrentBlockFromUi);
    connect(m_spinFrameCountY, &QSpinBox::valueChanged, this, &ConversationalEditorDialog::saveCurrentBlockFromUi);
    frameLayout->addWidget(new QLabel("B:")); frameLayout->addWidget(m_spinFrameWidth);
    frameLayout->addWidget(new QLabel("H:")); frameLayout->addWidget(m_spinFrameHeight);
    frameLayout->addWidget(new QLabel("nX:")); frameLayout->addWidget(m_spinFrameCountX);
    frameLayout->addWidget(new QLabel("nY:")); frameLayout->addWidget(m_spinFrameCountY);
    m_stackDrillPattern->addWidget(pageFrame);

    // Index 6: Manuell — Positions-Tabelle (Hurco WinMax Style)
    auto* pageManual = new QWidget(this);
    auto* manualLayout = new QVBoxLayout(pageManual);
    manualLayout->setContentsMargins(0, 0, 0, 0);
    manualLayout->setSpacing(2);

    m_tblManualPositions = new QTableWidget(0, 3, this);
    m_tblManualPositions->setHorizontalHeaderLabels({QStringLiteral("X"), QStringLiteral("Y"), QStringLiteral("Z")});
    m_tblManualPositions->setFixedHeight(110);
    m_tblManualPositions->setStyleSheet(
        "QTableWidget { background-color: #1A202C; color: #EDF2F7; gridline-color: #2D3748; font-size: 11px; }"
        "QHeaderView::section { background-color: #2D3748; color: #A0AEC0; font-weight: bold; border: 1px solid #4A5568; padding: 2px; }"
        "QTableWidget::item { padding: 1px; }"
        "QTableWidget::item:selected { background-color: #2B6CB0; }");
    m_tblManualPositions->verticalHeader()->setDefaultSectionSize(22);
    m_tblManualPositions->horizontalHeader()->setStretchLastSection(true);
    m_tblManualPositions->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_tblManualPositions->setSelectionMode(QAbstractItemView::SingleSelection);
    connect(m_tblManualPositions, &QTableWidget::cellChanged, this, &ConversationalEditorDialog::saveCurrentBlockFromUi);
    manualLayout->addWidget(m_tblManualPositions);

    auto* manBtnRow = new QHBoxLayout();
    auto* btnAddPos = new QPushButton(QStringLiteral("+ Position"), this);
    auto* btnDelPos = new QPushButton(QStringLiteral("- Löschen"), this);
    auto* btnInsPos = new QPushButton(QStringLiteral("↑ Einfügen"), this);
    QString manBtnStyle = "padding: 3px 8px; font-size: 10px; border-radius: 3px;";
    btnAddPos->setStyleSheet(manBtnStyle + "background-color: #38A169; color: white;");
    btnDelPos->setStyleSheet(manBtnStyle + "background-color: #E53E3E; color: white;");
    btnInsPos->setStyleSheet(manBtnStyle + "background-color: #2B6CB0; color: white;");

    connect(btnAddPos, &QPushButton::clicked, this, [this]() {
        int row = m_tblManualPositions->rowCount();
        m_tblManualPositions->insertRow(row);
        m_tblManualPositions->setItem(row, 0, new QTableWidgetItem("0.000"));
        m_tblManualPositions->setItem(row, 1, new QTableWidgetItem("0.000"));
        m_tblManualPositions->setItem(row, 2, new QTableWidgetItem("0.000"));
        saveCurrentBlockFromUi();
    });
    connect(btnDelPos, &QPushButton::clicked, this, [this]() {
        int row = m_tblManualPositions->currentRow();
        if (row >= 0) {
            m_tblManualPositions->removeRow(row);
            saveCurrentBlockFromUi();
        }
    });
    connect(btnInsPos, &QPushButton::clicked, this, [this]() {
        int row = std::max(0, m_tblManualPositions->currentRow());
        m_tblManualPositions->insertRow(row);
        m_tblManualPositions->setItem(row, 0, new QTableWidgetItem("0.000"));
        m_tblManualPositions->setItem(row, 1, new QTableWidgetItem("0.000"));
        m_tblManualPositions->setItem(row, 2, new QTableWidgetItem("0.000"));
        saveCurrentBlockFromUi();
    });

    manBtnRow->addWidget(btnAddPos);
    manBtnRow->addWidget(btnInsPos);
    manBtnRow->addWidget(btnDelPos);
    manBtnRow->addStretch();
    manualLayout->addLayout(manBtnRow);
    m_stackDrillPattern->addWidget(pageManual);

    // Muster-Dropdown steuert StackedWidget
    connect(m_cmbDrillPattern, &QComboBox::currentIndexChanged,
            m_stackDrillPattern, &QStackedWidget::setCurrentIndex);
    m_stackDrillPattern->setCurrentIndex(0);

    lDrill->addRow(QStringLiteral("Parameter:"), m_stackDrillPattern);

    // ── Zyklusdaten ──
    auto* drillOptRow = new QHBoxLayout();
    m_spinPeckDepth = makeSpinMM(0.1, 50, 2);
    m_spinDwellTime = new QDoubleSpinBox(this); m_spinDwellTime->setRange(0, 10); m_spinDwellTime->setValue(0.5); m_spinDwellTime->setSuffix(" s"); m_spinDwellTime->setDecimals(2);
    connect(m_spinDwellTime, &QDoubleSpinBox::valueChanged, this, &ConversationalEditorDialog::saveCurrentBlockFromUi);
    drillOptRow->addWidget(new QLabel("Q (Span):")); drillOptRow->addWidget(m_spinPeckDepth);
    drillOptRow->addWidget(new QLabel("P (Verweil):")); drillOptRow->addWidget(m_spinDwellTime);
    lDrill->addRow(QStringLiteral("Zyklusdaten:"), drillOptRow);

    m_stackParams->addWidget(pageDrill); // Index 3

    // ════════════════════════════════════════════
    // Seite 4: Raw NC
    // ════════════════════════════════════════════
    auto* pageNC = new QWidget(this);
    auto* lNC = new QVBoxLayout(pageNC);
    m_txtRawGCode = new QTextEdit(this);
    m_txtRawGCode->setFixedHeight(60);
    m_txtRawGCode->setPlaceholderText(QStringLiteral("G-Code / Klipper-Makros hier eingeben..."));
    connect(m_txtRawGCode, &QTextEdit::textChanged, this, &ConversationalEditorDialog::saveCurrentBlockFromUi);
    lNC->addWidget(m_txtRawGCode);
    m_stackParams->addWidget(pageNC); // Index 4

    // ════════════════════════════════════════════
    // Seite 5: Langloch (Slot)
    // ════════════════════════════════════════════
    auto* pageSlot = new QWidget(this);
    auto* lSlot = new QFormLayout(pageSlot);
    lSlot->setSpacing(6);

    auto* slotDimRow = new QHBoxLayout();
    m_spinSlotLength = makeSpinMM(1, 1000, 50);
    m_spinSlotWidth = makeSpinMM(1, 200, 12);
    slotDimRow->addWidget(new QLabel("Länge:")); slotDimRow->addWidget(m_spinSlotLength);
    slotDimRow->addWidget(new QLabel("Breite:")); slotDimRow->addWidget(m_spinSlotWidth);
    lSlot->addRow(QStringLiteral("Nutmaße:"), slotDimRow);

    m_spinSlotAngle = makeSpinMM(0, 360, 0);
    m_spinSlotAngle->setSuffix("°"); m_spinSlotAngle->setDecimals(1);
    lSlot->addRow(QStringLiteral("Drehwinkel:"), m_spinSlotAngle);

    m_spinSlotCornerR = makeSpinMM(0, 50, 0);
    lSlot->addRow(QStringLiteral("Eckenradius:"), m_spinSlotCornerR);

    auto* slotRepeatRow = new QHBoxLayout();
    m_spinSlotCount = new QSpinBox(this); m_spinSlotCount->setRange(1, 50); m_spinSlotCount->setValue(1);
    connect(m_spinSlotCount, &QSpinBox::valueChanged, this, &ConversationalEditorDialog::saveCurrentBlockFromUi);
    m_spinSlotSpacing = makeSpinMM(1, 500, 25);
    slotRepeatRow->addWidget(new QLabel("Anzahl:")); slotRepeatRow->addWidget(m_spinSlotCount);
    slotRepeatRow->addWidget(new QLabel("Abstand:")); slotRepeatRow->addWidget(m_spinSlotSpacing);
    lSlot->addRow(QStringLiteral("Wiederholung:"), slotRepeatRow);

    m_stackParams->addWidget(pageSlot); // Index 5

    // ════════════════════════════════════════════
    // Seite 6: Helix / Gewindefräsen
    // ════════════════════════════════════════════
    auto* pageHelix = new QWidget(this);
    auto* lHelix = new QFormLayout(pageHelix);
    lHelix->setSpacing(6);

    m_spinHelixDia = makeSpinMM(1, 500, 20);
    lHelix->addRow(QStringLiteral("Durchmesser:"), m_spinHelixDia);

    m_spinHelixPitch = makeSpinMM(0.1, 50, 2);
    lHelix->addRow(QStringLiteral("Steigung (mm/U):"), m_spinHelixPitch);

    m_cmbHelixType = new QComboBox(this);
    m_cmbHelixType->addItems({QStringLiteral("Innengewinde / Bohrung"), QStringLiteral("Außengewinde / Zapfen")});
    connect(m_cmbHelixType, &QComboBox::currentIndexChanged, this, &ConversationalEditorDialog::saveCurrentBlockFromUi);
    lHelix->addRow(QStringLiteral("Typ:"), m_cmbHelixType);

    m_cmbHelixDir = new QComboBox(this);
    m_cmbHelixDir->addItems({QStringLiteral("Rechtsgewinde (CW)"), QStringLiteral("Linksgewinde (CCW)")});
    connect(m_cmbHelixDir, &QComboBox::currentIndexChanged, this, &ConversationalEditorDialog::saveCurrentBlockFromUi);
    lHelix->addRow(QStringLiteral("Drehrichtung:"), m_cmbHelixDir);

    m_spinHelixStarts = new QSpinBox(this); m_spinHelixStarts->setRange(1, 8); m_spinHelixStarts->setValue(1);
    connect(m_spinHelixStarts, &QSpinBox::valueChanged, this, &ConversationalEditorDialog::saveCurrentBlockFromUi);
    lHelix->addRow(QStringLiteral("Gänge:"), m_spinHelixStarts);

    m_stackParams->addWidget(pageHelix); // Index 6

    // ════════════════════════════════════════════
    // Seite 7: 3D-STL Freiformflächen-Fräsen
    // ════════════════════════════════════════════
    auto* pageStl = new QWidget(this);
    auto* lStl = new QFormLayout(pageStl);
    lStl->setSpacing(6);

    m_lblStlInfo = new QLabel(QStringLiteral("Kein 3D-Modell geladen"), this);
    m_lblStlInfo->setStyleSheet("color: #63B3ED; font-weight: bold; padding: 4px; background-color: #2D3748; border-radius: 3px;");

    m_btnLoadStlFile = new QPushButton(QStringLiteral("📂 STL-Datei laden / wechseln..."), this);
    m_btnLoadStlFile->setStyleSheet("background-color: #4A5568; color: white; padding: 4px; font-weight: bold; border-radius: 3px;");
    connect(m_btnLoadStlFile, &QPushButton::clicked, this, [this]() {
        QString path = QFileDialog::getOpenFileName(this, QStringLiteral("STL 3D-Modell laden"), QString(), QStringLiteral("STL Dateien (*.stl)"));
        if (!path.isEmpty()) {
            auto res = Geometry::StlLoader::loadFromFile(path, Geometry::MeshRole::TargetPart);
            if (res.success) {
                m_partMesh = res.mesh;
                m_partMesh.alignToOrigin(true, true);
                if (m_selectedBlockIndex >= 0 && m_selectedBlockIndex < static_cast<int>(m_program.size())) {
                    m_program[m_selectedBlockIndex].stlFilePath = path;
                    m_program[m_selectedBlockIndex].directStlMesh = m_partMesh;
                }
                updateStlInfoLabel();
                updateTargetZForCurrentMesh();
                emit partMeshChanged(m_partMesh);
                onCalculateProgramClicked();
            } else {
                QMessageBox::warning(this, QStringLiteral("STL-Import"), res.errorMessage);
            }
        }
    });

    auto* stlFileRow = new QHBoxLayout();
    stlFileRow->addWidget(m_lblStlInfo, 1);
    stlFileRow->addWidget(m_btnLoadStlFile);
    lStl->addRow(QStringLiteral("3D-Modell:"), stlFileRow);

    // Drehen & Ausrichten
    auto* rotLayout = new QHBoxLayout();
    m_btnRotX = new QPushButton(QStringLiteral("↻ +90° X"), this);
    m_btnRotY = new QPushButton(QStringLiteral("↻ +90° Y"), this);
    m_btnRotZ = new QPushButton(QStringLiteral("↻ +90° Z"), this);
    m_btnCenterOrigin = new QPushButton(QStringLiteral("⌖ Z=0 Oben / Zentrieren"), this);

    QString rotBtnStyle = "background-color: #2C5282; color: white; font-weight: bold; padding: 4px 6px; border-radius: 3px;";
    m_btnRotX->setStyleSheet(rotBtnStyle);
    m_btnRotY->setStyleSheet(rotBtnStyle);
    m_btnRotZ->setStyleSheet(rotBtnStyle);
    m_btnCenterOrigin->setStyleSheet("background-color: #276749; color: white; font-weight: bold; padding: 4px 6px; border-radius: 3px;");

    connect(m_btnRotX, &QPushButton::clicked, this, [this]() {
        if (!m_partMesh.isEmpty()) {
            m_partMesh.rotateX(90.0, true);
            m_partMesh.alignToOrigin(true, true);
            if (m_selectedBlockIndex >= 0 && m_selectedBlockIndex < static_cast<int>(m_program.size())) {
                m_program[m_selectedBlockIndex].directStlMesh = m_partMesh;
            }
            updateStlInfoLabel();
            updateTargetZForCurrentMesh();
            emit partMeshChanged(m_partMesh);
            onCalculateProgramClicked();
        }
    });
    connect(m_btnRotY, &QPushButton::clicked, this, [this]() {
        if (!m_partMesh.isEmpty()) {
            m_partMesh.rotateY(90.0, true);
            m_partMesh.alignToOrigin(true, true);
            if (m_selectedBlockIndex >= 0 && m_selectedBlockIndex < static_cast<int>(m_program.size())) {
                m_program[m_selectedBlockIndex].directStlMesh = m_partMesh;
            }
            updateStlInfoLabel();
            updateTargetZForCurrentMesh();
            emit partMeshChanged(m_partMesh);
            onCalculateProgramClicked();
        }
    });
    connect(m_btnRotZ, &QPushButton::clicked, this, [this]() {
        if (!m_partMesh.isEmpty()) {
            m_partMesh.rotateZ(90.0, true);
            m_partMesh.alignToOrigin(true, true);
            if (m_selectedBlockIndex >= 0 && m_selectedBlockIndex < static_cast<int>(m_program.size())) {
                m_program[m_selectedBlockIndex].directStlMesh = m_partMesh;
            }
            updateStlInfoLabel();
            updateTargetZForCurrentMesh();
            emit partMeshChanged(m_partMesh);
            onCalculateProgramClicked();
        }
    });
    connect(m_btnCenterOrigin, &QPushButton::clicked, this, [this]() {
        if (!m_partMesh.isEmpty()) {
            m_partMesh.alignToOrigin(true, true);
            if (m_selectedBlockIndex >= 0 && m_selectedBlockIndex < static_cast<int>(m_program.size())) {
                m_program[m_selectedBlockIndex].directStlMesh = m_partMesh;
            }
            updateStlInfoLabel();
            updateTargetZForCurrentMesh();
            emit partMeshChanged(m_partMesh);
            onCalculateProgramClicked();
        }
    });

    rotLayout->addWidget(m_btnRotX);
    rotLayout->addWidget(m_btnRotY);
    rotLayout->addWidget(m_btnRotZ);
    rotLayout->addWidget(m_btnCenterOrigin);
    lStl->addRow(QStringLiteral("3D-Drehung:"), rotLayout);

    // Skalierung (Größe anpassen)
    auto* scaleLayout = new QHBoxLayout();
    m_spinStlScale = new QDoubleSpinBox(this);
    m_spinStlScale->setRange(0.01, 100.0);
    m_spinStlScale->setValue(1.0);
    m_spinStlScale->setSingleStep(0.1);
    m_spinStlScale->setDecimals(3);
    m_spinStlScale->setSuffix(QStringLiteral(" ×"));
    m_spinStlScale->setStyleSheet("font-weight: bold; padding: 3px;");

    m_btnApplyStlScale = new QPushButton(QStringLiteral("Anwenden"), this);
    m_btnApplyStlScale->setStyleSheet("background-color: #DD6B20; color: white; font-weight: bold; padding: 4px 8px; border-radius: 3px;");
    connect(m_btnApplyStlScale, &QPushButton::clicked, this, [this]() {
        if (m_partMesh.isEmpty()) return;
        double factor = m_spinStlScale->value();
        if (std::abs(factor - 1.0) < 1e-6) return;
        m_partMesh.scaleUniform(factor);
        m_partMesh.alignToOrigin(true, true);
        if (m_selectedBlockIndex >= 0 && m_selectedBlockIndex < static_cast<int>(m_program.size())) {
            m_program[m_selectedBlockIndex].directStlMesh = m_partMesh;
        }
        m_spinStlScale->setValue(1.0);
        updateStlInfoLabel();
        updateTargetZForCurrentMesh();
        emit partMeshChanged(m_partMesh);
        onCalculateProgramClicked();
    });

    scaleLayout->addWidget(m_spinStlScale);
    scaleLayout->addWidget(m_btnApplyStlScale);
    lStl->addRow(QStringLiteral("Größe:"), scaleLayout);

    // CAM Strategie
    m_cmbStlMode = new QComboBox(this);
    m_cmbStlMode->addItems({
        QStringLiteral("1. Komplett: Z-Ebenen Schruppen + Freiform-Schlichten X (Empfohlen)"),
        QStringLiteral("2. Komplett: Z-Ebenen Schruppen + Freiform-Schlichten Y"),
        QStringLiteral("3. Nur Z-Ebenen Schruppen (Waterline)"),
        QStringLiteral("4. Nur Freiform-Schlichten (Parallel X)"),
        QStringLiteral("5. Nur Freiform-Schlichten (Parallel Y)"),
        QStringLiteral("6. Perfekte Oberfläche: Kreuzraster-Schlichten (X + Y)"),
        QStringLiteral("7. Steile Wände: Z-Ebenen-Schlichten (Waterline Finish)")
    });
    connect(m_cmbStlMode, &QComboBox::currentIndexChanged, this, &ConversationalEditorDialog::saveCurrentBlockFromUi);
    lStl->addRow(QStringLiteral("CAM-Strategie:"), m_cmbStlMode);

    // Schnittparameter
    auto* stlParamRow = new QHBoxLayout();
    m_spinStlRoughStepDown = makeSpinMM(0.1, 50, 2.0);
    m_spinStlFinishStepOver = makeSpinMM(0.05, 50, 1.0);
    m_spinStlAllowance = makeSpinMM(0.0, 10, 0.0);
    m_spinStlSampleStep = makeSpinMM(0.1, 20, 0.8);

    stlParamRow->addWidget(new QLabel("ap (Z):")); stlParamRow->addWidget(m_spinStlRoughStepDown);
    stlParamRow->addWidget(new QLabel("ae (Zeile):")); stlParamRow->addWidget(m_spinStlFinishStepOver);
    stlParamRow->addWidget(new QLabel("Aufmaß:")); stlParamRow->addWidget(m_spinStlAllowance);
    stlParamRow->addWidget(new QLabel("dx:")); stlParamRow->addWidget(m_spinStlSampleStep);
    lStl->addRow(QStringLiteral("Schnittdaten:"), stlParamRow);

    auto* stlOptRow = new QHBoxLayout();
    m_chkStlUseStockDims = new QCheckBox(QStringLiteral("Gesamtes Rohteil abfahren (Standard: Nur Bauteilbereich)"), this);
    m_chkStlUseStockDims->setChecked(false);
    connect(m_chkStlUseStockDims, &QCheckBox::toggled, this, &ConversationalEditorDialog::saveCurrentBlockFromUi);

    m_btnOptimizeStlParams = new QPushButton(QStringLiteral("⚡ Automatisch optimal anpassen"), this);
    m_btnOptimizeStlParams->setStyleSheet("background-color: #805AD5; color: white; font-weight: bold; padding: 4px 8px; border-radius: 3px;");
    connect(m_btnOptimizeStlParams, &QPushButton::clicked, this, [this]() {
        int toolId = m_cmbTool->currentData().toInt();
        double toolDia = 6.0;
        for (const auto& t : m_toolLibrary) {
            if (t.id == toolId) {
                toolDia = t.diameter;
                break;
            }
        }
        m_spinStlFinishStepOver->setValue(std::max(0.5, toolDia * 0.25));
        m_spinStlRoughStepDown->setValue(std::max(1.0, toolDia * 0.5));
        m_spinStlSampleStep->setValue(std::max(0.5, toolDia * 0.15));
        saveCurrentBlockFromUi();
        onCalculateProgramClicked();
    });

    stlOptRow->addWidget(m_chkStlUseStockDims);
    stlOptRow->addWidget(m_btnOptimizeStlParams);
    lStl->addRow(QStringLiteral("Optionen:"), stlOptRow);

    m_stackParams->addWidget(pageStl); // Index 7

    detailLayout->addWidget(m_stackParams);

    // ─── Untere Technologie-Tabs (Hurco WinMax Vorbild) ───
    m_techTabWidget = new QTabWidget(this);

    // Tab 1: Schruppen (Roughing)
    auto* tabRough = new QWidget(this);
    auto* lRough = new QFormLayout(tabRough);
    lRough->setSpacing(6);

    m_cmbTool = new QComboBox(this);
    for (const auto& t : m_toolLibrary) {
        m_cmbTool->addItem(QString("T%1: %2 (Ø %3mm)").arg(t.id).arg(t.name).arg(t.diameter, 0, 'f', 1), t.id);
    }
    connect(m_cmbTool, &QComboBox::currentIndexChanged, this, &ConversationalEditorDialog::onToolChanged);
    lRough->addRow(QStringLiteral("WERKZEUG:"), m_cmbTool);

    m_cmbMillingType = new QComboBox(this);
    m_cmbMillingType->addItems({
        QStringLiteral("AUF KONTUR (ON)"),
        QStringLiteral("INNEN (INSIDE)"),
        QStringLiteral("AUSSEN (OUTSIDE)"),
        QStringLiteral("TASCHE (POCKET)")
    });
    connect(m_cmbMillingType, &QComboBox::currentIndexChanged, this, &ConversationalEditorDialog::saveCurrentBlockFromUi);
    m_lblMillingType = new QLabel(QStringLiteral("FRÄSART:"), this);
    lRough->addRow(m_lblMillingType, m_cmbMillingType);

    auto* roughFeedRow = new QHBoxLayout();
    m_spinFeed = makeSpinMM(10, 10000, 1500); m_spinFeed->setSuffix(" mm/min");
    m_spinPlunge = makeSpinMM(10, 5000, 500); m_spinPlunge->setSuffix(" mm/min");
    roughFeedRow->addWidget(new QLabel("Vorschub F:")); roughFeedRow->addWidget(m_spinFeed);
    roughFeedRow->addWidget(new QLabel("Eintauchen Fz:")); roughFeedRow->addWidget(m_spinPlunge);
    lRough->addRow(QStringLiteral("GESCHWINDIGKEIT:"), roughFeedRow);

    auto* roughCutRow = new QHBoxLayout();
    m_spinRpm = makeSpinMM(100, 60000, 18000); m_spinRpm->setSuffix(" U/min");
    m_spinStepDown = makeSpinMM(0.1, 50, 2.0);
    roughCutRow->addWidget(new QLabel("Drehzahl S:")); roughCutRow->addWidget(m_spinRpm);
    roughCutRow->addWidget(new QLabel("Zustellung ap:")); roughCutRow->addWidget(m_spinStepDown);
    lRough->addRow(QStringLiteral("SCHNITTWERTE:"), roughCutRow);

    m_techTabWidget->addTab(tabRough, QStringLiteral("SCHRUPPEN (ROUGHING)"));

    // Tab 2: Schlichten (Finishing)
    auto* tabFinish = new QWidget(this);
    auto* lFinish = new QFormLayout(tabFinish);
    lFinish->setSpacing(6);
    m_chkFinishPass = new QCheckBox(QStringLiteral("Schlichtgang aktivieren"), this);
    connect(m_chkFinishPass, &QCheckBox::toggled, this, &ConversationalEditorDialog::saveCurrentBlockFromUi);
    m_spinFinishStep = makeSpinMM(0.01, 5, 0.2);
    
    m_cmbFinishTool = new QComboBox(this);
    m_cmbFinishTool->addItem(QStringLiteral("== Wie Schrupp-Werkzeug =="), -1);
    for (const auto& t : m_toolLibrary) {
        m_cmbFinishTool->addItem(QString("T%1: %2 (Ø %3mm)").arg(t.id).arg(t.name).arg(t.diameter, 0, 'f', 1), t.id);
    }
    connect(m_cmbFinishTool, &QComboBox::currentIndexChanged, this, &ConversationalEditorDialog::saveCurrentBlockFromUi);

    lFinish->addRow(m_chkFinishPass);
    lFinish->addRow(QStringLiteral("Schlicht-Werkzeug:"), m_cmbFinishTool);
    lFinish->addRow(QStringLiteral("Schlichtaufmaß:"), m_spinAllowance);
    lFinish->addRow(QStringLiteral("Schlicht-ap:"), m_spinFinishStep);
    m_techTabWidget->addTab(tabFinish, QStringLiteral("SCHLICHTEN (FINISHING)"));

    // Tab 3: Kühlmittel & SFQ
    auto* tabCool = new QWidget(this);
    auto* lCool = new QFormLayout(tabCool);
    lCool->setSpacing(6);
    m_chkCoolant = new QCheckBox(QStringLiteral("Kühlmittel aktiv"), this);
    m_chkCoolant->setChecked(true);
    connect(m_chkCoolant, &QCheckBox::toggled, this, &ConversationalEditorDialog::saveCurrentBlockFromUi);
    m_cmbMillDirection = new QComboBox(this);
    m_cmbMillDirection->addItems({QStringLiteral("Gleichlauf (Climb)"), QStringLiteral("Gegenlauf (Conventional)")});
    connect(m_cmbMillDirection, &QComboBox::currentIndexChanged, this, &ConversationalEditorDialog::saveCurrentBlockFromUi);
    m_cmbApproach = new QComboBox(this);
    m_cmbApproach->addItems({QStringLiteral("Direkt"), QStringLiteral("Tangential"), QStringLiteral("Rampe")});
    connect(m_cmbApproach, &QComboBox::currentIndexChanged, this, &ConversationalEditorDialog::saveCurrentBlockFromUi);
    m_spinClearanceZ = makeSpinMM(0.5, 200, 5.0);
    lCool->addRow(m_chkCoolant);
    lCool->addRow(QStringLiteral("Fräsrichtung:"), m_cmbMillDirection);
    lCool->addRow(QStringLiteral("Anfahrtyp:"), m_cmbApproach);
    lCool->addRow(QStringLiteral("Sicherheitsebene:"), m_spinClearanceZ);
    m_techTabWidget->addTab(tabCool, QStringLiteral("KÜHLMITTEL & SFQ"));

    // Tab 4: Werkstoff
    auto* tabMat = new QWidget(this);
    auto* lMat = new QFormLayout(tabMat);
    lMat->setSpacing(6);
    m_cmbMaterial = new QComboBox(this);
    for (const auto& mat : m_materialDb.materials()) {
        m_cmbMaterial->addItem(QString("%1 (%2)").arg(mat.name).arg(mat.category), mat.id);
    }
    connect(m_cmbMaterial, &QComboBox::currentIndexChanged, this, &ConversationalEditorDialog::onMaterialChanged);
    lMat->addRow(QStringLiteral("Werkstoff (Vc/fz):"), m_cmbMaterial);
    m_techTabWidget->addTab(tabMat, QStringLiteral("WERKSTOFF (MATERIAL)"));

    detailLayout->addWidget(m_techTabWidget);
    mainLayout->addWidget(detailGroup);

    // 5. Haupt-Aktion: Gesamtprogramm berechnen
    auto* btnCalcProg = new QPushButton(QStringLiteral("⚡ Gesamtprogramm berechnen (Alle Blöcke)"), this);
    btnCalcProg->setStyleSheet("padding: 10px; font-weight: bold; background-color: #DD6B20; color: white; border-radius: 4px; font-size: 13px;");
    connect(btnCalcProg, &QPushButton::clicked, this, &ConversationalEditorDialog::onCalculateProgramClicked);
    mainLayout->addWidget(btnCalcProg);

    auto* progFileLayout = new QHBoxLayout();
    auto* btnSaveProg = new QPushButton(QStringLiteral("💾 Programm speichern (.gprog)"), this);
    auto* btnLoadProg = new QPushButton(QStringLiteral("📂 Programm laden (.gprog)"), this);
    connect(btnSaveProg, &QPushButton::clicked, this, &ConversationalEditorDialog::onSaveProgramClicked);
    connect(btnLoadProg, &QPushButton::clicked, this, &ConversationalEditorDialog::onLoadProgramClicked);
    progFileLayout->addWidget(btnSaveProg);
    progFileLayout->addWidget(btnLoadProg);
    mainLayout->addLayout(progFileLayout);

    m_lblProgramSummary = new QLabel(QStringLiteral("Keine Fräsbahnen berechnet."), this);
    m_lblProgramSummary->setStyleSheet("background-color: #1A202C; padding: 6px; border-radius: 4px; color: #CBD5E0; font-family: Consolas, monospace;");
    m_lblProgramSummary->setWordWrap(true);
    mainLayout->addWidget(m_lblProgramSummary);

    mainLayout->addStretch(1);

    // ──── Master-Stack befüllen ────
    m_masterStack->addWidget(m_blockEditorPage);  // Index 0: Block-Editor

    // ──── SEITE 1: Hurco WinMax Datensatz-Editor ────
    m_segmentEditor = new ContourSegmentEditorDialog(this);
    m_masterStack->addWidget(m_segmentEditor);     // Index 1: Segment-Datensatz-Editor

    // Segment-Editor → Live-Kontur-Update → Toolpath senden
    connect(m_segmentEditor, &ContourSegmentEditorDialog::contourUpdated, this, [this](const Geometry::Contour& contour) {
        if (m_selectedBlockIndex >= 0 && m_selectedBlockIndex < static_cast<int>(m_program.size())) {
            auto& b = m_program[m_selectedBlockIndex];
            if (m_segmentEditorMode == SegmentEditorMode::BlockContour) {
                b.contour = contour;
                b.segments = m_segmentEditor->segments();
            } else if (m_segmentEditorMode == SegmentEditorMode::PocketIsland) {
                // Live preview of island could be sent here, but for now we just update the data
                if (m_editingIslandIndex >= 0 && m_editingIslandIndex < static_cast<int>(b.pocketIslands.size())) {
                    b.pocketIslands[m_editingIslandIndex] = m_segmentEditor->segments();
                }
            }
            Core::ToolDefinition activeTool(1, "Tool", Core::ToolType::EndMill, 6.0);
            Core::ToolDefinition finishTool = activeTool;
            if (!m_toolLibrary.isEmpty()) activeTool = m_toolLibrary.first();
            for (const auto& t : m_toolLibrary) {
                if (t.id == b.toolId) activeTool = t;
                if (t.id == b.finishToolId) finishTool = t;
            }
            if (b.finishToolId <= 0) finishTool = activeTool;
            emit toolpathGenerated(b.generateToolpath(activeTool, finishTool, m_stockMesh.boundingBox, m_partMesh));
        }
    });

    // Segment-Editor → Kontext-Prompt weiterleiten
    connect(m_segmentEditor, &ContourSegmentEditorDialog::promptChanged, this, &ConversationalEditorDialog::promptChanged);

    // Segment-Editor → "Kontur übernehmen" → zurück zum Block-Editor
    connect(m_segmentEditor, &ContourSegmentEditorDialog::accepted, this, [this]() {
        if (m_selectedBlockIndex >= 0 && m_selectedBlockIndex < static_cast<int>(m_program.size())) {
            auto& b = m_program[m_selectedBlockIndex];
            if (m_segmentEditorMode == SegmentEditorMode::BlockContour) {
                b.segments = m_segmentEditor->segments();
                b.contour = m_segmentEditor->compiledContour();
                m_lblContourStatus->setText(QString("%1 Schritte (%2 Pkt)").arg(b.segments.size()).arg(b.contour.points.size()));
            } else if (m_segmentEditorMode == SegmentEditorMode::PocketIsland) {
                if (m_editingIslandIndex >= 0 && m_editingIslandIndex < static_cast<int>(b.pocketIslands.size())) {
                    b.pocketIslands[m_editingIslandIndex] = m_segmentEditor->segments();
                }
                loadBlockToUi(m_selectedBlockIndex); // Refresh UI to update island count label
            }
        }
        m_masterStack->setCurrentIndex(0);  // Zurück zum Block-Editor
        emit segmentEditorVisibilityChanged(false);
        onCalculateProgramClicked();
    });

    m_masterStack->setCurrentIndex(0);
}

void ConversationalEditorDialog::refreshBlockList() {
    m_isUpdatingUi = true;
    m_blockList->clear();

    for (size_t i = 0; i < m_program.size(); ++i) {
        const auto& b = m_program[i];
        auto* item = new QListWidgetItem(m_blockList);
        item->setText(QString("[%1] %2").arg(CAM::blockTypeToString(b.type)).arg(b.name));
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        item->setCheckState(b.enabled ? Qt::Checked : Qt::Unchecked);
    }

    if (m_selectedBlockIndex >= static_cast<int>(m_program.size())) {
        m_selectedBlockIndex = static_cast<int>(m_program.size()) - 1;
    }
    if (m_selectedBlockIndex >= 0) {
        m_blockList->setCurrentRow(m_selectedBlockIndex);
    }
    m_isUpdatingUi = false;
}

void ConversationalEditorDialog::loadBlockToUi(int index) {
    if (index < 0 || index >= static_cast<int>(m_program.size())) return;
    m_isUpdatingUi = true;
    m_selectedBlockIndex = index;

    const auto& b = m_program[index];
    m_editName->setText(b.name);
    m_chkVisible->setChecked(b.visible);

    // Werkzeug auswählen
    for (int i = 0; i < m_cmbTool->count(); ++i) {
        if (m_cmbTool->itemData(i).toInt() == b.toolId) {
            m_cmbTool->setCurrentIndex(i);
            break;
        }
    }
    
    // Schlicht-Werkzeug auswählen
    m_cmbFinishTool->setCurrentIndex(0); // Default == Wie Schrupp-Werkzeug ==
    for (int i = 0; i < m_cmbFinishTool->count(); ++i) {
        if (m_cmbFinishTool->itemData(i).toInt() == b.finishToolId) {
            m_cmbFinishTool->setCurrentIndex(i);
            break;
        }
    }

    // Material auswählen
    for (int i = 0; i < m_cmbMaterial->count(); ++i) {
        if (m_cmbMaterial->itemData(i).toInt() == b.materialId) {
            m_cmbMaterial->setCurrentIndex(i);
            break;
        }
    }

    m_lblBlockBigHeader->setText(QString("BLOCK %1    %2").arg(index + 1).arg(CAM::blockTypeToString(b.type).toUpper()));
    m_cmbMillingType->setCurrentIndex(static_cast<int>(b.millingType));
    m_cmbStartSide->setCurrentIndex(static_cast<int>(b.startSide));

    m_spinRpm->setValue(b.spindleRpm);
    m_spinFeed->setValue(b.feedRate);
    m_spinPlunge->setValue(b.plungeFeedRate);

    m_spinStartZ->setValue(b.startZ);
    m_spinTargetZ->setValue(b.targetZ);
    m_spinStepDown->setValue(b.stepDown);
    m_spinStepOver->setValue(b.stepOver);
    m_spinClearanceZ->setValue(b.clearanceZ);

    m_spinPosX->setValue(b.posX);
    m_spinPosY->setValue(b.posY);
    m_cmbMillDirection->setCurrentIndex(b.millingDirection);
    m_chkCoolant->setChecked(b.coolantOn);
    m_chkFinishPass->setChecked(b.finishPass);
    m_spinFinishStep->setValue(b.finishStepDown);
    m_cmbApproach->setCurrentIndex(b.approachType);

    // Fräsart-Dropdown nur bei relevanten Blocktypen anzeigen
    bool showMillingType = (b.type == CAM::BlockType::Contour ||
                            b.type == CAM::BlockType::Pocket ||
                            b.type == CAM::BlockType::Slot);
    m_cmbMillingType->setVisible(showMillingType);
    m_lblMillingType->setVisible(showMillingType);

    // Kontextseite einstellen
    switch (b.type) {
        case CAM::BlockType::Facing:
            m_stackParams->setCurrentIndex(0);
            m_spinFaceWidth->setValue(b.areaWidth);
            m_spinFaceDepth->setValue(b.areaDepth);
            m_chkUseStockDims->setChecked(b.useStockDimensions);
            break;
        case CAM::BlockType::Contour:
            m_stackParams->setCurrentIndex(1);
            m_cmbContourSide->setCurrentIndex(static_cast<int>(b.contourSide));
            m_spinAllowance->setValue(b.finishAllowance);
            m_cmbLeadType->setCurrentIndex(b.leadType);
            m_spinLeadRadius->setValue(b.leadRadius);
            m_chkUseTabs->setChecked(b.useTabs);
            m_spinTabCount->setValue(b.tabCount);
            m_spinTabWidth->setValue(b.tabWidth);
            m_spinTabHeight->setValue(b.tabHeight);
            m_lblContourStatus->setText(b.contour.empty() ? QStringLiteral("Standard-Rechteck") : QString("%1 Punkte").arg(b.contour.points.size()));
            break;
        case CAM::BlockType::Pocket:
            m_stackParams->setCurrentIndex(2);
            m_cmbPocketShape->setCurrentIndex(static_cast<int>(b.pocketShape));
            m_spinPocketWidthX->setValue(b.pocketWidthX);
            m_spinPocketDepthY->setValue(b.pocketDepthY);
            m_spinPocketRadius->setValue(b.pocketRadius);
            m_spinPocketCornerR->setValue(b.pocketCornerR);
            m_cmbPocketStrategy->setCurrentIndex(b.pocketStrategy);
            
            // Finishing
            m_grpFinishing->setChecked(b.enableFinishing);
            m_spinFinishAllowanceXY->setValue(b.finishAllowanceXY);
            m_spinFinishAllowanceZ->setValue(b.finishAllowanceZ);
            m_spinFinishFeed->setValue(b.finishFeedRate);
            m_spinFinishSpindle->setValue(b.finishSpindleRpm);
            
            // Island count
            if (b.pocketIslands.empty()) {
                m_lblIslandsCount->setText("Keine Inseln definiert");
            } else {
                m_lblIslandsCount->setText(QString("%1 Insel(n)").arg(b.pocketIslands.size()));
            }
            break;
        case CAM::BlockType::Drill:
            m_stackParams->setCurrentIndex(3);
            m_cmbDrillCycle->setCurrentIndex(b.drillCycle);
            m_cmbDrillPattern->setCurrentIndex(static_cast<int>(b.drillPattern));
            m_spinBoltRadius->setValue(b.boltCircleRadius);
            m_spinBoltCount->setValue(b.boltCircleHoleCount);
            m_spinBoltStartAngle->setValue(b.boltCircleStartAngle);
            m_spinPeckDepth->setValue(b.peckDepth);
            m_spinDwellTime->setValue(b.dwellTimeSec);
            m_spinGridCols->setValue(b.gridCols);
            m_spinGridRows->setValue(b.gridRows);
            m_spinGridPitchX->setValue(b.gridPitchX);
            m_spinGridPitchY->setValue(b.gridPitchY);
            // Lochreihe
            m_spinLineCount->setValue(b.lineHoleCount);
            m_spinLineSpacing->setValue(b.lineSpacing);
            m_spinLineAngle->setValue(b.lineAngleDeg);
            // Bogenreihe
            m_spinArcRadius->setValue(b.arcRadius);
            m_spinArcCount->setValue(b.arcHoleCount);
            m_spinArcStartAngle->setValue(b.arcStartAngle);
            m_spinArcEndAngle->setValue(b.arcEndAngle);
            // Rahmen
            m_spinFrameWidth->setValue(b.frameWidth);
            m_spinFrameHeight->setValue(b.frameHeight);
            m_spinFrameCountX->setValue(b.frameCountX);
            m_spinFrameCountY->setValue(b.frameCountY);
            // Manuell: Tabelle befüllen
            {
                m_tblManualPositions->blockSignals(true);
                m_tblManualPositions->setRowCount(0);
                for (const auto& [mx, my] : b.manualPositions) {
                    int row = m_tblManualPositions->rowCount();
                    m_tblManualPositions->insertRow(row);
                    m_tblManualPositions->setItem(row, 0, new QTableWidgetItem(QString::number(mx, 'f', 3)));
                    m_tblManualPositions->setItem(row, 1, new QTableWidgetItem(QString::number(my, 'f', 3)));
                    m_tblManualPositions->setItem(row, 2, new QTableWidgetItem("0.000"));
                }
                m_tblManualPositions->blockSignals(false);
            }
            break;
        case CAM::BlockType::Slot:
            m_stackParams->setCurrentIndex(5);
            m_spinSlotLength->setValue(b.slotLength);
            m_spinSlotWidth->setValue(b.slotWidth);
            m_spinSlotAngle->setValue(b.slotAngleDeg);
            m_spinSlotCornerR->setValue(b.slotCornerR);
            m_spinSlotCount->setValue(b.slotCount);
            m_spinSlotSpacing->setValue(b.slotSpacing);
            break;
        case CAM::BlockType::HelixThread:
            m_stackParams->setCurrentIndex(6);
            m_spinHelixDia->setValue(b.helixDiameter);
            m_spinHelixPitch->setValue(b.helixPitch);
            m_cmbHelixType->setCurrentIndex(b.helixInternal ? 0 : 1);
            m_cmbHelixDir->setCurrentIndex(b.helixCW ? 0 : 1);
            m_spinHelixStarts->setValue(b.helixStarts);
            break;
        case CAM::BlockType::Stl3D:
            m_stackParams->setCurrentIndex(7);
            updateStlInfoLabel();
            m_cmbStlMode->setCurrentIndex(static_cast<int>(b.stlStrategy));
            m_spinStlFinishStepOver->setValue(b.stlStepOver);
            m_spinStlRoughStepDown->setValue(b.stlStepDown);
            m_spinStlAllowance->setValue(b.stlAllowance);
            m_spinStlSampleStep->setValue(b.stlSampleStep);
            m_chkStlUseStockDims->setChecked(b.stlUseStockDims);

            // Wenn Z-Tiefe noch auf dem 2D-Default (-2.0 mm) steht:
            // Automatisch mit der realen Bauteiltiefe des STL-Modells vorbefüllen!
            if (std::abs(b.targetZ - (-2.0)) < 1e-3 && !m_partMesh.isEmpty()) {
                double realDepth = m_partMesh.boundingBox.minPoint.z;
                if (m_stockMesh.boundingBox.isValid()) {
                    realDepth = std::max(realDepth, m_stockMesh.boundingBox.minPoint.z);
                }
                m_spinTargetZ->setValue(realDepth);
                m_program[index].targetZ = realDepth;
            }
            break;
        case CAM::BlockType::RawNC:
            m_stackParams->setCurrentIndex(4);
            m_txtRawGCode->setPlainText(b.rawGCode);
            break;
    }

    // Header-Werkzeug auf den Block synchronisieren
    Core::ToolDefinition blockTool(b.toolId, "Fräser", Core::ToolType::EndMill, 6.0);
    for (const auto& t : m_toolLibrary) {
        if (t.id == b.toolId) { blockTool = t; break; }
    }
    emit activeToolChanged(blockTool);

    m_isUpdatingUi = false;
}

void ConversationalEditorDialog::saveCurrentBlockFromUi() {
    if (m_isUpdatingUi || m_selectedBlockIndex < 0 || m_selectedBlockIndex >= static_cast<int>(m_program.size())) return;

    auto& b = m_program[m_selectedBlockIndex];
    b.name = m_editName->text();
    b.visible = m_chkVisible->isChecked();
    b.toolId = m_cmbTool->currentData().toInt();
    b.finishToolId = m_cmbFinishTool->currentData().toInt();
    b.materialId = m_cmbMaterial->currentData().toInt();
    b.spindleRpm = m_spinRpm->value();
    b.feedRate = m_spinFeed->value();
    b.plungeFeedRate = m_spinPlunge->value();

    b.startZ = m_spinStartZ->value();
    b.targetZ = m_spinTargetZ->value();
    b.stepDown = m_spinStepDown->value();
    b.stepOver = m_spinStepOver->value();
    b.clearanceZ = m_spinClearanceZ->value();

    b.posX = m_spinPosX->value();
    b.posY = m_spinPosY->value();
    b.millingType = static_cast<CAM::MillingType>(m_cmbMillingType->currentIndex());
    b.startSide = static_cast<CAM::FrameStartSide>(m_cmbStartSide->currentIndex());
    b.millingDirection = m_cmbMillDirection->currentIndex();
    b.coolantOn = m_chkCoolant->isChecked();
    b.finishPass = m_chkFinishPass->isChecked();
    b.finishStepDown = m_spinFinishStep->value();
    b.approachType = m_cmbApproach->currentIndex();

    if (b.type == CAM::BlockType::Facing) {
        b.areaWidth = m_spinFaceWidth->value();
        b.areaDepth = m_spinFaceDepth->value();
        b.useStockDimensions = m_chkUseStockDims->isChecked();
    } else if (b.type == CAM::BlockType::Contour) {
        b.contourSide = static_cast<CAM::ContourSide>(m_cmbContourSide->currentIndex());
        b.finishAllowance = m_spinAllowance->value();
        b.leadType = m_cmbLeadType->currentIndex();
        b.leadRadius = m_spinLeadRadius->value();
        b.useTabs = m_chkUseTabs->isChecked();
        b.tabCount = m_spinTabCount->value();
        b.tabWidth = m_spinTabWidth->value();
        b.tabHeight = m_spinTabHeight->value();
    } else if (b.type == CAM::BlockType::Pocket) {
        b.pocketShape = static_cast<CAM::PocketShape>(m_cmbPocketShape->currentIndex());
        b.pocketWidthX = m_spinPocketWidthX->value();
        b.pocketDepthY = m_spinPocketDepthY->value();
        b.pocketRadius = m_spinPocketRadius->value();
        b.pocketCornerR = m_spinPocketCornerR->value();
        b.pocketStrategy = m_cmbPocketStrategy->currentIndex();
        
        b.enableFinishing = m_grpFinishing->isChecked();
        b.finishAllowanceXY = m_spinFinishAllowanceXY->value();
        b.finishAllowanceZ = m_spinFinishAllowanceZ->value();
        b.finishFeedRate = m_spinFinishFeed->value();
        b.finishSpindleRpm = m_spinFinishSpindle->value();
    } else if (b.type == CAM::BlockType::Drill) {
        b.drillCycle = m_cmbDrillCycle->currentIndex();
        b.drillPattern = static_cast<CAM::DrillPattern>(m_cmbDrillPattern->currentIndex());
        b.boltCircleRadius = m_spinBoltRadius->value();
        b.boltCircleHoleCount = m_spinBoltCount->value();
        b.boltCircleStartAngle = m_spinBoltStartAngle->value();
        b.peckDepth = m_spinPeckDepth->value();
        b.dwellTimeSec = m_spinDwellTime->value();
        b.gridCols = m_spinGridCols->value();
        b.gridRows = m_spinGridRows->value();
        b.gridPitchX = m_spinGridPitchX->value();
        b.gridPitchY = m_spinGridPitchY->value();
        // Lochreihe
        b.lineHoleCount = m_spinLineCount->value();
        b.lineSpacing = m_spinLineSpacing->value();
        b.lineAngleDeg = m_spinLineAngle->value();
        // Bogenreihe
        b.arcRadius = m_spinArcRadius->value();
        b.arcHoleCount = m_spinArcCount->value();
        b.arcStartAngle = m_spinArcStartAngle->value();
        b.arcEndAngle = m_spinArcEndAngle->value();
        // Rahmen
        b.frameWidth = m_spinFrameWidth->value();
        b.frameHeight = m_spinFrameHeight->value();
        b.frameCountX = m_spinFrameCountX->value();
        b.frameCountY = m_spinFrameCountY->value();
        // Manuell: Positionen aus Tabelle lesen
        b.manualPositions.clear();
        for (int r = 0; r < m_tblManualPositions->rowCount(); ++r) {
            auto* itemX = m_tblManualPositions->item(r, 0);
            auto* itemY = m_tblManualPositions->item(r, 1);
            if (itemX && itemY) {
                double mx = itemX->text().toDouble();
                double my = itemY->text().toDouble();
                b.manualPositions.push_back({mx, my});
            }
        }
    } else if (b.type == CAM::BlockType::Slot) {
        b.slotLength = m_spinSlotLength->value();
        b.slotWidth = m_spinSlotWidth->value();
        b.slotAngleDeg = m_spinSlotAngle->value();
        b.slotCornerR = m_spinSlotCornerR->value();
        b.slotCount = m_spinSlotCount->value();
        b.slotSpacing = m_spinSlotSpacing->value();
    } else if (b.type == CAM::BlockType::HelixThread) {
        b.helixDiameter = m_spinHelixDia->value();
        b.helixPitch = m_spinHelixPitch->value();
        b.helixInternal = (m_cmbHelixType->currentIndex() == 0);
        b.helixCW = (m_cmbHelixDir->currentIndex() == 0);
        b.helixStarts = m_spinHelixStarts->value();
    } else if (b.type == CAM::BlockType::Stl3D) {
        b.stlStrategy = static_cast<CAM::StlMillingStrategy>(m_cmbStlMode->currentIndex());
        b.stlStepOver = m_spinStlFinishStepOver->value();
        b.stlStepDown = m_spinStlRoughStepDown->value();
        b.stlAllowance = m_spinStlAllowance->value();
        b.stlSampleStep = m_spinStlSampleStep->value();
        b.stlUseStockDims = m_chkStlUseStockDims->isChecked();
        if (!m_partMesh.isEmpty()) {
            b.directStlMesh = m_partMesh;
        }
    } else if (b.type == CAM::BlockType::RawNC) {
        b.rawGCode = m_txtRawGCode->toPlainText();
    }

    // Listenbeschriftung aktualisieren
    auto* item = m_blockList->item(m_selectedBlockIndex);
    if (item) {
        item->setText(QString("[%1] %2").arg(CAM::blockTypeToString(b.type)).arg(b.name));
    }
}

void ConversationalEditorDialog::onAddBlockClicked(CAM::BlockType type) {
    int nextId = static_cast<int>(m_program.size()) + 1;
    QString name = QString("%1: %2").arg(nextId).arg(CAM::blockTypeToString(type));
    CAM::ConversationalBlock newBlock(nextId, type, name);

    if (type == CAM::BlockType::Stl3D) {
        newBlock.stlStrategy = CAM::StlMillingStrategy::RoughAndFinishX;
        if (!m_partMesh.isEmpty()) {
            double realDepth = m_partMesh.boundingBox.minPoint.z;
            if (m_stockMesh.boundingBox.isValid()) {
                realDepth = std::max(realDepth, m_stockMesh.boundingBox.minPoint.z);
            }
            newBlock.targetZ = realDepth;
        }
    }

    m_program.addBlock(newBlock);
    m_selectedBlockIndex = static_cast<int>(m_program.size()) - 1;
    refreshBlockList();
    loadBlockToUi(m_selectedBlockIndex);
    onCalculateTechnology();
}

void ConversationalEditorDialog::onRemoveBlockClicked() {
    if (m_program.empty()) return;
    m_program.removeBlock(m_selectedBlockIndex);
    refreshBlockList();
    loadBlockToUi(m_selectedBlockIndex);
    onCalculateProgramClicked();
}

void ConversationalEditorDialog::onMoveUpClicked() {
    if (m_program.moveBlockUp(m_selectedBlockIndex)) {
        m_selectedBlockIndex--;
        refreshBlockList();
    }
}

void ConversationalEditorDialog::onMoveDownClicked() {
    if (m_program.moveBlockDown(m_selectedBlockIndex)) {
        m_selectedBlockIndex++;
        refreshBlockList();
    }
}

void ConversationalEditorDialog::onDuplicateClicked() {
    m_program.duplicateBlock(m_selectedBlockIndex);
    refreshBlockList();
}

void ConversationalEditorDialog::onBlockSelectionChanged(int row) {
    if (!m_isUpdatingUi && row >= 0) {
        loadBlockToUi(row);
    }
}

void ConversationalEditorDialog::onBlockItemChanged(QListWidgetItem* item) {
    if (m_isUpdatingUi || !item) return;
    int row = m_blockList->row(item);
    if (row >= 0 && row < static_cast<int>(m_program.size())) {
        m_program[row].enabled = (item->checkState() == Qt::Checked);
    }
}

void ConversationalEditorDialog::onMaterialChanged(int index) {
    Q_UNUSED(index);
    onCalculateTechnology();
}

void ConversationalEditorDialog::onToolChanged(int index) {
    Q_UNUSED(index);
    onCalculateTechnology();
}

void ConversationalEditorDialog::onCalculateTechnology() {
    if (m_selectedBlockIndex < 0 || m_selectedBlockIndex >= static_cast<int>(m_program.size())) return;

    int matId = m_cmbMaterial->currentData().toInt();
    int toolId = m_cmbTool->currentData().toInt();

    auto mat = m_materialDb.findById(matId);
    Core::ToolDefinition tool(toolId, "Fräser", Core::ToolType::EndMill, 6.0);
    for (const auto& t : m_toolLibrary) {
        if (t.id == toolId) { tool = t; break; }
    }

    // Header-Werkzeug synchronisieren
    emit activeToolChanged(tool);

    auto tech = Core::TechnologyCalculator::calculate(mat, tool);

    m_spinRpm->setValue(tech.spindleRpm);
    m_spinFeed->setValue(tech.feedRate);
    m_spinPlunge->setValue(tech.plungeFeedRate);
    m_spinStepDown->setValue(tech.recommendedStepDown);
    m_spinStepOver->setValue(tech.recommendedStepOver);

    saveCurrentBlockFromUi();
}

void ConversationalEditorDialog::onPickContourClicked() {
    emit pickingModeRequested(true);
    QMessageBox::information(this, QStringLiteral("3D-Viewport Picking aktiv"),
                            QStringLiteral("Klicke jetzt im 3D-Viewport auf die gewünschte 2D-Kontur.\nSie wird leuchtend gelb markiert und automatisch übernommen."));
}

void ConversationalEditorDialog::applyPickedContour(int index, const Geometry::Contour& contour) {
    Q_UNUSED(index);
    if (m_selectedBlockIndex >= 0 && m_selectedBlockIndex < static_cast<int>(m_program.size())) {
        auto& b = m_program[m_selectedBlockIndex];
        b.contour = contour;
        m_lblContourStatus->setText(QString("Kontur (%1 Pkt)").arg(contour.points.size()));
    }
}

void ConversationalEditorDialog::onCalculateProgramClicked() {
    saveCurrentBlockFromUi();

    Core::BoundingBox stockBounds = m_stockMesh.boundingBox.isValid()
        ? m_stockMesh.boundingBox
        : Core::BoundingBox({-50, -40, -10}, {50, 40, 0});

    m_currentToolpath = m_program.generateFullToolpath(m_toolLibrary, stockBounds, m_partMesh);

    // Kollisionsprüfung für das Gesamtprogramm
    Core::ToolDefinition activeTool(1, "Tool", Core::ToolType::EndMill, 6.0);
    if (!m_toolLibrary.isEmpty()) activeTool = m_toolLibrary.first();

    auto rep = CAM::CollisionDetector::verifyToolpath(m_currentToolpath, m_machineConfig, activeTool, stockBounds);

    QString collText;
    if (rep.hasErrors) {
        collText = QString("⚠ %1 KRITISCH: %2").arg(rep.totalViolations).arg(rep.violations.front().description);
        QString tip = QString("Gefundene Kollisionen (%1):\n").arg(rep.totalViolations);
        for (size_t i = 0; i < rep.violations.size() && i < 8; ++i) {
            tip += QString("• Seg %1: %2 [X=%3, Y=%4, Z=%5]\n")
                .arg(rep.violations[i].segmentIndex)
                .arg(rep.violations[i].description)
                .arg(rep.violations[i].position.x, 0, 'f', 2)
                .arg(rep.violations[i].position.y, 0, 'f', 2)
                .arg(rep.violations[i].position.z, 0, 'f', 2);
        }
        if (rep.violations.size() > 8) {
            tip += QString("... und %1 weitere").arg(rep.violations.size() - 8);
        }
        m_lblProgramSummary->setToolTip(tip);
        m_lblProgramSummary->setStyleSheet("background-color: #742A2A; padding: 6px; border-radius: 4px; color: #FED7D7; font-weight: bold; font-family: Consolas;");
    } else {
        collText = QStringLiteral("✔ OK (Kollisionsfrei)");
        m_lblProgramSummary->setToolTip(QStringLiteral("Alle Fräsbahnen liegen innerhalb der Maschinengrenzen und Werkzeugauskragung."));
        m_lblProgramSummary->setStyleSheet("background-color: #22543D; padding: 6px; border-radius: 4px; color: #C6F6D5; font-weight: bold; font-family: Consolas;");
    }

    QString summary = QString("Blöcke: %1 | Segmente: %2 | Weg: %3 mm\nLaufzeit: %4 min | Kollisionen: %5")
        .arg(m_program.size())
        .arg(m_currentToolpath.size())
        .arg(m_currentToolpath.totalLength(), 0, 'f', 1)
        .arg(m_currentToolpath.estimatedTotalTimeSeconds() / 60.0, 0, 'f', 1)
        .arg(collText);

    m_lblProgramSummary->setText(summary);

    emit toolpathGenerated(m_currentToolpath);
}

void ConversationalEditorDialog::onSaveProgramClicked() {
    saveCurrentBlockFromUi();
    QString path = QFileDialog::getSaveFileName(this, QStringLiteral("Programm speichern"), QStringLiteral("programm.gprog"), QStringLiteral("Conversational Programm (*.gprog)"));
    if (path.isEmpty()) return;

    if (m_program.saveToFile(path)) {
        QMessageBox::information(this, QStringLiteral("Gespeichert"), QString("Programm erfolgreich gespeichert:\n%1").arg(path));
    }
}

void ConversationalEditorDialog::onLoadProgramClicked() {
    QString path = QFileDialog::getOpenFileName(this, QStringLiteral("Programm laden"), QString(), QStringLiteral("Conversational Programm (*.gprog)"));
    if (path.isEmpty()) return;

    m_program = CAM::ConversationalProgram::loadFromFile(path);
    m_selectedBlockIndex = 0;
    refreshBlockList();
    loadBlockToUi(0);
}

void ConversationalEditorDialog::onEditContourSegmentsClicked() {
    showSegmentEditor();
}

void ConversationalEditorDialog::onManageIslandsClicked() {
    if (m_selectedBlockIndex < 0 || m_selectedBlockIndex >= static_cast<int>(m_program.size())) return;
    auto& b = m_program[m_selectedBlockIndex];
    
    // For now, we only support adding a new island or editing the first one.
    // Let's create an island if none exists.
    if (b.pocketIslands.empty()) {
        Geometry::ContourSegment startSeg;
        startSeg.type = Geometry::ContourSegmentType::StartPoint;
        startSeg.x = b.posX;
        startSeg.y = b.posY;
        b.pocketIslands.push_back({startSeg});
    }
    
    m_editingIslandIndex = 0; // Edit the first island
    m_segmentEditorMode = SegmentEditorMode::PocketIsland;
    
    m_segmentEditor->setSegments(b.pocketIslands[0]);
    m_masterStack->setCurrentIndex(1);
    emit segmentEditorVisibilityChanged(true);
}

void ConversationalEditorDialog::showSegmentEditor() {
    if (m_selectedBlockIndex < 0 || m_selectedBlockIndex >= static_cast<int>(m_program.size())) return;
    const auto& b = m_program[m_selectedBlockIndex];

    m_segmentEditorMode = SegmentEditorMode::BlockContour;

    // Segmente in den Datensatz-Editor laden
    if (!b.segments.empty()) {
        m_segmentEditor->setSegments(b.segments);
    } else {
        // Neuer Block: Standard-Startpunkt erstellen
        Geometry::ContourSegment startSeg;
        startSeg.type = Geometry::ContourSegmentType::StartPoint;
        startSeg.x = b.posX;
        startSeg.y = b.posY;
        m_segmentEditor->setSegments({startSeg});
    }

    // Zur Datensatz-Ansicht umschalten (Seite 1)
    m_masterStack->setCurrentIndex(1);
    emit segmentEditorVisibilityChanged(true);
}

void ConversationalEditorDialog::hideSegmentEditor() {
    if (m_masterStack->currentIndex() != 1) return;

    // Segmente und Kontur in den aktiven Block übernehmen
    if (m_selectedBlockIndex >= 0 && m_selectedBlockIndex < static_cast<int>(m_program.size())) {
        auto& b = m_program[m_selectedBlockIndex];
        b.segments = m_segmentEditor->segments();
        b.contour = m_segmentEditor->compiledContour();
        m_lblContourStatus->setText(QString("%1 Schritte (%2 Pkt)").arg(b.segments.size()).arg(b.contour.points.size()));
    }

    // Zurück zum Block-Editor (Seite 0)
    m_masterStack->setCurrentIndex(0);
    emit segmentEditorVisibilityChanged(false);
}

void ConversationalEditorDialog::addContourSegment(Geometry::ContourSegmentType type) {
    // Falls der Segment-Editor noch nicht sichtbar ist, erst öffnen
    if (m_masterStack->currentIndex() != 1) {
        showSegmentEditor();
    }
    m_segmentEditor->addSegment(type);
}

void ConversationalEditorDialog::navigateSegmentNext() {
    if (m_masterStack->currentIndex() == 1) {
        m_segmentEditor->navigateNext();
    }
}

void ConversationalEditorDialog::navigateSegmentPrev() {
    if (m_masterStack->currentIndex() == 1) {
        m_segmentEditor->navigatePrevious();
    }
}

void ConversationalEditorDialog::deleteCurrentSegment() {
    if (m_masterStack->currentIndex() == 1) {
        m_segmentEditor->deleteCurrentSegment();
    }
}

void ConversationalEditorDialog::storeSegmentCalculatedValue() {
    if (m_masterStack->currentIndex() == 1) {
        m_segmentEditor->storeCalculatedValue();
    }
}

void ConversationalEditorDialog::findSegmentAlternativeSolution() {
    if (m_masterStack->currentIndex() == 1) {
        m_segmentEditor->findAlternativeSolution();
    }
}

bool ConversationalEditorDialog::isSegmentEditorVisible() const {
    return m_masterStack && m_masterStack->currentIndex() == 1;
}

} // namespace GeminiCNC::UI

