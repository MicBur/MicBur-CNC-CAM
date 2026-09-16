#include "ContourSegmentEditorDialog.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <cmath>
#include <algorithm>
#include <optional>

namespace GeminiCNC::UI {

ContourSegmentEditorDialog::ContourSegmentEditorDialog(QWidget* parent) : QWidget(parent) {
    setupUi();

    // Standard: 1 Startpunkt + 1 Linie
    m_segments = {
        {Geometry::ContourSegmentType::StartPoint, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {Geometry::ContourSegmentType::Line, 50.0, 0.0, 0.0, 0.0, 0.0, 0.0, 50.0}
    };
    loadStep(1);
    recompileContour();
}

void ContourSegmentEditorDialog::setupUi() {
    setStyleSheet(QStringLiteral(
        "QWidget { background-color: #0E1318; color: #F1F5F9; font-family: 'Segoe UI', sans-serif; }"
        "QGroupBox { border: 1px solid #243040; border-radius: 8px; margin-top: 10px; background-color: #141B24; padding: 10px; }"
        "QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 6px; color: #00D2FF; font-weight: bold; font-size: 11px; }"
        "QLineEdit { background-color: #0A0F15; border: 1px solid #2B3A4C; border-radius: 4px; padding: 6px; color: #38BDF8; font-family: Consolas, monospace; font-size: 13px; font-weight: bold; }"
        "QLineEdit:focus { border: 1px solid #00D2FF; background-color: #0D1622; }"
        "QDoubleSpinBox { background-color: #0A0F15; border: 1px solid #2B3A4C; border-radius: 4px; padding: 5px; color: #F1F5F9; font-family: Consolas, monospace; font-size: 12px; }"
        "QComboBox { background-color: #0A0F15; border: 1px solid #2B3A4C; border-radius: 4px; padding: 5px; color: #F1F5F9; }"
        "QComboBox QAbstractItemView { background-color: #141B24; selection-background-color: #0284C7; color: white; }"
        "QTabWidget::pane { border: 1px solid #243040; border-radius: 6px; background-color: #141B24; }"
        "QTabBar::tab { background-color: #0E1318; color: #94A3B8; padding: 8px 18px; border: 1px solid #243040; border-bottom: none; border-top-left-radius: 6px; border-top-right-radius: 6px; margin-right: 2px; font-weight: bold; font-size: 11px; }"
        "QTabBar::tab:selected { background-color: #141B24; color: #00D2FF; border-top: 2px solid #00D2FF; }"
    ));

    auto* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(12, 10, 12, 10);
    mainLayout->setSpacing(8);

    // ─── 1. Kopfbereich (Hurco: BLOCK 2 MILL CONTOUR / SEGMENT 1 LINE) ───
    auto* headerWidget = new QWidget(this);
    auto* headerLayout = new QHBoxLayout(headerWidget);
    headerLayout->setContentsMargins(0, 0, 0, 0);

    m_lblBlockHeader = new QLabel(QStringLiteral("BLOCK 2    MILL CONTOUR"), this);
    m_lblBlockHeader->setStyleSheet(QStringLiteral("font-size: 14px; font-weight: 800; color: #00D2FF; letter-spacing: 1px;"));

    m_lblSegmentHeader = new QLabel(QStringLiteral("SEGMENT 1    LINE"), this);
    m_lblSegmentHeader->setStyleSheet(QStringLiteral("font-size: 14px; font-weight: 800; color: #F8FAFC; letter-spacing: 1px;"));

    headerLayout->addWidget(m_lblBlockHeader);
    headerLayout->addSpacing(30);
    headerLayout->addWidget(m_lblSegmentHeader);
    headerLayout->addStretch();

    mainLayout->addWidget(headerWidget);

    // ─── 2. Geometrie-Bereich (2 Spalten nach Hurco Vorbild) ───
    auto* geomGroup = new QGroupBox(QStringLiteral("GEOMETRIE & POSITION"), this);
    auto* geomLayout = new QGridLayout(geomGroup);
    geomLayout->setSpacing(10);

    // Linke Spalte: EINGABEN (X END, Y END, LENGTH, ANGLE etc.)
    auto* leftBox = new QWidget(this);
    auto* leftLayout = new QFormLayout(leftBox);
    leftLayout->setSpacing(8);

    m_editLineEndX = new QLineEdit(this); m_editLineEndX->setPlaceholderText(QStringLiteral("[ Auto / Eingabe ]"));
    m_editLineEndY = new QLineEdit(this); m_editLineEndY->setPlaceholderText(QStringLiteral("[ Auto / Eingabe ]"));
    m_editLineZEnd = new QLineEdit(QStringLiteral("0.0000"), this);
    m_editZStart = new QLineEdit(QStringLiteral("0.0000"), this);
    connect(m_editLineZEnd, &QLineEdit::editingFinished, this, &ContourSegmentEditorDialog::onDepthEdited);
    connect(m_editZStart, &QLineEdit::editingFinished, this, &ContourSegmentEditorDialog::onDepthEdited);
    m_editLineLength = new QLineEdit(this); m_editLineLength->setPlaceholderText(QStringLiteral("[ Länge ]"));
    m_editLineAngle = new QLineEdit(this); m_editLineAngle->setPlaceholderText(QStringLiteral("[ Winkel ° ]"));

    connect(m_editLineEndX, &QLineEdit::textEdited, this, &ContourSegmentEditorDialog::onInputEdited);
    connect(m_editLineEndY, &QLineEdit::textEdited, this, &ContourSegmentEditorDialog::onInputEdited);
    connect(m_editLineLength, &QLineEdit::textEdited, this, &ContourSegmentEditorDialog::onInputEdited);
    connect(m_editLineAngle, &QLineEdit::textEdited, this, &ContourSegmentEditorDialog::onInputEdited);

    m_leftForm = leftLayout;
    m_lblXCaption = new QLabel(QStringLiteral("X END:"));
    m_lblYCaption = new QLabel(QStringLiteral("Y END:"));
    m_lblZEndCaption = new QLabel(QStringLiteral("Z END:"));
    leftLayout->addRow(m_lblXCaption, m_editLineEndX);
    leftLayout->addRow(m_lblYCaption, m_editLineEndY);
    leftLayout->addRow(new QLabel(QStringLiteral("Z START:")), m_editZStart);
    leftLayout->addRow(m_lblZEndCaption, m_editLineZEnd);
    leftLayout->addRow(new QLabel(QStringLiteral("XY LENGTH:")), m_editLineLength);
    leftLayout->addRow(new QLabel(QStringLiteral("XY ANGLE:")), m_editLineAngle);

    // ARC-Eingaben (Kreisbogen): Richtung, Endpunkt, Mittelpunkt, Radius, Winkel
    m_arcInputsWidget = new QWidget(this);
    m_arcForm = new QFormLayout(m_arcInputsWidget);
    m_arcForm->setSpacing(8);
    m_cmbArcDirection = new QComboBox(this);
    m_cmbArcDirection->addItems({QStringLiteral("UZS (CW)"), QStringLiteral("GUZS (CCW)")});
    auto makeArcEdit = [this](const QString& placeholder) {
        auto* edit = new QLineEdit(this);
        edit->setPlaceholderText(placeholder);
        connect(edit, &QLineEdit::textEdited, this, &ContourSegmentEditorDialog::onArcInputEdited);
        return edit;
    };
    m_editArcEndX = makeArcEdit(QStringLiteral("[ X Ende ]"));
    m_editArcEndY = makeArcEdit(QStringLiteral("[ Y Ende ]"));
    m_editArcZEnd = new QLineEdit(QStringLiteral("0.0000"), this);
    connect(m_editArcZEnd, &QLineEdit::editingFinished, this, &ContourSegmentEditorDialog::onDepthEdited);
    m_editArcCenterX = makeArcEdit(QStringLiteral("[ X Mittelpunkt ]"));
    m_editArcCenterY = makeArcEdit(QStringLiteral("[ Y Mittelpunkt ]"));
    m_editArcRadius = makeArcEdit(QStringLiteral("[ Radius ]"));
    m_editArcSweepAngle = makeArcEdit(QStringLiteral("[ Winkel ° ]"));
    connect(m_cmbArcDirection, &QComboBox::currentIndexChanged, this, [this]() {
        if (!m_isLoading) onArcInputEdited();
    });
    m_arcForm->addRow(new QLabel(QStringLiteral("RICHTUNG:")), m_cmbArcDirection);
    m_arcForm->addRow(new QLabel(QStringLiteral("X END:")), m_editArcEndX);
    m_arcForm->addRow(new QLabel(QStringLiteral("Y END:")), m_editArcEndY);
    m_arcForm->addRow(new QLabel(QStringLiteral("Z END:")), m_editArcZEnd);
    m_arcForm->addRow(new QLabel(QStringLiteral("X MITTELPUNKT:")), m_editArcCenterX);
    m_arcForm->addRow(new QLabel(QStringLiteral("Y MITTELPUNKT:")), m_editArcCenterY);
    m_arcForm->addRow(new QLabel(QStringLiteral("RADIUS:")), m_editArcRadius);
    m_arcForm->addRow(new QLabel(QStringLiteral("WINKEL:")), m_editArcSweepAngle);
    m_arcInputsWidget->setVisible(false);

    // Rechte Spalte: KONTEXT (X START, Y START, Z START aus vorherigem Schritt)
    auto* rightBox = new QWidget(this);
    auto* rightLayout = new QFormLayout(rightBox);
    rightLayout->setSpacing(8);

    m_lblStartX = new QLabel(QStringLiteral("0.0000"), this);
    m_lblStartY = new QLabel(QStringLiteral("0.0000"), this);
    m_lblStartZ = new QLabel(QStringLiteral("0.0000"), this);

    QString roStyle = QStringLiteral("background-color: #0A0F15; border: 1px solid #1E293B; border-radius: 4px; padding: 6px; color: #94A3B8; font-family: Consolas; font-size: 13px; font-weight: bold;");
    m_lblStartX->setStyleSheet(roStyle);
    m_lblStartY->setStyleSheet(roStyle);
    m_lblStartZ->setStyleSheet(roStyle);

    rightLayout->addRow(new QLabel(QStringLiteral("X START:")), m_lblStartX);
    rightLayout->addRow(new QLabel(QStringLiteral("Y START:")), m_lblStartY);
    rightLayout->addRow(new QLabel(QStringLiteral("Z START:")), m_lblStartZ);

    m_lineInputsWidget = leftBox;
    geomLayout->addWidget(leftBox, 0, 0);
    geomLayout->addWidget(m_arcInputsWidget, 0, 0); // gleiche Zelle, je nach Segmenttyp sichtbar
    geomLayout->addWidget(rightBox, 0, 1);

    // Auto-Berechnungsstatus-Zeile
    m_lblCalcStatus = new QLabel(QStringLiteral("Geben Sie Maße ein – die Steuerung berechnet fehlende Koordinaten automatisch."), this);
    m_lblCalcStatus->setStyleSheet(QStringLiteral("font-size: 11px; color: #00D2FF; font-style: italic; padding-top: 4px;"));
    geomLayout->addWidget(m_lblCalcStatus, 1, 0, 1, 2);

    mainLayout->addWidget(geomGroup);

    // ─── 3. Untere Tabs: [SCHRUPPEN] [SCHLICHTEN] [KÜHLMITTEL] ───
    m_techTabs = new QTabWidget(this);

    // Tab 1: Schruppen (Roughing)
    auto* tabRoughing = new QWidget(this);
    auto* lRough = new QFormLayout(tabRoughing);
    lRough->setSpacing(8);

    m_cmbTool = new QComboBox(this); // Einträge kommen aus der Werkzeugbibliothek (setToolLibrary)

    m_cmbMillingType = new QComboBox(this);
    m_cmbMillingType->addItems({
        QStringLiteral("AUF KONTUR (ON)"),
        QStringLiteral("INNEN (INSIDE)"),
        QStringLiteral("AUSSEN (OUTSIDE)")
    });
    m_cmbMillingType->setCurrentIndex(0);
    connect(m_cmbMillingType, &QComboBox::currentIndexChanged, this, &ContourSegmentEditorDialog::onMillingTypeChanged);

    auto* feedRow = new QHBoxLayout();
    m_spinFeed = new QDoubleSpinBox(this); m_spinFeed->setRange(10, 10000); m_spinFeed->setValue(1500); m_spinFeed->setSuffix(" mm/min");
    m_spinPlunge = new QDoubleSpinBox(this); m_spinPlunge->setRange(10, 5000); m_spinPlunge->setValue(500); m_spinPlunge->setSuffix(" mm/min");
    feedRow->addWidget(new QLabel(QStringLiteral("Vorschub:"))); feedRow->addWidget(m_spinFeed);
    feedRow->addWidget(new QLabel(QStringLiteral("Eintauchen:"))); feedRow->addWidget(m_spinPlunge);

    auto* speedRow = new QHBoxLayout();
    m_spinRpm = new QDoubleSpinBox(this); m_spinRpm->setRange(100, 60000); m_spinRpm->setValue(18000); m_spinRpm->setSuffix(" RPM");
    m_spinPeckDepth = new QDoubleSpinBox(this); m_spinPeckDepth->setRange(0.1, 50); m_spinPeckDepth->setValue(1.0); m_spinPeckDepth->setSuffix(" mm (ap)");
    speedRow->addWidget(new QLabel(QStringLiteral("Drehzahl:"))); speedRow->addWidget(m_spinRpm);
    speedRow->addWidget(new QLabel(QStringLiteral("Zustellung:"))); speedRow->addWidget(m_spinPeckDepth);

    lRough->addRow(QStringLiteral("WERKZEUG:"), m_cmbTool);
    lRough->addRow(QStringLiteral("FRÄSART:"), m_cmbMillingType);
    lRough->addRow(QStringLiteral("GESCHWINDIGKEIT:"), feedRow);
    lRough->addRow(QStringLiteral("SCHNITTWERTE:"), speedRow);

    // Technologie-Änderungen an den Block zurückmelden
    connect(m_cmbTool, &QComboBox::currentIndexChanged, this, &ContourSegmentEditorDialog::onTechnologyEdited);
    connect(m_cmbMillingType, &QComboBox::currentIndexChanged, this, &ContourSegmentEditorDialog::onTechnologyEdited);
    for (auto* spin : {m_spinFeed, m_spinPlunge, m_spinRpm, m_spinPeckDepth}) {
        connect(spin, &QDoubleSpinBox::valueChanged, this, &ContourSegmentEditorDialog::onTechnologyEdited);
    }

    m_techTabs->addTab(tabRoughing, QStringLiteral("SCHRUPPEN (ROUGHING)"));

    // Tab 2: Schlichten (Finishing)
    auto* tabFinishing = new QWidget(this);
    auto* lFinish = new QFormLayout(tabFinishing);
    lFinish->setSpacing(8);
    auto* spinFinishAllowance = new QDoubleSpinBox(this); spinFinishAllowance->setRange(0, 10); spinFinishAllowance->setValue(0.2); spinFinishAllowance->setSuffix(" mm");
    auto* spinFinishFeed = new QDoubleSpinBox(this); spinFinishFeed->setRange(10, 10000); spinFinishFeed->setValue(800); spinFinishFeed->setSuffix(" mm/min");
    lFinish->addRow(QStringLiteral("SCHLICHTAUFMASS:"), spinFinishAllowance);
    lFinish->addRow(QStringLiteral("SCHLICHTVORSCHUB:"), spinFinishFeed);
    m_techTabs->addTab(tabFinishing, QStringLiteral("SCHLICHTEN (FINISHING)"));

    // Tab 3: Kühlmittel (Coolant)
    auto* tabCoolant = new QWidget(this);
    auto* lCool = new QFormLayout(tabCoolant);
    auto* cmbCoolant = new QComboBox(this);
    cmbCoolant->addItems({QStringLiteral("FLUTKÜHLUNG (FLOOD)"), QStringLiteral("MINIMALMENGEN (MIST)"), QStringLiteral("DRUCKLUFT (AIR)"), QStringLiteral("AUS (OFF)")});
    lCool->addRow(QStringLiteral("KÜHLMITTEL-MODUS:"), cmbCoolant);
    m_techTabs->addTab(tabCoolant, QStringLiteral("KÜHLMITTEL (COOLANT)"));

    mainLayout->addWidget(m_techTabs);
    mainLayout->addStretch();
}

void ContourSegmentEditorDialog::setSegments(const std::vector<Geometry::ContourSegment>& segments) {
    m_segments = segments;
    if (m_segments.empty()) {
        m_segments.push_back({Geometry::ContourSegmentType::StartPoint, 0, 0, 0, 0, 0, 0, 0});
    }
    m_currentIndex = std::min(1, static_cast<int>(m_segments.size()) - 1);
    loadStep(m_currentIndex);
    recompileContour();
}

void ContourSegmentEditorDialog::loadStep(int index) {
    if (index < 0 || index >= static_cast<int>(m_segments.size())) return;
    m_isLoading = true;
    m_currentIndex = index;

    const auto& seg = m_segments[index];

    // Vorheriger Punkt als Kontext (X Start, Y Start)
    double sX = 0.0;
    double sY = 0.0;
    double sZ = 0.0;
    if (index > 0) {
        sX = m_segments[index - 1].x;
        sY = m_segments[index - 1].y;
        sZ = m_segments[index - 1].z;
    }
    m_lblStartX->setText(QString::number(sX, 'f', 4));
    m_lblStartY->setText(QString::number(sY, 'f', 4));
    m_lblStartZ->setText(QString::number(sZ, 'f', 4));

    // Kopfzeile aktualisieren
    QString typeStr = (seg.type == Geometry::ContourSegmentType::StartPoint) ? QStringLiteral("START")
                    : (seg.type == Geometry::ContourSegmentType::ArcCW || seg.type == Geometry::ContourSegmentType::ArcCCW) ? QStringLiteral("ARC")
                    : QStringLiteral("LINE");
    m_lblSegmentHeader->setText(QString("SEGMENT %1    %2").arg(index).arg(typeStr));

    // Eingabefelder füllen
    m_editLineEndX->setText(QString::number(seg.x, 'f', 4));
    m_editLineEndY->setText(QString::number(seg.y, 'f', 4));
    m_editLineZEnd->setText(QString::number(seg.z, 'f', 4));
    m_editZStart->setText(QString::number(seg.zStart, 'f', 4));

    // Kreisbogen: eigene Eingabemaske mit Richtung, Endpunkt, Mittelpunkt, Radius, Winkel
    const bool isArc = isArcStep(index);
    m_lineInputsWidget->setVisible(!isArc);
    m_arcInputsWidget->setVisible(isArc);
    m_cachedArcSolutions.clear();
    m_currentSolutionIndex = 0;
    if (isArc) {
        constexpr double kPi = 3.14159265358979323846;
        const bool cw = (seg.type == Geometry::ContourSegmentType::ArcCW);
        double cx = seg.centerX;
        double cy = seg.centerY;
        if (!seg.hasCenter) {
            // Mittelpunkt aus dem Radius wie beim Konturaufbau (kürzerer Bogen)
            const double dx = seg.x - sX;
            const double dy = seg.y - sY;
            const double d = std::hypot(dx, dy);
            const double r = std::max({0.1, std::abs(seg.radius), 0.5 * d});
            const double h = std::sqrt(std::max(0.0, r * r - 0.25 * d * d));
            double nx = d > 1e-9 ? -dy / d : 0.0;
            double ny = d > 1e-9 ? dx / d : 0.0;
            if (cw) {
                nx = -nx;
                ny = -ny;
            }
            cx = 0.5 * (sX + seg.x) + h * nx;
            cy = 0.5 * (sY + seg.y) + h * ny;
        }
        const double radius = std::hypot(sX - cx, sY - cy);
        double sweep = std::atan2(seg.y - cy, seg.x - cx) - std::atan2(sY - cy, sX - cx);
        if (cw && sweep >= 0.0) sweep -= 2.0 * kPi;
        if (!cw && sweep <= 0.0) sweep += 2.0 * kPi;

        m_cmbArcDirection->setCurrentIndex(cw ? 0 : 1);
        m_editArcEndX->setText(QString::number(seg.x, 'f', 4));
        m_editArcEndY->setText(QString::number(seg.y, 'f', 4));
        m_editArcZEnd->setText(QString::number(seg.z, 'f', 4));
        m_editArcCenterX->setText(QString::number(cx, 'f', 4));
        m_editArcCenterY->setText(QString::number(cy, 'f', 4));
        m_editArcRadius->setText(QString::number(radius, 'f', 4));
        m_editArcSweepAngle->setText(QString::number(std::abs(sweep) * 180.0 / kPi, 'f', 2));

        if (seg.hasCenter) {
            m_arcKnownFields = {m_editArcEndX, m_editArcEndY, m_editArcCenterX, m_editArcCenterY};
        } else {
            m_arcKnownFields = {m_editArcEndX, m_editArcEndY, m_editArcRadius};
        }
    }

    // Segment 0 (START): X/Y START, Z START und Z UNTEN (Tiefe gilt für die Folgesegmente)
    const bool isStart = (index == 0 && seg.type == Geometry::ContourSegmentType::StartPoint);
    m_lblXCaption->setText(isStart ? QStringLiteral("X START:") : QStringLiteral("X END:"));
    m_lblYCaption->setText(isStart ? QStringLiteral("Y START:") : QStringLiteral("Y END:"));
    m_lblZEndCaption->setText(isStart ? QStringLiteral("Z UNTEN:") : QStringLiteral("Z END:"));
    m_leftForm->setRowVisible(m_editZStart, isStart);
    m_leftForm->setRowVisible(m_editLineLength, !isStart);
    m_leftForm->setRowVisible(m_editLineAngle, !isStart);

    double dx = seg.x - sX;
    double dy = seg.y - sY;
    double len = std::hypot(dx, dy);
    double ang = std::atan2(dy, dx) * (180.0 / 3.141592653589793);
    if (ang < 0) ang += 360.0;

    m_editLineLength->setText(QString::number(len, 'f', 4));
    m_editLineAngle->setText(QString::number(ang, 'f', 1));

    updateContextPrompt();
    m_isLoading = false;
}

void ContourSegmentEditorDialog::saveCurrentStep() {
    if (m_currentIndex < 0 || m_currentIndex >= static_cast<int>(m_segments.size())) return;
    auto& seg = m_segments[m_currentIndex];

    const bool isArc = isArcStep(m_currentIndex);
    auto parse = [](const QLineEdit* edit, bool* ok) {
        return edit->text().trimmed().replace(QLatin1Char(','), QLatin1Char('.')).toDouble(ok);
    };
    bool okX = false, okY = false, okZ = false, okZStart = false;
    double vx = parse(isArc ? m_editArcEndX : m_editLineEndX, &okX);
    double vy = parse(isArc ? m_editArcEndY : m_editLineEndY, &okY);
    double vz = parse(isArc ? m_editArcZEnd : m_editLineZEnd, &okZ);
    double vzStart = parse(m_editZStart, &okZStart);

    if (okX) seg.x = vx;
    if (okY) seg.y = vy;

    if (m_currentIndex == 0 && seg.type == Geometry::ContourSegmentType::StartPoint) {
        // Tiefe von Segment 0 an alle Folgesegmente mit bisheriger Tiefe weitergeben
        Geometry::Contour::applyStartDepth(m_segments, okZStart ? vzStart : seg.zStart, okZ ? vz : seg.z);
    } else if (okZ) {
        seg.z = vz;
    }
}

void ContourSegmentEditorDialog::onDepthEdited() {
    if (m_isLoading) return;
    saveCurrentStep();
    recompileContour(); // meldet die Änderung an den Block (Z-Ebenen)
}

void ContourSegmentEditorDialog::onInputEdited() {
    if (m_isLoading || m_currentIndex <= 0 || isArcStep(m_currentIndex)) return;

    // Startkoordinaten aus vorherigem Schritt
    double sX = m_segments[m_currentIndex - 1].x;
    double sY = m_segments[m_currentIndex - 1].y;

    Geometry::LineSolveInput input;
    input.startX = sX;
    input.startY = sY;

    bool okX = false, okY = false, okLen = false, okAng = false;
    double vx = m_editLineEndX->text().toDouble(&okX);
    double vy = m_editLineEndY->text().toDouble(&okY);
    double vLen = m_editLineLength->text().toDouble(&okLen);
    double vAng = m_editLineAngle->text().toDouble(&okAng);

    if (okX && !m_editLineEndX->text().isEmpty()) input.endX = vx;
    if (okY && !m_editLineEndY->text().isEmpty()) input.endY = vy;
    if (okLen && !m_editLineLength->text().isEmpty()) input.length = vLen;
    if (okAng && !m_editLineAngle->text().isEmpty()) input.angleDeg = vAng;

    // Solver ausführen
    auto result = Geometry::ContourSolver::solveLine(input);
    if (result.success) {
        m_lblCalcStatus->setText(QString("⚡ Automatisch gelöst: Endpunkt (%1, %2) | Länge: %3 mm | Winkel: %4° [F4 zum Festschreiben]")
            .arg(result.endX, 0, 'f', 3)
            .arg(result.endY, 0, 'f', 3)
            .arg(result.length, 0, 'f', 2)
            .arg(result.angleDeg, 0, 'f', 1));

        // Live-Übernahme in temporäres Segment für 3D-Vorschau
        m_segments[m_currentIndex].x = result.endX;
        m_segments[m_currentIndex].y = result.endY;
        recompileContour();
    } else {
        m_lblCalcStatus->setText(QStringLiteral("Geben Sie Maße ein (z. B. Länge + Winkel oder Endpunkt X)..."));
    }
}

void ContourSegmentEditorDialog::storeCalculatedValue() {
    // F4 Taste: Berechnete Werte fest in die Textfelder und ins Segment schreiben
    if (m_currentIndex < 0 || m_currentIndex >= static_cast<int>(m_segments.size())) return;

    if (isArcStep(m_currentIndex)) {
        // Bogen: aktuelle Lösung übernehmen und als vorgegebene Werte (Endpunkt + Mittelpunkt) festschreiben
        if (!m_cachedArcSolutions.empty()) applyArcSolution();
        m_arcKnownFields = {m_editArcEndX, m_editArcEndY, m_editArcCenterX, m_editArcCenterY};
        emit promptChanged(QStringLiteral("Kreisbogen im Datensatz gespeichert."));
        return;
    }

    double sX = (m_currentIndex > 0) ? m_segments[m_currentIndex - 1].x : 0.0;
    double sY = (m_currentIndex > 0) ? m_segments[m_currentIndex - 1].y : 0.0;

    Geometry::LineSolveInput input;
    input.startX = sX;
    input.startY = sY;

    bool okX = false, okY = false, okLen = false, okAng = false;
    double vx = m_editLineEndX->text().toDouble(&okX);
    double vy = m_editLineEndY->text().toDouble(&okY);
    double vLen = m_editLineLength->text().toDouble(&okLen);
    double vAng = m_editLineAngle->text().toDouble(&okAng);

    if (okX) input.endX = vx;
    if (okY) input.endY = vy;
    if (okLen) input.length = vLen;
    if (okAng) input.angleDeg = vAng;

    auto result = Geometry::ContourSolver::solveLine(input);
    if (result.success) {
        m_isLoading = true;
        m_editLineEndX->setText(QString::number(result.endX, 'f', 4));
        m_editLineEndY->setText(QString::number(result.endY, 'f', 4));
        m_editLineLength->setText(QString::number(result.length, 'f', 4));
        m_editLineAngle->setText(QString::number(result.angleDeg, 'f', 1));
        m_isLoading = false;

        m_segments[m_currentIndex].x = result.endX;
        m_segments[m_currentIndex].y = result.endY;
        recompileContour();

        emit promptChanged(QStringLiteral("Werte erfolgreich im Datensatz gespeichert."));
    }
}

void ContourSegmentEditorDialog::findAlternativeSolution() {
    // F5 Taste: Nächste Lösung bei Kreisbögen mit 2 Mittelpunkten
    if (!isArcStep(m_currentIndex) || m_cachedArcSolutions.size() < 2) {
        emit promptChanged(QStringLiteral("Keine alternative Lösung vorhanden (nur bei Bogen mit Endpunkt und Radius)."));
        return;
    }
    m_currentSolutionIndex = (m_currentSolutionIndex + 1) % static_cast<int>(m_cachedArcSolutions.size());
    applyArcSolution();
    emit promptChanged(QStringLiteral("Alternative Lösung %1 von %2 angewendet.")
                           .arg(m_currentSolutionIndex + 1).arg(m_cachedArcSolutions.size()));
}

void ContourSegmentEditorDialog::navigateNext() {
    saveCurrentStep();
    if (m_currentIndex + 1 < static_cast<int>(m_segments.size())) {
        loadStep(m_currentIndex + 1);
    } else {
        // Neuen Schritt anhängen
        addSegment(Geometry::ContourSegmentType::Line);
    }
}

void ContourSegmentEditorDialog::navigatePrevious() {
    saveCurrentStep();
    if (m_currentIndex > 0) {
        loadStep(m_currentIndex - 1);
    }
}

void ContourSegmentEditorDialog::addSegment(Geometry::ContourSegmentType type) {
    saveCurrentStep();
    double lastX = m_segments.empty() ? 0.0 : m_segments.back().x;
    double lastY = m_segments.empty() ? 0.0 : m_segments.back().y;
    double lastZ = m_segments.empty() ? 0.0 : m_segments.back().z;

    Geometry::ContourSegment newSeg;
    newSeg.type = type;
    newSeg.x = lastX + 20.0;
    newSeg.y = lastY;
    newSeg.z = lastZ;
    if (type == Geometry::ContourSegmentType::ArcCW || type == Geometry::ContourSegmentType::ArcCCW) {
        // Vorschlag: Halbkreis R10 mit Mittelpunkt zwischen Start und Ende
        newSeg.radius = 10.0;
        newSeg.hasCenter = true;
        newSeg.centerX = lastX + 10.0;
        newSeg.centerY = lastY;
    }
    m_segments.push_back(newSeg);

    loadStep(static_cast<int>(m_segments.size()) - 1);
    recompileContour();
}

void ContourSegmentEditorDialog::deleteCurrentSegment() {
    if (m_segments.size() <= 1) return;
    m_segments.erase(m_segments.begin() + m_currentIndex);
    if (m_currentIndex >= static_cast<int>(m_segments.size())) {
        m_currentIndex = static_cast<int>(m_segments.size()) - 1;
    }
    loadStep(m_currentIndex);
    recompileContour();
}

void ContourSegmentEditorDialog::recompileContour() {
    // Erkennen ob die Kontur geschlossen ist:
    // Wenn der letzte Punkt (Line/Arc) auf den Startpunkt zurückführt → geschlossen.
    bool isClosed = false;
    if (m_segments.size() >= 3) {
        const auto& first = m_segments.front();
        const auto& last = m_segments.back();
        if (first.type == Geometry::ContourSegmentType::StartPoint
            && (last.type == Geometry::ContourSegmentType::Line
                || last.type == Geometry::ContourSegmentType::ArcCW
                || last.type == Geometry::ContourSegmentType::ArcCCW)) {
            if (std::abs(first.x - last.x) < 1e-3 && std::abs(first.y - last.y) < 1e-3) {
                isClosed = true;
            }
        }
    }
    m_compiledContour = Geometry::Contour::createFromSegments(m_segments, isClosed);
    emit contourUpdated(m_compiledContour);
}

void ContourSegmentEditorDialog::onMillingTypeChanged(int idx) {
    Q_UNUSED(idx);
    recompileContour();
}

void ContourSegmentEditorDialog::setToolLibrary(const QList<Core::ToolDefinition>& tools) {
    const int currentId = m_cmbTool->currentData().toInt();
    m_isSyncingTechnology = true;
    m_cmbTool->clear();
    for (const auto& t : tools) {
        m_cmbTool->addItem(QString("T%1: %2 (Ø %3 mm)").arg(t.id).arg(t.name).arg(t.diameter, 0, 'f', 1), t.id);
    }
    const int idx = m_cmbTool->findData(currentId);
    if (idx >= 0) m_cmbTool->setCurrentIndex(idx);
    m_isSyncingTechnology = false;
}

void ContourSegmentEditorDialog::setTechnology(int toolId, int contourSide, double feed, double plunge, double rpm, double stepDown) {
    m_isSyncingTechnology = true;
    const int idx = m_cmbTool->findData(toolId);
    if (idx >= 0) m_cmbTool->setCurrentIndex(idx);
    // Auswahl: 0 = Auf Kontur, 1 = Innen, 2 = Außen  ↔  ContourSide: 0 = Außen, 1 = Innen, 2 = Auf Kontur
    m_cmbMillingType->setCurrentIndex(contourSide == 2 ? 0 : (contourSide == 1 ? 1 : 2));
    m_spinFeed->setValue(feed);
    m_spinPlunge->setValue(plunge);
    m_spinRpm->setValue(rpm);
    m_spinPeckDepth->setValue(stepDown);
    m_isSyncingTechnology = false;
}

void ContourSegmentEditorDialog::onTechnologyEdited() {
    if (m_isSyncingTechnology) return;
    constexpr int sideForIndex[] = {2, 1, 0};
    const int idx = std::clamp(m_cmbMillingType->currentIndex(), 0, 2);
    emit technologyChanged(m_cmbTool->currentData().toInt(), sideForIndex[idx],
                           m_spinFeed->value(), m_spinPlunge->value(), m_spinRpm->value(), m_spinPeckDepth->value());
}

bool ContourSegmentEditorDialog::isArcStep(int index) const {
    return index > 0 && index < static_cast<int>(m_segments.size())
        && (m_segments[index].type == Geometry::ContourSegmentType::ArcCW
            || m_segments[index].type == Geometry::ContourSegmentType::ArcCCW);
}

void ContourSegmentEditorDialog::onArcInputEdited() {
    if (m_isLoading || !isArcStep(m_currentIndex)) return;

    // Das bearbeitete Feld gilt als vorgegeben; widersprüchliche Vorgaben werden zu berechneten Werten
    if (auto* edited = qobject_cast<QLineEdit*>(sender())) {
        m_arcKnownFields.insert(edited);
        if (edited == m_editArcRadius) {
            m_arcKnownFields.remove(m_editArcCenterX);
            m_arcKnownFields.remove(m_editArcCenterY);
            m_arcKnownFields.remove(m_editArcSweepAngle);
        } else if (edited == m_editArcCenterX || edited == m_editArcCenterY) {
            m_arcKnownFields.remove(m_editArcRadius);
        } else if (edited == m_editArcSweepAngle) {
            m_arcKnownFields.remove(m_editArcEndX);
            m_arcKnownFields.remove(m_editArcEndY);
            m_arcKnownFields.remove(m_editArcRadius);
        } else if (edited == m_editArcEndX || edited == m_editArcEndY) {
            m_arcKnownFields.remove(m_editArcSweepAngle);
        }
    }

    const auto& prev = m_segments[m_currentIndex - 1];
    Geometry::ArcSolveInput in;
    in.startX = prev.x;
    in.startY = prev.y;
    in.isCW = (m_cmbArcDirection->currentIndex() == 0);
    auto known = [this](QLineEdit* edit) -> std::optional<double> {
        if (!m_arcKnownFields.contains(edit)) return std::nullopt;
        bool ok = false;
        const double v = edit->text().trimmed().replace(QLatin1Char(','), QLatin1Char('.')).toDouble(&ok);
        return ok ? std::optional<double>(v) : std::nullopt;
    };
    in.endX = known(m_editArcEndX);
    in.endY = known(m_editArcEndY);
    in.centerX = known(m_editArcCenterX);
    in.centerY = known(m_editArcCenterY);
    in.radius = known(m_editArcRadius);
    in.sweepAngleDeg = known(m_editArcSweepAngle);

    const auto solutions = Geometry::ContourSolver::solveArc(in);
    if (solutions.empty()) {
        m_cachedArcSolutions.clear();
        m_lblCalcStatus->setText(QStringLiteral("Bogen: Endpunkt + Mittelpunkt, Mittelpunkt + Winkel oder Endpunkt + Radius eingeben..."));
        return;
    }
    const bool sameCount = solutions.size() == m_cachedArcSolutions.size();
    m_cachedArcSolutions = solutions;
    if (!sameCount) m_currentSolutionIndex = 0;
    applyArcSolution();
}

void ContourSegmentEditorDialog::applyArcSolution() {
    if (m_cachedArcSolutions.empty() || !isArcStep(m_currentIndex)) return;
    const int count = static_cast<int>(m_cachedArcSolutions.size());
    const int idx = ((m_currentSolutionIndex % count) + count) % count;
    const auto& sol = m_cachedArcSolutions[idx];

    auto& seg = m_segments[m_currentIndex];
    seg.type = (m_cmbArcDirection->currentIndex() == 0) ? Geometry::ContourSegmentType::ArcCW
                                                        : Geometry::ContourSegmentType::ArcCCW;
    seg.x = sol.endX;
    seg.y = sol.endY;
    seg.centerX = sol.centerX;
    seg.centerY = sol.centerY;
    seg.radius = sol.radius;
    seg.hasCenter = true;

    // Berechnete (nicht vorgegebene) Felder anzeigen
    m_isLoading = true;
    auto show = [this](QLineEdit* edit, double value, int decimals) {
        if (!m_arcKnownFields.contains(edit)) edit->setText(QString::number(value, 'f', decimals));
    };
    show(m_editArcEndX, sol.endX, 4);
    show(m_editArcEndY, sol.endY, 4);
    show(m_editArcCenterX, sol.centerX, 4);
    show(m_editArcCenterY, sol.centerY, 4);
    show(m_editArcRadius, sol.radius, 4);
    show(m_editArcSweepAngle, sol.sweepAngleDeg, 2);
    m_isLoading = false;

    QString status = QString("⚡ Bogen gelöst: Mittelpunkt (%1, %2) | R %3 | Winkel %4°")
        .arg(sol.centerX, 0, 'f', 3).arg(sol.centerY, 0, 'f', 3)
        .arg(sol.radius, 0, 'f', 3).arg(sol.sweepAngleDeg, 0, 'f', 1);
    if (count > 1) status += QString("  [Lösung %1/%2 – F5 nächste Lösung]").arg(idx + 1).arg(count);
    m_lblCalcStatus->setText(status);
    recompileContour();
}

void ContourSegmentEditorDialog::updateContextPrompt() {
    if (m_currentIndex == 0) {
        emit promptChanged(QStringLiteral("Startpunkt der Kontur eingeben (X START, Y START)."));
    } else {
        emit promptChanged(QStringLiteral("Endpunkt eingeben, falls bekannt. (Oder Länge / Winkel eingeben)."));
    }
}

} // namespace GeminiCNC::UI
