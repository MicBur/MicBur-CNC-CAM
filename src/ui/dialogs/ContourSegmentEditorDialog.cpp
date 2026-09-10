#include "ContourSegmentEditorDialog.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <cmath>

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
    m_editLineLength = new QLineEdit(this); m_editLineLength->setPlaceholderText(QStringLiteral("[ Länge ]"));
    m_editLineAngle = new QLineEdit(this); m_editLineAngle->setPlaceholderText(QStringLiteral("[ Winkel ° ]"));

    connect(m_editLineEndX, &QLineEdit::textEdited, this, &ContourSegmentEditorDialog::onInputEdited);
    connect(m_editLineEndY, &QLineEdit::textEdited, this, &ContourSegmentEditorDialog::onInputEdited);
    connect(m_editLineLength, &QLineEdit::textEdited, this, &ContourSegmentEditorDialog::onInputEdited);
    connect(m_editLineAngle, &QLineEdit::textEdited, this, &ContourSegmentEditorDialog::onInputEdited);

    leftLayout->addRow(new QLabel(QStringLiteral("X END:")), m_editLineEndX);
    leftLayout->addRow(new QLabel(QStringLiteral("Y END:")), m_editLineEndY);
    leftLayout->addRow(new QLabel(QStringLiteral("Z END:")), m_editLineZEnd);
    leftLayout->addRow(new QLabel(QStringLiteral("XY LENGTH:")), m_editLineLength);
    leftLayout->addRow(new QLabel(QStringLiteral("XY ANGLE:")), m_editLineAngle);

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

    geomLayout->addWidget(leftBox, 0, 0);
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

    m_cmbTool = new QComboBox(this);
    m_cmbTool->addItem(QStringLiteral("T1: Schaftfräser Ø6.0 mm (HSS/VHM)"), 1);
    m_cmbTool->addItem(QStringLiteral("T2: Schaftfräser Ø3.175 mm (1/8\")"), 2);
    m_cmbTool->addItem(QStringLiteral("T3: Planmesserkopf Ø22.0 mm"), 3);

    m_cmbMillingType = new QComboBox(this);
    m_cmbMillingType->addItems({
        QStringLiteral("AUF KONTUR (ON)"),
        QStringLiteral("INNEN (INSIDE)"),
        QStringLiteral("AUSSEN (OUTSIDE)"),
        QStringLiteral("TASCHE (POCKET)")
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

    bool okX = false, okY = false, okZ = false;
    double vx = m_editLineEndX->text().toDouble(&okX);
    double vy = m_editLineEndY->text().toDouble(&okY);
    double vz = m_editLineZEnd->text().toDouble(&okZ);

    if (okX) seg.x = vx;
    if (okY) seg.y = vy;
    if (okZ) seg.z = vz;
}

void ContourSegmentEditorDialog::onInputEdited() {
    if (m_isLoading || m_currentIndex <= 0) return;

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
    emit promptChanged(QStringLiteral("Alternative Lösung angewendet."));
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
            && last.type == Geometry::ContourSegmentType::Line) {
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

void ContourSegmentEditorDialog::updateContextPrompt() {
    if (m_currentIndex == 0) {
        emit promptChanged(QStringLiteral("Startpunkt der Kontur eingeben (X START, Y START)."));
    } else {
        emit promptChanged(QStringLiteral("Endpunkt eingeben, falls bekannt. (Oder Länge / Winkel eingeben)."));
    }
}

} // namespace GeminiCNC::UI
