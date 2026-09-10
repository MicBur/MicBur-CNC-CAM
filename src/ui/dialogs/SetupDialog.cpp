#include "SetupDialog.h"
#include "geometry/StlLoader.h"
#include "geometry/DxfLoader.h"
#include "geometry/ObjLoader.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <QFileDialog>
#include <QMessageBox>

namespace GeminiCNC::UI {

SetupDialog::SetupDialog(QWidget* parent) : QWidget(parent) {
    auto* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(6, 6, 6, 6);
    mainLayout->setSpacing(10);

    // ═══════════════════════════════════════════════════════
    // Gruppe 1: Fertigteil-Import
    // ═══════════════════════════════════════════════════════
    auto* partGroup = new QGroupBox(QStringLiteral("1. Fertigteil / Ziel-Bauteil"), this);
    auto* partLayout = new QVBoxLayout(partGroup);

    auto* btnImportPart = new QPushButton(QStringLiteral("📂 Bauteil laden (STL oder DXF)..."), this);
    btnImportPart->setStyleSheet("padding: 8px; font-weight: bold; background-color: #3182CE; color: white; border-radius: 4px;");
    connect(btnImportPart, &QPushButton::clicked, this, &SetupDialog::onImportPartClicked);
    partLayout->addWidget(btnImportPart);

    m_lblPartInfo = new QLabel(QStringLiteral("Kein Bauteil geladen."), this);
    m_lblPartInfo->setStyleSheet("color: #A0AEC0; font-size: 11px;");
    partLayout->addWidget(m_lblPartInfo);

    mainLayout->addWidget(partGroup);

    // ═══════════════════════════════════════════════════════
    // Gruppe 2: Rohteil-Geometrie (Hurco WinMax-Stil)
    // ═══════════════════════════════════════════════════════
    auto* stockGroup = new QGroupBox(QStringLiteral("2. Rohteil-Geometrie"), this);
    auto* stockLayout = new QVBoxLayout(stockGroup);

    // Rohteil-Typ Auswahl
    auto* typeRow = new QHBoxLayout();
    typeRow->addWidget(new QLabel(QStringLiteral("Rohteil-Typ:"), this));
    m_cmbStockType = new QComboBox(this);
    m_cmbStockType->addItems({
        QStringLiteral("Quader (Box)"),
        QStringLiteral("Zylinder"),
        QStringLiteral("STL-Datei")
    });
    m_cmbStockType->setStyleSheet("padding: 4px; font-weight: bold;");
    typeRow->addWidget(m_cmbStockType, 1);
    
    typeRow->addWidget(new QLabel(QStringLiteral(" Material:"), this));
    m_cmbMaterialPreset = new QComboBox(this);
    m_cmbMaterialPreset->addItems({
        QStringLiteral("Aluminium"),
        QStringLiteral("Messing"),
        QStringLiteral("Stahl"),
        QStringLiteral("Holz"),
        QStringLiteral("POM"),
        QStringLiteral("Edelstahl")
    });
    // Default ist Alu (0 im SetupDialog, aber 0 im Viewport = Alu)
    m_cmbMaterialPreset->setCurrentIndex(0); 
    connect(m_cmbMaterialPreset, &QComboBox::currentIndexChanged, this, &SetupDialog::onMaterialPresetChanged);
    typeRow->addWidget(m_cmbMaterialPreset, 1);
    
    stockLayout->addLayout(typeRow);

    // Parameter-Stack (3 Seiten)
    m_stockParamStack = new QStackedWidget(this);

    // ─── Seite 0: Box ───
    auto* boxPage = new QWidget(this);
    auto* boxLayout = new QHBoxLayout(boxPage);
    boxLayout->setContentsMargins(0, 4, 0, 4);

    m_spinStockX = new QDoubleSpinBox(this);
    m_spinStockX->setRange(1.0, 2000.0);
    m_spinStockX->setValue(100.0);
    m_spinStockX->setPrefix("X: ");
    m_spinStockX->setSuffix(" mm");

    m_spinStockY = new QDoubleSpinBox(this);
    m_spinStockY->setRange(1.0, 2000.0);
    m_spinStockY->setValue(80.0);
    m_spinStockY->setPrefix("Y: ");
    m_spinStockY->setSuffix(" mm");

    m_spinStockZ = new QDoubleSpinBox(this);
    m_spinStockZ->setRange(1.0, 500.0);
    m_spinStockZ->setValue(30.0);
    m_spinStockZ->setPrefix("Z: ");
    m_spinStockZ->setSuffix(" mm");

    boxLayout->addWidget(m_spinStockX);
    boxLayout->addWidget(m_spinStockY);
    boxLayout->addWidget(m_spinStockZ);
    m_stockParamStack->addWidget(boxPage);

    // ─── Seite 1: Zylinder ───
    auto* cylPage = new QWidget(this);
    auto* cylLayout = new QVBoxLayout(cylPage);
    cylLayout->setContentsMargins(0, 4, 0, 4);

    auto* cylRow1 = new QHBoxLayout();
    m_spinStockRadius = new QDoubleSpinBox(this);
    m_spinStockRadius->setRange(1.0, 1000.0);
    m_spinStockRadius->setValue(25.0);
    m_spinStockRadius->setPrefix("Ø ");
    m_spinStockRadius->setSuffix(" mm");
    m_spinStockRadius->setDecimals(1);

    m_spinStockCylHeight = new QDoubleSpinBox(this);
    m_spinStockCylHeight->setRange(1.0, 500.0);
    m_spinStockCylHeight->setValue(20.0);
    m_spinStockCylHeight->setPrefix("H: ");
    m_spinStockCylHeight->setSuffix(" mm");
    m_spinStockCylHeight->setDecimals(1);

    cylRow1->addWidget(new QLabel(QStringLiteral("Radius:"), this));
    cylRow1->addWidget(m_spinStockRadius);
    cylRow1->addWidget(new QLabel(QStringLiteral("Höhe:"), this));
    cylRow1->addWidget(m_spinStockCylHeight);
    cylLayout->addLayout(cylRow1);

    auto* cylRow2 = new QHBoxLayout();
    m_cmbCylAxis = new QComboBox(this);
    m_cmbCylAxis->addItems({
        QStringLiteral("Z (Vertikal — Standard)"),
        QStringLiteral("X (Horizontal)"),
        QStringLiteral("Y (Horizontal)")
    });

    m_cmbCylDir = new QComboBox(this);
    m_cmbCylDir->addItems({
        QStringLiteral("Negativ (−Z Standard CNC)"),
        QStringLiteral("Positiv (+)")
    });

    cylRow2->addWidget(new QLabel(QStringLiteral("Achse:"), this));
    cylRow2->addWidget(m_cmbCylAxis, 1);
    cylRow2->addWidget(new QLabel(QStringLiteral("Richtung:"), this));
    cylRow2->addWidget(m_cmbCylDir, 1);
    cylLayout->addLayout(cylRow2);
    m_stockParamStack->addWidget(cylPage);

    // ─── Seite 2: STL-Datei ───
    auto* stlPage = new QWidget(this);
    auto* stlLayout = new QVBoxLayout(stlPage);
    stlLayout->setContentsMargins(0, 4, 0, 4);

    auto* btnImportStock = new QPushButton(QStringLiteral("📂 Rohteil aus STL laden..."), this);
    btnImportStock->setStyleSheet("padding: 6px; background-color: #4A5568; color: white; border-radius: 4px;");
    connect(btnImportStock, &QPushButton::clicked, this, &SetupDialog::onImportStockClicked);
    stlLayout->addWidget(btnImportStock);
    m_stockParamStack->addWidget(stlPage);

    stockLayout->addWidget(m_stockParamStack);

    // Erzeugen-Button (wird bei STL-Seite ausgeblendet)
    m_btnGenStock = new QPushButton(QStringLiteral("⚙ Rohteil erzeugen"), this);
    m_btnGenStock->setStyleSheet("padding: 6px; background-color: #DD6B20; color: white; font-weight: bold; border-radius: 4px;");
    connect(m_btnGenStock, &QPushButton::clicked, this, &SetupDialog::onGenerateStockClicked);
    stockLayout->addWidget(m_btnGenStock);

    // Stock Type Combo → Seite wechseln, Button ein-/ausblenden
    connect(m_cmbStockType, &QComboBox::currentIndexChanged, this, [this](int index) {
        m_stockParamStack->setCurrentIndex(index);
        m_btnGenStock->setVisible(index < 2); // Kein Erzeugen-Button für STL
    });

    m_lblStockInfo = new QLabel(QStringLiteral("Kein Rohteil geladen."), this);
    m_lblStockInfo->setStyleSheet("color: #A0AEC0; font-size: 11px;");
    stockLayout->addWidget(m_lblStockInfo);

    mainLayout->addWidget(stockGroup);

    // ═══════════════════════════════════════════════════════
    // Gruppe 3: Nullpunkt & Ausrichtung
    // ═══════════════════════════════════════════════════════
    auto* alignGroup = new QGroupBox(QStringLiteral("3. Werkstück-Nullpunkt (WCS)"), this);
    auto* alignLayout = new QVBoxLayout(alignGroup);

    // ─── Referenzpunkt (Hurco WinMax Style) ───
    auto* refRow = new QHBoxLayout();
    refRow->addWidget(new QLabel(QStringLiteral("Referenzpunkt:"), this));
    m_cmbRefPoint = new QComboBox(this);
    m_cmbRefPoint->addItems({
        QStringLiteral("Mitte Oben"),              // 0: XY-Mitte, Z=Oberkante
        QStringLiteral("Vorne Links Oben"),         // 1: X-min, Y-min, Z=Oberkante
        QStringLiteral("Vorne Rechts Oben"),        // 2: X-max, Y-min, Z=Oberkante
        QStringLiteral("Hinten Links Oben"),        // 3: X-min, Y-max, Z=Oberkante
        QStringLiteral("Hinten Rechts Oben"),       // 4: X-max, Y-max, Z=Oberkante
        QStringLiteral("Mitte Unten"),              // 5: XY-Mitte, Z=Unterkante
        QStringLiteral("Vorne Links Unten"),        // 6: X-min, Y-min, Z=Unterkante
        QStringLiteral("Vorne Rechts Unten"),       // 7: X-max, Y-min, Z=Unterkante
        QStringLiteral("Hinten Links Unten"),       // 8: X-min, Y-max, Z=Unterkante
        QStringLiteral("Hinten Rechts Unten")       // 9: X-max, Y-max, Z=Unterkante
    });
    m_cmbRefPoint->setCurrentIndex(1); // Standard: Vorne Links Oben
    connect(m_cmbRefPoint, &QComboBox::currentIndexChanged, this, &SetupDialog::onAlignOriginClicked);
    refRow->addWidget(m_cmbRefPoint);
    alignLayout->addLayout(refRow);

    // Die alten Checkboxen bleiben intern, werden aber ausgeblendet
    m_chkCenterXY = new QCheckBox(QStringLiteral("Werkstück mittig auf XY ausrichten"), this);
    m_chkCenterXY->setChecked(true);
    m_chkCenterXY->setVisible(false);

    m_chkZeroTopZ = new QCheckBox(QStringLiteral("Z=0 an Bauteil-Oberkante setzen"), this);
    m_chkZeroTopZ->setChecked(true);
    m_chkZeroTopZ->setVisible(false);

    // ─── Manuelle Nullpunkt-Verschiebung (G54-Offset) ───
    auto* originRow = new QHBoxLayout();
    auto makeOriginSpin = [this](const QString& suffix) {
        auto* sp = new QDoubleSpinBox(this);
        sp->setRange(-2000, 2000);
        sp->setDecimals(3);
        sp->setValue(0.0);
        sp->setSuffix(suffix);
        sp->setSingleStep(0.1);
        connect(sp, &QDoubleSpinBox::valueChanged, this, &SetupDialog::onAlignOriginClicked);
        return sp;
    };
    m_spinOriginX = makeOriginSpin(" mm");
    m_spinOriginY = makeOriginSpin(" mm");
    m_spinOriginZ = makeOriginSpin(" mm");

    originRow->addWidget(new QLabel(QStringLiteral("X₀:"), this));
    originRow->addWidget(m_spinOriginX);
    originRow->addWidget(new QLabel(QStringLiteral("Y₀:"), this));
    originRow->addWidget(m_spinOriginY);
    originRow->addWidget(new QLabel(QStringLiteral("Z₀:"), this));
    originRow->addWidget(m_spinOriginZ);
    alignLayout->addLayout(originRow);

    auto* rotLayout = new QHBoxLayout();
    auto* btnRotX = new QPushButton(QStringLiteral("↻ +90° X"), this);
    auto* btnRotY = new QPushButton(QStringLiteral("↻ +90° Y"), this);
    auto* btnRotZ = new QPushButton(QStringLiteral("↻ +90° Z"), this);
    QString rotStyle = "padding: 5px; background-color: #2C5282; color: white; border-radius: 4px; font-weight: bold;";
    btnRotX->setStyleSheet(rotStyle);
    btnRotY->setStyleSheet(rotStyle);
    btnRotZ->setStyleSheet(rotStyle);

    connect(btnRotX, &QPushButton::clicked, this, [this]() {
        if (!m_partMesh.isEmpty()) {
            m_partMesh.rotateX(90.0, true);
            onAlignOriginClicked();
        }
    });
    connect(btnRotY, &QPushButton::clicked, this, [this]() {
        if (!m_partMesh.isEmpty()) {
            m_partMesh.rotateY(90.0, true);
            onAlignOriginClicked();
        }
    });
    connect(btnRotZ, &QPushButton::clicked, this, [this]() {
        if (!m_partMesh.isEmpty()) {
            m_partMesh.rotateZ(90.0, true);
            onAlignOriginClicked();
        }
    });

    rotLayout->addWidget(btnRotX);
    rotLayout->addWidget(btnRotY);
    rotLayout->addWidget(btnRotZ);
    alignLayout->addLayout(rotLayout);

    auto* btnAlign = new QPushButton(QStringLiteral("⌖ Nullpunkt neu ausrichten"), this);
    btnAlign->setStyleSheet("padding: 6px; background-color: #38A169; color: white; border-radius: 4px;");
    connect(btnAlign, &QPushButton::clicked, this, &SetupDialog::onAlignOriginClicked);
    alignLayout->addWidget(btnAlign);

    mainLayout->addWidget(alignGroup);

    // ═══════════════════════════════════════════════════════
    // Gruppe 4: Maschinenparameter
    // ═══════════════════════════════════════════════════════
    auto* machineGroup = new QGroupBox(QStringLiteral("4. Maschinen-Parameter"), this);
    auto* machineLayout = new QFormLayout(machineGroup);
    machineLayout->setLabelAlignment(Qt::AlignRight);

    // Preset-Auswahl
    m_cmbMachinePreset = new QComboBox(this);
    m_cmbMachinePreset->addItems({
        QStringLiteral("CNC 6040 (Standard)"),
        QStringLiteral("CNC 3018 (Klein)"),
        QStringLiteral("Portalfräse Groß"),
        QStringLiteral("Hurco VMX 30"),
        QStringLiteral("Benutzerdefiniert")
    });
    m_cmbMachinePreset->setStyleSheet("padding: 4px; font-weight: bold;");
    connect(m_cmbMachinePreset, &QComboBox::currentIndexChanged, this, &SetupDialog::onMachinePresetChanged);
    machineLayout->addRow(QStringLiteral("Maschinen-Typ:"), m_cmbMachinePreset);

    // Verfahrwege
    auto* travelRow = new QHBoxLayout();
    auto makeSpinTravel = [this](const QString& prefix, double val, double maxVal) {
        auto* spin = new QDoubleSpinBox(this);
        spin->setRange(1.0, maxVal);
        spin->setValue(val);
        spin->setPrefix(prefix);
        spin->setSuffix(" mm");
        spin->setDecimals(0);
        connect(spin, &QDoubleSpinBox::valueChanged, this, &SetupDialog::applyMachineUiToConfig);
        return spin;
    };
    m_spinTravelX = makeSpinTravel("X: ", 600.0, 5000.0);
    m_spinTravelY = makeSpinTravel("Y: ", 400.0, 5000.0);
    m_spinTravelZ = makeSpinTravel("Z: ", 80.0, 1000.0);
    travelRow->addWidget(m_spinTravelX);
    travelRow->addWidget(m_spinTravelY);
    travelRow->addWidget(m_spinTravelZ);
    machineLayout->addRow(QStringLiteral("Verfahrwege:"), travelRow);

    // Eilgang
    auto* rapidRow = new QHBoxLayout();
    m_spinRapidXY = new QDoubleSpinBox(this);
    m_spinRapidXY->setRange(100.0, 50000.0);
    m_spinRapidXY->setValue(4000.0);
    m_spinRapidXY->setPrefix("XY: ");
    m_spinRapidXY->setSuffix(" mm/min");
    m_spinRapidXY->setDecimals(0);
    connect(m_spinRapidXY, &QDoubleSpinBox::valueChanged, this, &SetupDialog::applyMachineUiToConfig);

    m_spinRapidZ = new QDoubleSpinBox(this);
    m_spinRapidZ->setRange(100.0, 50000.0);
    m_spinRapidZ->setValue(2000.0);
    m_spinRapidZ->setPrefix("Z: ");
    m_spinRapidZ->setSuffix(" mm/min");
    m_spinRapidZ->setDecimals(0);
    connect(m_spinRapidZ, &QDoubleSpinBox::valueChanged, this, &SetupDialog::applyMachineUiToConfig);

    rapidRow->addWidget(m_spinRapidXY);
    rapidRow->addWidget(m_spinRapidZ);
    machineLayout->addRow(QStringLiteral("Max. Eilgang:"), rapidRow);

    // Spindel
    auto* spindleRow = new QHBoxLayout();
    m_spinSpindleMin = new QDoubleSpinBox(this);
    m_spinSpindleMin->setRange(0.0, 100000.0);
    m_spinSpindleMin->setValue(3000.0);
    m_spinSpindleMin->setPrefix("Min: ");
    m_spinSpindleMin->setSuffix(" RPM");
    m_spinSpindleMin->setDecimals(0);
    connect(m_spinSpindleMin, &QDoubleSpinBox::valueChanged, this, &SetupDialog::applyMachineUiToConfig);

    m_spinSpindleMax = new QDoubleSpinBox(this);
    m_spinSpindleMax->setRange(100.0, 100000.0);
    m_spinSpindleMax->setValue(24000.0);
    m_spinSpindleMax->setPrefix("Max: ");
    m_spinSpindleMax->setSuffix(" RPM");
    m_spinSpindleMax->setDecimals(0);
    connect(m_spinSpindleMax, &QDoubleSpinBox::valueChanged, this, &SetupDialog::applyMachineUiToConfig);

    spindleRow->addWidget(m_spinSpindleMin);
    spindleRow->addWidget(m_spinSpindleMax);
    machineLayout->addRow(QStringLiteral("Spindel-Drehzahl:"), spindleRow);

    // Beschleunigung
    m_spinAccel = new QDoubleSpinBox(this);
    m_spinAccel->setRange(10.0, 10000.0);
    m_spinAccel->setValue(500.0);
    m_spinAccel->setSuffix(" mm/s²");
    m_spinAccel->setDecimals(0);
    connect(m_spinAccel, &QDoubleSpinBox::valueChanged, this, &SetupDialog::applyMachineUiToConfig);
    machineLayout->addRow(QStringLiteral("Beschleunigung:"), m_spinAccel);

    mainLayout->addWidget(machineGroup);

    mainLayout->addStretch(1);

    // Initialen Standard-Rohteilquader erzeugen + CNC 6040 Preset laden
    onGenerateStockClicked();
    onMachinePresetChanged(0);
}

void SetupDialog::onImportPartClicked() {
    QString filter = QStringLiteral("Geometriedateien (*.stl *.obj *.dxf);;STL 3D-Modell (*.stl);;Wavefront OBJ (*.obj);;DXF 2D-Zeichnung (*.dxf)");
    QString path = QFileDialog::getOpenFileName(this, QStringLiteral("Bauteil importieren"), QString(), filter);
    if (path.isEmpty()) return;

    if (path.endsWith(QStringLiteral(".stl"), Qt::CaseInsensitive)) {
        auto res = Geometry::StlLoader::loadFromFile(path, Geometry::MeshRole::TargetPart);
        if (!res.success) {
            QMessageBox::warning(this, QStringLiteral("STL-Import fehlgeschlagen"), res.errorMessage);
            return;
        }
        m_partMesh = res.mesh;
        m_partMesh.alignToOrigin(m_chkCenterXY->isChecked(), m_chkZeroTopZ->isChecked());
        m_contours.clear();
        emit partMeshChanged(m_partMesh);
    } else if (path.endsWith(QStringLiteral(".obj"), Qt::CaseInsensitive)) {
        auto res = Geometry::ObjLoader::loadFromFile(path, Geometry::MeshRole::TargetPart);
        if (!res.success) {
            QMessageBox::warning(this, QStringLiteral("OBJ-Import fehlgeschlagen"), res.errorMessage);
            return;
        }
        m_partMesh = res.mesh;
        m_partMesh.alignToOrigin(m_chkCenterXY->isChecked(), m_chkZeroTopZ->isChecked());
        m_contours.clear();
        emit partMeshChanged(m_partMesh);
    } else if (path.endsWith(QStringLiteral(".dxf"), Qt::CaseInsensitive)) {
        auto res = Geometry::DxfLoader::loadFromFile(path);
        if (!res.success) {
            QMessageBox::warning(this, QStringLiteral("DXF-Import fehlgeschlagen"), res.errorMessage);
            return;
        }
        m_contours = res.contours;
        m_partMesh = Geometry::DxfLoader::extrudeContours(m_contours, 10.0, Geometry::MeshRole::TargetPart);
        m_partMesh.alignToOrigin(m_chkCenterXY->isChecked(), m_chkZeroTopZ->isChecked());
        emit contoursChanged(m_contours);
        emit partMeshChanged(m_partMesh);
    }

    onAlignOriginClicked();
    updateInfoLabels();
}

void SetupDialog::onImportStockClicked() {
    QString path = QFileDialog::getOpenFileName(this, QStringLiteral("Rohteil STL laden"), QString(), QStringLiteral("STL Dateien (*.stl)"));
    if (path.isEmpty()) return;

    auto res = Geometry::StlLoader::loadFromFile(path, Geometry::MeshRole::Stock);
    if (!res.success) {
        QMessageBox::warning(this, QStringLiteral("Rohteil-Import Fehler"), res.errorMessage);
        return;
    }

    m_stockMesh = res.mesh;
    onAlignOriginClicked();
    updateInfoLabels();
    emit stockMeshChanged(m_stockMesh);
}

void SetupDialog::onGenerateStockClicked() {
    int stockType = m_cmbStockType->currentIndex();

    if (stockType == 0) {
        // Quader (Box)
        double wx = m_spinStockX->value();
        double dy = m_spinStockY->value();
        double hz = m_spinStockZ->value();
        m_stockMesh = Geometry::Mesh::createBoxStock(wx, dy, hz, {-wx * 0.5, -dy * 0.5, -hz});
    } else if (stockType == 1) {
        // Zylinder
        double radius = m_spinStockRadius->value();
        double height = m_spinStockCylHeight->value();
        int axis = m_cmbCylAxis->currentIndex(); // 0=Z, 1=X, 2=Y
        bool positive = (m_cmbCylDir->currentIndex() == 1);
        m_stockMesh = Geometry::Mesh::createCylinderStock(radius, height, 48, axis, positive);
    }
    // stockType == 2 (STL) wird über onImportStockClicked() geladen

    onAlignOriginClicked();
    updateInfoLabels();
    emit stockMeshChanged(m_stockMesh);
}

void SetupDialog::onMaterialPresetChanged(int index) {
    emit materialPresetChanged(index);
}

void SetupDialog::onAlignOriginClicked() {
    int ref = m_cmbRefPoint->currentIndex();

    // Referenzpunkt → Alignment-Logik
    // Index 0-4: Z=Oberkante (zeroTopZ=true)
    // Index 5-9: Z=Unterkante (zeroTopZ=false → minZ=0)
    // Index 0,5: XY-Mitte (centerXY=true)
    // Index 1-4, 6-9: Ecke (centerXY=false → minX=minY=0, dann Eckenshift)
    bool zeroTop = (ref < 5);
    bool centerXY = (ref == 0 || ref == 5);

    // Manueller Offset (G54-Verschiebung)
    double ox = m_spinOriginX->value();
    double oy = m_spinOriginY->value();
    double oz = m_spinOriginZ->value();

    auto alignMesh = [&](Geometry::Mesh& mesh) {
        if (mesh.isEmpty()) return;

        // Basisausrichtung: alignToOrigin setzt je nach Modus
        // centerXY=false → minX=0, minY=0 (Vorne Links)
        // centerXY=true → Mitte bei 0,0
        // zeroTop=true → maxZ=0 (Oberkante)
        // zeroTop=false → minZ=0 (Unterkante)
        mesh.alignToOrigin(centerXY, zeroTop);

        // Ecken-Verschiebung für nicht-Mitte-Referenzpunkte
        // Nach alignToOrigin(false, ...) ist minX=0, minY=0
        // Für "Rechts" brauchen wir maxX=0, für "Hinten" maxY=0
        if (!centerXY) {
            mesh.computeBoundingBox();
            auto& bb = mesh.boundingBox;
            double shiftX = 0, shiftY = 0;
            int cornerType = ref < 5 ? ref : ref - 5; // 1=VL, 2=VR, 3=HL, 4=HR

            switch (cornerType) {
                case 1: // Vorne Links → minX=0, minY=0 (default)
                    break;
                case 2: // Vorne Rechts → maxX=0, minY=0
                    shiftX = -bb.maxPoint.x;
                    break;
                case 3: // Hinten Links → minX=0, maxY=0
                    shiftY = -bb.maxPoint.y;
                    break;
                case 4: // Hinten Rechts → maxX=0, maxY=0
                    shiftX = -bb.maxPoint.x;
                    shiftY = -bb.maxPoint.y;
                    break;
            }
            if (std::abs(shiftX) > 1e-6 || std::abs(shiftY) > 1e-6)
                mesh.translate({shiftX, shiftY, 0.0});
        }

        // Manuellen Offset anwenden
        if (std::abs(ox) > 1e-6 || std::abs(oy) > 1e-6 || std::abs(oz) > 1e-6)
            mesh.translate({-ox, -oy, -oz});
    };

    if (!m_partMesh.isEmpty()) {
        alignMesh(m_partMesh);
        emit partMeshChanged(m_partMesh);
    }
    if (!m_stockMesh.isEmpty()) {
        alignMesh(m_stockMesh);
        emit stockMeshChanged(m_stockMesh);
    }

    updateInfoLabels();
}

void SetupDialog::onMachinePresetChanged(int index) {
    m_isUpdatingMachineUi = true;

    Core::MachineConfig preset;
    switch (index) {
        case 0: preset = Core::MachineConfig::presetCNC6040(); break;
        case 1: preset = Core::MachineConfig::presetCNC3018(); break;
        case 2: preset = Core::MachineConfig::presetLargePortal(); break;
        case 3: preset = Core::MachineConfig::presetHurcoVMX30(); break;
        default: // Benutzerdefiniert: UI-Werte beibehalten
            m_isUpdatingMachineUi = false;
            return;
    }

    // UI mit Preset-Werten befüllen
    m_spinTravelX->setValue(preset.maxLimits.x - preset.minLimits.x);
    m_spinTravelY->setValue(preset.maxLimits.y - preset.minLimits.y);
    m_spinTravelZ->setValue(std::abs(preset.minLimits.z)); // Schnitttiefe (unter Z=0)
    m_spinRapidXY->setValue(preset.maxRapidSpeeds.x);
    m_spinRapidZ->setValue(preset.maxRapidSpeeds.z);
    m_spinSpindleMin->setValue(preset.minSpindleRpm);
    m_spinSpindleMax->setValue(preset.maxSpindleRpm);
    m_spinAccel->setValue(preset.maxAcceleration);

    m_machineConfig = preset;
    m_isUpdatingMachineUi = false;

    emit machineConfigChanged(m_machineConfig);
}

void SetupDialog::applyMachineUiToConfig() {
    if (m_isUpdatingMachineUi) return;

    // Preset auf "Benutzerdefiniert" setzen, wenn der User manuell ändert
    if (m_cmbMachinePreset->currentIndex() != 4) {
        m_isUpdatingMachineUi = true;
        m_cmbMachinePreset->setCurrentIndex(4);
        m_isUpdatingMachineUi = false;
    }

    double travelX = m_spinTravelX->value();
    double travelY = m_spinTravelY->value();
    double travelZ = m_spinTravelZ->value();

    m_machineConfig.machineName = QStringLiteral("Benutzerdefiniert");
    m_machineConfig.minLimits = Core::Vector3D(-travelX * 0.5, -travelY * 0.5, -travelZ, -360.0);
    m_machineConfig.maxLimits = Core::Vector3D(travelX * 0.5, travelY * 0.5, 50.0, 360.0);
    m_machineConfig.maxRapidSpeeds = Core::Vector3D(m_spinRapidXY->value(), m_spinRapidXY->value(),
                                                     m_spinRapidZ->value(), 7200.0);
    m_machineConfig.maxAcceleration = m_spinAccel->value();
    m_machineConfig.minSpindleRpm = m_spinSpindleMin->value();
    m_machineConfig.maxSpindleRpm = m_spinSpindleMax->value();

    emit machineConfigChanged(m_machineConfig);
}

void SetupDialog::updateInfoLabels() {
    if (!m_partMesh.isEmpty()) {
        const auto& b = m_partMesh.boundingBox;
        m_lblPartInfo->setText(QString("Bauteil: %1 Dreiecke\nAbmessungen: %2 x %3 x %4 mm")
            .arg(m_partMesh.triangleCount())
            .arg(b.widthX(), 0, 'f', 1)
            .arg(b.depthY(), 0, 'f', 1)
            .arg(b.heightZ(), 0, 'f', 1));
    }

    if (!m_stockMesh.isEmpty()) {
        const auto& b = m_stockMesh.boundingBox;
        QString typeStr;
        switch (m_cmbStockType->currentIndex()) {
            case 0: typeStr = "Quader"; break;
            case 1: typeStr = "Zylinder"; break;
            case 2: typeStr = "STL"; break;
            default: typeStr = "Rohteil"; break;
        }
        m_lblStockInfo->setText(QString("%1: %2 Dreiecke\nAbmessungen: %3 x %4 x %5 mm")
            .arg(typeStr)
            .arg(m_stockMesh.triangleCount())
            .arg(b.widthX(), 0, 'f', 1)
            .arg(b.depthY(), 0, 'f', 1)
            .arg(b.heightZ(), 0, 'f', 1));
    }
}

} // namespace GeminiCNC::UI
