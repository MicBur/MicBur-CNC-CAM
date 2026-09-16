#include "ToolManagerDialog.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QSplitter>
#include <QGroupBox>
#include <QColorDialog>
#include <QMessageBox>
#include <QFont>

namespace GeminiCNC::UI {

ToolManagerDialog::ToolManagerDialog(QWidget* parent) : QWidget(parent) {
    // Gespeicherte Werkzeugbibliothek laden, sonst Standardwerkzeuge
    int activeId = -1;
    if (Core::ToolDefinition::loadLibrary(Core::ToolDefinition::defaultLibraryPath(), m_tools, activeId)) {
        for (int i = 0; i < m_tools.size(); ++i) {
            if (m_tools[i].id == activeId) m_activeToolIndex = i;
        }
    } else {
        m_tools = Core::ToolDefinition::createDefaultLibrary();
    }
    m_materialDb = Core::MaterialDatabase::createDefault();
    setupUi();
    populateToolList();
    if (!m_tools.isEmpty()) {
        m_toolList->setCurrentRow(0);
        loadToolToEditor(0);
    }
}

void ToolManagerDialog::setupUi() {
    auto* mainLayout = new QHBoxLayout(this);
    mainLayout->setContentsMargins(4, 4, 4, 4);
    mainLayout->setSpacing(6);

    // ═══════════════════════════════════════════════════════
    // LINKE SPALTE: Werkzeugliste + Typ + Parameter
    // ═══════════════════════════════════════════════════════
    auto* leftWidget = new QWidget(this);
    auto* leftLayout = new QVBoxLayout(leftWidget);
    leftLayout->setContentsMargins(0, 0, 0, 0);
    leftLayout->setSpacing(6);

    // --- Werkzeugliste ---
    auto* listGroup = new QGroupBox(QStringLiteral("Werkzeugmagazin"), this);
    auto* listLayout = new QVBoxLayout(listGroup);

    m_toolList = new QListWidget(this);
    m_toolList->setStyleSheet(
        "QListWidget { background-color: #111827; color: #E2E8F0; font-family: Consolas; font-size: 12px; "
        "border: 1px solid #1F2937; } "
        "QListWidget::item { padding: 4px 6px; border-bottom: 1px solid #1F2937; } "
        "QListWidget::item:selected { background-color: #1E40AF; color: #FFFFFF; } "
        "QListWidget::item:hover { background-color: #1E293B; }");
    m_toolList->setMinimumHeight(100);
    connect(m_toolList, &QListWidget::currentRowChanged, this, [this](int row) {
        if (row >= 0) {
            if (m_editingIndex >= 0 && m_editingIndex < m_tools.size()) {
                saveEditorToTool(m_editingIndex);
            }
            loadToolToEditor(row);
        }
    });
    listLayout->addWidget(m_toolList);

    auto* listBtnRow = new QHBoxLayout();
    m_btnActivate = new QPushButton(QStringLiteral("✔ Aktivieren"), this);
    m_btnActivate->setStyleSheet("padding: 5px 10px; font-weight: bold; background-color: #38A169; color: white; border-radius: 3px;");
    connect(m_btnActivate, &QPushButton::clicked, this, &ToolManagerDialog::onActivateClicked);

    m_btnAdd = new QPushButton(QStringLiteral("+"), this);
    m_btnAdd->setStyleSheet("padding: 5px 10px; font-weight: bold; background-color: #3182CE; color: white; border-radius: 3px;");
    m_btnAdd->setFixedWidth(36);
    connect(m_btnAdd, &QPushButton::clicked, this, &ToolManagerDialog::onAddToolClicked);

    m_btnDelete = new QPushButton(QStringLiteral("✕"), this);
    m_btnDelete->setStyleSheet("padding: 5px 10px; font-weight: bold; background-color: #E53E3E; color: white; border-radius: 3px;");
    m_btnDelete->setFixedWidth(36);
    connect(m_btnDelete, &QPushButton::clicked, this, &ToolManagerDialog::onDeleteToolClicked);

    listBtnRow->addWidget(m_btnActivate, 1);
    listBtnRow->addWidget(m_btnAdd);
    listBtnRow->addWidget(m_btnDelete);
    listLayout->addLayout(listBtnRow);
    leftLayout->addWidget(listGroup);

    // --- Typ-Auswahl ---
    auto* typeGroup = new QGroupBox(QStringLiteral("Werkzeug-Typ"), this);
    auto* typeLayout = new QHBoxLayout(typeGroup);
    m_cmbType = new QComboBox(this);
    m_cmbType->addItems({
        QStringLiteral("Schaftfräser"),
        QStringLiteral("Kugelkopffräser"),
        QStringLiteral("Fasenfräser"),
        QStringLiteral("Planfräser"),
        QStringLiteral("Bohrer")
    });
    m_cmbType->setStyleSheet("padding: 4px; font-weight: bold; font-size: 12px;");
    connect(m_cmbType, &QComboBox::currentIndexChanged, this, &ToolManagerDialog::onToolTypeChanged);
    typeLayout->addWidget(m_cmbType);
    leftLayout->addWidget(typeGroup);

    // --- Geometrie-Parameter ---
    auto* paramGroup = new QGroupBox(QStringLiteral("Werkzeug-Geometrie"), this);
    auto* paramLayout = new QFormLayout(paramGroup);
    paramLayout->setLabelAlignment(Qt::AlignRight);
    paramLayout->setSpacing(4);

    auto makeSpin = [this](double min, double max, double val, int decimals, const QString& suffix) {
        auto* spin = new QDoubleSpinBox(this);
        spin->setRange(min, max);
        spin->setValue(val);
        spin->setDecimals(decimals);
        spin->setSuffix(suffix);
        spin->setStyleSheet("padding: 3px; font-family: Consolas;");
        connect(spin, &QDoubleSpinBox::valueChanged, this, &ToolManagerDialog::onParameterChanged);
        return spin;
    };

    m_spinDiameter     = makeSpin(0.1, 200.0, 6.0, 2, " mm");
    m_spinFluteLength  = makeSpin(0.5, 200.0, 18.0, 1, " mm");
    m_spinShaftDia     = makeSpin(0.5, 100.0, 6.0, 2, " mm");
    m_spinStickOut     = makeSpin(1.0, 300.0, 28.0, 1, " mm");
    m_spinOverallLength= makeSpin(5.0, 500.0, 50.0, 1, " mm");
    m_spinHolderDia    = makeSpin(5.0, 100.0, 32.0, 1, " mm");

    m_spinFlutes = new QSpinBox(this);
    m_spinFlutes->setRange(1, 12);
    m_spinFlutes->setValue(2);
    m_spinFlutes->setStyleSheet("padding: 3px; font-family: Consolas;");
    connect(m_spinFlutes, &QSpinBox::valueChanged, this, &ToolManagerDialog::onParameterChanged);

    m_btnColor = new QPushButton(QStringLiteral("■ #FFE614"), this);
    m_btnColor->setStyleSheet("padding: 4px 8px; font-weight: bold; background-color: #FFE614; color: black; border-radius: 3px;");
    connect(m_btnColor, &QPushButton::clicked, this, &ToolManagerDialog::onColorClicked);

    m_editName = new QLineEdit(this);
    m_editName->setStyleSheet("padding: 3px; font-weight: bold;");
    connect(m_editName, &QLineEdit::textEdited, this, &ToolManagerDialog::onParameterChanged);
    paramLayout->addRow(QStringLiteral("Bezeichnung:"), m_editName);
    paramLayout->addRow(QStringLiteral("Ø Schneide:"), m_spinDiameter);
    paramLayout->addRow(QStringLiteral("Schneidenlänge:"), m_spinFluteLength);
    paramLayout->addRow(QStringLiteral("Ø Schaft:"), m_spinShaftDia);
    paramLayout->addRow(QStringLiteral("Auskragung:"), m_spinStickOut);
    paramLayout->addRow(QStringLiteral("Gesamtlänge:"), m_spinOverallLength);
    paramLayout->addRow(QStringLiteral("Ø Halter:"), m_spinHolderDia);
    paramLayout->addRow(QStringLiteral("Zähnezahl (z):"), m_spinFlutes);
    paramLayout->addRow(QStringLiteral("Frässpur-Farbe:"), m_btnColor);

    leftLayout->addWidget(paramGroup);
    leftLayout->addStretch(1);

    // ═══════════════════════════════════════════════════════
    // RECHTE SPALTE: Grafik + Schnittdaten
    // ═══════════════════════════════════════════════════════
    auto* rightWidget = new QWidget(this);
    auto* rightLayout = new QVBoxLayout(rightWidget);
    rightLayout->setContentsMargins(0, 0, 0, 0);
    rightLayout->setSpacing(6);

    // --- Grafische Werkzeug-Silhouette ---
    m_toolGraphic = new ToolGraphicsWidget(this);
    rightLayout->addWidget(m_toolGraphic, 3);

    // --- Material-Auswahl ---
    auto* matGroup = new QGroupBox(QStringLiteral("Werkstoff → Schnittdaten-Berechnung"), this);
    auto* matLayout = new QVBoxLayout(matGroup);

    m_cmbMaterial = new QComboBox(this);
    for (const auto& mat : m_materialDb.materials()) {
        m_cmbMaterial->addItem(mat.name, mat.id);
    }
    m_cmbMaterial->setStyleSheet("padding: 4px; font-weight: bold;");
    connect(m_cmbMaterial, &QComboBox::currentIndexChanged, this, &ToolManagerDialog::onMaterialChanged);
    matLayout->addWidget(m_cmbMaterial);

    // Vc-Anzeige
    m_lblVc = new QLabel(QStringLiteral("Vc = 260 m/min"), this);
    m_lblVc->setStyleSheet("color: #94A3B8; font-family: Consolas; font-size: 10px;");
    matLayout->addWidget(m_lblVc);

    rightLayout->addWidget(matGroup);

    // --- Berechnete Schnittdaten (read-only) ---
    auto* cutGroup = new QGroupBox(QStringLiteral("Berechnete Schnittdaten"), this);
    cutGroup->setStyleSheet(
        "QGroupBox { font-weight: bold; color: #00D2FF; border: 1px solid #1E40AF; border-radius: 4px; padding-top: 14px; } "
        "QGroupBox::title { padding: 2px 8px; }");
    auto* cutLayout = new QFormLayout(cutGroup);
    cutLayout->setLabelAlignment(Qt::AlignRight);
    cutLayout->setSpacing(3);

    QString valStyle = "color: #63B3ED; font-family: Consolas; font-size: 13px; font-weight: bold;";
    QString lblStyle = "color: #94A3B8; font-family: Consolas; font-size: 11px;";

    m_lblRpm    = new QLabel("—", this); m_lblRpm->setStyleSheet(valStyle);
    m_lblFeed   = new QLabel("—", this); m_lblFeed->setStyleSheet(valStyle);
    m_lblFz     = new QLabel("—", this); m_lblFz->setStyleSheet(valStyle);
    m_lblPlunge = new QLabel("—", this); m_lblPlunge->setStyleSheet(valStyle);
    m_lblAp     = new QLabel("—", this); m_lblAp->setStyleSheet(valStyle);
    m_lblAe     = new QLabel("—", this); m_lblAe->setStyleSheet(valStyle);

    auto addCutRow = [&](const QString& label, QLabel* val) {
        auto* lbl = new QLabel(label, this);
        lbl->setStyleSheet(lblStyle);
        cutLayout->addRow(lbl, val);
    };

    addCutRow("Drehzahl S:", m_lblRpm);
    addCutRow("Vorschub F:", m_lblFeed);
    addCutRow("fz pro Zahn:", m_lblFz);
    addCutRow("Eintauchen Fz:", m_lblPlunge);
    addCutRow("Zustellung ap:", m_lblAp);
    addCutRow("Zustellung ae:", m_lblAe);

    rightLayout->addWidget(cutGroup);
    rightLayout->addStretch(1);

    // ═══════════════════════════════════════════════════════
    // Zusammenführung mit QSplitter
    // ═══════════════════════════════════════════════════════
    auto* splitter = new QSplitter(Qt::Horizontal, this);
    splitter->addWidget(leftWidget);
    splitter->addWidget(rightWidget);
    splitter->setStretchFactor(0, 1);
    splitter->setStretchFactor(1, 1);
    splitter->setSizes({350, 350});

    mainLayout->addWidget(splitter);
}

Core::ToolDefinition ToolManagerDialog::activeTool() const {
    if (m_activeToolIndex >= 0 && m_activeToolIndex < m_tools.size()) {
        return m_tools[m_activeToolIndex];
    }
    return Core::ToolDefinition(1, QStringLiteral("6mm Standardfräser"), Core::ToolType::EndMill, 6.0);
}

void ToolManagerDialog::populateToolList() {
    m_toolList->blockSignals(true);
    m_toolList->clear();

    for (int i = 0; i < m_tools.size(); ++i) {
        const auto& t = m_tools[i];
        QString prefix = (i == m_activeToolIndex) ? QStringLiteral("● ") : QStringLiteral("  ");
        QString text = QString("%1T%2  %3  Ø%4mm")
            .arg(prefix)
            .arg(t.id, 2)
            .arg(t.name)
            .arg(t.diameter, 0, 'f', 1);
        auto* item = new QListWidgetItem(text);

        if (i == m_activeToolIndex) {
            item->setForeground(QColor(72, 187, 120));
            QFont f = item->font();
            f.setBold(true);
            item->setFont(f);
        }

        // Farbstreifen links
        QColor toolCol(t.color.isEmpty() ? QStringLiteral("#FFE614") : t.color);
        item->setIcon(QIcon()); // Platzhalter
        item->setBackground(QColor(toolCol.red(), toolCol.green(), toolCol.blue(), 25));

        m_toolList->addItem(item);
    }

    m_toolList->blockSignals(false);
}

void ToolManagerDialog::loadToolToEditor(int index) {
    if (index < 0 || index >= m_tools.size()) return;
    m_isLoadingEditor = true;
    m_editingIndex = index;

    const auto& t = m_tools[index];

    m_cmbType->setCurrentIndex(static_cast<int>(t.type));
    m_editName->setText(t.name);
    m_spinDiameter->setValue(t.diameter);
    m_spinFluteLength->setValue(t.fluteLength);
    m_spinShaftDia->setValue(t.shaftDiameter);
    m_spinStickOut->setValue(t.stickOutLength);
    m_spinOverallLength->setValue(t.overallLength);
    m_spinHolderDia->setValue(t.holderDiameter);
    m_spinFlutes->setValue(t.flutes);

    // Farb-Button aktualisieren
    QColor col(t.color.isEmpty() ? QStringLiteral("#FFE614") : t.color);
    m_btnColor->setText(QString("■ %1").arg(col.name().toUpper()));
    m_btnColor->setStyleSheet(QString("padding: 4px 8px; font-weight: bold; background-color: %1; color: %2; border-radius: 3px;")
        .arg(col.name(), col.lightness() > 140 ? "black" : "white"));

    m_isLoadingEditor = false;

    updateGraphics();
    recalculateCuttingData();
}

void ToolManagerDialog::saveEditorToTool(int index) {
    if (index < 0 || index >= m_tools.size() || m_isLoadingEditor) return;

    auto& t = m_tools[index];
    t.type = static_cast<Core::ToolType>(m_cmbType->currentIndex());
    if (!m_editName->text().trimmed().isEmpty()) t.name = m_editName->text().trimmed();
    t.diameter = m_spinDiameter->value();
    t.fluteLength = m_spinFluteLength->value();
    t.shaftDiameter = m_spinShaftDia->value();
    t.stickOutLength = m_spinStickOut->value();
    t.overallLength = m_spinOverallLength->value();
    t.holderDiameter = m_spinHolderDia->value();
    t.flutes = m_spinFlutes->value();
}

void ToolManagerDialog::onToolListSelectionChanged() {
    int row = m_toolList->currentRow();
    if (row >= 0) loadToolToEditor(row);
}

void ToolManagerDialog::onAddToolClicked() {
    // Eindeutige T-Nummer: höchste vorhandene + 1
    int newId = 1;
    for (const auto& t : m_tools) newId = std::max(newId, t.id + 1);
    Core::ToolDefinition newTool(newId, QString("Werkzeug T%1").arg(newId), Core::ToolType::EndMill, 8.0);
    newTool.fluteLength = 22.0;
    newTool.stickOutLength = 35.0;
    newTool.shaftDiameter = 8.0;
    newTool.flutes = 3;
    newTool.color = QStringLiteral("#FFE614");
    m_tools.append(newTool);
    populateToolList();
    m_toolList->setCurrentRow(m_tools.size() - 1);
    persistLibrary();
}

void ToolManagerDialog::onDeleteToolClicked() {
    int row = m_toolList->currentRow();
    if (row < 0 || row >= m_tools.size()) return;
    if (m_tools.size() <= 1) {
        QMessageBox::warning(this, QStringLiteral("Achtung"),
                             QStringLiteral("Es muss mindestens ein Werkzeug vorhanden sein."));
        return;
    }

    m_tools.removeAt(row);
    if (m_activeToolIndex >= m_tools.size()) {
        m_activeToolIndex = m_tools.size() - 1;
    }
    if (m_activeToolIndex == row && m_activeToolIndex > 0) {
        m_activeToolIndex--;
    }
    m_editingIndex = -1;
    populateToolList();
    m_toolList->setCurrentRow(std::min(row, static_cast<int>(m_tools.size()) - 1));
    persistLibrary();
    emit activeToolChanged(m_tools[m_activeToolIndex]);
}

void ToolManagerDialog::onActivateClicked() {
    int row = m_toolList->currentRow();
    if (row >= 0 && row < m_tools.size()) {
        saveEditorToTool(row);
        m_activeToolIndex = row;
        populateToolList();
        m_toolList->setCurrentRow(row);
        persistLibrary();
        emit activeToolChanged(m_tools[m_activeToolIndex]);
    }
}

void ToolManagerDialog::onToolTypeChanged(int index) {
    if (m_isLoadingEditor) return;
    if (m_editingIndex >= 0 && m_editingIndex < m_tools.size()) {
        m_tools[m_editingIndex].type = static_cast<Core::ToolType>(index);
        updateGraphics();
        recalculateCuttingData();
        populateToolList();
        m_toolList->setCurrentRow(m_editingIndex);
        persistLibrary();
        if (m_editingIndex == m_activeToolIndex) {
            emit activeToolChanged(m_tools[m_activeToolIndex]);
        }
    }
}

void ToolManagerDialog::onParameterChanged() {
    if (m_isLoadingEditor) return;
    if (m_editingIndex >= 0 && m_editingIndex < m_tools.size()) {
        saveEditorToTool(m_editingIndex);
        updateGraphics();
        recalculateCuttingData();
        // Liste aktualisieren (Durchmesser könnte sich geändert haben)
        populateToolList();
        m_toolList->blockSignals(true);
        m_toolList->setCurrentRow(m_editingIndex);
        m_toolList->blockSignals(false);
        persistLibrary();
        if (m_editingIndex == m_activeToolIndex) {
            emit activeToolChanged(m_tools[m_activeToolIndex]);
        }
    }
}

void ToolManagerDialog::onMaterialChanged(int index) {
    Q_UNUSED(index);
    recalculateCuttingData(); // übernimmt neue Schnittdaten ins Werkzeug
    persistLibrary();
}

void ToolManagerDialog::persistLibrary() {
    const int activeId = (m_activeToolIndex >= 0 && m_activeToolIndex < m_tools.size()) ? m_tools[m_activeToolIndex].id : -1;
    const QString path = Core::ToolDefinition::defaultLibraryPath();
    if (!Core::ToolDefinition::saveLibrary(path, m_tools, activeId) && !m_saveErrorShown) {
        m_saveErrorShown = true; // nur einmal melden, nicht bei jedem Tastendruck
        QMessageBox::warning(this, QStringLiteral("Werkzeugbibliothek"),
                             QStringLiteral("Die Werkzeugbibliothek konnte nicht gespeichert werden:\n%1").arg(path));
    }
    emit toolLibraryChanged(m_tools);
}

void ToolManagerDialog::onColorClicked() {
    if (m_editingIndex < 0 || m_editingIndex >= m_tools.size()) return;

    QColor initial(m_tools[m_editingIndex].color.isEmpty()
                       ? QStringLiteral("#FFE614") : m_tools[m_editingIndex].color);
    QColor chosen = QColorDialog::getColor(initial, this, QStringLiteral("Frässpur-Farbe wählen"));
    if (chosen.isValid()) {
        m_tools[m_editingIndex].color = chosen.name();
        m_btnColor->setText(QString("■ %1").arg(chosen.name().toUpper()));
        m_btnColor->setStyleSheet(QString("padding: 4px 8px; font-weight: bold; background-color: %1; color: %2; border-radius: 3px;")
            .arg(chosen.name(), chosen.lightness() > 140 ? "black" : "white"));
        populateToolList();
        m_toolList->blockSignals(true);
        m_toolList->setCurrentRow(m_editingIndex);
        m_toolList->blockSignals(false);
        persistLibrary();
        if (m_editingIndex == m_activeToolIndex) {
            emit activeToolChanged(m_tools[m_activeToolIndex]);
        }
    }
}

void ToolManagerDialog::updateGraphics() {
    if (m_editingIndex >= 0 && m_editingIndex < m_tools.size()) {
        m_toolGraphic->setToolDefinition(m_tools[m_editingIndex]);
    }
}

void ToolManagerDialog::recalculateCuttingData() {
    if (m_editingIndex < 0 || m_editingIndex >= m_tools.size()) return;

    int matId = m_cmbMaterial->currentData().toInt();
    Core::Material mat = m_materialDb.findById(matId);
    const auto& tool = m_tools[m_editingIndex];

    Core::CuttingParameters params = Core::TechnologyCalculator::calculate(mat, tool);

    // Zahnvorschub fz berechnen
    double fz = (params.spindleRpm > 0 && tool.flutes > 0)
        ? params.feedRate / (params.spindleRpm * tool.flutes)
        : 0.0;

    m_lblRpm->setText(QString("S = %1 U/min").arg(static_cast<int>(params.spindleRpm)));
    m_lblFeed->setText(QString("F = %1 mm/min").arg(static_cast<int>(params.feedRate)));
    m_lblFz->setText(QString("fz = %1 mm/Zahn").arg(fz, 0, 'f', 4));
    m_lblPlunge->setText(QString("Fz = %1 mm/min").arg(static_cast<int>(params.plungeFeedRate)));
    m_lblAp->setText(QString("ap = %1 mm").arg(params.recommendedStepDown, 0, 'f', 2));
    m_lblAe->setText(QString("ae = %1 mm").arg(params.recommendedStepOver, 0, 'f', 2));
    m_lblVc->setText(QString("Vc = %1 m/min  |  fz(Basis) = %2 mm/Zahn")
        .arg(mat.vc, 0, 'f', 0).arg(mat.fzBase, 0, 'f', 4));

    // Berechnete Werte auch ins Werkzeug übernehmen
    m_tools[m_editingIndex].spindleSpeed = params.spindleRpm;
    m_tools[m_editingIndex].defaultFeedRate = params.feedRate;
    m_tools[m_editingIndex].plungeFeedRate = params.plungeFeedRate;
    m_tools[m_editingIndex].maxStepDown = params.recommendedStepDown;
    m_tools[m_editingIndex].stepOverPercentage = (tool.diameter > 0.1)
        ? (params.recommendedStepOver / tool.diameter) * 100.0 : 50.0;
}

} // namespace GeminiCNC::UI
