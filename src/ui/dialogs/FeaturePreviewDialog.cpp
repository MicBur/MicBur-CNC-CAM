#include "FeaturePreviewDialog.h"

#include <QHeaderView>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QApplication>
#include <QStyle>

namespace GeminiCNC::UI {

FeaturePreviewDialog::FeaturePreviewDialog(QWidget* parent)
    : QDialog(parent)
{
    setWindowTitle(QStringLiteral("STL Feature-Analyse — Erkannte Bearbeitungsfeatures"));
    setMinimumSize(700, 450);
    setupUi();
}

void FeaturePreviewDialog::setupUi()
{
    auto* mainLayout = new QVBoxLayout(this);
    mainLayout->setSpacing(8);

    // Zusammenfassung
    m_lblSummary = new QLabel(QStringLiteral("Analyse läuft..."), this);
    m_lblSummary->setStyleSheet(
        "background-color: #1A202C; padding: 8px; border-radius: 4px; "
        "color: #CBD5E0; font-family: Consolas, monospace; font-size: 12px;");
    m_lblSummary->setWordWrap(true);
    mainLayout->addWidget(m_lblSummary);

    // Feature-Tabelle
    m_table = new QTableWidget(0, 7, this);
    m_table->setHorizontalHeaderLabels({
        QStringLiteral("✓"),
        QStringLiteral("Feature"),
        QStringLiteral("Typ"),
        QStringLiteral("Z oben"),
        QStringLiteral("Z unten"),
        QStringLiteral("Werkzeug"),
        QStringLiteral("Vol. (mm³)")
    });
    m_table->horizontalHeader()->setStretchLastSection(true);
    m_table->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    m_table->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
    m_table->horizontalHeader()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
    m_table->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_table->setAlternatingRowColors(true);
    m_table->setStyleSheet(
        "QTableWidget { background-color: #2D3748; color: #E2E8F0; gridline-color: #4A5568; font-size: 12px; }"
        "QTableWidget::item { padding: 4px; }"
        "QHeaderView::section { background-color: #1A202C; color: #A0AEC0; padding: 4px; border: 1px solid #4A5568; font-weight: bold; }");
    mainLayout->addWidget(m_table, 1);

    // Buttons
    auto* btnLayout = new QHBoxLayout();
    btnLayout->addStretch(1);

    m_btnCancel = new QPushButton(QStringLiteral("Abbrechen"), this);
    m_btnCancel->setMinimumWidth(120);
    connect(m_btnCancel, &QPushButton::clicked, this, &QDialog::reject);
    btnLayout->addWidget(m_btnCancel);

    m_btnAccept = new QPushButton(QStringLiteral("✅ Übernehmen"), this);
    m_btnAccept->setMinimumWidth(160);
    m_btnAccept->setStyleSheet(
        "padding: 8px 16px; font-weight: bold; background-color: #48BB78; "
        "color: white; border-radius: 4px; font-size: 13px;");
    connect(m_btnAccept, &QPushButton::clicked, this, &FeaturePreviewDialog::onAcceptClicked);
    btnLayout->addWidget(m_btnAccept);

    mainLayout->addLayout(btnLayout);
}

void FeaturePreviewDialog::runAnalysis(
    const Geometry::Mesh& stockMesh,
    const Geometry::Mesh& partMesh,
    const QList<Core::ToolDefinition>& toolLibrary)
{
    m_stockMesh = stockMesh;
    m_partMesh = partMesh;
    m_toolLibrary = toolLibrary;

    // Feature-Erkennung starten
    QApplication::setOverrideCursor(Qt::WaitCursor);
    m_result = CAM::FeatureRecognizer::analyze(partMesh, stockMesh, toolLibrary);
    QApplication::restoreOverrideCursor();

    m_lblSummary->setText(m_result.summary);
    populateTable();
}

void FeaturePreviewDialog::populateTable()
{
    m_table->setRowCount(static_cast<int>(m_result.features.size()));

    for (int row = 0; row < static_cast<int>(m_result.features.size()); ++row) {
        const auto& f = m_result.features[row];

        // Spalte 0: Checkbox (aktiviert)
        auto* chk = new QCheckBox(this);
        chk->setChecked(true);
        auto* chkWidget = new QWidget(this);
        auto* chkLayout = new QHBoxLayout(chkWidget);
        chkLayout->addWidget(chk);
        chkLayout->setAlignment(Qt::AlignCenter);
        chkLayout->setContentsMargins(0, 0, 0, 0);
        m_table->setCellWidget(row, 0, chkWidget);

        // Spalte 1: Feature-Name
        auto* nameItem = new QTableWidgetItem(f.name);
        nameItem->setFlags(nameItem->flags() & ~Qt::ItemIsEditable);
        m_table->setItem(row, 1, nameItem);

        // Spalte 2: Typ
        auto* typeItem = new QTableWidgetItem(featureTypeName(f.type));
        typeItem->setFlags(typeItem->flags() & ~Qt::ItemIsEditable);
        m_table->setItem(row, 2, typeItem);

        // Spalte 3: Z oben
        auto* zTopItem = new QTableWidgetItem(QString::number(f.topZ, 'f', 2));
        zTopItem->setFlags(zTopItem->flags() & ~Qt::ItemIsEditable);
        zTopItem->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
        m_table->setItem(row, 3, zTopItem);

        // Spalte 4: Z unten
        auto* zBotItem = new QTableWidgetItem(QString::number(f.bottomZ, 'f', 2));
        zBotItem->setFlags(zBotItem->flags() & ~Qt::ItemIsEditable);
        zBotItem->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
        m_table->setItem(row, 4, zBotItem);

        // Spalte 5: Werkzeug (Dropdown)
        auto* cmbTool = new QComboBox(this);
        int selectedIdx = 0;
        for (int t = 0; t < m_toolLibrary.size(); ++t) {
            const auto& tool = m_toolLibrary[t];
            cmbTool->addItem(
                QStringLiteral("T%1: %2 (Ø%3)")
                    .arg(tool.id)
                    .arg(tool.name)
                    .arg(tool.diameter, 0, 'f', 1),
                tool.id
            );
            if (tool.id == f.suggestedToolId) {
                selectedIdx = t;
            }
        }
        cmbTool->setCurrentIndex(selectedIdx);
        m_table->setCellWidget(row, 5, cmbTool);

        // Spalte 6: Volumen
        auto* volItem = new QTableWidgetItem(QString::number(f.estimatedVolume, 'f', 1));
        volItem->setFlags(volItem->flags() & ~Qt::ItemIsEditable);
        volItem->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
        m_table->setItem(row, 6, volItem);
    }
}

QString FeaturePreviewDialog::featureTypeName(CAM::FeatureType type) const
{
    switch (type) {
        case CAM::FeatureType::Facing:          return QStringLiteral("Planfräsen");
        case CAM::FeatureType::ExternalContour: return QStringLiteral("Außenkontur");
        case CAM::FeatureType::Pocket:          return QStringLiteral("Tasche");
        case CAM::FeatureType::ThroughPocket:   return QStringLiteral("Durchgangstasche");
        case CAM::FeatureType::Slot:            return QStringLiteral("Nut");
        case CAM::FeatureType::CircularHole:    return QStringLiteral("Bohrung");
        case CAM::FeatureType::FreeformSurface: return QStringLiteral("3D-Freiform");
        case CAM::FeatureType::Step:            return QStringLiteral("Absatz");
    }
    return QStringLiteral("Unbekannt");
}

void FeaturePreviewDialog::onAcceptClicked()
{
    // Werkzeug-Overrides aus der Tabelle anwenden
    for (int row = 0; row < m_table->rowCount(); ++row) {
        // Checkbox prüfen
        auto* chkWidget = m_table->cellWidget(row, 0);
        auto* chk = chkWidget->findChild<QCheckBox*>();
        if (!chk || !chk->isChecked()) {
            // Feature deaktiviert — Type auf Step setzen (wird übersprungen)
            m_result.features[row].type = CAM::FeatureType::Step;
            continue;
        }

        // Werkzeug-Override auslesen
        auto* cmbTool = qobject_cast<QComboBox*>(m_table->cellWidget(row, 5));
        if (cmbTool) {
            int toolId = cmbTool->currentData().toInt();
            m_result.features[row].suggestedToolId = toolId;
        }
    }

    // Deaktivierte Features rausfiltern (Step-Typ)
    CAM::RecognitionResult filteredResult;
    filteredResult.hasFreeformSurfaces = m_result.hasFreeformSurfaces;
    filteredResult.totalRemovalVolume = 0.0;
    for (const auto& f : m_result.features) {
        if (f.type != CAM::FeatureType::Step) {
            filteredResult.features.push_back(f);
            filteredResult.totalRemovalVolume += f.estimatedVolume;
        }
    }

    if (filteredResult.features.empty()) {
        QMessageBox::information(this, QStringLiteral("Keine Features"),
            QStringLiteral("Alle Features wurden deaktiviert. Es wird kein Programm erzeugt."));
        return;
    }

    // Programm generieren
    CAM::AutoProgramGenerator::Settings settings;
    settings.clearanceZ = 5.0;
    settings.finishAllowance = 0.1;
    settings.generateFinishPasses = true;

    m_program = CAM::AutoProgramGenerator::generate(
        filteredResult,
        m_toolLibrary,
        m_stockMesh.boundingBox,
        m_partMesh,
        settings
    );

    emit programGenerated(m_program);
    accept();
}

} // namespace GeminiCNC::UI
