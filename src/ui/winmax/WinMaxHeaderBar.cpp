#include "WinMaxHeaderBar.h"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QFrame>

namespace GeminiCNC::UI {

WinMaxHeaderBar::WinMaxHeaderBar(QWidget* parent) : QWidget(parent) {
    setFixedHeight(72);
    setStyleSheet(QStringLiteral("background-color: #1B1F23; border-bottom: 2px solid #374151;"));
    setupUi();
    setCoordinates(0.0, 0.0, 25.0);
    setProgramInfo(QStringLiteral("UNNAMED.GPROG"), QStringLiteral("Block 1/1: Bereit"));
    setMachineState(QStringLiteral("BEREIT"), QStringLiteral("#10B981"));
}

void WinMaxHeaderBar::setupUi() {
    auto* mainLayout = new QHBoxLayout(this);
    mainLayout->setContentsMargins(10, 4, 10, 4);
    mainLayout->setSpacing(12);

    // ═══════════════════════════════════════════════════════
    // 1. Logo & Programm-Info (links)
    // ═══════════════════════════════════════════════════════
    auto* leftLayout = new QVBoxLayout();
    leftLayout->setSpacing(2);

    auto* topRow = new QHBoxLayout();
    topRow->setSpacing(8);

    m_lblLogo = new QLabel(QStringLiteral("GEMINI CNC"), this);
    m_lblLogo->setStyleSheet(QStringLiteral(
        "font-size: 16px; font-weight: 900; color: #F59E0B; letter-spacing: 3px; "
        "font-family: 'Segoe UI', Arial;"));

    m_lblStateBadge = new QLabel(QStringLiteral("BEREIT"), this);
    m_lblStateBadge->setStyleSheet(QStringLiteral(
        "background-color: #10B981; color: #1B1F23; font-weight: bold; font-size: 11px; "
        "padding: 2px 10px; border-radius: 3px;"));

    topRow->addWidget(m_lblLogo);
    topRow->addWidget(m_lblStateBadge);
    topRow->addStretch();
    leftLayout->addLayout(topRow);

    m_lblProgramInfo = new QLabel(this);
    m_lblProgramInfo->setStyleSheet(QStringLiteral(
        "color: #9CA3AF; font-size: 11px; font-family: Consolas, monospace;"));

    leftLayout->addWidget(m_lblProgramInfo);
    mainLayout->addLayout(leftLayout);

    // Trennlinie
    auto* line1 = new QFrame(this);
    line1->setFrameShape(QFrame::VLine);
    line1->setStyleSheet(QStringLiteral("color: #374151;"));
    mainLayout->addWidget(line1);

    // ═══════════════════════════════════════════════════════
    // 2. DRO – Große Koordinatenanzeige (vertikal gestapelt)
    // ═══════════════════════════════════════════════════════
    auto* droLayout = new QVBoxLayout();
    droLayout->setSpacing(0);

    const QString droStyle = QStringLiteral(
        "color: #10B981; font-family: Consolas, monospace; font-size: 18px; "
        "font-weight: bold; padding: 0px 8px; background: transparent;");

    auto* droHeader = new QLabel(QStringLiteral("G54"), this);
    droHeader->setStyleSheet(QStringLiteral(
        "color: #6B7280; font-weight: bold; font-size: 10px; padding-left: 8px;"));

    auto* droRow = new QHBoxLayout();
    droRow->setSpacing(4);

    m_lblCoordX = new QLabel(QStringLiteral("X:  +0.000"), this);
    m_lblCoordX->setStyleSheet(droStyle);
    m_lblCoordY = new QLabel(QStringLiteral("Y:  +0.000"), this);
    m_lblCoordY->setStyleSheet(droStyle);
    m_lblCoordZ = new QLabel(QStringLiteral("Z: +25.000"), this);
    m_lblCoordZ->setStyleSheet(droStyle);

    droRow->addWidget(m_lblCoordX);
    droRow->addWidget(m_lblCoordY);
    droRow->addWidget(m_lblCoordZ);

    droLayout->addWidget(droHeader);
    droLayout->addLayout(droRow);
    mainLayout->addLayout(droLayout, 1);

    // Trennlinie
    auto* line2 = new QFrame(this);
    line2->setFrameShape(QFrame::VLine);
    line2->setStyleSheet(QStringLiteral("color: #374151;"));
    mainLayout->addWidget(line2);

    // ═══════════════════════════════════════════════════════
    // 3. Spindel, Vorschub & Werkzeug (rechts)
    // ═══════════════════════════════════════════════════════
    auto* rightLayout = new QVBoxLayout();
    rightLayout->setSpacing(2);

    m_lblTechnology = new QLabel(QStringLiteral("S: 0 U/min (100%)  F: 0 mm/min (100%)"), this);
    m_lblTechnology->setStyleSheet(QStringLiteral(
        "color: #9CA3AF; font-size: 11px; font-family: Consolas, monospace; font-weight: bold;"));

    m_lblTool = new QLabel(QStringLiteral("T1: Standard-Fräser"), this);
    m_lblTool->setStyleSheet(QStringLiteral(
        "color: #F59E0B; font-size: 12px; font-weight: bold;"));

    rightLayout->addWidget(m_lblTechnology);
    rightLayout->addWidget(m_lblTool);
    mainLayout->addLayout(rightLayout);
}

void WinMaxHeaderBar::setProgramInfo(const QString& programName, const QString& activeBlockInfo) {
    m_lblProgramInfo->setText(QString("PROG: %1 | %2").arg(programName, activeBlockInfo));
}

void WinMaxHeaderBar::setMachineState(const QString& stateText, const QString& badgeColor) {
    m_lblStateBadge->setText(stateText);
    m_lblStateBadge->setStyleSheet(QString(
        "background-color: %1; color: #1B1F23; font-weight: bold; font-size: 11px; "
        "padding: 2px 10px; border-radius: 3px;").arg(badgeColor));
}

void WinMaxHeaderBar::setCoordinates(double x, double y, double z) {
    m_lblCoordX->setText(QString("X: %1%2").arg(x >= 0 ? "+" : "").arg(x, 8, 'f', 3));
    m_lblCoordY->setText(QString("Y: %1%2").arg(y >= 0 ? "+" : "").arg(y, 8, 'f', 3));
    m_lblCoordZ->setText(QString("Z: %1%2").arg(z >= 0 ? "+" : "").arg(z, 8, 'f', 3));
}

void WinMaxHeaderBar::setTechnology(double rpm, double feed, int rpmOverride, int feedOverride) {
    m_lblTechnology->setText(QString("S: %1 U/min (%2%)  F: %3 mm/min (%4%)")
        .arg(static_cast<int>(rpm)).arg(rpmOverride)
        .arg(static_cast<int>(feed)).arg(feedOverride));
}

void WinMaxHeaderBar::setActiveTool(const Core::ToolDefinition& tool) {
    m_lblTool->setText(QString("T%1: %2 (Ø %3mm)").arg(tool.id).arg(tool.name).arg(tool.diameter, 0, 'f', 1));
}

} // namespace GeminiCNC::UI
