#include "JogDialog.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>

namespace GeminiCNC::UI {

JogDialog::JogDialog(Hardware::JogController* jogCtrl, QWidget* parent)
    : QWidget(parent), m_jogCtrl(jogCtrl) {

    auto* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(6, 6, 6, 6);
    mainLayout->setSpacing(8);

    // 1. Schrittweiten-Auswahl
    auto* stepGroup = new QGroupBox(QStringLiteral("Schrittweite"), this);
    auto* stepLayout = new QHBoxLayout(stepGroup);
    m_stepGroup = new QButtonGroup(this);

    auto addStepRadio = [this, stepLayout](const QString& text, double val, int id, bool checked = false) {
        auto* rb = new QRadioButton(text, this);
        rb->setChecked(checked);
        m_stepGroup->addButton(rb, id);
        stepLayout->addWidget(rb);
    };

    addStepRadio("0.01 mm", 0.01, 1);
    addStepRadio("0.1 mm", 0.1, 2);
    addStepRadio("1.0 mm", 1.0, 3, true);
    addStepRadio("10 mm", 10.0, 4);
    addStepRadio("50 mm", 50.0, 5);

    connect(m_stepGroup, &QButtonGroup::idClicked, this, &JogDialog::onStepDistanceChanged);
    mainLayout->addWidget(stepGroup);

    // 2. Jog-Tastenfeld (XY & Z & A)
    auto* padGroup = new QGroupBox(QStringLiteral("Manuelle Achsensteuerung"), this);
    auto* padLayout = new QGridLayout(padGroup);

    auto makeJogBtn = [this](const QString& text, const QString& bg) {
        auto* b = new QPushButton(text, this);
        b->setMinimumSize(54, 44);
        b->setStyleSheet(QString("font-weight: bold; font-size: 14px; background-color: %1; color: white; border-radius: 6px;").arg(bg));
        return b;
    };

    auto* btnYPlus = makeJogBtn("Y+", "#2B6CB0");
    auto* btnYMinus = makeJogBtn("Y-", "#2B6CB0");
    auto* btnXMinus = makeJogBtn("X-", "#2B6CB0");
    auto* btnXPlus = makeJogBtn("X+", "#2B6CB0");

    auto* btnZPlus = makeJogBtn("Z+", "#D69E2E");
    auto* btnZMinus = makeJogBtn("Z-", "#D69E2E");

    auto* btnAPlus = makeJogBtn("A+ ↻", "#805AD5");
    auto* btnAMinus = makeJogBtn("A- ↺", "#805AD5");

    auto* btnHomeAll = new QPushButton(QStringLiteral("⌂ Home Alle"), this);
    btnHomeAll->setStyleSheet("background-color: #38A169; color: white; font-weight: bold; border-radius: 4px; padding: 6px;");

    // Layout Anordnung:
    //      Y+        Z+   A+
    //  X-  Home  X+
    //      Y-        Z-   A-
    padLayout->addWidget(btnYPlus, 0, 1);
    padLayout->addWidget(btnXMinus, 1, 0);
    padLayout->addWidget(btnHomeAll, 1, 1);
    padLayout->addWidget(btnXPlus, 1, 2);
    padLayout->addWidget(btnYMinus, 2, 1);

    padLayout->addWidget(btnZPlus, 0, 3);
    padLayout->addWidget(btnZMinus, 2, 3);

    padLayout->addWidget(btnAPlus, 0, 4);
    padLayout->addWidget(btnAMinus, 2, 4);

    mainLayout->addWidget(padGroup);

    // Jog Signale verbinden
    if (m_jogCtrl) {
        connect(btnYPlus, &QPushButton::clicked, this, [this]() { m_jogCtrl->jogAxis(Hardware::JogAxis::Y, +1); });
        connect(btnYMinus, &QPushButton::clicked, this, [this]() { m_jogCtrl->jogAxis(Hardware::JogAxis::Y, -1); });
        connect(btnXPlus, &QPushButton::clicked, this, [this]() { m_jogCtrl->jogAxis(Hardware::JogAxis::X, +1); });
        connect(btnXMinus, &QPushButton::clicked, this, [this]() { m_jogCtrl->jogAxis(Hardware::JogAxis::X, -1); });
        connect(btnZPlus, &QPushButton::clicked, this, [this]() { m_jogCtrl->jogAxis(Hardware::JogAxis::Z, +1); });
        connect(btnZMinus, &QPushButton::clicked, this, [this]() { m_jogCtrl->jogAxis(Hardware::JogAxis::Z, -1); });
        connect(btnAPlus, &QPushButton::clicked, this, [this]() { m_jogCtrl->jogAxis(Hardware::JogAxis::A, +1); });
        connect(btnAMinus, &QPushButton::clicked, this, [this]() { m_jogCtrl->jogAxis(Hardware::JogAxis::A, -1); });
        connect(btnHomeAll, &QPushButton::clicked, this, [this]() { m_jogCtrl->homeAll(); });
    }

    // 3. Nullpunkt-Tasten
    auto* zeroGroup = new QGroupBox(QStringLiteral("Nullpunkt setzen (G92)"), this);
    auto* zeroLayout = new QHBoxLayout(zeroGroup);

    auto* btnZeroAll = new QPushButton(QStringLiteral("X/Y/Z/A = 0"), this);
    btnZeroAll->setStyleSheet("background-color: #4A5568; color: white; padding: 6px; border-radius: 4px;");
    auto* btnZeroZ = new QPushButton(QStringLiteral("Z = 0"), this);
    btnZeroZ->setStyleSheet("background-color: #4A5568; color: white; padding: 6px; border-radius: 4px;");

    zeroLayout->addWidget(btnZeroAll);
    zeroLayout->addWidget(btnZeroZ);
    mainLayout->addWidget(zeroGroup);

    if (m_jogCtrl) {
        connect(btnZeroAll, &QPushButton::clicked, this, [this]() { m_jogCtrl->zeroAll(); });
        connect(btnZeroZ, &QPushButton::clicked, this, [this]() { m_jogCtrl->zeroAxis(Hardware::JogAxis::Z); });
    }

    // 4. Manuelle Spindelsteuerung
    auto* spindleGroup = new QGroupBox(QStringLiteral("Spindel"), this);
    auto* spindleLayout = new QHBoxLayout(spindleGroup);

    m_btnSpindle = new QPushButton(QStringLiteral("Spindel START"), this);
    m_btnSpindle->setStyleSheet("background-color: #38A169; color: white; font-weight: bold; padding: 8px; border-radius: 4px;");
    connect(m_btnSpindle, &QPushButton::clicked, this, &JogDialog::onSpindleToggleClicked);

    m_spinSpindleRpm = new QDoubleSpinBox(this);
    m_spinSpindleRpm->setRange(1000, 30000);
    m_spinSpindleRpm->setValue(18000);
    m_spinSpindleRpm->setSingleStep(1000);
    m_spinSpindleRpm->setSuffix(" U/min");

    spindleLayout->addWidget(m_btnSpindle);
    spindleLayout->addWidget(m_spinSpindleRpm);
    mainLayout->addWidget(spindleGroup);

    mainLayout->addStretch(1);
}

void JogDialog::onStepDistanceChanged(int id) {
    if (!m_jogCtrl) return;
    double dist = 1.0;
    if (id == 1) dist = 0.01;
    else if (id == 2) dist = 0.1;
    else if (id == 3) dist = 1.0;
    else if (id == 4) dist = 10.0;
    else if (id == 5) dist = 50.0;

    m_jogCtrl->setStepDistance(dist);
}

void JogDialog::onSpindleToggleClicked() {
    if (!m_jogCtrl) return;

    m_spindleRunning = !m_spindleRunning;
    if (m_spindleRunning) {
        m_btnSpindle->setText(QStringLiteral("Spindel STOP"));
        m_btnSpindle->setStyleSheet("background-color: #E53E3E; color: white; font-weight: bold; padding: 8px; border-radius: 4px;");
        m_jogCtrl->setSpindleState(true, m_spinSpindleRpm->value());
    } else {
        m_btnSpindle->setText(QStringLiteral("Spindel START"));
        m_btnSpindle->setStyleSheet("background-color: #38A169; color: white; font-weight: bold; padding: 8px; border-radius: 4px;");
        m_jogCtrl->setSpindleState(false);
    }
}

} // namespace GeminiCNC::UI
