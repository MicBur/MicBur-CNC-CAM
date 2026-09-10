#ifndef GEMINI_CNC_JOGDIALOG_H
#define GEMINI_CNC_JOGDIALOG_H

#include <QWidget>
#include <QPushButton>
#include <QRadioButton>
#include <QDoubleSpinBox>
#include <QButtonGroup>
#include "hardware/JogController.h"

namespace GeminiCNC::UI {

/**
 * @brief Dialog für Schritt 5: Manuelle Achsen-Bedieneinheit (Jogging, Nullung, Spindel).
 */
class JogDialog : public QWidget {
    Q_OBJECT
public:
    explicit JogDialog(Hardware::JogController* jogCtrl, QWidget* parent = nullptr);

private slots:
    void onStepDistanceChanged(int id);
    void onSpindleToggleClicked();

private:
    Hardware::JogController* m_jogCtrl{nullptr};
    QButtonGroup* m_stepGroup{nullptr};

    QPushButton* m_btnSpindle{nullptr};
    QDoubleSpinBox* m_spinSpindleRpm{nullptr};
    bool m_spindleRunning{false};
};

} // namespace GeminiCNC::UI

#endif // GEMINI_CNC_JOGDIALOG_H
