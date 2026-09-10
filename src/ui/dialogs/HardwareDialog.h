#ifndef GEMINI_CNC_HARDWAREDIALOG_H
#define GEMINI_CNC_HARDWAREDIALOG_H

#include <QWidget>
#include <QLineEdit>
#include <QSpinBox>
#include <QPushButton>
#include <QCheckBox>
#include <QLabel>
#include <QTextEdit>
#include "hardware/MoonrakerClient.h"

namespace GeminiCNC::UI {

/**
 * @brief Dialog für Schritt 6: Klipper/Moonraker-Schnittstelle, TMC2209-Treiber & Hardware-Profile.
 */
class HardwareDialog : public QWidget {
    Q_OBJECT
public:
    explicit HardwareDialog(Hardware::MoonrakerClient* client, QWidget* parent = nullptr);

private slots:
    void onConnectClicked();
    void onMockModeToggled(bool checked);
    void onQueryTmcClicked(const QString& stepper);

    void onClientStateChanged(Hardware::ConnectionState state);
    void onClientResponse(const QString& resp);
    void onTmcStatus(const QString& stepper, const QJsonObject& status);

private:
    Hardware::MoonrakerClient* m_client{nullptr};

    QLineEdit* m_editHost{nullptr};
    QSpinBox* m_spinPort{nullptr};
    QCheckBox* m_chkMockMode{nullptr};
    QCheckBox* m_chkEnableFourthAxis{nullptr};

    QPushButton* m_btnConnect{nullptr};
    QLabel* m_lblConnStatus{nullptr};

    QTextEdit* m_txtLog{nullptr};
};

} // namespace GeminiCNC::UI

#endif // GEMINI_CNC_HARDWAREDIALOG_H
