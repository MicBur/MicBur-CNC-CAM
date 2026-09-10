#ifndef GEMINI_CNC_DIALOGSTACK_H
#define GEMINI_CNC_DIALOGSTACK_H

#include <QStackedWidget>
#include <QVBoxLayout>
#include <QLabel>

namespace GeminiCNC::UI {

enum class DialogPage {
    Setup = 0,        // Geometrie-Import & Rohteil
    ToolManager = 1,  // Werkzeugverwaltung
    Programming = 2,  // Hurco Conversational Arbeitsplan
    Simulation = 3,   // 3D-Simulation & Kollisionsprüfung
    Jog = 4,          // Manuelle Achs-Bedieneinheit
    Probe = 5,        // Werkzeug- & Werkstück-Einmessen
    Hardware = 6      // Klipper & TMC2209 Konfiguration
};

/**
 * @brief Seiten-Container für Dialog-Panels.
 *
 * Hurco WinMax-Prinzip: KEINE Prev/Next-Buttons.
 * Seitenwechsel erfolgt ausschließlich über die Softkeys.
 * Zeigt nur den aktuellen Seitentitel als Überschrift.
 */
class DialogStack : public QWidget {
    Q_OBJECT
public:
    explicit DialogStack(QWidget* parent = nullptr);

    void addPage(DialogPage page, QWidget* widget, const QString& title);
    void setCurrentPage(DialogPage page);
    [[nodiscard]] DialogPage currentPage() const;

signals:
    void pageChanged(DialogPage newPage);

private:
    QStackedWidget* m_stackedWidget{nullptr};
    QLabel* m_lblTitle{nullptr};
    QList<QString> m_pageTitles;
};

} // namespace GeminiCNC::UI

#endif // GEMINI_CNC_DIALOGSTACK_H
