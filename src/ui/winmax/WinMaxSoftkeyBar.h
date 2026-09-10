#ifndef GEMINI_CNC_WINMAXSOFTKEYBAR_H
#define GEMINI_CNC_WINMAXSOFTKEYBAR_H

#include <QWidget>
#include <QPushButton>
#include <QLabel>
#include <vector>
#include <QString>

namespace GeminiCNC::UI {

/**
 * @brief Hurco WinMax Menühierarchie für die 8 Softkeys (F1–F8).
 */
enum class SoftkeyMenu {
    Main,           // Hauptmenü (Setup, Tools, Prog, Params, Probe, Offsets, Grafik, ProgMgr)
    NewBlock,       // Block hinzufügen (Position, Bohrung, Fräsen, Muster, Sonstiges, NC-Aufruf)
    Milling,        // Fräsen (Linien/Bögen, Kreis, Rahmen/Tasche, Planfräsen, Ellipse, 3D-Form)
    Holes,          // Bohrungen (Bohren, Gewinde, Reiben, Lochkreis, Einzelpunkte)
    BlockEdit,      // Block-Bearbeitung (Vorheriger, Nächster, Params, Setup, Tools, Exit)
    ContourEdit,    // Kontur-Bearbeitung (Vorheriger, Nächster, Wert speichern, Nächste Lösung, Exit)
    NewSegment,     // Segment-Typ (Gerade, Bogen, Rundung, Helix, 3D Bogen)
    ManualJog,      // Handbetrieb (X-, X+, Y-, Y+, Z-, Z+, Nullen, Home)
    Simulation      // 3D-Simulation & Grafik (Start, Pause, Satz einzeln, Reset, ISO, XY, XZ, Exit)
};

struct SoftkeyDef {
    int keyIndex{1};     // 1..8
    QString label;       // Hauptbeschriftung
    QString actionKey;   // Routing-Key
    bool enabled{true};  // Verfügbarkeit
    QString subtitle;    // Optionale Unterzeile (z. B. "G81")
};

/**
 * @brief Hurco WinMax Softkey-Leiste: VERTIKAL RECHTS, F1–F8.
 *
 * Modernes Dark-Glass / High-Tech Cyber-Industrial Styling:
 * - 8 gleichmäßig verteilte Softkeys
 * - Feiner Neon-Rand (#2563EB / #00D2FF)
 * - F-Tastennummer in der rechten unteren Ecke
 * - Sanfter Glow-Effekt bei Hover
 */
class WinMaxSoftkeyBar : public QWidget {
    Q_OBJECT
public:
    explicit WinMaxSoftkeyBar(QWidget* parent = nullptr);

    void setMenu(SoftkeyMenu menu);
    [[nodiscard]] SoftkeyMenu currentMenu() const { return m_currentMenu; }

    void triggerKey(int fKeyNumber); // 1..8

signals:
    void softkeyTriggered(SoftkeyMenu menu, int fKeyNumber, const QString& actionKey);

private slots:
    void onButtonClicked(int fKeyNumber);

private:
    void setupUi();
    void updateButtons();

    static constexpr int NUM_SOFTKEYS = 8;

    SoftkeyMenu m_currentMenu{SoftkeyMenu::Main};
    std::vector<QPushButton*> m_buttons;
    std::vector<QLabel*> m_labelMains;
    std::vector<QLabel*> m_labelFKeys;
    std::vector<SoftkeyDef> m_currentDefs;
};

} // namespace GeminiCNC::UI

#endif // GEMINI_CNC_WINMAXSOFTKEYBAR_H
