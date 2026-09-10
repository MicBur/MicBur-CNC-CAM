#include "WinMaxSoftkeyBar.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QFrame>

namespace GeminiCNC::UI {

WinMaxSoftkeyBar::WinMaxSoftkeyBar(QWidget* parent) : QWidget(parent) {
    setupUi();
    setMenu(SoftkeyMenu::Main);
}

void WinMaxSoftkeyBar::setupUi() {
    // 165px Breite für perfekte Lesbarkeit der Hurco-Rechtsspalte
    setFixedWidth(165);
    setStyleSheet(QStringLiteral(
        "QWidget { background-color: #0E1318; border-left: 1px solid #1F2937; }"));

    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(5, 6, 5, 6);
    layout->setSpacing(6);

    // 8 Softkeys vertikal erzeugen
    for (int i = 0; i < NUM_SOFTKEYS; ++i) {
        auto* btn = new QPushButton(this);
        btn->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        btn->setFocusPolicy(Qt::NoFocus);
        btn->setCursor(Qt::PointingHandCursor);

        // Modernes High-Tech Dark-Glass Styling mit Cyan-Akzenten
        btn->setStyleSheet(QStringLiteral(
            "QPushButton {"
            "  background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #1A232E, stop:1 #121820);"
            "  border: 1px solid #2B3A4C;"
            "  border-left: 3px solid #0284C7;"
            "  border-radius: 6px;"
            "  padding: 2px;"
            "}"
            "QPushButton:hover {"
            "  background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #223142, stop:1 #18222E);"
            "  border: 1px solid #00D2FF;"
            "  border-left: 4px solid #00D2FF;"
            "}"
            "QPushButton:pressed {"
            "  background-color: #0F1722;"
            "  border: 1px solid #38BDF8;"
            "  border-left: 2px solid #38BDF8;"
            "}"
            "QPushButton:disabled {"
            "  background-color: #0D1217;"
            "  border: 1px solid #1E293B;"
            "  border-left: 3px solid #1E293B;"
            "}"));

        auto* btnLayout = new QVBoxLayout(btn);
        btnLayout->setContentsMargins(6, 4, 8, 4);
        btnLayout->setSpacing(1);

        auto* lblMain = new QLabel(btn);
        lblMain->setAlignment(Qt::AlignCenter);
        lblMain->setWordWrap(true);
        lblMain->setAttribute(Qt::WA_TransparentForMouseEvents);
        lblMain->setStyleSheet(QStringLiteral("color: #F8FAFC; font-weight: bold; font-size: 11px; background: transparent; border: none;"));

        auto* lblFKey = new QLabel(btn);
        lblFKey->setAlignment(Qt::AlignRight | Qt::AlignBottom);
        lblFKey->setAttribute(Qt::WA_TransparentForMouseEvents);
        lblFKey->setStyleSheet(QStringLiteral("color: #00D2FF; font-weight: bold; font-family: Consolas, monospace; font-size: 10px; background: transparent; border: none;"));

        btnLayout->addWidget(lblMain, 1);
        btnLayout->addWidget(lblFKey, 0);

        int keyNum = i + 1;
        connect(btn, &QPushButton::clicked, this, [this, keyNum]() {
            onButtonClicked(keyNum);
        });

        layout->addWidget(btn);
        m_buttons.push_back(btn);
        m_labelMains.push_back(lblMain);
        m_labelFKeys.push_back(lblFKey);
    }
}

void WinMaxSoftkeyBar::setMenu(SoftkeyMenu menu) {
    m_currentMenu = menu;
    m_currentDefs.clear();

    switch (menu) {
        // ─── 1. Hauptmenü (Home) ───
        case SoftkeyMenu::Main:
            m_currentDefs = {
                {1, QStringLiteral("WERKSTÜCK\nSETUP"), QStringLiteral("setup"), true, ""},
                {2, QStringLiteral("WERKZEUGE\nSETUP"), QStringLiteral("tools"), true, ""},
                {3, QStringLiteral("ARBEITSPLAN\nPROGRAMM"), QStringLiteral("prog"), true, ""},
                {4, QStringLiteral("PROGRAMM\nPARAMETER"), QStringLiteral("params"), true, ""},
                {5, QStringLiteral("EINMESSEN\nTASTEN"), QStringLiteral("probe"), true, ""},
                {6, QStringLiteral("NULLPUNKTE\nG54–G59"), QStringLiteral("offsets"), true, ""},
                {7, QStringLiteral("3D-GRAFIK\nMAX5"), QStringLiteral("graphics"), true, ""},
                {8, QStringLiteral("HANDBETRIEB\nMANUAL"), QStringLiteral("menu_jog"), true, ""}
            };
            break;

        // ─── 2. Neuer Block (New Block) ───
        case SoftkeyMenu::NewBlock:
            m_currentDefs = {
                {1, QStringLiteral("POSITION"), QStringLiteral("blk_pos"), true, ""},
                {2, QStringLiteral("BOHRUNGEN\n(HOLES)"), QStringLiteral("menu_holes"), true, ""},
                {3, QStringLiteral("FRÄSEN\n(MILLING)"), QStringLiteral("menu_milling"), true, ""},
                {4, QStringLiteral("MUSTER\n(PATTERNS)"), QStringLiteral("blk_patterns"), true, ""},
                {5, QStringLiteral("SONSTIGES\n(MISC)"), QStringLiteral("blk_misc"), true, ""},
                {6, QStringLiteral("NC-AUFRUF\n(NC CALL)"), QStringLiteral("blk_nc"), true, ""},
                {7, QStringLiteral("NOT-HALT"), QStringLiteral("estop"), true, ""},
                {8, QStringLiteral("ZURÜCK\n(EXIT)"), QStringLiteral("back_prog"), true, ""}
            };
            break;

        // ─── 3. Fräsen (Milling) ───
        case SoftkeyMenu::Milling:
            m_currentDefs = {
                {1, QStringLiteral("LINIEN &\nBÖGEN"), QStringLiteral("prog_contour"), true, ""},
                {2, QStringLiteral("KREIS\n(CIRCLE)"), QStringLiteral("prog_circle"), true, ""},
                {3, QStringLiteral("RAHMEN &\nTASCHE"), QStringLiteral("prog_frame"), true, ""},
                {4, QStringLiteral("PLANFRÄSEN\n(FACE)"), QStringLiteral("prog_face"), true, ""},
                {5, QStringLiteral("LANGLOCH\n(SLOT)"), QStringLiteral("prog_slot"), true, ""},
                {6, QStringLiteral("HELIX &\nGEWINDE"), QStringLiteral("prog_helix"), true, ""},
                {7, QStringLiteral("3D-FORM\n(3D MOLD)"), QStringLiteral("prog_3d"), true, ""},
                {8, QStringLiteral("ZURÜCK\n(EXIT)"), QStringLiteral("back_newblock"), true, ""}
            };
            break;

        // ─── 4. Bohrungen (Holes) ───
        case SoftkeyMenu::Holes:
            m_currentDefs = {
                {1, QStringLiteral("BOHREN\n(DRILL)"), QStringLiteral("hole_drill"), true, "G81/G83"},
                {2, QStringLiteral("GEWINDE\n(TAP)"), QStringLiteral("hole_tap"), true, "G84"},
                {3, QStringLiteral("REIBEN &\nAUSDREHEN"), QStringLiteral("hole_bore"), true, "G85"},
                {4, QStringLiteral("ZENTRIEREN\nCENTER"), QStringLiteral("hole_center"), true, ""},
                {5, QStringLiteral("LOCHKREIS\nBOLT CIRCLE"), QStringLiteral("hole_bolt"), true, ""},
                {6, QStringLiteral("RASTER\nGRID"), QStringLiteral("hole_grid"), true, ""},
                {7, QStringLiteral("PUNKTE\nLOCATIONS"), QStringLiteral("hole_locations"), true, ""},
                {8, QStringLiteral("ZURÜCK\n(EXIT)"), QStringLiteral("back_newblock"), true, ""}
            };
            break;

        // ─── 5. Block-Bearbeitung (Block Edit) ───
        case SoftkeyMenu::BlockEdit:
            m_currentDefs = {
                {1, QStringLiteral("VORHERIGER\nBLOCK"), QStringLiteral("blk_prev"), true, ""},
                {2, QStringLiteral("NÄCHSTER\nBLOCK"), QStringLiteral("blk_next"), true, ""},
                {3, QStringLiteral("NEUER\nBLOCK"), QStringLiteral("menu_newblock"), true, ""},
                {4, QStringLiteral("BLOCK\nLÖSCHEN"), QStringLiteral("blk_del"), true, ""},
                {5, QStringLiteral("WERKSTÜCK\nSETUP"), QStringLiteral("setup"), true, ""},
                {6, QStringLiteral("WERKZEUG\nSETUP"), QStringLiteral("tools"), true, ""},
                {7, QStringLiteral("BERECHNEN\n(RUN)"), QStringLiteral("prog_calc"), true, ""},
                {8, QStringLiteral("HAUPTMENÜ\n(EXIT)"), QStringLiteral("back_main"), true, ""}
            };
            break;

        // ─── 6. Kontur-Bearbeitung (Contour Edit) ───
        case SoftkeyMenu::ContourEdit:
            m_currentDefs = {
                {1, QStringLiteral("VORHERIGES\nSEGMENT"), QStringLiteral("seg_prev"), true, ""},
                {2, QStringLiteral("NÄCHSTES\nSEGMENT"), QStringLiteral("seg_next"), true, ""},
                {3, QStringLiteral("NEUES\nSEGMENT"), QStringLiteral("menu_newseg"), true, ""},
                {4, QStringLiteral("WERTE\nSPEICHERN"), QStringLiteral("seg_store"), true, ""},
                {5, QStringLiteral("NÄCHSTE\nLÖSUNG"), QStringLiteral("seg_find_alt"), true, ""},
                {6, QStringLiteral("SEGMENT\nLÖSCHEN"), QStringLiteral("seg_del"), true, ""},
                {7, QStringLiteral("KONTUR\nSCHLIESSEN"), QStringLiteral("seg_close"), true, ""},
                {8, QStringLiteral("ZURÜCK\n(EXIT)"), QStringLiteral("back_block"), true, ""}
            };
            break;

        // ─── 7. Neues Segment (New Segment) ───
        case SoftkeyMenu::NewSegment:
            m_currentDefs = {
                {1, QStringLiteral("GERADE\n(LINE)"), QStringLiteral("newseg_line"), true, ""},
                {2, QStringLiteral("BOGEN\n(ARC)"), QStringLiteral("newseg_arc"), true, ""},
                {3, QStringLiteral("RUNDUNG\n(BLEND ARC)"), QStringLiteral("newseg_blend"), true, ""},
                {4, QStringLiteral("HELIX\n(HELIX)"), QStringLiteral("newseg_helix"), true, ""},
                {5, QStringLiteral("3D-BOGEN\n(3D ARC)"), QStringLiteral("newseg_3darc"), true, ""},
                {6, QStringLiteral(""), QStringLiteral(""), false, ""},
                {7, QStringLiteral(""), QStringLiteral(""), false, ""},
                {8, QStringLiteral("ZURÜCK\n(EXIT)"), QStringLiteral("back_contouredit"), true, ""}
            };
            break;

        // ─── 8. Handbetrieb (Manual Jog) ───
        case SoftkeyMenu::ManualJog:
            m_currentDefs = {
                {1, QStringLiteral("X  −"), QStringLiteral("jog_xm"), true, ""},
                {2, QStringLiteral("X  +"), QStringLiteral("jog_xp"), true, ""},
                {3, QStringLiteral("Y  −"), QStringLiteral("jog_ym"), true, ""},
                {4, QStringLiteral("Y  +"), QStringLiteral("jog_yp"), true, ""},
                {5, QStringLiteral("Z  −"), QStringLiteral("jog_zm"), true, ""},
                {6, QStringLiteral("Z  +"), QStringLiteral("jog_zp"), true, ""},
                {7, QStringLiteral("G54 NULLEN\nHOME ALLE"), QStringLiteral("jog_zero"), true, ""},
                {8, QStringLiteral("HAUPTMENÜ\n(EXIT)"), QStringLiteral("back_main"), true, ""}
            };
            break;

        // ─── 9. 3D-Simulation & Grafik (Max5) ───
        case SoftkeyMenu::Simulation:
            m_currentDefs = {
                {1, QStringLiteral("START\n(RUN)"), QStringLiteral("sim_start"), true, ""},
                {2, QStringLiteral("PAUSE\n(HALT)"), QStringLiteral("sim_pause"), true, ""},
                {3, QStringLiteral("SATZ EINZELN\n(STEP)"), QStringLiteral("sim_step"), true, ""},
                {4, QStringLiteral("RESET\n(ANFANG)"), QStringLiteral("sim_reset"), true, ""},
                {5, QStringLiteral("ANSICHT\nISOMETRISCH"), QStringLiteral("sim_view_iso"), true, ""},
                {6, QStringLiteral("ANSICHT\nXY (OBEN)"), QStringLiteral("sim_view_xy"), true, ""},
                {7, QStringLiteral("ANSICHT\nXZ (VORNE)"), QStringLiteral("sim_view_xz"), true, ""},
                {8, QStringLiteral("HAUPTMENÜ\n(EXIT)"), QStringLiteral("back_main"), true, ""}
            };
            break;
    }

    updateButtons();
}

void WinMaxSoftkeyBar::updateButtons() {
    for (int i = 0; i < NUM_SOFTKEYS; ++i) {
        if (i < static_cast<int>(m_currentDefs.size())) {
            const auto& def = m_currentDefs[i];
            m_buttons[i]->setEnabled(def.enabled);

            if (def.enabled && !def.label.isEmpty()) {
                m_labelMains[i]->setText(def.label);
                m_labelFKeys[i]->setText(QString("F%1").arg(def.keyIndex));
                m_labelMains[i]->setStyleSheet(QStringLiteral("color: #F8FAFC; font-weight: bold; font-size: 11px; background: transparent; border: none;"));
                m_labelFKeys[i]->setStyleSheet(QStringLiteral("color: #00D2FF; font-weight: bold; font-family: Consolas, monospace; font-size: 10px; background: transparent; border: none;"));
            } else {
                m_labelMains[i]->setText(QString());
                m_labelFKeys[i]->setText(QString());
            }
        } else {
            m_buttons[i]->setEnabled(false);
            m_labelMains[i]->setText(QString());
            m_labelFKeys[i]->setText(QString());
        }
    }
}

void WinMaxSoftkeyBar::triggerKey(int fKeyNumber) {
    if (fKeyNumber >= 1 && fKeyNumber <= NUM_SOFTKEYS) {
        onButtonClicked(fKeyNumber);
    }
}

void WinMaxSoftkeyBar::onButtonClicked(int fKeyNumber) {
    int idx = fKeyNumber - 1;
    if (idx >= 0 && idx < static_cast<int>(m_currentDefs.size())) {
        const auto& def = m_currentDefs[idx];
        if (def.enabled && !def.actionKey.isEmpty()) {
            emit softkeyTriggered(m_currentMenu, fKeyNumber, def.actionKey);
        }
    }
}

} // namespace GeminiCNC::UI
