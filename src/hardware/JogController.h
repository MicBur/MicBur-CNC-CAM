#ifndef GEMINI_CNC_JOGCONTROLLER_H
#define GEMINI_CNC_JOGCONTROLLER_H

#include "IKlipperClient.h"
#include <QObject>

namespace GeminiCNC::Hardware {

enum class JogAxis {
    X,
    Y,
    Z,
    A
};

/**
 * @brief Manuelle Achs-Jog-Steuerung mit einstellbaren Schrittweiten und Sicherheitsfunktionen.
 */
class JogController : public QObject {
    Q_OBJECT
public:
    explicit JogController(IKlipperClient* client, QObject* parent = nullptr);

    void setStepDistance(double distanceMm);
    [[nodiscard]] double stepDistance() const { return m_stepDistance; }

    void setJogFeedRate(double feedMmMin);
    [[nodiscard]] double jogFeedRate() const { return m_jogFeedRate; }

    void jogAxis(JogAxis axis, int direction); // direction: +1 oder -1
    void homeAxis(JogAxis axis);
    void homeAll();
    void zeroAxis(JogAxis axis);
    void zeroAll();

    void setSpindleState(bool on, double rpm = 18000.0);

private:
    IKlipperClient* m_client{nullptr};
    double m_stepDistance{1.0}; // Standard 1mm
    double m_jogFeedRate{1200.0}; // 1200 mm/min
    bool m_spindleOn{false};
};

} // namespace GeminiCNC::Hardware

#endif // GEMINI_CNC_JOGCONTROLLER_H
