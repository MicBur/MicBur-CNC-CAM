#ifndef GEMINI_CNC_WINMAXHEADERBAR_H
#define GEMINI_CNC_WINMAXHEADERBAR_H

#include <QWidget>
#include <QLabel>
#include <QString>
#include "core/Vector3D.h"
#include "core/ToolDefinition.h"

namespace GeminiCNC::UI {

/**
 * @brief Hurco WinMax-typische Kopfzeile mit DRO, Programmstatus und Werkzeuganzeige.
 */
class WinMaxHeaderBar : public QWidget {
    Q_OBJECT
public:
    explicit WinMaxHeaderBar(QWidget* parent = nullptr);

    void setProgramInfo(const QString& programName, const QString& activeBlockInfo);
    void setMachineState(const QString& stateText, const QString& badgeColor = "#38A169");
    void setCoordinates(double x, double y, double z);
    void setTechnology(double rpm, double feed, int rpmOverride = 100, int feedOverride = 100);
    void setActiveTool(const Core::ToolDefinition& tool);

private:
    void setupUi();

    QLabel* m_lblLogo{nullptr};
    QLabel* m_lblProgramInfo{nullptr};
    QLabel* m_lblStateBadge{nullptr};

    QLabel* m_lblCoordX{nullptr};
    QLabel* m_lblCoordY{nullptr};
    QLabel* m_lblCoordZ{nullptr};

    QLabel* m_lblTechnology{nullptr};
    QLabel* m_lblTool{nullptr};
};

} // namespace GeminiCNC::UI

#endif // GEMINI_CNC_WINMAXHEADERBAR_H
