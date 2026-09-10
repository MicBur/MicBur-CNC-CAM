#ifndef GEMINI_CNC_TOOLGRAPHICSWIDGET_H
#define GEMINI_CNC_TOOLGRAPHICSWIDGET_H

#include <QWidget>
#include <QPainter>
#include "core/ToolDefinition.h"

namespace GeminiCNC::UI {

/**
 * @brief Maßstäbliche 2D-Schnittzeichnung eines CNC-Werkzeugs im Hurco WinMax-Stil.
 *
 * Zeichnet per QPainter eine typabhängige Werkzeug-Silhouette mit Bemaßungslinien,
 * Farbcodierung (Schneide=Silber, Schaft=Hellgrau, Halter=Anthrazit) und TCP-Markierung.
 * Skaliert automatisch auf die verfügbare Widget-Größe.
 */
class ToolGraphicsWidget : public QWidget {
    Q_OBJECT
public:
    explicit ToolGraphicsWidget(QWidget* parent = nullptr);

    void setToolDefinition(const Core::ToolDefinition& tool);
    [[nodiscard]] const Core::ToolDefinition& toolDefinition() const { return m_tool; }

protected:
    void paintEvent(QPaintEvent* event) override;
    [[nodiscard]] QSize minimumSizeHint() const override { return {200, 250}; }
    [[nodiscard]] QSize sizeHint() const override { return {320, 400}; }

private:
    void drawToolSilhouette(QPainter& p, const QRectF& area);
    void drawDimensionLine(QPainter& p, const QPointF& from, const QPointF& to,
                           const QString& label, bool horizontal, double offset);
    void drawTcpMarker(QPainter& p, const QPointF& tcp);

    Core::ToolDefinition m_tool;
};

} // namespace GeminiCNC::UI

#endif // GEMINI_CNC_TOOLGRAPHICSWIDGET_H
