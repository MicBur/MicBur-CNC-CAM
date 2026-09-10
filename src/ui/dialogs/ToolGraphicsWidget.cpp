#include "ToolGraphicsWidget.h"
#include <QPainterPath>
#include <cmath>

namespace GeminiCNC::UI {

ToolGraphicsWidget::ToolGraphicsWidget(QWidget* parent) : QWidget(parent) {
    setMinimumSize(200, 250);
    setStyleSheet("background-color: #0D1117; border: 1px solid #1F2937; border-radius: 4px;");
}

void ToolGraphicsWidget::setToolDefinition(const Core::ToolDefinition& tool) {
    m_tool = tool;
    update();
}

void ToolGraphicsWidget::paintEvent(QPaintEvent* event) {
    Q_UNUSED(event);
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);

    // Dunkler Hintergrund mit feinem Rand
    p.fillRect(rect(), QColor(13, 17, 23));
    p.setPen(QPen(QColor(31, 41, 55), 1));
    p.drawRect(rect().adjusted(0, 0, -1, -1));

    // Titel
    QFont titleFont("Consolas", 10, QFont::Bold);
    p.setFont(titleFont);
    p.setPen(QColor(148, 163, 184));
    QString title = QString("T%1  %2").arg(m_tool.id).arg(Core::toolTypeToString(m_tool.type));
    p.drawText(QRectF(0, 4, width(), 20), Qt::AlignHCenter, title);

    // Zeichenbereich (mit Rand für Bemaßungen)
    QRectF drawArea(50.0, 30.0, width() - 100.0, height() - 50.0);
    if (drawArea.width() < 80 || drawArea.height() < 100) return;

    drawToolSilhouette(p, drawArea);
}

void ToolGraphicsWidget::drawToolSilhouette(QPainter& p, const QRectF& area) {
    // Maße aus dem Werkzeug
    const double diaFlute = m_tool.diameter;
    const double lenFlute = m_tool.fluteLength;
    const double diaShaft = m_tool.shaftDiameter;
    const double lenStick = m_tool.stickOutLength;
    const double diaHolder = m_tool.holderDiameter;
    const double lenHolder = 25.0;

    // Gesamthöhe und maximale Breite für Skalierung
    const double totalH = lenStick + lenHolder;
    const double maxDia = std::max({diaFlute, diaShaft, diaHolder});

    if (totalH < 0.1 || maxDia < 0.1) return;

    // Skalierungsfaktor: Werkzeug passt in 80% des Zeichenbereichs
    double scaleX = (area.width() * 0.65) / maxDia;
    double scaleY = (area.height() * 0.85) / totalH;
    double scale = std::min(scaleX, scaleY);

    // Nullpunkt: Mitte-Unten im Zeichenbereich = TCP (Werkzeugspitze)
    double cx = area.center().x();
    double bottomY = area.bottom() - 20.0;

    // Lambda: Werkzeug-Koordinaten → Bildschirm (Y nach oben, Z=0 ist Spitze)
    auto toScreen = [&](double xMm, double zMm) -> QPointF {
        return QPointF(cx + xMm * scale, bottomY - zMm * scale);
    };

    // ═══════════════════════════════════════════════════════
    // 1. Halter (Dunkelgrau / Anthrazit)
    // ═══════════════════════════════════════════════════════
    {
        double hz0 = lenStick;
        double hz1 = lenStick + lenHolder;
        double hr = diaHolder * 0.5;

        QRectF holderRect(toScreen(-hr, hz1), toScreen(hr, hz0));
        QLinearGradient holderGrad(holderRect.topLeft(), holderRect.topRight());
        holderGrad.setColorAt(0.0, QColor(50, 55, 65));
        holderGrad.setColorAt(0.3, QColor(85, 90, 100));
        holderGrad.setColorAt(0.7, QColor(85, 90, 100));
        holderGrad.setColorAt(1.0, QColor(50, 55, 65));
        p.setBrush(holderGrad);
        p.setPen(QPen(QColor(40, 45, 55), 1.5));
        p.drawRoundedRect(holderRect, 2, 2);
    }

    // ═══════════════════════════════════════════════════════
    // 2. Schaft (Hellgrau / Stahl)
    // ═══════════════════════════════════════════════════════
    if (lenStick > lenFlute + 0.5) {
        double sz0 = lenFlute;
        double sz1 = lenStick;
        double sr = diaShaft * 0.5;

        QRectF shankRect(toScreen(-sr, sz1), toScreen(sr, sz0));
        QLinearGradient shankGrad(shankRect.topLeft(), shankRect.topRight());
        shankGrad.setColorAt(0.0, QColor(140, 145, 155));
        shankGrad.setColorAt(0.35, QColor(195, 200, 210));
        shankGrad.setColorAt(0.65, QColor(195, 200, 210));
        shankGrad.setColorAt(1.0, QColor(140, 145, 155));
        p.setBrush(shankGrad);
        p.setPen(QPen(QColor(110, 115, 125), 1.2));
        p.drawRect(shankRect);
    }

    // ═══════════════════════════════════════════════════════
    // 3. Schneide (Silber / Hartmetall) — typabhängig
    // ═══════════════════════════════════════════════════════
    {
        double fr = diaFlute * 0.5;

        QLinearGradient fluteGrad(toScreen(-fr, lenFlute), toScreen(fr, lenFlute));
        fluteGrad.setColorAt(0.0, QColor(160, 165, 175));
        fluteGrad.setColorAt(0.2, QColor(210, 215, 225));
        fluteGrad.setColorAt(0.5, QColor(230, 235, 240));
        fluteGrad.setColorAt(0.8, QColor(210, 215, 225));
        fluteGrad.setColorAt(1.0, QColor(160, 165, 175));
        p.setBrush(fluteGrad);
        p.setPen(QPen(QColor(130, 135, 145), 1.5));

        QPainterPath flutePath;

        switch (m_tool.type) {
            case Core::ToolType::BallMill: {
                // Rechteck oben + Halbkreis unten
                QPointF tl = toScreen(-fr, lenFlute);
                QPointF br = toScreen(fr, fr); // Zylinder bis Kugelradius
                if (lenFlute > fr) {
                    flutePath.addRect(QRectF(tl, br));
                }
                // Halbkreis
                QPointF ballCenter = toScreen(0, fr);
                double ballScreenR = fr * scale;
                QRectF ballRect(ballCenter.x() - ballScreenR, ballCenter.y() - ballScreenR,
                                ballScreenR * 2, ballScreenR * 2);
                flutePath.moveTo(toScreen(-fr, fr));
                flutePath.arcTo(ballRect, 180.0, 180.0);
                break;
            }

            case Core::ToolType::ChamferMill: {
                // Trapez: oben breit, unten schmal (10% vom Radius)
                double tipR = fr * 0.1;
                flutePath.moveTo(toScreen(-tipR, 0));
                flutePath.lineTo(toScreen(-fr, lenFlute));
                flutePath.lineTo(toScreen(fr, lenFlute));
                flutePath.lineTo(toScreen(tipR, 0));
                flutePath.closeSubpath();
                break;
            }

            case Core::ToolType::Drill: {
                // Dreieckspitze unten + Zylinder oben
                double tipH = fr * 0.6;
                // Spitze
                flutePath.moveTo(toScreen(0, 0));
                flutePath.lineTo(toScreen(-fr, tipH));
                flutePath.lineTo(toScreen(fr, tipH));
                flutePath.closeSubpath();
                // Zylindrischer Körper
                if (lenFlute > tipH) {
                    QRectF bodyRect(toScreen(-fr, lenFlute), toScreen(fr, tipH));
                    flutePath.addRect(bodyRect);
                }
                break;
            }

            case Core::ToolType::EndMill:
            case Core::ToolType::FaceMill:
            default: {
                // Gerader Zylinder (Rechteck)
                QRectF fluteRect(toScreen(-fr, lenFlute), toScreen(fr, 0));
                flutePath.addRect(fluteRect);
                break;
            }
        }

        p.drawPath(flutePath);
    }

    // ═══════════════════════════════════════════════════════
    // 4. Schneidenrillen (Verzierung für Realismus)
    // ═══════════════════════════════════════════════════════
    {
        int numFlutes = std::max(1, m_tool.flutes);
        double fr = diaFlute * 0.5;
        p.setPen(QPen(QColor(130, 135, 145, 80), 0.8, Qt::DashLine));

        double fluteVisibleH = lenFlute;
        if (m_tool.type == Core::ToolType::Drill) {
            fluteVisibleH = lenFlute - fr * 0.6;
        }

        for (int i = 0; i < numFlutes && i < 6; ++i) {
            double xOff = -fr * 0.7 + (fr * 1.4 * (i + 1)) / (numFlutes + 1);
            double z0 = (m_tool.type == Core::ToolType::Drill) ? fr * 0.6 : 0.0;
            p.drawLine(toScreen(xOff, z0), toScreen(xOff, z0 + fluteVisibleH));
        }
    }

    // ═══════════════════════════════════════════════════════
    // 5. TCP-Markierung (roter Punkt an der Spitze)
    // ═══════════════════════════════════════════════════════
    drawTcpMarker(p, toScreen(0, 0));

    // ═══════════════════════════════════════════════════════
    // 6. Bemaßungslinien
    // ═══════════════════════════════════════════════════════
    QFont dimFont("Consolas", 8);
    p.setFont(dimFont);

    double fr = diaFlute * 0.5;
    double hr = diaHolder * 0.5;

    // Ø Schneide (horizontal unten)
    drawDimensionLine(p, toScreen(-fr, 0), toScreen(fr, 0),
                      QString("Ø%1").arg(diaFlute, 0, 'f', 1), true, 18.0);

    // Schneidenlänge (vertikal links)
    drawDimensionLine(p, toScreen(-fr, 0), toScreen(-fr, lenFlute),
                      QString("%1").arg(lenFlute, 0, 'f', 1), false, -30.0);

    // Auskragung (vertikal ganz links)
    drawDimensionLine(p, toScreen(-hr, 0), toScreen(-hr, lenStick),
                      QString("L=%1").arg(lenStick, 0, 'f', 1), false, -55.0);

    // Ø Halter (horizontal oben)
    double holderMidZ = lenStick + lenHolder * 0.5;
    drawDimensionLine(p, toScreen(-hr, holderMidZ), toScreen(hr, holderMidZ),
                      QString("Ø%1").arg(diaHolder, 0, 'f', 0), true, -14.0);

    // Zähne-Info rechts oben
    p.setPen(QColor(100, 116, 139));
    p.drawText(QPointF(area.right() - 35, area.top() + 15),
               QString("z=%1").arg(m_tool.flutes));
}

void ToolGraphicsWidget::drawDimensionLine(QPainter& p, const QPointF& from, const QPointF& to,
                                            const QString& label, bool horizontal, double offset) {
    QPen dimPen(QColor(0, 210, 255, 200), 0.8);
    p.setPen(dimPen);

    QPointF f, t;
    if (horizontal) {
        f = QPointF(from.x(), from.y() + offset);
        t = QPointF(to.x(), to.y() + offset);

        // Verlängerungslinien
        p.setPen(QPen(QColor(0, 210, 255, 100), 0.5));
        p.drawLine(from, f);
        p.drawLine(to, t);

        // Maßlinie
        p.setPen(dimPen);
        p.drawLine(f, t);

        // Pfeile
        double arrowW = 4.0;
        p.drawLine(f, QPointF(f.x() + arrowW, f.y() - arrowW));
        p.drawLine(f, QPointF(f.x() + arrowW, f.y() + arrowW));
        p.drawLine(t, QPointF(t.x() - arrowW, t.y() - arrowW));
        p.drawLine(t, QPointF(t.x() - arrowW, t.y() + arrowW));
    } else {
        f = QPointF(from.x() + offset, from.y());
        t = QPointF(to.x() + offset, to.y());

        // Verlängerungslinien
        p.setPen(QPen(QColor(0, 210, 255, 100), 0.5));
        p.drawLine(from, f);
        p.drawLine(to, t);

        // Maßlinie
        p.setPen(dimPen);
        p.drawLine(f, t);

        // Pfeile
        double arrowH = 4.0;
        p.drawLine(f, QPointF(f.x() - arrowH, f.y() - arrowH));
        p.drawLine(f, QPointF(f.x() + arrowH, f.y() - arrowH));
        p.drawLine(t, QPointF(t.x() - arrowH, t.y() + arrowH));
        p.drawLine(t, QPointF(t.x() + arrowH, t.y() + arrowH));
    }

    // Label
    p.setPen(QColor(0, 210, 255, 230));
    QFont dimFont("Consolas", 8, QFont::Bold);
    p.setFont(dimFont);

    QPointF mid((f.x() + t.x()) * 0.5, (f.y() + t.y()) * 0.5);
    QRectF labelRect;
    if (horizontal) {
        labelRect = QRectF(mid.x() - 30, mid.y() - 14, 60, 14);
    } else {
        labelRect = QRectF(mid.x() - 35, mid.y() - 7, 35, 14);
    }

    // Hintergrund für Lesbarkeit
    p.fillRect(labelRect.adjusted(-2, -1, 2, 1), QColor(13, 17, 23, 220));
    p.drawText(labelRect, Qt::AlignCenter, label);
}

void ToolGraphicsWidget::drawTcpMarker(QPainter& p, const QPointF& tcp) {
    // Rotes Fadenkreuz am TCP
    double sz = 8.0;
    p.setPen(QPen(QColor(255, 60, 60), 1.5));
    p.drawLine(tcp.x() - sz, tcp.y(), tcp.x() + sz, tcp.y());
    p.drawLine(tcp.x(), tcp.y() - sz, tcp.x(), tcp.y() + sz);

    // Kleiner Kreis
    p.setPen(QPen(QColor(255, 60, 60), 1.0));
    p.setBrush(Qt::NoBrush);
    p.drawEllipse(tcp, 4.0, 4.0);

    // Label
    QFont tcpFont("Consolas", 7, QFont::Bold);
    p.setFont(tcpFont);
    p.setPen(QColor(255, 100, 100));
    p.drawText(QPointF(tcp.x() + 10, tcp.y() + 4), "TCP");
}

} // namespace GeminiCNC::UI
