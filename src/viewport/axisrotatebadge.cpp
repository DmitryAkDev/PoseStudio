/**
 * @file axisrotatebadge.cpp
 * @brief Implementation of AxisRotateBadge. See axisrotatebadge.h.
 */

#include "axisrotatebadge.h"

#include <QColor>
#include <QFont>
#include <QPainter>
#include <QPainterPath>
#include <QPen>
#include <QString>

#include <cmath>

namespace pose {

AxisRotateBadge::AxisRotateBadge(QWidget* parent, int height) : QWidget(parent) {
    setFixedSize(66, height);
    setToolTip(tr("Roll the mouse wheel to rotate the selected joint about this axis"));
}

void AxisRotateBadge::setAxis(int axis) {
    m_axis = axis;
    update();
}

void AxisRotateBadge::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    const QRectF r = QRectF(rect()).adjusted(0.5, 0.5, -0.5, -0.5);
    p.setPen(QColor(0x55, 0x55, 0x55));
    p.setBrush(QColor(0x25, 0x26, 0x27));
    p.drawRoundedRect(r, 4.0, 4.0);
    if (m_axis < 0 || m_axis > 2) {
        return;
    }
    static const QColor kAxisColor[3] = {QColor(0xe0, 0x55, 0x55), QColor(0x5f, 0xc4, 0x5f),
                                         QColor(0x5b, 0x87, 0xcc)};
    static const char* const kAxisName[3] = {"X", "Y", "Z"};
    const QColor c = kAxisColor[m_axis];

    // Rotation glyph: a 270° arc (counter-clockwise from 45°) with an arrowhead on its end.
    const qreal  d = r.height() * 0.52;
    const QRectF arc(r.left() + 9.0, r.center().y() - d * 0.5, d, d);
    QPen         pen(c, 2.0);
    pen.setCapStyle(Qt::RoundCap);
    p.setPen(pen);
    p.setBrush(Qt::NoBrush);
    p.drawArc(arc, 45 * 16, 270 * 16);
    const qreal   endDeg = 45.0 + 270.0; // Qt angles: 0° = 3 o'clock, counter-clockwise
    const qreal   endRad = endDeg * 3.14159265 / 180.0;
    const QPointF tip(arc.center().x() + std::cos(endRad) * d * 0.5,
                      arc.center().y() - std::sin(endRad) * d * 0.5);
    const QPointF tangent(-std::sin(endRad), -std::cos(endRad)); // direction of travel (screen)
    const QPointF normal(-tangent.y(), tangent.x());
    const QPointF base = tip - tangent * 5.0;
    QPainterPath  head;
    head.moveTo(tip + tangent * 1.5);
    head.lineTo(base + normal * 3.5);
    head.lineTo(base - normal * 3.5);
    head.closeSubpath();
    p.setPen(Qt::NoPen);
    p.setBrush(c);
    p.drawPath(head);

    // The axis letter.
    QFont f = font();
    f.setBold(true);
    f.setPointSizeF(f.pointSizeF() + 1.0);
    p.setFont(f);
    p.setPen(c);
    p.drawText(QRectF(arc.right() + 8.0, r.top(), r.right() - arc.right() - 8.0, r.height()),
               Qt::AlignVCenter | Qt::AlignLeft, QString::fromLatin1(kAxisName[m_axis]));
}

} // namespace pose
