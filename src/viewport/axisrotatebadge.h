/**
 * @file axisrotatebadge.h
 * @brief The viewport strip's "rotating about an axis" badge.
 *
 * Shown under the strip's buttons for as long as X, Y, or Z is held with a joint selected — the
 * mouse wheel's meaning is MODAL during the hold (it rotates the joint instead of dollying), so
 * the mode must be visible. A rotation-arrow glyph and the axis letter in the axis colour
 * (X red, Y green, Z the accent blue — the DCC convention), on the same surface as the strip's
 * buttons. Painted, not an icon file: the colour is the message and there are three of them.
 */

#ifndef AXISROTATEBADGE_H
#define AXISROTATEBADGE_H

#include <QWidget>

class QPaintEvent;

namespace pose {

/**
 * @class AxisRotateBadge
 * @brief A fixed-size painted badge naming the axis the wheel currently rotates about.
 */
class AxisRotateBadge : public QWidget {
    Q_OBJECT
public:
    /// @p height matches the strip's field height so the badge rows align with the buttons.
    explicit AxisRotateBadge(QWidget* parent, int height);

    /// @p axis 0/1/2 = X/Y/Z; anything else paints the empty surface.
    void setAxis(int axis);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    int m_axis = -1;
};

} // namespace pose

#endif // AXISROTATEBADGE_H
