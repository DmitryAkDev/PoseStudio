/**
 * @file dragnumberbox.cpp
 * @brief Implementation of DragNumberBox. See dragnumberbox.h.
 */

#include "dragnumberbox.h"

#include <QEvent>
#include <QFocusEvent>
#include <QHideEvent>
#include <QKeyEvent>
#include <QLineEdit>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>

#include <algorithm>
#include <cmath>

namespace pose {

namespace {
// Pixels of horizontal movement before a press commits to being a scrub rather than a
// click. Deliberately smaller than QApplication::startDragDistance() (a drag-and-drop
// threshold, ~10px): a scrub should engage almost immediately, and an accidental
// couple-of-pixels twitch landing in edit mode instead is a cheap mistake (Esc).
constexpr double kDragThresholdPx = 3.0;

// Corner radius matching the app's input fields (spin boxes, HDRI button: 4px).
constexpr double kCornerRadius = 4.0;
} // namespace

DragNumberBox::DragNumberBox(QWidget* parent) : QWidget(parent) {
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    setFocusPolicy(Qt::ClickFocus); // so committing the editor has somewhere to return focus
    // Left-right arrow on hover: the affordance that this field scrubs horizontally. (The inline
    // editor is a child QLineEdit with its own I-beam, so edit mode is unaffected.)
    setCursor(Qt::SizeHorCursor);
}

void DragNumberBox::setRange(double min, double max) {
    m_min = min;
    m_max = std::max(min, max);
    setValue(m_value); // re-clamp into the new range
    update();
}

void DragNumberBox::setSingleStep(double step) {
    m_step = (step > 0.0) ? step : 0.01;
}

void DragNumberBox::setDecimals(int decimals) {
    m_decimals = std::max(0, decimals);
    update();
}

void DragNumberBox::setValue(double value) {
    const double clamped = std::clamp(value, m_min, m_max);
    // Half-a-step epsilon-of-step comparisons would fight typed precision; compare against a
    // tolerance far below anything the step/decimals can represent instead.
    if (std::abs(clamped - m_value) < 1e-9) {
        return;
    }
    m_value = clamped;
    update();
    emit valueChanged(m_value);
}

QSize DragNumberBox::sizeHint() const {
    return QSize(120, 24); // one text row + the border, matching the panel's other inputs
}

QSize DragNumberBox::minimumSizeHint() const {
    return QSize(48, 24);
}

QString DragNumberBox::formattedValue() const {
    return QString::number(m_value, 'f', m_decimals);
}

void DragNumberBox::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);

    // Half-pixel inset so the 1px border strokes on pixel centers (crisp, not smeared).
    const QRectF r = QRectF(rect()).adjusted(0.5, 0.5, -0.5, -0.5);
    QPainterPath box;
    box.addRoundedRect(r, kCornerRadius, kCornerRadius);

    p.fillPath(box, m_backgroundColor);

    // The range fill: left-to-right, proportional to where the value sits in [min, max].
    // Clipped to the rounded box so the fill's leading edge stays square while the
    // left corners keep the border radius.
    const double range = m_max - m_min;
    const double fraction = (range > 0.0) ? std::clamp((m_value - m_min) / range, 0.0, 1.0) : 0.0;
    if (fraction > 0.0) {
        p.save();
        p.setClipPath(box);
        p.fillRect(QRectF(r.left(), r.top(), r.width() * fraction, r.height()), m_fillColor);
        p.restore();
    }

    p.setPen(QPen(m_hovered ? m_hoverBorderColor : m_borderColor, 1.0));
    p.drawPath(box);

    p.setPen(m_textColor);
    p.drawText(rect(), Qt::AlignCenter, formattedValue());
}

void DragNumberBox::mousePressEvent(QMouseEvent* event) {
    if (event->button() != Qt::LeftButton) {
        QWidget::mousePressEvent(event);
        return;
    }
    m_pressed = true;
    m_dragging = false;
    m_pressPos = event->position();
    m_pressValue = m_value;
    event->accept();
}

void DragNumberBox::mouseMoveEvent(QMouseEvent* event) {
    if (!m_pressed) {
        QWidget::mouseMoveEvent(event);
        return;
    }
    const double dx = event->position().x() - m_pressPos.x();
    if (!m_dragging && std::abs(dx) < kDragThresholdPx) {
        return; // still within click territory
    }
    if (!m_dragging) {
        m_dragging = true;
        emit editingStarted(); // before the first scrub value lands, so listeners see pre-edit state
    }

    // Anchored scrub: a full-widget-width sweep covers the whole range, snapped to the step
    // so the readout lands on the same values the old slider/spin pair produced.
    const double range = m_max - m_min;
    const double raw = m_pressValue + (dx / std::max(1, width())) * range;
    const double snapped = m_min + std::round((raw - m_min) / m_step) * m_step;
    setValue(snapped);
    event->accept();
}

void DragNumberBox::mouseReleaseEvent(QMouseEvent* event) {
    if (event->button() != Qt::LeftButton || !m_pressed) {
        QWidget::mouseReleaseEvent(event);
        return;
    }
    const bool wasClick = !m_dragging;
    m_pressed = false;
    m_dragging = false;
    if (wasClick) {
        beginEdit(); // press + release with no scrub = type-in editing
    } else {
        emit editingFinished(); // scrub gesture committed
    }
    event->accept();
}

void DragNumberBox::resizeEvent(QResizeEvent* event) {
    QWidget::resizeEvent(event);
    if (m_editor && m_editing) {
        m_editor->setGeometry(rect());
    }
}

// The mouse release can be lost mid-gesture: a popup grab (Alt opening the menu bar, a context
// menu), the widget being hidden, or another window taking activation all steal the implicit
// mouse grab, and the release then goes to them. Without this, m_pressed stayed set and — worse
// — a scrub's editingStarted was never balanced, so a listener's depth-counted undo bracket
// stuck open for the rest of the session (the Environment panel registered no lighting undo
// entries after one such scrub). Ending the gesture here keeps the bracket balanced; the
// scrubbed value has already landed through setValue.
void DragNumberBox::abandonScrub() {
    if (!m_pressed) {
        return;
    }
    const bool wasScrub = m_dragging;
    m_pressed = false;
    m_dragging = false;
    if (wasScrub) {
        emit editingFinished(); // balances the editingStarted the scrub emitted
    }
}

bool DragNumberBox::event(QEvent* event) {
    if (event->type() == QEvent::UngrabMouse) {
        abandonScrub(); // Qt took the mouse grab away: no release is coming
    }
    return QWidget::event(event);
}

void DragNumberBox::hideEvent(QHideEvent* event) {
    abandonScrub();
    QWidget::hideEvent(event);
}

void DragNumberBox::focusOutEvent(QFocusEvent* event) {
    // Only the reasons that mean the input went elsewhere mid-gesture (a popup opened, the menu
    // bar took Alt, the window was deactivated). Other reasons — Tab, the inline editor taking
    // focus after a click — are ordinary focus traffic.
    switch (event->reason()) {
    case Qt::PopupFocusReason:
    case Qt::MenuBarFocusReason:
    case Qt::ActiveWindowFocusReason:
        abandonScrub();
        break;
    default:
        break;
    }
    QWidget::focusOutEvent(event);
}

void DragNumberBox::enterEvent(QEnterEvent*) {
    m_hovered = true;
    update();
}

void DragNumberBox::leaveEvent(QEvent*) {
    m_hovered = false;
    update();
}

void DragNumberBox::beginEdit() {
    if (!m_editor) {
        m_editor = new QLineEdit(this);
        m_editor->setObjectName(QStringLiteral("DragNumberBoxEditor"));
        m_editor->setAlignment(Qt::AlignCenter);
        m_editor->installEventFilter(this); // Esc = cancel (QLineEdit has no cancel signal)
        connect(m_editor, &QLineEdit::editingFinished, this, &DragNumberBox::commitEdit);
    }
    // Self-contained styling (the widget must look right in any container, styled or not):
    // same surface and text as the painted box, the hover-accent border to signal edit mode.
    // Built from the colour properties on every edit — not hard-coded — so a QSS re-theme
    // (qproperty-backgroundColor etc.) restyles the editor along with the painted look.
    m_editor->setStyleSheet(QStringLiteral(
                                "#DragNumberBoxEditor {"
                                "  background-color: %1;"
                                "  color: %2;"
                                "  border: 1px solid %3;"
                                "  border-radius: 4px;"
                                "  selection-background-color: %3;"
                                "}")
                                .arg(m_backgroundColor.name(), m_textColor.name(),
                                     m_hoverBorderColor.name()));
    m_editing = true;
    emit editingStarted(); // balanced by editingFinished in commitEdit/cancelEdit
    m_editor->setGeometry(rect());
    m_editor->setText(formattedValue());
    m_editor->selectAll();
    m_editor->show();
    m_editor->setFocus();
}

void DragNumberBox::commitEdit() {
    // editingFinished fires on Enter AND again on the focus-out that hiding causes; the
    // m_editing guard collapses that (and a cancelled edit) to a single no-op pass.
    if (!m_editing) {
        return;
    }
    m_editing = false;
    // Tolerate a comma decimal separator; QString::toDouble parses C-format only.
    QString text = m_editor->text().trimmed();
    text.replace(QLatin1Char(','), QLatin1Char('.'));
    bool ok = false;
    const double typed = text.toDouble(&ok);
    m_editor->hide();
    setFocus(Qt::OtherFocusReason);
    if (ok) {
        setValue(typed); // clamps to range; unparseable input just keeps the old value
    }
    update();
    emit editingFinished();
}

void DragNumberBox::cancelEdit() {
    m_editing = false; // must clear BEFORE hide(): hiding focuses away -> commitEdit re-entry
    m_editor->hide();
    setFocus(Qt::OtherFocusReason);
    update();
    emit editingFinished(); // balances beginEdit's editingStarted; no value change happened
}

bool DragNumberBox::eventFilter(QObject* watched, QEvent* event) {
    if (watched == m_editor && event->type() == QEvent::KeyPress) {
        if (static_cast<QKeyEvent*>(event)->key() == Qt::Key_Escape) {
            cancelEdit();
            return true;
        }
    }
    return QWidget::eventFilter(watched, event);
}

} // namespace pose
