/**
 * @file transformpanel.cpp
 * @brief Implementation of TransformPanel. See transformpanel.h.
 */

#include "transformpanel.h"

#include "dragnumberbox.h"
#include "jointtransform.h"
#include "propertyrows.h"
#include "viewportwidget.h"

#include <QFormLayout>
#include <QLabel>
#include <QPushButton>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QToolButton>
#include <QVBoxLayout>

namespace pose {

TransformPanel::TransformPanel(ViewportWidget* viewport, QWidget* parent)
    : QWidget(parent), m_viewport(viewport) {
    setObjectName(QStringLiteral("TransformPanel"));
    buildUi();
    if (m_viewport) {
        // The selection or the selected joint's pose changed — by a drag, the B / S / T keys, undo, a pose
        // load, one of these dials: the rows follow.
        connect(m_viewport, &ViewportWidget::jointTransformChanged, this, &TransformPanel::refresh);
    }
    refresh();
}

void TransformPanel::buildUi() {
    auto* outer = new QVBoxLayout(this);
    outer->setContentsMargins(0, 0, 0, 0);

    auto* scroll = new QScrollArea(this);
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    outer->addWidget(scroll);

    auto* content = new QWidget;
    content->setObjectName(QStringLiteral("TransformContent"));
    auto* col = new QVBoxLayout(content);
    col->setContentsMargins(12, 12, 12, 12);
    col->setSpacing(12);

    m_jointName = new QLabel;
    m_jointName->setObjectName(QStringLiteral("TransformJointName"));
    col->addWidget(m_jointName);

    // --- Rotation: the three dials, in JointDialKind order ---
    QFormLayout* rotation = propertyrows::addGroup(col, tr("Rotation"));
    const std::array<QString, 3> labels = {tr("Bend"), tr("Side-Side"), tr("Twist")};
    const std::array<QString, 3> tips = {
        tr("Bends the joint in its natural direction. 0 is the rest pose, 100 is fully bent; "
           "negative values bend it the other way, as far as the joint allows.\n"
           "In the viewport: press B and move the mouse."),
        tr("Bends the joint in its secondary direction. 0 is the rest pose, 100 is the joint's limit; "
           "negative values bend it the other way, as far as the joint allows.\n"
           "In the viewport: press S and move the mouse."),
        tr("Turns the joint about its own length. 0 is the rest pose, 100 is the joint's limit; "
           "negative values turn it the other way, as far as the joint allows.\n"
           "In the viewport: press T and move the mouse."),
    };
    for (int i = 0; i < 3; ++i) {
        Row& row = m_rows[static_cast<std::size_t>(i)];
        row.label = new QLabel(labels[static_cast<std::size_t>(i)]);
        row.label->setToolTip(tips[static_cast<std::size_t>(i)]);

        row.box = new DragNumberBox;
        row.box->setRange(-100.0, 100.0);
        row.box->setOrigin(0.0); // the rest pose: the bar fills from it, either way
        row.box->setSingleStep(1.0);
        row.box->setDecimals(0);
        row.box->setToolTip(tips[static_cast<std::size_t>(i)]);
        // One undo entry per completed gesture (a scrub or a type-in), not one per value it
        // passes through: the box brackets the user's changes with these two signals.
        connect(row.box, &DragNumberBox::editingStarted, this, [this]() {
            if (m_viewport) {
                m_viewport->beginJointTransformEdit();
            }
        });
        connect(row.box, &DragNumberBox::editingFinished, this, [this]() {
            if (m_viewport) {
                m_viewport->endJointTransformEdit();
            }
        });
        // Live: every value the box lands on poses the joint. The pose may rest short of it (a
        // joint limit, the body in the way), so the row is read back at once.
        connect(row.box, &DragNumberBox::valueChanged, this, [this, i](double value) {
            if (m_viewport) {
                m_viewport->setJointTransformDial(i, value);
                refresh();
            }
        });

        // Per-row restore: just this dial back to 0, the rest pose.
        row.restore = propertyrows::makeRestoreButton();
        row.restore->setToolTip(tr("Back to the rest pose"));
        connect(row.restore, &QToolButton::clicked, this, [this, i]() {
            if (m_viewport && m_viewport->beginJointTransformEdit()) {
                m_viewport->setJointTransformDial(i, 0.0);
                m_viewport->endJointTransformEdit(); // registers no entry if the dial was already at 0
                refresh();
            }
        });

        rotation->addRow(row.label, propertyrows::rowWithRestore(row.box, row.restore));
    }

    m_hint = new QLabel(tr("Click a joint on a figure to select it. Its rotation can then be "
                           "set here."));
    m_hint->setObjectName(QStringLiteral("TransformHint"));
    m_hint->setWordWrap(true);
    col->addWidget(m_hint);

    // --- Reset ---
    m_reset = new QPushButton(tr("Reset All"));
    m_reset->setObjectName(QStringLiteral("PropertyResetButton"));
    m_reset->setToolTip(tr("Returns all three dials to 0: the selected joint's rest pose"));
    col->addWidget(m_reset, 0, Qt::AlignLeft);
    connect(m_reset, &QPushButton::clicked, this, &TransformPanel::resetRotation);

    col->addStretch(1);
    scroll->setWidget(content);
}

void TransformPanel::refresh() {
    const JointTransform state = m_viewport ? m_viewport->jointTransform() : JointTransform{};
    m_jointName->setText(state.valid ? QString::fromStdString(state.name) : tr("No joint selected"));
    m_hint->setVisible(!state.valid);
    m_reset->setEnabled(state.valid);
    for (std::size_t i = 0; i < m_rows.size(); ++i) {
        Row&                        row = m_rows[i];
        const JointTransform::Dial& dial = state.dials[i];
        const bool                  on = state.valid && dial.enabled;
        {
            // Signals blocked: mirroring the pose into the row must not pose the figure back.
            const QSignalBlocker blocker(row.box);
            if (on) {
                row.box->setRange(dial.minValue, dial.maxValue);
                row.box->setValue(dial.value);
            } else {
                row.box->setRange(-100.0, 100.0);
                row.box->setValue(0.0);
            }
        }
        // (After the value: disabling a box mid-gesture ends the gesture, which closes its edit.)
        row.box->setEnabled(on);
        row.label->setEnabled(on);
        row.restore->setEnabled(on);
    }
}

void TransformPanel::resetRotation() {
    if (!m_viewport || !m_viewport->beginJointTransformEdit()) {
        return;
    }
    for (int i = 0; i < 3; ++i) {
        m_viewport->setJointTransformDial(i, 0.0);
    }
    m_viewport->endJointTransformEdit();
    refresh();
}

} // namespace pose
