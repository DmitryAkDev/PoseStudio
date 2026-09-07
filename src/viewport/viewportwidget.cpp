/**
 * @file viewportwidget.cpp
 * @brief Implementation of the viewport widget facade. See viewportwidget.h.
 */

#include "viewportwidget.h"

#include "scene/shademode.h"
#include "vulkanwindow.h"

#include <QAction>
#include <QActionGroup>
#include <QCoreApplication>
#include <QDebug>
#include <QEvent>
#include <QHBoxLayout>
#include <QHideEvent>
#include <QIcon>
#include <QLabel>
#include <QMenu>
#include <QMoveEvent>
#include <QPainter>
#include <QPainterPath>
#include <QPoint>
#include <QPushButton>
#include <QResizeEvent>
#include <QShowEvent>
#include <QSize>
#include <QVBoxLayout>
#include <QVersionNumber>
#include <QVulkanInstance>

#include <vulkan/vulkan.h>

namespace pose {

namespace {
// Single source of truth for the Vulkan version the app targets. 1.1 is supported by
// effectively every current driver and is enough for VMA's dedicated-allocation path.
// Bump here (and nowhere else) when the renderer starts relying on newer core features.
constexpr uint32_t kVulkanApiVersion = VK_API_VERSION_1_1;
} // namespace

/// The strip's "rotating about an axis" badge, shown under the buttons for as long as X, Y, or Z
/// is held with a joint selected — the wheel's meaning is modal, so the mode must be visible.
/// A rotation-arrow glyph and the axis letter in the axis colour (X red, Y green, Z the accent
/// blue — the DCC convention), on the same surface as the strip's buttons. Painted, not an icon
/// file: the colour is the message and there are three of them.
class AxisRotateBadge : public QWidget {
public:
    explicit AxisRotateBadge(QWidget* parent, int height) : QWidget(parent), m_height(height) {
        setFixedSize(66, height);
        setToolTip(tr("Roll the mouse wheel to rotate the selected joint about this axis"));
    }
    void setAxis(int axis) {
        m_axis = axis;
        update();
    }

protected:
    void paintEvent(QPaintEvent*) override {
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
        const qreal d = r.height() * 0.52;
        const QRectF arc(r.left() + 9.0, r.center().y() - d * 0.5, d, d);
        QPen pen(c, 2.0);
        pen.setCapStyle(Qt::RoundCap);
        p.setPen(pen);
        p.setBrush(Qt::NoBrush);
        p.drawArc(arc, 45 * 16, 270 * 16);
        const qreal endDeg = 45.0 + 270.0; // Qt angles: 0° = 3 o'clock, counter-clockwise
        const qreal endRad = endDeg * 3.14159265 / 180.0;
        const QPointF tip(arc.center().x() + std::cos(endRad) * d * 0.5,
                          arc.center().y() - std::sin(endRad) * d * 0.5);
        const QPointF tangent(-std::sin(endRad), -std::cos(endRad)); // direction of travel (screen)
        const QPointF normal(-tangent.y(), tangent.x());
        const QPointF base = tip - tangent * 5.0;
        QPainterPath head;
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

private:
    int m_axis = -1;
    int m_height = 28;
};

ViewportWidget::ViewportWidget(QWidget* parent) : QWidget(parent) {
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);

    m_instance = std::make_unique<QVulkanInstance>();
    m_instance->setApiVersion(QVersionNumber(1, 1));
#ifndef NDEBUG
    // Standard validation layer in debug builds; QVulkanInstance routes its messages to
    // qDebug() automatically. Released builds skip it for performance.
    m_instance->setLayers({QByteArrayLiteral("VK_LAYER_KHRONOS_validation")});
#endif

    if (!m_instance->create()) {
        auto* message = new QLabel(
            tr("3D viewport unavailable.\n\n"
               "Could not create a Vulkan instance — check that your GPU supports Vulkan "
               "and that your graphics drivers are up to date."),
            this);
        message->setAlignment(Qt::AlignCenter);
        message->setWordWrap(true);
        layout->addWidget(message);
        qCritical() << "[Vulkan] QVulkanInstance::create() failed, error code"
                    << m_instance->errorCode();
        m_instance.reset();
        return;
    }

    // Compiled shaders are mirrored next to the executable by the build (see CMakeLists).
    const QString shaderDir = QCoreApplication::applicationDirPath() + QStringLiteral("/shaders");

    m_window = new VulkanWindow(m_instance.get(), kVulkanApiVersion, shaderDir);
    // Signal-to-signal forward: undo/redo restoring a lighting state surfaces on the facade,
    // where the Environment panel listens.
    connect(m_window, &VulkanWindow::lightingRestored, this, &ViewportWidget::lightingRestored);
    m_container = QWidget::createWindowContainer(m_window, this);
    m_container->setFocusPolicy(Qt::StrongFocus); // so the viewport can receive wheel/keys
    layout->addWidget(m_container);

    createShaderOverlay(); // the floating shader-mode dropdown in the top-right corner
}

void ViewportWidget::importObj(const QString& path) {
    if (m_window) {
        m_window->importObj(path);
    }
}

void ViewportWidget::importFigure(const QString& path) {
    if (m_window) {
        m_window->importFigure(path);
    }
}

bool ViewportWidget::hasPosableFigure() const {
    return m_window && m_window->hasPosableFigure();
}

bool ViewportWidget::savePose(const QString& path) {
    return m_window && m_window->savePose(path);
}

bool ViewportWidget::loadPose(const QString& path) {
    return m_window && m_window->loadPose(path);
}

void ViewportWidget::setShadeMode(int mode) {
    if (m_window) {
        m_window->setShadeMode(mode);
    }
}

void ViewportWidget::deleteSelectedObject() {
    if (m_window) {
        m_window->deleteSelectedObject();
    }
}

void ViewportWidget::setShowSkeleton(bool on) {
    if (!m_window || m_window->showSkeleton() == on) {
        return;
    }
    m_window->setShowSkeleton(on);
    emit skeletonVisibilityChanged(on);
}

bool ViewportWidget::showSkeleton() const {
    return m_window && m_window->showSkeleton();
}


void ViewportWidget::resetView() {
    if (m_window) {
        m_window->resetView();
    }
}

void ViewportWidget::setAxisView(AxisView view) {
    if (m_window) {
        m_window->setAxisView(view);
    }
}

void ViewportWidget::flipView() {
    if (m_window) {
        m_window->flipView();
    }
}

void ViewportWidget::frameSelected() {
    if (m_window) {
        m_window->frameSelected();
    }
}

void ViewportWidget::groundFigure() {
    if (m_window) {
        m_window->groundFigure();
    }
}

void ViewportWidget::setEnvironment(const QString& hdrPath) {
    if (m_window) {
        m_window->setEnvironmentFile(hdrPath);
    }
}

void ViewportWidget::setLightingSettings(const LightingSettings& settings) {
    if (m_window) {
        m_window->setLightingSettings(settings);
    }
}

void ViewportWidget::undo() {
    if (m_window) {
        m_window->undo();
    }
}

void ViewportWidget::redo() {
    if (m_window) {
        m_window->redo();
    }
}

void ViewportWidget::registerLightingUndo(const LightingSettings& preEdit) {
    if (m_window) {
        m_window->registerLightingUndo(preEdit);
    }
}

QStringList ViewportWidget::shaderModeNames() {
    // Picker order = the table's order (scene/shademode.h); the index handed to setShadeMode is
    // a row of that table, and the row maps itself onto mesh.frag's mode + the draw variants.
    QStringList names;
    for (const ShadeMode& mode : kShadeModes) {
        names << QString::fromUtf8(mode.name);
    }
    return names;
}

void ViewportWidget::createShaderOverlay() {
    // A *top-level* frameless window owned by this widget — a child widget would be composited behind
    // the native viewport (see the SplashOverlay note). WA_ShowWithoutActivating so revealing it
    // doesn't steal focus from the app; the combo still takes clicks normally.
    //
    // Qt::Tool, not Qt::Window: this is an auxiliary control strip belonging to the main window, and
    // the tool type is what tells the OS/WM to keep it OUT of the window list and stacked with its
    // owner. A plain Qt::Window is a full application window: on X11 it shows up as a SECOND entry in
    // the taskbar/window switcher, and nothing guarantees it stays above the main window — the WM
    // stacks it wherever it pleases, so the strip sat BEHIND the main window until the user clicked
    // it to activate it. (On Windows the owned-window z-order happens to hold for Qt::Window too, which
    // is why this only showed up on Linux.)
    m_overlay = new QWidget(this, Qt::Tool | Qt::FramelessWindowHint);
    m_overlay->setObjectName(QStringLiteral("ViewportShaderOverlay"));
    m_overlay->setAttribute(Qt::WA_TranslucentBackground, true);
    m_overlay->setAttribute(Qt::WA_ShowWithoutActivating, true);

    // Two rows: the control strip, and under it (right-aligned, usually empty) the transient
    // axis-rotate badge. Hidden widgets take no space, so the strip is one row until a key is held.
    auto* column = new QVBoxLayout(m_overlay);
    column->setContentsMargins(0, 0, 0, 0);
    column->setSpacing(6);
    auto* lay = new QHBoxLayout();
    lay->setContentsMargins(0, 0, 0, 0);
    lay->setSpacing(6);
    column->addLayout(lay);

    // The selector is a push-button that opens a *real QMenu*, rather than a QComboBox. This is the one
    // way to get the picker's dropdown to look and behave EXACTLY like the File/Edit/Help menus — colours,
    // hover, spacing, and top-down drop — because it *is* a QMenu and so inherits the global QMenu QSS
    // (see _menumanager.qss). A styled QComboBox popup renders its own item states inconsistently.
    // The pickers' field style (the closed selector): the menu-bar surface + hover; the QMenu each
    // opens is themed globally. text-align:left so the label sits left of the drop arrow, like a
    // combo field. No cursor override on the overlay controls: Windows apps keep the standard
    // arrow over buttons (the hand cursor reads as a hyperlink there).
    const auto pickerStyle = [](const QString& objectName, int minWidthPx) {
        return QStringLiteral(
                   "#%1 {"
                   "  background-color: #252627;"
                   "  color: #e8e8ea;"
                   "  border: 1px solid #555555;"
                   "  border-radius: 4px;"
                   "  padding: 5px 10px;"
                   "  min-width: %2px;"
                   "  font-size: 12px;"
                   "  text-align: left;"
                   "}"
                   "#%1:hover { background-color: #314D7A; color: #ffffff; }"
                   "#%1::menu-indicator { subcontrol-position: right center;"
                   "  subcontrol-origin: padding; right: 8px; }")
            .arg(objectName)
            .arg(minWidthPx);
    };

    m_shaderButton = new QPushButton(m_overlay);
    m_shaderButton->setObjectName(QStringLiteral("ShaderModeButton"));
    m_shaderButton->setToolTip(tr("Viewport shading mode"));
    m_shaderButton->setStyleSheet(pickerStyle(QStringLiteral("ShaderModeButton"), 168));

    auto* menu = new QMenu(m_shaderButton);
    auto* group = new QActionGroup(menu);
    group->setExclusive(true);
    const QStringList names = shaderModeNames();
    // QPushButton with text-align:left ignores padding-left, so the field label gets a small leading
    // indent from a couple of spaces to keep the mode name off the left border (the menu items, which
    // honour padding normally, stay unindented).
    const auto fieldLabel = [](const QString& name) { return QStringLiteral("  ") + name; };
    // The default (PBR) and the group separators come from the table too (kDefaultShadeMode,
    // ShadeMode::separatorAfter), so the picker is a pure rendering of scene/shademode.h.
    for (int i = 0; i < names.size(); ++i) {
        QAction* action = menu->addAction(names.at(i));
        action->setCheckable(true);
        action->setChecked(i == kDefaultShadeMode);
        group->addAction(action);
        connect(action, &QAction::triggered, this, [this, i, name = names.at(i), fieldLabel] {
            setShadeMode(i);
            m_shaderButton->setText(fieldLabel(name));
        });
        if (kShadeModes[i].separatorAfter) {
            menu->addSeparator();
        }
    }
    m_shaderButton->setMenu(menu); // QPushButton drops the menu straight down from the button
    m_shaderButton->setText(fieldLabel(names.at(kDefaultShadeMode)));
    lay->addWidget(m_shaderButton);

    // The VIEW picker: the named camera views (the same ones the 1/3/7/5 keys reach), grouped by
    // axis, plus Home. Its label READS the current view — "Perspective" once the user orbits
    // away from a named one — following the window's viewPresetChanged, so the keys, the View
    // menu, and this picker all agree on where the camera is.
    m_viewButton = new QPushButton(m_overlay);
    m_viewButton->setObjectName(QStringLiteral("ViewPresetButton"));
    m_viewButton->setToolTip(tr("Camera view"));
    m_viewButton->setStyleSheet(pickerStyle(QStringLiteral("ViewPresetButton"), 132));
    auto* viewMenu = new QMenu(m_viewButton);
    // The global QMenu::item padding is tight on the right (built for the long menu-bar
    // entries); these short labels need breathing room so the dropdown doesn't read as a
    // sliver, and the menu is at least as wide as its field, like a combo's popup.
    viewMenu->setStyleSheet(QStringLiteral("QMenu::item { padding: 6px 28px 6px 6px; }"));
    struct ViewEntry {
        const char* label;
        ViewPreset  preset;
        bool        separatorAfter;
    };
    const ViewEntry viewEntries[] = {
        {"Home View",   ViewPreset::Home,   true},
        {"Top View",    ViewPreset::Top,    false},
        {"Bottom View", ViewPreset::Bottom, true},
        {"Front View",  ViewPreset::Front,  false},
        {"Back View",   ViewPreset::Back,   true},
        {"Left View",   ViewPreset::Left,   false},
        {"Right View",  ViewPreset::Right,  false},
    };
    for (const ViewEntry& entry : viewEntries) {
        QAction* action = viewMenu->addAction(QString::fromUtf8(entry.label));
        const ViewPreset preset = entry.preset;
        connect(action, &QAction::triggered, this, [this, preset] {
            switch (preset) {
            case ViewPreset::Top:    setAxisView(AxisView::Top);    break;
            case ViewPreset::Bottom: setAxisView(AxisView::Bottom); break;
            case ViewPreset::Front:  setAxisView(AxisView::Front);  break;
            case ViewPreset::Back:   setAxisView(AxisView::Back);   break;
            case ViewPreset::Left:   setAxisView(AxisView::Left);   break;
            case ViewPreset::Right:  setAxisView(AxisView::Right);  break;
            case ViewPreset::Home:
            case ViewPreset::Free:   resetView();                   break;
            }
        });
        if (entry.separatorAfter) {
            viewMenu->addSeparator();
        }
    }
    m_viewButton->setMenu(viewMenu);
    const auto viewLabel = [fieldLabel](ViewPreset preset) {
        switch (preset) {
        case ViewPreset::Top:    return fieldLabel(tr("Top View"));
        case ViewPreset::Bottom: return fieldLabel(tr("Bottom View"));
        case ViewPreset::Front:  return fieldLabel(tr("Front View"));
        case ViewPreset::Back:   return fieldLabel(tr("Back View"));
        case ViewPreset::Left:   return fieldLabel(tr("Left View"));
        case ViewPreset::Right:  return fieldLabel(tr("Right View"));
        case ViewPreset::Home:   return fieldLabel(tr("Home View"));
        case ViewPreset::Free:   break;
        }
        return fieldLabel(tr("Perspective View"));
    };
    m_viewButton->setText(viewLabel(m_window ? m_window->currentView() : ViewPreset::Home));
    viewMenu->setMinimumWidth(m_viewButton->sizeHint().width());
    if (m_window) {
        connect(m_window, &VulkanWindow::viewPresetChanged, this,
                [this, viewLabel](ViewPreset preset) { m_viewButton->setText(viewLabel(preset)); });
    }
    lay->addWidget(m_viewButton);

    // "Home": snap the camera back to the default perspective framing. Same surface/hover as the
    // shader field so the two read as one control strip; squared off to the field's own height.
    m_homeButton = new QPushButton(m_overlay);
    m_homeButton->setObjectName(QStringLiteral("ViewportHomeButton"));
    m_homeButton->setToolTip(tr("Reset to the default view"));
    m_homeButton->setIcon(QIcon(QStringLiteral(":/resources/icons/home.png")));
    m_homeButton->setIconSize(QSize(16, 16));
    m_homeButton->setStyleSheet(QStringLiteral(
        "#ViewportHomeButton {"
        "  background-color: #252627;"
        "  border: 1px solid #555555;"
        "  border-radius: 4px;"
        "}"
        "#ViewportHomeButton:hover { background-color: #314D7A; }"));
    // Match the field's height exactly (rather than guessing a size), so the strip aligns whatever
    // the font metrics work out to.
    const int fieldHeight = m_shaderButton->sizeHint().height();
    m_homeButton->setFixedSize(fieldHeight, fieldHeight);
    connect(m_homeButton, &QPushButton::clicked, this, &ViewportWidget::resetView);
    lay->addWidget(m_homeButton);

    // "Ground": drop the figure onto the floor plane — its CURRENT pose's lowest point lands on
    // y = 0 (a knee bend lifts the feet off the floor; this puts them back). Same treatment as
    // the Home button beside it.
    m_groundButton = new QPushButton(m_overlay);
    m_groundButton->setObjectName(QStringLiteral("ViewportGroundButton"));
    m_groundButton->setToolTip(tr("Move the selected figure to the ground"));
    m_groundButton->setIcon(QIcon(QStringLiteral(":/resources/icons/ground.png")));
    m_groundButton->setIconSize(QSize(16, 16));
    m_groundButton->setStyleSheet(QStringLiteral(
        "#ViewportGroundButton {"
        "  background-color: #252627;"
        "  border: 1px solid #555555;"
        "  border-radius: 4px;"
        "}"
        "#ViewportGroundButton:hover { background-color: #314D7A; }"));
    m_groundButton->setFixedSize(fieldHeight, fieldHeight);
    connect(m_groundButton, &QPushButton::clicked, this, &ViewportWidget::groundFigure);
    lay->addWidget(m_groundButton);

    // "Skeleton": toggle the skeleton overlay (the joint→parent bone lines drawn over the figure).
    // A persistent on/off view control (unlike the one-shot Home/Ground), so it's checkable and
    // shows its state. Off by default — joints are grabbed directly on the figure — but joints stay
    // clickable either way. Also reachable from the View menu; the two stay in sync via the
    // skeletonVisibilityChanged signal.
    m_skeletonButton = new QPushButton(m_overlay);
    m_skeletonButton->setObjectName(QStringLiteral("ViewportSkeletonButton"));
    m_skeletonButton->setToolTip(tr("Toggle the skeleton overlay"));
    m_skeletonButton->setIcon(QIcon(QStringLiteral(":/resources/icons/skeleton.png")));
    m_skeletonButton->setIconSize(QSize(16, 16));
    m_skeletonButton->setCheckable(true);
    m_skeletonButton->setStyleSheet(QStringLiteral(
        "#ViewportSkeletonButton {"
        "  background-color: #252627;"
        "  border: 1px solid #555555;"
        "  border-radius: 4px;"
        "}"
        "#ViewportSkeletonButton:hover { background-color: #3a3d40; }"
        "#ViewportSkeletonButton:checked { background-color: #314D7A; }"""));
    m_skeletonButton->setFixedSize(fieldHeight, fieldHeight);
    connect(m_skeletonButton, &QPushButton::toggled, this, [this](bool on) {
        // The View menu drives this button (and vice versa) via the signal; when we're the source
        // of the change, block the re-entrant setChecked so the signal loop stays one round-trip.
        if (on == (m_window && m_window->showSkeleton())) {
            return;
        }
        setShowSkeleton(on);
    });
    lay->addWidget(m_skeletonButton);

    // The axis-rotate badge, under the strip's right end (below the Skeleton button): visible
    // only while X/Y/Z is held with a joint selected (VulkanWindow::axisRotateKeyChanged).
    m_axisBadge = new AxisRotateBadge(m_overlay, fieldHeight);
    m_axisBadge->hide();
    auto* badgeRow = new QHBoxLayout();
    badgeRow->setContentsMargins(0, 0, 0, 0);
    badgeRow->addStretch(1);
    badgeRow->addWidget(m_axisBadge);
    column->addLayout(badgeRow);
    if (m_window) {
        connect(m_window, &VulkanWindow::axisRotateKeyChanged, this, [this](int axis) {
            m_axisBadge->setAxis(axis);
            m_axisBadge->setVisible(axis >= 0);
            syncOverlayPosition(); // the strip grew or shrank by a row
        });
    }
    // Keep the button's checked state in sync when the View menu (or anything else) toggles the
    // overlay: the signal is the single source of truth, and blockSignals keeps the round-trip to
    // one hop (no re-entrant toggled → setShowSkeleton → signal loop).
    connect(this, &ViewportWidget::skeletonVisibilityChanged, this, [this](bool visible) {
        if (m_skeletonButton->isChecked() == visible) {
            return;
        }
        m_skeletonButton->blockSignals(true);
        m_skeletonButton->setChecked(visible);
        m_skeletonButton->blockSignals(false);
    });

    m_overlay->adjustSize();
}

void ViewportWidget::syncOverlayPosition() {
    if (!m_overlay || !m_container || !m_container->isVisible()) {
        return;
    }
    m_overlay->adjustSize();
    constexpr int margin = 12;
    const QPoint topRight = m_container->mapToGlobal(QPoint(m_container->width(), 0));
    m_overlay->move(topRight.x() - m_overlay->width() - margin, topRight.y() + margin);
}

void ViewportWidget::showEvent(QShowEvent* event) {
    QWidget::showEvent(event);
    if (!m_overlay) {
        return;
    }
    // Follow the top-level window's moves/resizes (a child moveEvent doesn't fire when the whole app
    // window is dragged). Re-target the filter if we've been reparented into a different window.
    if (QWidget* w = window(); w && w != m_filteredWindow) {
        if (m_filteredWindow) {
            m_filteredWindow->removeEventFilter(this);
        }
        w->installEventFilter(this);
        m_filteredWindow = w;
    }
    m_overlay->show();
    m_overlay->raise();
    syncOverlayPosition();
}

void ViewportWidget::hideEvent(QHideEvent* event) {
    QWidget::hideEvent(event);
    if (m_overlay) {
        m_overlay->hide();
    }
}

void ViewportWidget::resizeEvent(QResizeEvent* event) {
    QWidget::resizeEvent(event);
    syncOverlayPosition();
}

void ViewportWidget::moveEvent(QMoveEvent* event) {
    QWidget::moveEvent(event);
    syncOverlayPosition();
}

bool ViewportWidget::eventFilter(QObject* watched, QEvent* event) {
    if (watched == m_filteredWindow) {
        const QEvent::Type t = event->type();
        if (t == QEvent::Move || t == QEvent::Resize || t == QEvent::WindowStateChange) {
            syncOverlayPosition();
        } else if (t == QEvent::WindowActivate) {
            // The main window was activated (focus, un-minimize, user click): re-raise the overlay
            // strip so it can't sit BEHIND the main window. On X11 a top-level strip's stacking
            // relative to its owner is the WM's call, so we re-assert it whenever the owner is
            // raised. WindowActivate, not ActivationChange (which is also sent on deactivation):
            // the strip must not be raised over a modal dialog or the splash while they hold the
            // focus.
            if (m_overlay && m_overlay->isVisible()) {
                m_overlay->raise();
            }
        }
    }
    return QWidget::eventFilter(watched, event);
}

ViewportWidget::~ViewportWidget() {
    // The floating overlay is an owned top-level window with no Vulkan resources; drop it first so it
    // can't briefly outlive the viewport it tracks. (Qt would also delete it as a child, but explicit
    // is clearer next to the ordered Vulkan teardown below.)
    delete m_overlay;
    m_overlay = nullptr;
    m_shaderButton = nullptr;
    m_homeButton = nullptr;
    m_groundButton = nullptr;

    // The QVulkanInstance (m_instance) owns the VkInstance, and the VulkanWindow's
    // device/renderer were created from it. Qt would otherwise destroy the window via the
    // base QWidget destructor — i.e. AFTER m_instance is gone — leaving the renderer to
    // call vkDestroyDevice on a dead instance. Destroy the container (hence the window,
    // hence all Vulkan objects) here, explicitly, while m_instance is still alive.
    delete m_container;
    m_container = nullptr;
    m_window = nullptr; // was owned by m_container; now dangling — clear it
}

} // namespace pose
