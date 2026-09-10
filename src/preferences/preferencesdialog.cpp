/**
 * @file preferencesdialog.cpp
 * @brief Implements the PreferencesDialog popup: left vertical tabs, right stacked panels.
 */

#include "preferencesdialog.h"
#include "preferencespanel.h"
#include "generalpreferencespanel.h"
#include "assetspreferencespanel.h"
#include "placeholderpreferencespanel.h"
#include "factoryresetpreferencespanel.h"

#include <QListWidget>
#include <QStackedWidget>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QFrame>
#include <QDialogButtonBox>

PreferencesDialog::PreferencesDialog(QWidget* parent)
    : QDialog(parent) {
    setObjectName(QStringLiteral("PreferencesDialog"));
    setWindowTitle(QStringLiteral("Preferences"));
    setMinimumSize(640, 460);

    auto* outer = new QVBoxLayout(this);
    outer->setContentsMargins(0, 0, 0, 0);
    outer->setSpacing(0);

    // --- Body: vertical tab nav (left) + stacked panels (right) ---
    auto* body = new QHBoxLayout();
    body->setContentsMargins(0, 0, 0, 0);
    body->setSpacing(0);

    m_nav = new QListWidget(this);
    m_nav->setObjectName(QStringLiteral("PreferencesNav"));
    m_nav->setFixedWidth(170);
    m_nav->setFrameShape(QFrame::NoFrame);
    m_nav->setFocusPolicy(Qt::NoFocus); // driven by selection, no focus rectangle needed
    body->addWidget(m_nav);

    m_stack = new QStackedWidget(this);
    m_stack->setObjectName(QStringLiteral("PreferencesStack"));
    body->addWidget(m_stack, 1);

    outer->addLayout(body, 1);

    // --- Footer: Close button, right-aligned ---
    auto* footer = new QHBoxLayout();
    footer->setContentsMargins(16, 12, 16, 12);
    auto* buttonBox = new QDialogButtonBox(QDialogButtonBox::Close, Qt::Horizontal, this);
    footer->addStretch(1);
    footer->addWidget(buttonBox);
    outer->addLayout(footer);

    connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
    connect(m_nav, &QListWidget::currentRowChanged, m_stack, &QStackedWidget::setCurrentIndex);

    // Register the tabs, in nav order. The PlaceholderPreferencesPanel entries hold a tab's
    // place until its real page exists — see preferencespanel.h for how to promote one.
    auto* assetsPanel = new AssetsPreferencesPanel(this);
    connect(assetsPanel, &AssetsPreferencesPanel::librariesChanged,
            this, &PreferencesDialog::assetLibrariesChanged);
    connect(assetsPanel, &AssetsPreferencesPanel::navigateToLibraryRequested, this, [this](const QString& path) {
        emit navigateToLibraryRequested(path);
        accept(); // close so the navigation in the main window behind us is actually visible
    });

    addPanel(QStringLiteral("General"),       new GeneralPreferencesPanel(this));
    addPanel(QStringLiteral("Interface"),
             new PlaceholderPreferencesPanel(
                 QStringLiteral("Interface"),
                 QStringLiteral("Interface and appearance settings will appear here."), this));
    addPanel(QStringLiteral("Assets"),        assetsPanel);
    addPanel(QStringLiteral("Input"),
             new PlaceholderPreferencesPanel(
                 QStringLiteral("Input"),
                 QStringLiteral("Keyboard and input settings will appear here."), this));
    addPanel(QStringLiteral("Navigation"),
             new PlaceholderPreferencesPanel(
                 QStringLiteral("Navigation"),
                 QStringLiteral("Viewport navigation settings will appear here."), this));
    addPanel(QStringLiteral("System"),
             new PlaceholderPreferencesPanel(
                 QStringLiteral("System"),
                 QStringLiteral("System, performance, and storage settings will appear here."), this));
    addPanel(QStringLiteral("Factory Reset"), new FactoryResetPreferencesPanel(this));

    m_nav->setCurrentRow(0);
}

void PreferencesDialog::selectTab(const QString& tabLabel) {
    const QList<QListWidgetItem*> matches = m_nav->findItems(tabLabel, Qt::MatchExactly);
    if (!matches.isEmpty()) m_nav->setCurrentItem(matches.first());
}

void PreferencesDialog::addPanel(const QString& tabLabel, PreferencesPanel* panel) {
    auto* item = new QListWidgetItem(tabLabel, m_nav);
    // A fixed row height in code rather than QSS ::item vertical padding — the same fix, for the
    // same reason, as the Assets page's library list: see AssetsPreferencesPanel::reloadLibraries.
    item->setSizeHint(QSize(0, 38));
    m_stack->addWidget(panel);
}
