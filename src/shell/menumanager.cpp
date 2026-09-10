/**
 * @file menumanager.cpp
 * @brief Builds the application's top menu bar (File / Edit / View / Help).
 *
 * Live today: File → Import (.OBJ meshes and .DUF character figures), Save/Load Pose and Quit;
 * Edit → Undo/Redo (the viewport's unified pose + lighting stack), Delete Selected Object, the
 * pose utilities (reset joint/limb/pose, mirror pose/limb) and Preferences; the whole View menu
 * (Show Skeleton, the axis views, Flip, Frame Selected and Home View — all with app-wide
 * shortcuts in Blender's numpad convention); Help → the website link and About. The remaining
 * entries (New/Open/Save, the other import formats, Export, clipboard, docs) are disabled
 * placeholders that establish the menu structure and icon conventions those features will slot
 * into — enabling one means replacing its disabled entry with a real handler here.
 */

#include "menumanager.h"
#include "splashoverlay.h"
#include "preferencesdialog.h"
#include "assetmanagerwidget.h"
#include "preferencesmanager.h"
#include "constants.h"
#include "viewport/viewportwidget.h"
#include <QMenu>
#include <QMenuBar>
#include <QAction>
#include <QApplication>
#include <QFileDialog>
#include <QFileInfo>
#include <QIcon>
#include <QMessageBox>
#include <QDesktopServices>
#include <QStandardPaths>
#include <QUrl>

/**
 * @brief Loads a normal/disabled icon pair following the "name.png" / "name-d.png" convention.
 */
static QIcon loadDualStateIcon(const QString& baseName) {
    QIcon icon;
    icon.addPixmap(QPixmap(QStringLiteral(":/resources/icons/%1.png").arg(baseName)), QIcon::Normal);
    icon.addPixmap(QPixmap(QStringLiteral(":/resources/icons/%1-d.png").arg(baseName)), QIcon::Disabled);
    return icon;
}

/**
 * @brief The folder a file dialog should start in: the one remembered under `prefKey` (the last
 * import / pose folder), falling back to Documents when nothing is saved yet or the stored folder
 * was moved/deleted. Shared by every File → Import and pose I/O entry point.
 */
static QString rememberedStartDir(const char* prefKey) {
    return PreferencesManager::instance().rememberedDirectory(
        QLatin1String(prefKey), QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation));
}

MenuManager::MenuManager(QMainWindow *parent) : QObject(parent), mainWindow(parent) {}

void MenuManager::setupMenus() {
    if (!mainWindow) return;

    // =========================================================================
    // FILE MENU — document lifecycle, import/export, and application exit
    // =========================================================================
    QMenu *fileMenu = mainWindow->menuBar()->addMenu("File");

    fileMenu->addAction(loadDualStateIcon("new"), "New...")->setEnabled(false);
    fileMenu->addAction(loadDualStateIcon("open"), "Open...")->setEnabled(false);
    fileMenu->addAction("Open Recent...")->setEnabled(false);
    fileMenu->addSeparator();

    fileMenu->addAction(loadDualStateIcon("save"), "Save")->setEnabled(false);
    fileMenu->addAction("Save As...")->setEnabled(false);
    fileMenu->addAction("Save Copy...")->setEnabled(false);
    fileMenu->addSeparator();

    // Import submenu: one entry per supported file format, kept alphabetical. An entry with a
    // handler is live; the rest are disabled placeholders until their importer lands — wiring a
    // new format is filling in its handler here. .DUF is the rigged character figure: its own
    // native scene format + pipeline (geometry + skeleton + morphs + materials), listed as a
    // file format alongside the mesh formats.
    QMenu *importMenu = fileMenu->addMenu(loadDualStateIcon("import"), "Import");
    struct ImportFormat {
        const char *label;
        void (MenuManager::*handler)(); // nullptr = importer not built yet
    };
    const ImportFormat importFormats[] = {
        {".ABC (Alembic)",                    nullptr},
        {".BVH (Biovision Hierarchy)",        nullptr},
        {".DAE (Collada)",                    nullptr},
        {".DUF (DUF File)",                   &MenuManager::importFigureFile},
        {".FBX (FBX File)",                   nullptr},
        {".GLB (GL Transmission Format .glTF)", nullptr},
        {".OBJ (Wavefront)",                  &MenuManager::importObjFile},
        {".PLY (Polygon File Format)",        nullptr},
        {".STL (Stereolithography)",          nullptr},
        {".USD (Universal Scene Description)", nullptr},
    };
    for (const ImportFormat &format : importFormats) {
        QAction *action = importMenu->addAction(format.label);
        if (format.handler) {
            QObject::connect(action, &QAction::triggered, mainWindow,
                             [this, handler = format.handler]() { (this->*handler)(); });
        } else {
            action->setEnabled(false); // importer not built yet
        }
    }

    fileMenu->addAction(loadDualStateIcon("export"), "Export...")->setEnabled(false);
    fileMenu->addSeparator();

    // Pose I/O: save the current figure's joint rotations to a small text file and restore them
    // later. Enabled regardless of scene state; the handlers warn if no figure is loaded.
    QAction *savePoseAction = fileMenu->addAction("Save Pose...");
    QObject::connect(savePoseAction, &QAction::triggered, mainWindow, [this]() { savePoseFile(); });
    QAction *loadPoseAction = fileMenu->addAction("Load Pose...");
    QObject::connect(loadPoseAction, &QAction::triggered, mainWindow, [this]() { loadPoseFile(); });
    fileMenu->addSeparator();

    QAction *quitAction = fileMenu->addAction("Quit");
    // Ctrl+Q covers Windows/Linux explicitly; QKeySequence::Quit adds the platform-standard
    // binding too (e.g. Cmd+Q on macOS), so both are listed rather than picking just one.
    quitAction->setShortcuts({QKeySequence("Ctrl+Q"), QKeySequence::Quit});
    QObject::connect(quitAction, &QAction::triggered, QApplication::instance(), &QApplication::quit);

    // =========================================================================
    // EDIT MENU — undo/redo history, clipboard, and preferences
    // =========================================================================
    QMenu *editMenu = mainWindow->menuBar()->addMenu("Edit");

    // Undo/Redo drive the viewport's shared edit stack (pose changes + Environment-panel lighting
    // gestures). The QAction shortcuts are what make Ctrl+Z/Ctrl+Y work app-wide — the panel's
    // controls take keyboard focus, so the viewport's own keyPressEvent alone wouldn't see the
    // keys after a dial edit. Text fields still keep their own Ctrl+Z: a focused QLineEdit accepts
    // the ShortcutOverride for its editing keys, which parks these window-level shortcuts.
    QAction *undoAction = editMenu->addAction(loadDualStateIcon("undo"), "Undo");
    undoAction->setShortcut(QKeySequence::Undo);
    QObject::connect(undoAction, &QAction::triggered, mainWindow, [this]() {
        if (viewportWidget) viewportWidget->undo();
    });
    QAction *redoAction = editMenu->addAction(loadDualStateIcon("redo"), "Redo");
    redoAction->setShortcut(QKeySequence::Redo);
    QObject::connect(redoAction, &QAction::triggered, mainWindow, [this]() {
        if (viewportWidget) viewportWidget->redo();
    });
    editMenu->addAction("Undo History...")->setEnabled(false);
    editMenu->addSeparator();

    editMenu->addAction(loadDualStateIcon("copy"), "Copy")->setEnabled(false);
    editMenu->addAction("Paste")->setEnabled(false);
    // Delete removes the SELECTED (outlined) viewport object. The "Del" shown here is a hint,
    // not a bound QKeySequence: the key itself is handled by the viewport while it has focus
    // (VulkanWindow::keyPressEvent). A window-level shortcut would hijack Delete from the Asset
    // Manager's tree and grid, where the key must stay theirs.
    QAction *deleteAction = editMenu->addAction("Delete Selected Object\tDel");
    QObject::connect(deleteAction, &QAction::triggered, mainWindow, [this]() {
        if (viewportWidget) viewportWidget->deleteSelectedObject();
    });
    editMenu->addSeparator();

    // Pose utilities, on the ACTIVE figure (the one whose joint was last clicked). "Limb" = the
    // selected joint and everything below it. Each is one undo step; pins are left alone. The
    // joint context menu in the viewport offers the per-joint ones too.
    QAction *resetJointAction = editMenu->addAction("Reset Selected Joint");
    QObject::connect(resetJointAction, &QAction::triggered, mainWindow, [this]() {
        if (viewportWidget) viewportWidget->resetSelectedJoint();
    });
    QAction *resetLimbAction = editMenu->addAction("Reset Limb");
    QObject::connect(resetLimbAction, &QAction::triggered, mainWindow, [this]() {
        if (viewportWidget) viewportWidget->resetSelectedLimb();
    });
    QAction *resetPoseAction = editMenu->addAction("Reset Pose");
    QObject::connect(resetPoseAction, &QAction::triggered, mainWindow, [this]() {
        if (viewportWidget) viewportWidget->resetPose();
    });
    editMenu->addSeparator();
    QAction *mirrorPoseAction = editMenu->addAction("Mirror Pose");
    QObject::connect(mirrorPoseAction, &QAction::triggered, mainWindow, [this]() {
        if (viewportWidget) viewportWidget->mirrorPose();
    });
    QAction *mirrorLimbAction = editMenu->addAction("Mirror Limb to Other Side");
    QObject::connect(mirrorLimbAction, &QAction::triggered, mainWindow, [this]() {
        if (viewportWidget) viewportWidget->mirrorSelectedLimb();
    });
    editMenu->addSeparator();

    QAction *preferencesAction = editMenu->addAction(loadDualStateIcon("preferences"), "Preferences");
    QObject::connect(preferencesAction, &QAction::triggered, mainWindow, [this]() {
        openPreferencesDialog();
    });

    // =========================================================================
    // VIEW MENU — viewport display options (skeleton overlay)
    // =========================================================================
    QMenu *viewMenu = mainWindow->menuBar()->addMenu("View");

    // "Show Skeleton": toggle the skeleton overlay (the joint→parent bone lines drawn over the
    // figure). Off by default — joints are grabbed directly on the figure — but joints stay
    // clickable either way. Kept in sync with the overlay's Skeleton button via the
    // skeletonVisibilityChanged signal. The viewport is registered after setupMenus() (see
    // setViewportWidget), so the action's connections are attached there.
    m_showSkeletonAction = viewMenu->addAction("Show Skeleton");
    m_showSkeletonAction->setCheckable(true);
    viewMenu->addSeparator();

    // Camera views in Blender's numpad convention, bound APP-WIDE (window shortcuts, so they
    // work whichever panel has focus — text fields still get their digits and periods: a
    // focused QLineEdit/spin box accepts the ShortcutOverride for text keys, which parks these).
    // Each has two bindings: the number-row key, which the shortcut map also matches for the
    // numpad digit with NumLock ON (it retries a keypad key without its keypad modifier), and
    // the navigation key the numpad sends with NumLock OFF (End/PageDown/Home/PageUp/Delete),
    // bound WITH the keypad modifier so the real End/Home/Delete keys stay untouched.
    struct AxisViewEntry {
        const char*    label;
        pose::AxisView view;
        Qt::Key        rowKey;   // number row / numpad with NumLock on
        Qt::Key        padKey;   // what the same numpad key sends with NumLock off
        bool           ctrl;
    };
    const AxisViewEntry axisViews[] = {
        {"Front View",  pose::AxisView::Front,  Qt::Key_1, Qt::Key_End,      false},
        {"Back View",   pose::AxisView::Back,   Qt::Key_1, Qt::Key_End,      true},
        {"Right View",  pose::AxisView::Right,  Qt::Key_3, Qt::Key_PageDown, false},
        {"Left View",   pose::AxisView::Left,   Qt::Key_3, Qt::Key_PageDown, true},
        {"Top View",    pose::AxisView::Top,    Qt::Key_7, Qt::Key_Home,     false},
        {"Bottom View", pose::AxisView::Bottom, Qt::Key_7, Qt::Key_Home,     true},
    };
    for (const AxisViewEntry& entry : axisViews) {
        QAction *action = viewMenu->addAction(entry.label);
        const Qt::KeyboardModifiers mods = entry.ctrl ? Qt::ControlModifier : Qt::NoModifier;
        action->setShortcuts({QKeySequence(mods | entry.rowKey),
                              QKeySequence(mods | Qt::KeypadModifier | entry.padKey)});
        const pose::AxisView view = entry.view;
        QObject::connect(action, &QAction::triggered, mainWindow, [this, view]() {
            if (viewportWidget) viewportWidget->setAxisView(view);
        });
    }
    viewMenu->addSeparator();
    QAction *flipViewAction = viewMenu->addAction("Flip View");
    flipViewAction->setShortcuts({QKeySequence(Qt::Key_9),
                                  QKeySequence(Qt::KeypadModifier | Qt::Key_PageUp)});
    QObject::connect(flipViewAction, &QAction::triggered, mainWindow, [this]() {
        if (viewportWidget) viewportWidget->flipView();
    });
    QAction *frameSelectedAction = viewMenu->addAction("Frame Selected");
    frameSelectedAction->setShortcuts({QKeySequence(Qt::Key_Period),
                                       QKeySequence(Qt::KeypadModifier | Qt::Key_Delete),
                                       QKeySequence(Qt::KeypadModifier | Qt::Key_Comma)});
    QObject::connect(frameSelectedAction, &QAction::triggered, mainWindow, [this]() {
        if (viewportWidget) viewportWidget->frameSelected();
    });
    // Home View = the viewport's Home button: the default perspective three-quarter framing.
    // On 5 — the centre of the numpad's view cluster (numpad 5 sends Clear with NumLock off).
    QAction *homeViewAction = viewMenu->addAction("Home View");
    homeViewAction->setShortcuts({QKeySequence(Qt::Key_5),
                                  QKeySequence(Qt::KeypadModifier | Qt::Key_Clear)});
    QObject::connect(homeViewAction, &QAction::triggered, mainWindow, [this]() {
        if (viewportWidget) viewportWidget->resetView();
    });

    // =========================================================================
    // HELP MENU — documentation, support links, and the About dialog
    QMenu *helpMenu = mainWindow->menuBar()->addMenu("Help");

    helpMenu->addAction("Release Notes")->setEnabled(false);
    helpMenu->addAction(loadDualStateIcon("tutorials"), "Tutorials")->setEnabled(false);
    helpMenu->addAction("Support")->setEnabled(false);
    helpMenu->addSeparator();

    QAction *websiteAction = helpMenu->addAction(QIcon(":/resources/icons/globe.png"), "PoseStudio.org");
    QObject::connect(websiteAction, &QAction::triggered, mainWindow, []() {
        QDesktopServices::openUrl(QUrl(QStringLiteral("https://posestudio.org")));
    });
    helpMenu->addSeparator();

    // "About" reuses the boot splash overlay for branding/version info
    QAction *aboutAction = helpMenu->addAction(loadDualStateIcon("about"), "About PoseStudio");
    QObject::connect(aboutAction, &QAction::triggered, mainWindow, [this]() {
        SplashOverlay *splash = new SplashOverlay(mainWindow);
        splash->show();
    });
}

void MenuManager::setAssetManagerWidget(AssetManagerWidget *widget) {
    assetManagerWidget = widget;
    if (!assetManagerWidget) return;

    QObject::connect(assetManagerWidget, &AssetManagerWidget::manageAssetFoldersRequested,
                      mainWindow, [this]() { openPreferencesDialog(QStringLiteral("Assets")); });
}

void MenuManager::setViewportWidget(pose::ViewportWidget *viewport) {
    viewportWidget = viewport;
    if (!viewport || !m_showSkeletonAction) {
        return;
    }
    // View → Show Skeleton drives the same overlay state as the viewport's Skeleton button; the
    // skeletonVisibilityChanged signal keeps the two in lockstep (blockSignals keeps the round-trip
    // to one hop).
    m_showSkeletonAction->setChecked(viewport->showSkeleton());
    QObject::connect(m_showSkeletonAction, &QAction::toggled, this, [this](bool on) {
        if (viewportWidget) viewportWidget->setShowSkeleton(on);
    });
    QObject::connect(viewport, &pose::ViewportWidget::skeletonVisibilityChanged, this, [this](bool visible) {
        if (!m_showSkeletonAction || m_showSkeletonAction->isChecked() == visible) {
            return;
        }
        m_showSkeletonAction->blockSignals(true);
        m_showSkeletonAction->setChecked(visible);
        m_showSkeletonAction->blockSignals(false);
    });
}

void MenuManager::importObjFile() {
    if (!viewportWidget) return;

    const QString path = QFileDialog::getOpenFileName(
        mainWindow, QStringLiteral("Import OBJ"), rememberedStartDir(Constants::PREF_LAST_IMPORT_DIR),
        QStringLiteral("Wavefront OBJ (*.obj)"));
    if (path.isEmpty()) return; // user cancelled

    // Remember the folder this model came from for the next import.
    PreferencesManager::instance().setValue(Constants::PREF_LAST_IMPORT_DIR,
                                            QFileInfo(path).absolutePath());
    viewportWidget->importObj(path);
}

void MenuManager::importFigureFile() {
    if (!viewportWidget) return;

    const QString path = QFileDialog::getOpenFileName(
        mainWindow, QStringLiteral("Import Character Figure"),
        rememberedStartDir(Constants::PREF_LAST_IMPORT_DIR),
        QStringLiteral("Figure Files (*.duf *.dsf)"));
    if (path.isEmpty()) return; // user cancelled

    PreferencesManager::instance().setValue(Constants::PREF_LAST_IMPORT_DIR,
                                            QFileInfo(path).absolutePath());
    viewportWidget->importFigure(path);
}

void MenuManager::savePoseFile() {
    if (!viewportWidget) return;
    if (!viewportWidget->hasPosableFigure()) {
        QMessageBox::information(mainWindow, QStringLiteral("Save Pose"),
                                QStringLiteral("Import a character figure before saving a pose."));
        return;
    }

    QString path = QFileDialog::getSaveFileName(mainWindow, QStringLiteral("Save Pose"),
                                                rememberedStartDir(Constants::PREF_LAST_POSE_DIR),
                                                QStringLiteral("Pose Files (*.pose)"));
    if (path.isEmpty()) return; // user cancelled
    if (!path.endsWith(QStringLiteral(".pose"), Qt::CaseInsensitive)) {
        path += QStringLiteral(".pose");
    }
    // Remember the pose folder for the next save/load, like Import remembers its folder.
    PreferencesManager::instance().setValue(Constants::PREF_LAST_POSE_DIR,
                                            QFileInfo(path).absolutePath());
    if (!viewportWidget->savePose(path)) {
        QMessageBox::warning(mainWindow, QStringLiteral("Save Pose"),
                             QStringLiteral("Could not write the pose file."));
    }
}

void MenuManager::loadPoseFile() {
    if (!viewportWidget) return;
    if (!viewportWidget->hasPosableFigure()) {
        QMessageBox::information(mainWindow, QStringLiteral("Load Pose"),
                                QStringLiteral("Import a character figure before loading a pose."));
        return;
    }

    const QString path = QFileDialog::getOpenFileName(mainWindow, QStringLiteral("Load Pose"),
                                                      rememberedStartDir(Constants::PREF_LAST_POSE_DIR),
                                                      QStringLiteral("Pose Files (*.pose)"));
    if (path.isEmpty()) return; // user cancelled
    PreferencesManager::instance().setValue(Constants::PREF_LAST_POSE_DIR,
                                            QFileInfo(path).absolutePath());
    if (!viewportWidget->loadPose(path)) {
        QMessageBox::warning(mainWindow, QStringLiteral("Load Pose"),
                             QStringLiteral("Could not read the pose file."));
    }
}

void MenuManager::openPreferencesDialog(const QString &initialTab) {
    PreferencesDialog dialog(mainWindow);
    if (!initialTab.isEmpty()) dialog.selectTab(initialTab);

    if (assetManagerWidget) {
        QObject::connect(&dialog, &PreferencesDialog::assetLibrariesChanged,
                          assetManagerWidget, &AssetManagerWidget::refreshAssetManager);
        QObject::connect(&dialog, &PreferencesDialog::navigateToLibraryRequested,
                          assetManagerWidget, &AssetManagerWidget::navigateToLibraryRoot);
    }
    dialog.exec();
}
