/**
 * @file main.cpp
 * @brief Application entry point: boots core services, then builds the main window.
 *
 * Pure wiring — the theming lives in shell/apptheme.*, the database bootstrap in core/database.*.
 * Startup order matters here — the database and preferences cache must be ready before any UI
 * that might read from them gets constructed, and the theme must be installed before the first
 * widget polishes.
 */

#include "database.h"
#include "menumanager.h"
#include "splashoverlay.h"
#include "constants.h"
#include "preferencesmanager.h"
#include "installping.h"
#include "assetmanagerwidget.h"
#include "apptheme.h"
#include "environmentpanel.h"
#include "viewport/viewportwidget.h"

#include <QApplication>
#include <QDebug>
#include <QMainWindow>
#include <QScreen>
#include <QSplitter>
#include <QTabWidget>

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);

    // App identity. Must be set before any QStandardPaths lookup: the per-user data dir the
    // database now lives in (AppDataLocation) is derived from these names, so initializeDatabase
    // below depends on it. Keep this first.
    QApplication::setApplicationName(QStringLiteral("PoseStudio"));
    QApplication::setApplicationDisplayName(QStringLiteral("PoseStudio"));

    // --- 1. Core services & data layer ---
    // Fail here, with a clear error, rather than confusingly the first time some unrelated widget
    // tries to run a query later.
    if (!initializeDatabase(DbInitMode::Normal)) {
        qCritical() << "Fatal Error: PoseStudio cannot launch without a valid database connection.";
        return -1;
    }

    PreferencesManager::instance().loadFromDatabase();

    // --- 2. Branding & theming (window icon, Fusion proxy style, the QSS modules, menu shadows) ---
    AppTheme::install(app);

    // --- 3. Main window layout ---
    QMainWindow mainWindow;
#ifdef NDEBUG
    mainWindow.setWindowTitle(QStringLiteral("%1 %2").arg(Constants::APP_NAME, Constants::APP_VERSION));
#else
    // Mark unoptimized builds in the title: a Debug build is ~5x slower at figure import (MSVC
    // debug-STL overhead), which has been mistaken for a real performance regression when a Debug
    // window was benchmarked against other applications' shipped binaries.
    mainWindow.setWindowTitle(QStringLiteral("%1 %2 [Debug build — slow imports]")
                                  .arg(Constants::APP_NAME, Constants::APP_VERSION));
#endif

    // Size the window relative to the screen rather than using a fixed pixel size,
    // so it's usable on both small laptop displays and large monitors. The height-derived
    // width must still be clamped to the screen: on portrait or 5:4 displays height × 1.4
    // exceeds the available width and the window would open wider than the desktop.
    if (QScreen *screen = app.primaryScreen()) {
        const QRect screenGeometry = screen->availableGeometry();
        const int width = qMin(static_cast<int>(screenGeometry.height() * 1.4),
                               static_cast<int>(screenGeometry.width() * 0.95));
        mainWindow.resize(width, static_cast<int>(screenGeometry.height() * 0.80));
    }

    MenuManager *menuManager = new MenuManager(&mainWindow);
    menuManager->setupMenus();

    QSplitter *mainSplitter = new QSplitter(Qt::Horizontal, &mainWindow);

    // The 3D viewport: a self-contained Vulkan-backed widget. Everything graphics-API
    // related lives behind this facade (see src/viewport/). If Vulkan is unavailable it
    // degrades to an inline message rather than failing to launch.
    pose::ViewportWidget *viewport = new pose::ViewportWidget(mainSplitter);

    QTabWidget *sidePanel = new QTabWidget(mainSplitter);
    sidePanel->setTabPosition(QTabWidget::West);

    AssetManagerWidget *assetsTab = new AssetManagerWidget();
    sidePanel->addTab(assetsTab, QStringLiteral("Asset Manager"));
    // The Environment tab: live image-based-lighting controls for the viewport (see EnvironmentPanel).
    sidePanel->addTab(new pose::EnvironmentPanel(viewport), QStringLiteral("Environment"));

    menuManager->setAssetManagerWidget(assetsTab);
    menuManager->setViewportWidget(viewport);

    // Double-clicking an asset in the grid imports it into the viewport — the same entry points the
    // menu and command-line "open with" use: an .obj as a static model, a character figure (.duf/.dsf)
    // through the figure pipeline.
    QObject::connect(assetsTab, &AssetManagerWidget::importModelRequested,
                     viewport, &pose::ViewportWidget::importObj);
    QObject::connect(assetsTab, &AssetManagerWidget::importFigureRequested,
                     viewport, &pose::ViewportWidget::importFigure);

    mainSplitter->addWidget(viewport);
    mainSplitter->addWidget(sidePanel);
    mainSplitter->setSizes({1500, 500});

    mainWindow.setCentralWidget(mainSplitter);

    // --- 4. Execution ---
    mainWindow.show();

    // Boot branding screen; dismisses itself on the user's next click anywhere
    SplashOverlay *splash = new SplashOverlay(&mainWindow);
    splash->show();

    // "Open with" / drag-onto-exe convenience: import any model/figure paths passed on the command
    // line. Two paths, depending on the platform: the viewport builds its renderer on the first
    // expose, and on Windows that expose is delivered synchronously inside mainWindow.show()
    // above, so here the renderer already exists and these imports run the ordinary interactive
    // path (progress dialog and all). Where the expose arrives later (other platforms), the
    // importers queue the paths and drain them, in this order, once the renderer is up — so the
    // calls are safe either way.
    const QStringList launchArgs = QCoreApplication::arguments();
    for (int i = 1; i < launchArgs.size(); ++i) {
        const QString& arg = launchArgs.at(i);
        if (arg.endsWith(QStringLiteral(".obj"), Qt::CaseInsensitive)) {
            viewport->importObj(arg);
        } else if (arg.endsWith(QStringLiteral(".duf"), Qt::CaseInsensitive) ||
                   arg.endsWith(QStringLiteral(".dsf"), Qt::CaseInsensitive)) {
            viewport->importFigure(arg);
        } else if (arg.endsWith(QStringLiteral(".pose"), Qt::CaseInsensitive)) {
            // A pose file applies onto whichever figure was imported (queued until the figure exists).
            viewport->loadPose(arg);
        }
    }

    // The anonymous per-launch install ping (see installping.h): armed now, it fires a few
    // seconds into the session so it never competes with startup, and is a no-op in builds
    // without a signing key or when the user has switched it off. Owned by the main window, NOT
    // the application object — see scheduleAtStartup for why that distinction matters at exit.
    InstallPing::scheduleAtStartup(&mainWindow);

    return app.exec();
}
