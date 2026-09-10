/**
 * @file apptheme.h
 * @brief The one place the application's look is installed: icon, Fusion proxy style, the
 *        concatenated QSS modules, tooltip effects, and the app-wide menu-shadow filter.
 *
 * main.cpp used to carry all of this inline (a MenuShadowFilter class, a loadStylesheets()
 * helper and a "Branding & theming" block). Keeping it here leaves main.cpp as pure wiring and
 * gives the theming decisions a single home; nothing in a widget constructor should set the
 * style, the stylesheet, or the tooltip effect flags — call AppTheme::install() once, and only
 * from main().
 */

#ifndef APPTHEME_H
#define APPTHEME_H

class QApplication;

namespace AppTheme {

/**
 * @brief Applies PoseStudio's theme to @p app. Call once, after the database and preferences
 *        are up (nothing here reads them today, but widgets built afterwards do) and BEFORE any
 *        widget is constructed, so every widget polishes under the final style + stylesheet.
 *
 * Installs, in order: the window icon; the tooltip effect toggles; AppProxyStyle over Fusion
 * (never the native OS style — the dark theme must render identically on every platform); the
 * QSS modules in cascade order; and the MenuShadowFilter that strips the native popup shadow
 * from every QMenu while keeping the rounded corners (see apptheme.cpp).
 */
void install(QApplication& app);

} // namespace AppTheme

#endif // APPTHEME_H
