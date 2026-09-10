/**
 * @file placeholderpreferencespanel.cpp
 * @brief Implements PlaceholderPreferencesPanel. See placeholderpreferencespanel.h.
 */

#include "placeholderpreferencespanel.h"

PlaceholderPreferencesPanel::PlaceholderPreferencesPanel(const QString& title,
                                                         const QString& message, QWidget* parent)
    : PreferencesPanel(title, parent) {
    addPlaceholder(message);
}
