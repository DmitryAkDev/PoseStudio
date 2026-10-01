/**
 * @file generalpreferencespanel.h
 * @brief "General" page of the Preferences dialog.
 */

#ifndef GENERALPREFERENCESPANEL_H
#define GENERALPREFERENCESPANEL_H

#include "preferencespanel.h"

/// General application settings: the UI language, the anonymous install ping toggle (see
/// installping.h) and the startup update check.
class GeneralPreferencesPanel : public PreferencesPanel {
    Q_OBJECT

public:
    explicit GeneralPreferencesPanel(QWidget* parent = nullptr);
};

#endif // GENERALPREFERENCESPANEL_H
