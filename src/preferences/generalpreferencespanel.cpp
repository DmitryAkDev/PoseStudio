/**
 * @file generalpreferencespanel.cpp
 * @brief Implements the "General" preferences page.
 */

#include "generalpreferencespanel.h"
#include "installping.h"

#include <QCheckBox>
#include <QVBoxLayout>

GeneralPreferencesPanel::GeneralPreferencesPanel(QWidget* parent)
    : PreferencesPanel(QStringLiteral("General"), parent) {

    // --- Anonymous install ping ---
    // The toggle is the user-facing half of InstallPing; the copy below is the disclosure of
    // exactly what the ping carries, so keep it in step with InstallPing::buildPayload().
    auto* pingToggle = new QCheckBox(QStringLiteral("Send an anonymous install ping"), this);
    pingToggle->setChecked(InstallPing::isEnabled());
    contentLayout()->addWidget(pingToggle);

    addDescription(
        "Each time it starts, PoseStudio lets posestudio.io know that this installation exists, so we can "
        "see how many people use it and which versions are out there. The ping carries a random "
        "install ID, the app version, your operating system and CPU type, whether the app was "
        "installed or is running portable, the Qt library version, and the time of the ping — "
        "nothing else: no names, no files, no usage data, and the ID isn't linked to you in any way.");
    // Shown, never minted here: opening Preferences must not create and persist an identifier
    // (in a keyless build, or with the toggle off, one may legitimately never exist).
    const QString id = InstallPing::existingInstallId();
    addDescription(id.isEmpty()
                       ? QStringLiteral("Install ID: not created yet (assigned when the first ping is sent)")
                       : QStringLiteral("Install ID: %1").arg(id));

    if (!InstallPing::isBuiltIn()) {
        // Local builds carry no signing key (it comes from the release pipeline's secret), so
        // the setting is honoured but moot — say so rather than leave a developer guessing.
        addDescription("This build has no ping key, so nothing is sent regardless of this setting.");
    }

    connect(pingToggle, &QCheckBox::toggled, this, [](bool enabled) {
        InstallPing::setEnabled(enabled);
    });
}
