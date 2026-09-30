/**
 * @file propertyrows.cpp
 * @brief Implementation of the shared properties-tab row builders. See propertyrows.h.
 */

#include "propertyrows.h"

#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QIcon>
#include <QToolButton>
#include <QVBoxLayout>

namespace pose::propertyrows {

QFormLayout* addGroup(QVBoxLayout* parent, const QString& title) {
    auto* box = new QGroupBox(title);
    box->setObjectName(QStringLiteral("PropertyGroup"));
    auto* form = new QFormLayout(box);
    form->setLabelAlignment(Qt::AlignLeft);
    form->setFormAlignment(Qt::AlignTop);
    form->setHorizontalSpacing(10);
    form->setVerticalSpacing(8);
    parent->addWidget(box);
    return form;
}

QToolButton* makeRestoreButton() {
    auto* restore = new QToolButton;
    restore->setObjectName(QStringLiteral("PropertyRowRestore"));
    restore->setIcon(QIcon(QStringLiteral(":/resources/icons/default.png")));
    restore->setIconSize(QSize(14, 14));
    restore->setToolTip(QObject::tr("Restore default"));
    restore->setCursor(Qt::PointingHandCursor);
    restore->setAutoRaise(true);
    return restore;
}

QWidget* rowWithRestore(QWidget* main, QToolButton* restore) {
    auto* field = new QWidget;
    auto* row = new QHBoxLayout(field);
    row->setContentsMargins(0, 0, 0, 0);
    row->setSpacing(4);
    row->addWidget(main, 1);
    row->addWidget(restore, 0);
    return field;
}

} // namespace pose::propertyrows
