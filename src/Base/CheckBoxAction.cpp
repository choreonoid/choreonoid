#include "CheckBoxAction.h"
#include <QBoxLayout>
#include <QLabel>

using namespace cnoid;


CheckBoxAction::CheckBoxAction(const QString& text, QObject* parent)
    : QWidgetAction(parent)
{
    auto widget = new QWidget;
    auto hbox = new QHBoxLayout;
    checkBox_ = new CheckBox;
    hbox->addWidget(checkBox_);
    auto label = new QLabel(text);
    label->setAlignment(Qt::AlignLeft);
    hbox->addWidget(label);
    hbox->addStretch();
    widget->setLayout(hbox);
    setDefaultWidget(widget);
}
