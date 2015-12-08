/**
   @author Shin'ichiro Nakaoka
*/

#include "CheckBox.h"

using namespace cnoid;

CheckBox::CheckBox(QWidget* parent)
    : QCheckBox(parent)
{
    initialize();
}


CheckBox::CheckBox(const QString& text, QWidget* parent)
    : QCheckBox(text, parent)
{
    initialize();
}


void CheckBox::initialize()
{
    connect(this, SIGNAL(stateChanged(int)), this, SLOT(onStateChanged(int)));
    connect(this, SIGNAL(toggled(bool)), this, SLOT(onToggled(bool)));
}


void CheckBox::onStateChanged(int state)
{
    sigStateChanged_(state);
}

void CheckBox::onToggled(bool checked)
{
    sigToggled_(checked);
}
