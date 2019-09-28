/**
   @author Shin'ichiro Nakaoka
*/

#include "CheckBox.h"

using namespace cnoid;

CheckBox::CheckBox(QWidget* parent)
    : QCheckBox(parent)
{
    sigStateChangedConnected = false;
    sigButtonToggledConnected = false;
}


CheckBox::CheckBox(const QString& text, QWidget* parent)
    : QCheckBox(text, parent)
{
    sigStateChangedConnected = false;
    sigButtonToggledConnected = false;
}


SignalProxy<void(int)> CheckBox::sigStateChanged()
{
    if(!sigStateChangedConnected){
        connect(this, SIGNAL(stateChanged(int)), this, SLOT(onStateChanged(int)));
        sigStateChangedConnected = true;
    }
    return sigStateChanged_;
}


SignalProxy<void(bool)> CheckBox::sigToggled()
{
    if(!sigButtonToggledConnected){
        connect(this, SIGNAL(toggled(bool)), this, SLOT(onToggled(bool)));
        sigButtonToggledConnected = true;
    }
    return sigToggled_;
}


void CheckBox::onStateChanged(int state)
{
    sigStateChanged_(state);
}


void CheckBox::onToggled(bool checked)
{
    sigToggled_(checked);
}
