#include "CheckBox.h"

using namespace cnoid;


CheckBox::CheckBox(QWidget* parent)
    : QCheckBox(parent)
{
    isUserInputEnabled_ = true;
}


CheckBox::CheckBox(const QString& text, QWidget* parent)
    : QCheckBox(text, parent)
{
    isUserInputEnabled_ = true;
}


SignalProxy<void(int)> CheckBox::sigStateChanged()
{
    if(!sigStateChanged_){
        sigStateChanged_.emplace();
        connect(this, (void(QCheckBox::*)(int)) &QCheckBox::stateChanged,
                [this](int state){ (*sigStateChanged_)(state); });
    }
    return *sigStateChanged_;
}


SignalProxy<void(bool)> CheckBox::sigToggled()
{
    if(!sigToggled_){
        sigToggled_.emplace();
        connect(this, (void(QCheckBox::*)(bool)) &QCheckBox::toggled,
                [this](bool checked){ (*sigToggled_)(checked); });
    }
    return *sigToggled_;
}


void CheckBox::keyPressEvent(QKeyEvent* event)
{
    if(isUserInputEnabled_){
        QCheckBox::keyPressEvent(event);
    }
}


void CheckBox::mousePressEvent(QMouseEvent* event)
{
    if(isUserInputEnabled_){
        QCheckBox::mousePressEvent(event);
    }
}
