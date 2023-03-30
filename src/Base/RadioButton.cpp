#include "RadioButton.h"

using namespace cnoid;


RadioButton::RadioButton(QWidget* parent)
    : QRadioButton(parent)
{
    isUserInputEnabled_ = true;
}


RadioButton::RadioButton(const QString& text, QWidget* parent)
    : QRadioButton(text, parent)
{
    isUserInputEnabled_ = true;
}


SignalProxy<void(bool)> RadioButton::sigToggled()
{
    if(!sigToggled_){
        stdx::emplace(sigToggled_);
        connect(this, (void(QRadioButton::*)(bool)) &QRadioButton::toggled,
                [this](bool checked){ (*sigToggled_)(checked); });
    }
    return *sigToggled_;
}


void RadioButton::keyPressEvent(QKeyEvent* event)
{
    if(isUserInputEnabled_){
        QRadioButton::keyPressEvent(event);
    }
}


void RadioButton::mousePressEvent(QMouseEvent* event)
{
    if(isUserInputEnabled_){
        QRadioButton::mousePressEvent(event);
    }
}
