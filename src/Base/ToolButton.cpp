#include "ToolButton.h"

using namespace cnoid;


ToolButton::ToolButton(QWidget* parent)
    : QToolButton(parent)
{
    isUserInputEnabled_ = true;
}


ToolButton::ToolButton(const QString& text, QWidget* parent)
    : ToolButton(parent)
{
    setText(text);
}


SignalProxy<void()> ToolButton::sigClicked()
{
    if(!sigClicked_){
        sigClicked_.emplace();
        connect(this, (void(QToolButton::*)()) &QToolButton::clicked,
                [this](){ (*sigClicked_)(); });
    }
    return *sigClicked_;
}


SignalProxy<void(bool)> ToolButton::sigToggled()
{
    if(!sigToggled_){
        sigToggled_.emplace();
        connect(this, (void(QToolButton::*)(bool)) &QToolButton::toggled,
                [this](bool checked){ (*sigToggled_)(checked); });
    }
    return *sigToggled_;
}


SignalProxy<void()> ToolButton::sigPressed()
{
    if(!sigPressed_){
        sigPressed_.emplace();
        connect(this, (void(QToolButton::*)()) &QToolButton::pressed,
                [this](){ (*sigPressed_)(); });
    }
    return *sigPressed_;
}
    
    
SignalProxy<void()> ToolButton::sigReleased()
{
    if(!sigReleased_){
        sigReleased_.emplace();
        connect(this, (void(QToolButton::*)()) &QToolButton::released,
                [this](){ (*sigReleased_)(); });
    }
    return *sigReleased_;
}


void ToolButton::keyPressEvent(QKeyEvent* event)
{
    if(isUserInputEnabled_){
        QToolButton::keyPressEvent(event);
    }
}


void ToolButton::mousePressEvent(QMouseEvent* event)
{
    if(isUserInputEnabled_){
        QToolButton::mousePressEvent(event);
    }
}
