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
        stdx::emplace(sigClicked_);
        connect(this, (void(QToolButton::*)()) &QToolButton::clicked,
                [this](){ (*sigClicked_)(); });
    }
    return *sigClicked_;
}


SignalProxy<void(bool)> ToolButton::sigToggled()
{
    if(!sigToggled_){
        stdx::emplace(sigToggled_);
        connect(this, (void(QToolButton::*)(bool)) &QToolButton::toggled,
                [this](bool checked){ (*sigToggled_)(checked); });
    }
    return *sigToggled_;
}


SignalProxy<void()> ToolButton::sigPressed()
{
    if(!sigPressed_){
        stdx::emplace(sigPressed_);
        connect(this, (void(QToolButton::*)()) &QToolButton::pressed,
                [this](){ (*sigPressed_)(); });
    }
    return *sigPressed_;
}
    
    
SignalProxy<void()> ToolButton::sigReleased()
{
    if(!sigReleased_){
        stdx::emplace(sigReleased_);
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
