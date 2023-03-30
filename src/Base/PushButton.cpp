#include "PushButton.h"

using namespace cnoid;


PushButton::PushButton(QWidget* parent)
    : QPushButton(parent)
{
    isUserInputEnabled_ = true;
}


PushButton::PushButton(const QString& text, QWidget* parent)
    : QPushButton(text, parent)
{
    isUserInputEnabled_ = true;
}


PushButton::PushButton(const QIcon & icon, const QString & text, QWidget* parent)
    : QPushButton(icon, text, parent)
{
    isUserInputEnabled_ = true;
}


SignalProxy<void()> PushButton::sigClicked()
{
    if(!sigClicked_){
        stdx::emplace(sigClicked_);
        connect(this, (void(QPushButton::*)()) &QPushButton::clicked,
                [this](){ (*sigClicked_)(); });
    }
    return *sigClicked_;
}


SignalProxy<void(bool)> PushButton::sigToggled()
{
    if(!sigToggled_){
        stdx::emplace(sigToggled_);
        connect(this, (void(QPushButton::*)(bool)) &QPushButton::toggled,
                [this](bool checked){ (*sigToggled_)(checked); });
    }
    return *sigToggled_;
}


void PushButton::keyPressEvent(QKeyEvent* event)
{
    if(isUserInputEnabled_){
        QPushButton::keyPressEvent(event);
    }
}


void PushButton::mousePressEvent(QMouseEvent* event)
{
    if(isUserInputEnabled_){
        QPushButton::mousePressEvent(event);
    }
}
