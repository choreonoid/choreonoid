#include "ToggleButton.h"

using namespace cnoid;


ToggleButton::ToggleButton(QWidget* parent)
    : PushButton(parent)
{
    setCheckable(true);
}


ToggleButton::ToggleButton(const QString& text, QWidget* parent)
    : PushButton(text, parent)
{
    setCheckable(true);
}
    

ToggleButton::ToggleButton(const QIcon& icon, const QString& text, QWidget* parent)
    : PushButton(icon, text, parent)
{
    setCheckable(true);
}
