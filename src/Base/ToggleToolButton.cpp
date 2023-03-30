#include "ToggleToolButton.h"

using namespace cnoid;


ToggleToolButton::ToggleToolButton(QWidget* parent)
    : ToolButton(parent)
{
    setCheckable(true);
}
