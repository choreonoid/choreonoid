#ifndef CNOID_BASE_TOGGLE_TOOL_BUTTON_H
#define CNOID_BASE_TOGGLE_TOOL_BUTTON_H

#include "ToolButton.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT ToggleToolButton : public ToolButton
{
public:
    ToggleToolButton(QWidget* parent = nullptr);
};

}

#endif
