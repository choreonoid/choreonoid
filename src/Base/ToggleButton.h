#ifndef CNOID_BASE_TOGGLE_BUTTON_H
#define CNOID_BASE_TOGGLE_BUTTON_H

#include "PushButton.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT ToggleButton : public PushButton
{
public:
    ToggleButton(QWidget* parent = nullptr);
    ToggleButton(const QString& text, QWidget* parent = nullptr);
    ToggleButton(const QIcon& icon, const QString& text, QWidget* parent = nullptr);
};

}

#endif
