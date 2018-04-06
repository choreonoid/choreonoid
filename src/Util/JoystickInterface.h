/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_JOYSTICK_INTERFACE_H
#define CNOID_UTIL_JOYSTICK_INTERFACE_H

#include <cmath>

namespace cnoid {

class JoystickInterface
{
public:
    virtual int numAxes() const = 0;
    virtual int numButtons() const = 0;
    virtual bool readCurrentState() = 0;
    virtual double getPosition(int axis) const = 0;
    virtual bool getButtonState(int button) const = 0;
};

}

#endif
