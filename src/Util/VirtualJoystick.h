/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_VIRTUAL_JOYSTICK_H
#define CNOID_UTIL_VIRTUAL_JOYSTICK_H

#include "Signal.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT VirtualJoystick
{
public:
    static void registerJoystick(const char* name, VirtualJoystick* joystick);
    static VirtualJoystick* findJoystick(const char* name);

    virtual ~VirtualJoystick() = 0;
    virtual bool isReady() const = 0;
    virtual int numAxes() const = 0;
    virtual int numButtons() const = 0;
    virtual bool readCurrentState() = 0;
    virtual double getPosition(int axis) const = 0;
    virtual bool getButtonState(int button) const = 0;
    virtual bool isActive() const = 0;
    virtual SignalProxy<void(int id, bool isPressed)> sigButton() = 0;
    virtual SignalProxy<void(int id, double position)> sigAxis() = 0;
};

}

#endif
