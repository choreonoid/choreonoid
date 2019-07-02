/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_EXT_JOYSTICK_H
#define CNOID_UTIL_EXT_JOYSTICK_H

#include "Signal.h"
#include <string>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT ExtJoystick
{
    Signal<void()> sigDestroyed_;
    
public:
    static void registerJoystick(const std::string& name, ExtJoystick* joystick);
    static ExtJoystick* findJoystick(const std::string& name);

    SignalProxy<void()> sigDestroyed() { return sigDestroyed_; }

    virtual ~ExtJoystick();
    virtual int numAxes() const = 0;
    virtual int numButtons() const = 0;
    virtual bool readCurrentState() = 0;
    virtual double getPosition(int axis) const = 0;
    virtual bool getButtonState(int button) const = 0;
    virtual bool isActive() const = 0;
    virtual SignalProxy<void(int id, double position)> sigAxis() = 0;
    virtual SignalProxy<void(int id, bool isPressed)> sigButton() = 0;
};

}

#endif
