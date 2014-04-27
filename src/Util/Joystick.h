/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_JOYSTICK_H
#define CNOID_UTIL_JOYSTICK_H

#include "exportdecl.h"

namespace cnoid {

class JoystickImpl;

class CNOID_EXPORT Joystick
{
public:
    Joystick();
    Joystick(const char* device);
    virtual ~Joystick();

    int fileDescriptor() const;

    bool isReady() const;
    const char* errorMessage() const;
        
    int numAxes() const;
    int numButtons() const;
    bool readCurrentState();
    double getPosition(int axis) const;
    bool getButtonState(int button) const;

protected:
    enum EventType { BUTTON, AXIS };
    virtual void onJoystickEvent(EventType type, int id, double position);
       
private:
    JoystickImpl* impl;
    friend class JoystickImpl;
};

}

#endif
