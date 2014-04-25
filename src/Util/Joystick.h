/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_JOYSTICK_H_INCLUDED
#define CNOID_UTIL_JOYSTICK_H_INCLUDED

#include "exportdecl.h"

namespace cnoid {

class JoystickImpl;

class CNOID_EXPORT Joystick
{
public:
    Joystick();
    Joystick(const char* device);
    ~Joystick();

    bool isReady() const;
    const char* errorMessage() const;
        
    int numAxes() const;
    int numButtons() const;
    bool readCurrentState();
    double getPosition(int axis) const;
    bool getButtonState(int button) const;
       
private:
    JoystickImpl* impl;
};
}

#endif
