/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_JOYSTICK_CAPTURE_H
#define CNOID_BASE_JOYSTICK_CAPTURE_H

#include <cnoid/Signal>
#include "exportdecl.h"

namespace cnoid {

class JoystickCaptureImpl;

class CNOID_EXPORT JoystickCapture
{
public:
    JoystickCapture();
    ~JoystickCapture();

    bool setDevice(const char* device);
    bool isReady() const;
    void releaseDevice();

    SignalProxy<void(int id, bool isPressed)> sigButton();
    SignalProxy<void(int id, double position)> sigAxis();

    int numAxes() const;
    void setAxisEnabled(int axis, bool on);
    int numButtons() const;
    bool readCurrentState();
    double getPosition(int axis) const;
    bool getButtonState(int button) const;
    bool isActive() const;
       
private:
    JoystickCaptureImpl* impl;
};

}

#endif
