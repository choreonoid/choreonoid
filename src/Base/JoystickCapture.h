/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_JOYSTICK_CAPTURE_H
#define CNOID_BASE_JOYSTICK_CAPTURE_H

#include <boost/signals.hpp>
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

    boost::signal<void(int id, bool isPressed)>& sigButton();
    boost::signal<void(int id, double position)>& sigAxis();

    int numAxes() const;
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
