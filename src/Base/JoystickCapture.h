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

    boost::signal<void(int id, double position)>& sigButton();
    boost::signal<void(int id, double position)>& sigAxis();
       
private:
    JoystickCaptureImpl* impl;
};

}

#endif
