/*!
  @author Shin'ichiro Nakaoka
*/

#include "JoystickCapture.h"
#include "SocketNotifier.h"
#include <cnoid/Joystick>
#include <boost/bind.hpp>

using namespace boost;
using namespace cnoid;

namespace {

class JoystickEx : public Joystick
{
    JoystickCaptureImpl* capture;
public:
    JoystickEx(JoystickCaptureImpl* capture, const char* device)
        : Joystick(device), capture(capture) { }
    virtual void onJoystickButtonEvent(int id, bool isPressed);
    virtual void onJoystickAxisEvent(int id, double position);
};

}

namespace cnoid {

class JoystickCaptureImpl
{
public:
    JoystickEx* joystick;
    SocketNotifier* notifier;
    
    signal<void(int id, bool isPressed)> sigButton;
    signal<void(int id, double position)> sigAxis;
    
    JoystickCaptureImpl();
    ~JoystickCaptureImpl();
    bool setDevice(const char* device);
};

}


JoystickCapture::JoystickCapture()
{
    impl = new JoystickCaptureImpl;
}


JoystickCaptureImpl::JoystickCaptureImpl()
{
    joystick = 0;
    notifier = 0;
}


JoystickCapture::~JoystickCapture()
{
    delete impl;
}


JoystickCaptureImpl::~JoystickCaptureImpl()
{
    setDevice(0);
}


bool JoystickCapture::setDevice(const char* device)
{
    return impl->setDevice(device);
}


bool JoystickCaptureImpl::setDevice(const char* device)
{
    if(notifier){
        delete notifier;
    }
    if(joystick){
        delete joystick;
    }
    if(device){
        joystick = new JoystickEx(this, device);
        if(joystick->isReady()){
            notifier = new SocketNotifier(joystick->fileDescriptor(), QSocketNotifier::Read);
            notifier->sigActivated().connect(
                bind(&Joystick::readCurrentState, joystick));
            return true;
        }
    }
    return false;
}


void JoystickCapture::releaseDevice()
{
    impl->setDevice(0);
}


boost::signal<void(int id, bool isPressed)>& JoystickCapture::sigButton()
{
    return impl->sigButton;
}


boost::signal<void(int id, double position)>& JoystickCapture::sigAxis()
{
    return impl->sigAxis;
}


bool JoystickCapture::isReady() const
{
    return (impl->joystick && impl->joystick->isReady());
}


void JoystickEx::onJoystickButtonEvent(int id, bool isPressed)
{
    capture->sigButton(id, isPressed);
}


void JoystickEx::onJoystickAxisEvent(int id, double position)
{
    capture->sigAxis(id, position);
}


int JoystickCapture::numAxes() const
{
    return impl->joystick ? impl->joystick->numAxes() : 0;
}

int JoystickCapture::numButtons() const
{
    return impl->joystick ? impl->joystick->numButtons() : 0;
}

bool JoystickCapture::readCurrentState()
{
    return impl->joystick ? impl->joystick->readCurrentState() : false;
}

double JoystickCapture::getPosition(int axis) const
{
    return impl->joystick ? impl->joystick->getPosition(axis) : 0.0;
}

bool JoystickCapture::getButtonState(int button) const
{
    return impl->joystick ? impl->joystick->getButtonState(button) : false;
}


bool JoystickCapture::isActive() const
{
    return impl->joystick ? impl->joystick->isActive() : false;
}
