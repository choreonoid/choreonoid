/*!
  @author Shin'ichiro Nakaoka
*/

#include "JoystickCapture.h"
#include "SocketNotifier.h"
#include <cnoid/Joystick>

using namespace cnoid;

namespace cnoid {

class JoystickCaptureImpl
{
public:
    Joystick* joystick;
    SocketNotifier* notifier;
    
    JoystickCaptureImpl();
    ~JoystickCaptureImpl();
    bool setDevice(const char* device);
    void onNotifierActivated();
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
        notifier = 0;
    }
    if(joystick){
        delete joystick;
        joystick = 0;
    }
    if(device){
        joystick = new Joystick(device);
        if(joystick->isReady()){
            notifier = new SocketNotifier(joystick->fileDescriptor(), QSocketNotifier::Read);
            notifier->sigActivated().connect(std::bind(&JoystickCaptureImpl::onNotifierActivated, this));
            return true;
        }
    }
    return false;
}


void JoystickCapture::releaseDevice()
{
    impl->setDevice(0);
}


SignalProxy<void(int id, bool isPressed)> JoystickCapture::sigButton()
{
    return impl->joystick->sigButton();
}


SignalProxy<void(int id, double position)> JoystickCapture::sigAxis()
{
    return impl->joystick->sigAxis();
}


bool JoystickCapture::isReady() const
{
    return (impl->joystick && impl->joystick->isReady());
}


void JoystickCaptureImpl::onNotifierActivated()
{
    notifier->setEnabled(false);
    joystick->readCurrentState();
    notifier->setEnabled(true);
}


int JoystickCapture::numAxes() const
{
    return impl->joystick ? impl->joystick->numAxes() : 0;
}

void JoystickCapture::setAxisEnabled(int axis, bool on)
{
    if(impl->joystick){
        impl->joystick->setAxisEnabled(axis, on);
    }
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
