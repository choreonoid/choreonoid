/*!
  @author Shin'ichiro Nakaoka
*/

#include "Joystick.h"
#include <boost/format.hpp>
#include <linux/joystick.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <string>
#include <vector>
#include <cmath>
#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;

namespace cnoid {

class JoystickImpl
{
public:
    Joystick* self;
    int fd;
    vector<double> axes;
    vector<bool> buttons;
    string errorMessage;

    JoystickImpl(Joystick* self, const char* device);
    ~JoystickImpl();
    bool openDevice(const char* device);
    void closeDevice();
    bool readCurrentState();
    bool readEvent();
};
}


Joystick::Joystick()
{
    impl = new JoystickImpl(this, "/dev/input/js0");
}


Joystick::Joystick(const char* device)
{
    impl = new JoystickImpl(this, device);
}


JoystickImpl::JoystickImpl(Joystick* self, const char* device)
    : self(self)
{
    openDevice(device);
}


bool JoystickImpl::openDevice(const char* device)
{
    closeDevice();
    
    fd = open(device, O_RDONLY | O_NONBLOCK);

    if(fd < 0){
        errorMessage = str(format(_("Device \"%1%\": %2%")) % device % strerror(errno));
        return false;
    }
    errorMessage.clear();
        
    char numAxes;
    ioctl(fd, JSIOCGAXES, &numAxes);
    axes.resize(numAxes, 0.0);

    char numButtons;
    ioctl(fd, JSIOCGBUTTONS, &numButtons);
    buttons.resize(numButtons, false);

    // read initial state
    return readCurrentState();
}    


Joystick::~Joystick()
{
    delete impl;
}


JoystickImpl::~JoystickImpl()
{
    closeDevice();
}


void JoystickImpl::closeDevice()
{
    if(fd >= 0){
        close(fd);
        fd = -1;
    }
}


bool Joystick::isReady() const
{
    return (impl->fd >= 0);
}


const char* Joystick::errorMessage() const
{
    return impl->errorMessage.c_str();
}


int Joystick::fileDescriptor() const
{
    return impl->fd;
}


int Joystick::numAxes() const
{
    return impl->axes.size();
}


int Joystick::numButtons() const
{
    return impl->buttons.size();
}


void Joystick::onJoystickEvent(EventType type, int id, double position)
{

}


bool Joystick::readCurrentState()
{
    return impl->readCurrentState();
}


bool JoystickImpl::readCurrentState()
{
    while(readEvent());
    return (fd >= 0);
}


bool JoystickImpl::readEvent()
{
    if(fd < 0){
        return false;
    }
    
    js_event event;
    const float MAX_VALUE_16BIT = 32767.0f;
    
    // read data of joystick 
    int len = read(fd, &event, sizeof(js_event));
    
    if(len <= 0) {
        if(errno == EAGAIN){
            return false;
        } else {
            errorMessage = strerror(errno);
            closeDevice();
            return false;
        }
    }
    if(len < (int)sizeof(js_event)){
        return false;
    }

    const int id = event.number;
    double pos = (double)event.value / MAX_VALUE_16BIT;
    if(event.type & JS_EVENT_BUTTON) {
        // button 
        buttons[id] = (pos > 0.0);
        self->onJoystickEvent(Joystick::BUTTON, id, pos);
    } else if(event.type & JS_EVENT_AXIS){
        // normalize value (-1.0〜1.0)
        pos = nearbyint(pos * 10.0) / 10.0;
        double prevPos = axes[id];
        if(pos != prevPos){
            axes[id] = pos;
            self->onJoystickEvent(Joystick::AXIS, id, pos);
        }
    }
    return true;
}


double Joystick::getPosition(int axis) const
{
    if(axis < impl->axes.size()){
        return impl->axes[axis];
    }
    return 0.0;
}


bool Joystick::getButtonState(int button) const
{
    if(button < impl->buttons.size()){
        return impl->buttons[button];
    }
    return false;
}


bool Joystick::isActive() const
{
    for(size_t i=0; i < impl->axes.size(); ++i){
        if(impl->axes[i] != 0.0){
            return true;
        }
    }
    for(size_t i=0; i < impl->buttons.size(); ++i){
        if(impl->buttons[i]){
            return true;
        }
    }
    return false;
}
