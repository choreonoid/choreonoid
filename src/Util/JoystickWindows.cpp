/**
   @author Shizuko Hattori
   @author Shin'ichiro Nakaoka
*/

#include "Joystick.h"
#include "ExtJoystick.h"
#include <boost/dynamic_bitset.hpp>
#include <windows.h>
#include <mmsystem.h>
#include <vector>
#include <string>

using namespace std;
using namespace cnoid;


namespace cnoid {

class JoystickImpl
{
public:
    Joystick* self;
    ExtJoystick* extJoystick;
    int id;
    int numAvailableAxes;
    vector<double> axes;
    boost::dynamic_bitset<> axisEnabled;
    vector<bool> buttons;
    string errorMessage;
    Signal<void(int id, bool isPressed)> sigButton;
    Signal<void(int id, double position)> sigAxis;

    JoystickImpl();
    bool openDevice();
    bool readCurrentState();
};

}


Joystick::Joystick()
{
    impl = new JoystickImpl();
}


Joystick::Joystick(const char* device)
{
    impl = new JoystickImpl();
}


JoystickImpl::JoystickImpl()
{
    id = -1;
    extJoystick = 0;

    if (!openDevice()){
        extJoystick = ExtJoystick::findJoystick("*");
    }
}


bool JoystickImpl::openDevice()
{
    numAvailableAxes = 0;
    
    std::vector<UINT> devIds;
    JOYINFOEX info;

    info.dwSize  = sizeof(JOYINFOEX);
    info.dwFlags = JOY_RETURNALL;

    int i = 0;
    while(true){
        MMRESULT result = joyGetPosEx(i, &info);
        if(result == JOYERR_PARMS){
            break;
        }
        if(result == JOYERR_NOERROR){
            devIds.push_back(i);
        }
        ++i;
    }

    if(!devIds.empty()){
        // Pick up the first device
        id = devIds.front();
        JOYCAPS caps;
        joyGetDevCaps(id, &caps, sizeof(caps));
        numAvailableAxes = caps.wNumAxes;
        axes.resize(numAvailableAxes, 0.0);
        axisEnabled.resize(numAvailableAxes, true);
        buttons.resize(caps.wNumButtons, false);

        return readCurrentState();
    }

    return false;
}


Joystick::~Joystick()
{
    delete impl;
}


bool Joystick::isReady() const
{
    return impl->extJoystick ? true : (impl->id >= 0);
}


const char* Joystick::errorMessage() const
{
    return impl->errorMessage.c_str();
}


int Joystick::fileDescriptor() const
{
    return impl->id;
}


int Joystick::numAxes() const
{
    return impl->extJoystick ? impl->extJoystick->numAxes() : impl->numAvailableAxes;
}


void Joystick::setAxisEnabled(int axis, bool on)
{
    if (!impl->extJoystick){
        if (axis < impl->axes.size()){
            impl->axes[axis] = 0.0;
            impl->axisEnabled[axis] = on;
        }
    }
}


int Joystick::numButtons() const
{
    return impl->extJoystick ? impl->extJoystick->numButtons() : impl->buttons.size();
}


bool Joystick::readCurrentState()
{
    return impl->extJoystick ? impl->extJoystick->readCurrentState() : impl->readCurrentState();
}


bool JoystickImpl::readCurrentState()
{
    if(id < 0){
        return false;
    }
    
    JOYINFOEX info;
    info.dwSize  = sizeof(info);
    info.dwFlags = JOY_RETURNBUTTONS | JOY_RETURNX | JOY_RETURNY | JOY_RETURNZ;

    if(joyGetPosEx(id, &info) != JOYERR_NOERROR) {
        return false;

    } else {
        // buttons
        for(size_t i=0; i < buttons.size(); ++i){
            if(info.dwButtons & JOY_BUTTON1){
                buttons[i] = true;
            } else {
                buttons[i] = false;
            }
            info.dwButtons >>= 1;
        }
        // axes. normalize value (-1.0 to 1.0)
        if (axisEnabled[0])
            axes[0] = ((double)info.dwXpos - 32767.0) / 32768.0;
        if (axisEnabled[1])
            axes[1] = ((double)info.dwYpos - 32767.0) / 32768.0;
        if (axisEnabled[2])
            axes[2] = ((double)info.dwZpos - 32767.0) / 32768.0;
        // axis 3 and 4 are exchanged to be the same as Linux
        if (axisEnabled[4])
            axes[4] = ((double)info.dwRpos - 32767.0) / 32768.0;
        if (axisEnabled[3])
            axes[3] = ((double)info.dwUpos - 32767.0) / 32768.0;
        if (axisEnabled[5])
            axes[5] = ((double)info.dwVpos - 32767.0) / 32768.0;

        return true;
    }

    return false;
}


double Joystick::getPosition(int axis) const
{
    if (impl->extJoystick){
        return impl->extJoystick->getPosition(axis);
    }

    if(axis < impl->axes.size()){
        return impl->axes[axis];
    }
    return 0.0;
}


bool Joystick::getButtonState(int button) const
{
    if (impl->extJoystick){
        return impl->extJoystick->getButtonState(button);
    }

    if(button < impl->buttons.size()){
        return impl->buttons[button];
    }
    return false;
}


bool Joystick::isActive() const
{
    if (impl->extJoystick){
        return impl->extJoystick->isActive();
    }

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


SignalProxy<void(int id, bool isPressed)> Joystick::sigButton()
{
    return impl->sigButton;
}


SignalProxy<void(int id, double position)> Joystick::sigAxis()
{
    return impl->sigAxis;
}

