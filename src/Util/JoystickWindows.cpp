/**
   @author Shizuko Hattori
   @author Shin'ichiro Nakaoka
*/

#include "Joystick.h"
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
    int id;
    int numAvailableAxes;
    vector<double> axes;
    vector<bool> buttons;
    string errorMessage;
    Signal<void(int id, bool isPressed)> sigButton;
    Signal<void(int id, double position)> sigAxis;

    JoystickImpl();
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
        axes.resize(6, 0.0);
        buttons.resize(caps.wNumButtons, false);

        readCurrentState();
    }
}


Joystick::~Joystick()
{
    delete impl;
}


bool Joystick::isReady() const
{
    return (impl->id >= 0);
}


const char* Joystick::errorMessage() const
{
    return impl->errorMessage.c_str();
}


int Joystick::fileDescriptor() const
{
    return -1; // invalid
}


int Joystick::numAxes() const
{
    return impl->numAvailableAxes;
}


//! \todo implement this function
void Joystick::setAxisEnabled(int axis, bool on)
{

}


int Joystick::numButtons() const
{
    return impl->buttons.size();
}


bool Joystick::readCurrentState()
{
    return impl->readCurrentState();
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
        // axes. normalize value (-1.0〜1.0)
        axes[0] = ((double)info.dwXpos - 32767.0) / 32768.0;
        axes[1] = ((double)info.dwYpos - 32767.0) / 32768.0;
        axes[2] = ((double)info.dwZpos - 32767.0) / 32768.0;
        // axis 3 and 4 are exchanged to be the same as Linux
        axes[4] = ((double)info.dwRpos - 32767.0) / 32768.0;
        axes[3] = ((double)info.dwUpos - 32767.0) / 32768.0;
        axes[5] = ((double)info.dwVpos - 32767.0) / 32768.0;
    }

    return false;
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


SignalProxy<void(int id, bool isPressed)> Joystick::sigButton()
{
    return impl->sigButton;
}


SignalProxy<void(int id, double position)> Joystick::sigAxis()
{
    return impl->sigAxis;
}

