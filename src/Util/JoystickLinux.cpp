/*!
  @author Shin'ichiro Nakaoka
*/

#include "Joystick.h"
#include "ExtJoystick.h"
#include <boost/format.hpp>
#include <linux/joystick.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <string>
#include <vector>
#include <map>

using namespace std;
using namespace cnoid;
using boost::format;

namespace {

enum Model { PS4, PS3, XBOX, F310_XInput, F310_DirectInput, UNSUPPORTED };

// Left-Stick, Right-Stick, DirectionalPad, L2, R2
const int NUM_STD_AXES = 8;    

const int PS4_Axes[]         = {  0,  1,  2,  5,  6,  7,  3,  4 };
const int PS3_Axes[]         = {  0,  1,  2,  3, -1, -1, 12, 13 };
const int XBOX_Axes[]        = {  0,  1,  3,  4,  6,  7,  2,  5 };
const int F310X_Axes[]       = {  0,  1,  3,  4,  6,  7,  2,  5 };
const int F310D_Axes[]       = {  0,  1,  2,  3,  4,  5, -1, -1 };
const int Unsupported_Axes[] = { -1, -1, -1, -1, -1, -1, -1, -1 };

// A, B, X, Y, L1, R1, SELECT(PS4:share), START(PS4:option), Left-Stick, Right-Stick, Logo(PS4:PS)
const int NUM_STD_BUTTONS = 11; 
//const int PS4_Buttons[]   =     {  0,  1,  2,  3,  4,  5, 10, 11,  9,  8, 12 };
const int PS4_Buttons[]   =       {  1,  2,  0,  3,  4,  5,  8,  9, 10, 11, 12 };
const int PS3_Buttons[]   =       { 14, 13, 15, 12, 10, 11,  0,  3,  1,  2, 16 };
const int XBOX_Buttons[]  =       {  0,  1,  2,  3,  4,  5,  6,  7,  9, 10,  8 };
const int F310X_Buttons[] =       {  0,  1,  2,  3,  4,  5,  6,  7,  9, 10,  8 };
const int F310D_Buttons[] =       {  1,  2,  0,  3,  4,  5,  8,  9, 10, 11, -1 };
const int Unsupported_Buttons[] = { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 };


struct ModelInfo {
    Model id;
    const int* axisMap;
    const int* buttonMap;
};

const map<string, ModelInfo> modelMap = {

    // CUH-ZCT1J or CUH-ZCT2J
    { "Sony Computer Entertainment Wireless Controller",    { PS4,  PS4_Axes,  PS4_Buttons } },
    
    { "Sony Interactive Entertainment Wireless Controller", { PS4,  PS4_Axes,  PS4_Buttons } },
    { "Sony PLAYSTATION(R)3 Controller",                    { PS3,  PS3_Axes,  PS3_Buttons } },
    { "Microsoft X-Box 360 pad",                            { XBOX, XBOX_Axes, XBOX_Axes } },
    { "Microsoft X-Box One pad",                            { XBOX, XBOX_Axes, XBOX_Axes } },
    { "Logitech Gamepad F310",                              { F310_XInput, F310X_Axes, F310D_Axes } }
};

ModelInfo unsupportedModel { UNSUPPORTED, Unsupported_Axes, Unsupported_Buttons };

}

namespace cnoid {

class JoystickImpl
{
public:
    Joystick* self;
    ExtJoystick* extJoystick;
    int fd;
    vector<double> axes;
    vector<bool> axisEnabled;
    vector<bool> buttons;
    string errorMessage;
    Signal<void(int id, bool isPressed)> sigButton;
    Signal<void(int id, double position)> sigAxis;

    ModelInfo currentModel;
    
    vector<bool> record_init_pos;
    vector<double> initial_pos;
    vector<bool> initialized;
    vector<double> PS_axes_default_pos;

    JoystickImpl(Joystick* self, const char* device);
    ~JoystickImpl();
    bool openDevice(const char* device);
    void closeDevice();
    bool readCurrentState();
    bool readEvent();
    double readDirectionPadAsAxis(int axis);
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
    fd = -1;

    extJoystick = ExtJoystick::findJoystick(device);

    if(!extJoystick){
        if(!openDevice(device)){
            extJoystick = ExtJoystick::findJoystick("*");
        }
    }
}


bool JoystickImpl::openDevice(const char* device)
{
    closeDevice();
    
    fd = open(device, O_RDONLY | O_NONBLOCK);

    if(fd < 0){
        errorMessage = str(format("Device \"%1%\": %2%") % device % strerror(errno));
        return false;
    }
    errorMessage.clear();
        
    char numAxes;
    ioctl(fd, JSIOCGAXES, &numAxes);
    axes.resize(numAxes, 0.0);
    axisEnabled.resize(numAxes, true);

    char numButtons;
    ioctl(fd, JSIOCGBUTTONS, &numButtons);
    buttons.resize(numButtons, false);

    char identifier[1024];
    ioctl(fd, JSIOCGNAME(sizeof(identifier)), identifier);

    auto iter = modelMap.find(identifier);
    if(iter != modelMap.end()){
        currentModel = iter->second;
    } else {
        currentModel = unsupportedModel;
    }

    if(currentModel.id == PS4 || currentModel.id == PS3){
        record_init_pos.assign(axisEnabled.size(), false);
        initial_pos.assign(axisEnabled.size(), 0.0);
        initialized.assign(axisEnabled.size(), false);
        vector<double> PS_analog_input_defaults;
        if(currentModel.id == PS4){
            // Lstick_H, Lstick_V, Rstick_H, L2, R2, Rstick_V
            PS_analog_input_defaults = { 0.0, 0.0, 0.0, -1.0, -1.0, 0.0 };
        } else if(currentModel.id == PS3){
            // Lstick_H, Lstick_V, Rstick_H, Rstick_V, empty*4, UP, RIGHT, DOWN, LEFT, L2, R2
            PS_analog_input_defaults = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, -1.0 };
        }
        vector<double> PS_default_positions(axisEnabled.size() - PS_analog_input_defaults.size(), 0.0);
        copy(PS_analog_input_defaults.begin(), PS_analog_input_defaults.end(), back_inserter(PS_axes_default_pos));
        copy(PS_default_positions.begin(), PS_default_positions.end(), back_inserter(PS_axes_default_pos));
    }
    
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
    return impl->extJoystick ? true : (impl->fd >= 0);
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
    return impl->extJoystick ? impl->extJoystick->numAxes() : impl->axes.size();
}


void Joystick::setAxisEnabled(int axis, bool on)
{
    if(!impl->extJoystick){
        if(axis < impl->axes.size()){
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
        bool isPressed = (pos > 0.0);
        buttons[id] = isPressed;
        sigButton(id, isPressed);
    } else if(event.type & JS_EVENT_AXIS){
        if(axisEnabled[id]){
            // normalize value (-1.0 to 1.0)
            pos = nearbyint(pos * 10.0) / 10.0;

            if(currentModel.id == PS4 || currentModel.id == PS3){
                // analog stick input of PS4 is -1.0 before first operation.
                if(!record_init_pos[id]){
                    initial_pos[id] = pos;
                    pos = PS_axes_default_pos[id]; // replace input for default value
                    record_init_pos[id] = true;
                    initialized[id] = false;
                } else {
                    if(!initialized[id]){
                        if(pos == initial_pos[id]){
                            pos = PS_axes_default_pos[id]; // replace input for default value
                        } else {
                            initialized[id]=true; // analog stick input changed
                        }
                    }
                }
            }
            
            double prevPos = axes[id];
            if(pos != prevPos){
                axes[id] = pos;
                sigAxis(id, pos);
            }
        }
    }
    return true;
}

double JoystickImpl::readDirectionPadAsAxis(int axis)
{
    switch(currentModel.id){

    case PS3:
        if(axis == 4){
            if(buttons[5]){
                return 1.0;
            } else if(buttons[7]){
                return -1.0;
            }
        } else if(axis==5){
            if(buttons[4]){
                return -1.0;
            } else if(buttons[6]){
                return 1.0;
            }
        }
        break;

    case F310_DirectInput:
        if(axis==6){
            if(buttons[6]){
                return 1.0;
            } else {
                return -1.0;
            }
        } else if(axis==7){
            if(buttons[7]){
                return 1.0;
            }else{
                return -1.0;
            }
        }
        break;

    default:
        return 0.0;
    }
}


double Joystick::getPosition(int axis) const
{
    if(impl->extJoystick){
        return impl->extJoystick->getPosition(axis);
    }

    if(impl->currentModel.id == UNSUPPORTED || axis >= NUM_STD_AXES){
        return getNativePosition(axis);
    }

    int axisIndex = impl->currentModel.axisMap[axis];
    if(axisIndex >= 0){
      return impl->axes[axisIndex];
    } else {
      return impl->readDirectionPadAsAxis(axis);
    }

    return 0.0;
}


bool Joystick::getButtonState(int button) const
{
    if(impl->extJoystick){
        return impl->extJoystick->getButtonState(button);
    }

    if(impl->currentModel.id == UNSUPPORTED || button >= NUM_STD_BUTTONS){
        return getNativeButtonState(button);
    }

    int buttonIndex = impl->currentModel.buttonMap[button];
    if(buttonIndex >= 0){
        return impl->buttons[buttonIndex];
    }
    
    return false;
}

double Joystick::getNativePosition(int axis) const
{
    if(impl->extJoystick){
        return impl->extJoystick->getPosition(axis);
    }

    if(axis < impl->axes.size()){
        return impl->axes[axis];
    }
    return 0.0;
}


bool Joystick::getNativeButtonState(int button) const
{
    if(impl->extJoystick){
        return impl->extJoystick->getButtonState(button);
    }
    
    if(button < impl->buttons.size()){
        return impl->buttons[button];
    }
    return false;
}


bool Joystick::isActive() const
{
    if(impl->extJoystick){
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
