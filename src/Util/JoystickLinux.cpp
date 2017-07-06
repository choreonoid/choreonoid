/*!
  @author Shin'ichiro Nakaoka
*/

#include "Joystick.h"
#include "ExtJoystick.h"
#include <boost/format.hpp>
#include <boost/dynamic_bitset.hpp>
#include <linux/joystick.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <string>
#include <vector>
#include <cmath>

using namespace std;
using namespace boost;
using namespace cnoid;

namespace cnoid {

class JoystickImpl
{
public:
    enum Model { PS4=0, PS3=1, XBOX=2, F310_XInput=3, F310_DirectInput=4, OTHER=5 };
    static const int STANDARD_AXIS_NUM = 8;    // LeftStick, RightStick, DirectionalPad, L2, R2
    static const int STANDARD_BUTTON_NUM = 11; // square,cross,circle,triangle,L1,R1,L3,R3,START(PS4:option),SELECT(PS4:share),Logo(PS4:PS)
    Joystick* self;
    ExtJoystick* extJoystick;
    int fd;
    vector<double> axes;
    dynamic_bitset<> axisEnabled;
    vector<bool> buttons;
    string errorMessage;
    Signal<void(int id, bool isPressed)> sigButton;
    Signal<void(int id, double position)> sigAxis;
    Model model;
    vector<int> axis_map;
    vector<int> button_map;
    vector<bool> record_init_pos;
    vector<double> initial_pos;
    vector<bool> initialized;
    vector<double> PS4_axes_default_pos;

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
    string model_id = identifier;
    axis_map.resize(STANDARD_AXIS_NUM);
    static const vector<int> PS4_AM{ 0, 1, 2, 5, 6, 7, 3, 4 };
    static const vector<int> PS3_AM{ 0, 1, 2, 3, -1, -1, 12, 13 };
    static const vector<int> XBOX_AM{ 0, 1, 3, 4, 6, 7, 2, 5 };
    static const vector<int> F310_X_AM{ 0, 1, 3, 4, 6, 7, 2, 5 };
    static const vector<int> F310_D_AM{ 0, 1, 2, 3, 4, 5, -1, -1 };
    static const vector<int> OTHER_AM(STANDARD_AXIS_NUM, -1);
    button_map.resize(STANDARD_BUTTON_NUM);
    // Mapping square, cross, circle, triangle, L1, R1, L3, R3, START, SELECT, LOGO
    static const vector<int> PS4_BM{ 0, 1, 2, 3, 4, 5, 10, 11, 9, 8, 12 };
    static const vector<int> PS3_BM{ 15, 14, 13, 12, 10, 11, 1, 2, 3, 0, 16 };
    static const vector<int> XBOX_BM{ 2, 0, 1, 3, 4, 5, 9, 10, 7, 6, 8 };
    static const vector<int> F310_X_BM{ 2, 0, 1, 3, 4, 5, 9, 10, 7, 6, 8 };
    static const vector<int> F310_D_BM{ 0, 1, 2, 3, 4, 5, 10, 11, 9, 8, -1 };
    static const vector<int> OTHER_BM(STANDARD_BUTTON_NUM, -1);
    if(model_id == "Sony Computer Entertainment Wireless Controller" || model_id == "Sony Interactive Entertainment Wireless Controller"){
      // CUH-ZCT1J or CUH-ZCT2J
      model = PS4;
      axis_map = PS4_AM;
      button_map = PS4_BM;
      record_init_pos.assign(axisEnabled.size(), false);
      initial_pos.assign(axisEnabled.size(), 0.0);
      initialized.assign(axisEnabled.size(), false);
      vector<double> PS4_analog_input_defaults = { 0.0, 0.0, 0.0, -1.0, -1.0, 0.0 };  // Lstick_H, Lstick_V, Rstick_H, L2, R2, Rstick_V
      vector<double> PS4_default_positions(axisEnabled.size()-PS4_analog_input_defaults.size(), 0.0);
      copy(PS4_analog_input_defaults.begin(), PS4_analog_input_defaults.end(), back_inserter(PS4_axes_default_pos));
      copy(PS4_default_positions.begin(), PS4_default_positions.end(), back_inserter(PS4_axes_default_pos));
    }else if(model_id == "Sony PLAYSTATION(R)3 Controller"){
      model = PS3;
      axis_map = PS3_AM;
      button_map = PS3_BM;
    }else if(model_id == "Microsoft X-Box 360 pad" || model_id == "Microsoft X-Box One pad"){
      model = XBOX;
      axis_map = XBOX_AM;
      button_map = XBOX_BM;
    }else if(model_id == "Logitech Gamepad F310"){
      model = F310_XInput;
      axis_map = F310_X_AM;
      button_map = F310_X_BM;
    }else if(model_id == "Logicool Logicool Dual Action"){
      model = F310_DirectInput;
      axis_map = F310_D_AM;
      button_map = F310_D_BM;
    }else{
      model = OTHER;
      axis_map = OTHER_AM;
      button_map = OTHER_BM;
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
            // normalize value (-1.0〜1.0)
            pos = nearbyint(pos * 10.0) / 10.0;
            if(model==PS4){
                // analog stick input of PS4 is -1.0 before first operation.
                if(!record_init_pos[id]){
                    initial_pos[id] = pos;
                    pos = PS4_axes_default_pos[id];    // replace input for default value
                    record_init_pos[id] = true;
                    initialized[id] = false;
                } else {
                    if(!initialized[id]){
                        if(pos == initial_pos[id]){
                            pos = PS4_axes_default_pos[id];    // replace input for default value
                        } else {
                            initialized[id]=true;     // analog stick input changed
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
    if(model==PS3 && axis==4){
      if(buttons[5]){
        return 1.0;
      }else if(buttons[7]){
        return -1.0;
      }
    }else if(model==PS3 && axis==5){
      if(buttons[4]){
        return -1.0;
      }else if(buttons[6]){
        return 1.0;
      }
    }else if(model==F310_DirectInput && axis==6){
      if(buttons[6]){
        return 1.0;
      }else{
        return -1.0;
      }
    }else if(model==F310_DirectInput && axis==7){
      if(buttons[7]){
        return 1.0;
      }else{
        return -1.0;
      }
    }
    return 0.0;
}

double Joystick::getPosition(int axis) const
{
    if(impl->extJoystick){
        return impl->extJoystick->getPosition(axis);
    }

    if(impl->model == JoystickImpl::OTHER || axis >= JoystickImpl::STANDARD_AXIS_NUM){
      return getNativePosition(axis);
    }

    if(impl->axis_map[axis]==-1){
      return impl->readDirectionPadAsAxis(axis);
    }else{
      return impl->axes[impl->axis_map[axis]];
    }
    return 0.0;
}


bool Joystick::getButtonState(int button) const
{
    if(impl->extJoystick){
        return impl->extJoystick->getButtonState(button);
    }

    if(impl->model == JoystickImpl::OTHER || button >= JoystickImpl::STANDARD_BUTTON_NUM){
      return getNativeButtonState(button);
    }

    if(impl->button_map[button]!=-1){
      return impl->buttons[impl->button_map[button]];
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
