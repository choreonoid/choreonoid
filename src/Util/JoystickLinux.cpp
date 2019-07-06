/*!
  @author Shin'ichiro Nakaoka
*/

#include "Joystick.h"
#include "ExtJoystick.h"
#include <cnoid/Config>
#include <cnoid/stdx/filesystem>
#include <fmt/format.h>
#include <linux/joystick.h>
#include <sys/ioctl.h>
#include <string>
#include <vector>
#include <map>
#include <chrono>
#include <cerrno>
#include <cstring>
#include <cmath>
#include <fcntl.h>
#include <unistd.h>
#include <regex>

using namespace std;
using namespace cnoid;
using fmt::format;
namespace filesystem = stdx::filesystem;

namespace {

enum ModelID {
    PS4 = 0, // old hid-sony driver (kernel version 4.9 or earlier)
    PS4v2,   // new hid-sony driver (kernel version 4.10 or later)
    PS3,     // old hid-sony driver (kernel version 4.9 or earlier)
    PS3v2,   // new hid-sony driver (kernel version 4.10 or later)
    XBOX,
    F310,
    UNSUPPORTED,
    NUM_MODELS
};

enum AxisID {
    L_STICK_H_AXIS = Joystick::L_STICK_H_AXIS,
    L_STICK_V_AXIS = Joystick::L_STICK_V_AXIS,
    R_STICK_H_AXIS = Joystick::R_STICK_H_AXIS,
    R_STICK_V_AXIS = Joystick::R_STICK_V_AXIS,
    DIRECTIONAL_PAD_H_AXIS = Joystick::DIRECTIONAL_PAD_H_AXIS,
    DIRECTIONAL_PAD_V_AXIS = Joystick::DIRECTIONAL_PAD_V_AXIS,
    L_TRIGGER_AXIS = Joystick::L_TRIGGER_AXIS,
    R_TRIGGER_AXIS = Joystick::R_TRIGGER_AXIS,
    NUM_AXES
};

const int INVALID_AXIS = -1;

enum ButtonID {
    A_BUTTON = Joystick::A_BUTTON,
    B_BUTTON = Joystick::B_BUTTON,
    X_BUTTON = Joystick::X_BUTTON,
    Y_BUTTON = Joystick::Y_BUTTON,
    L_BUTTON = Joystick::L_BUTTON,
    R_BUTTON = Joystick::R_BUTTON,
    SELECT_BUTTON = Joystick::SELECT_BUTTON,
    START_BUTTON = Joystick::START_BUTTON,
    L_STICK_BUTTON = Joystick::L_STICK_BUTTON,
    R_STICK_BUTTON = Joystick::R_STICK_BUTTON,
    LOGO_BUTTON = Joystick::LOGO_BUTTON,
    NUM_BUTTONS = Joystick::NUM_STD_BUTTONS,

    DIRECTIONAL_PAD_LEFT_BUTTON = 200,
    DIRECTIONAL_PAD_RIGHT_BUTTON = 201,
    DIRECTIONAL_PAD_UP_BUTTON = 202,
    DIRECTIONAL_PAD_DOWN_BUTTON = 203,
    
};

const int INVALID_BUTTON = -1;


const int PS4_Axes[] = {
    L_STICK_H_AXIS, L_STICK_V_AXIS, R_STICK_H_AXIS, L_TRIGGER_AXIS, R_TRIGGER_AXIS, R_STICK_V_AXIS,
    DIRECTIONAL_PAD_H_AXIS, DIRECTIONAL_PAD_V_AXIS, INVALID_AXIS, INVALID_AXIS, INVALID_AXIS, INVALID_AXIS };

const int PS4_Buttons[] = {
    X_BUTTON, A_BUTTON, B_BUTTON, Y_BUTTON, L_BUTTON, R_BUTTON, INVALID_BUTTON, INVALID_BUTTON,
    SELECT_BUTTON, START_BUTTON, L_STICK_BUTTON, R_STICK_BUTTON, LOGO_BUTTON, INVALID_BUTTON };

const int PS4v2_Axes[] = {
    L_STICK_H_AXIS,  L_STICK_V_AXIS, L_TRIGGER_AXIS, R_STICK_H_AXIS, R_STICK_V_AXIS, R_TRIGGER_AXIS,
    DIRECTIONAL_PAD_H_AXIS, DIRECTIONAL_PAD_V_AXIS, INVALID_AXIS, INVALID_AXIS, INVALID_AXIS, INVALID_AXIS };

const int PS4v2_Buttons[] = {
    A_BUTTON, B_BUTTON, Y_BUTTON, X_BUTTON, L_BUTTON, R_BUTTON, INVALID_BUTTON, INVALID_BUTTON,
    SELECT_BUTTON, START_BUTTON, LOGO_BUTTON, L_STICK_BUTTON, R_STICK_BUTTON };

const int PS3_Axes[] = {
    L_STICK_H_AXIS, L_STICK_V_AXIS, R_STICK_H_AXIS, R_STICK_V_AXIS,
    // 4 - 11
    INVALID_AXIS, INVALID_AXIS, INVALID_AXIS, INVALID_AXIS, INVALID_AXIS, INVALID_AXIS, INVALID_AXIS, INVALID_AXIS,
    L_TRIGGER_AXIS, R_TRIGGER_AXIS,
    // 14 - 26
    INVALID_AXIS, INVALID_AXIS, INVALID_AXIS, INVALID_AXIS, INVALID_AXIS, INVALID_AXIS, INVALID_AXIS,
    INVALID_AXIS, INVALID_AXIS, INVALID_AXIS, INVALID_AXIS, INVALID_AXIS, INVALID_AXIS };

const int PS3_Buttons[] = {
    SELECT_BUTTON, L_STICK_BUTTON, R_STICK_BUTTON, START_BUTTON,
    DIRECTIONAL_PAD_UP_BUTTON, DIRECTIONAL_PAD_RIGHT_BUTTON, DIRECTIONAL_PAD_DOWN_BUTTON, DIRECTIONAL_PAD_LEFT_BUTTON,
    INVALID_BUTTON, INVALID_BUTTON, L_BUTTON, R_BUTTON,
    Y_BUTTON, B_BUTTON, A_BUTTON, X_BUTTON,
    LOGO_BUTTON, INVALID_BUTTON, INVALID_BUTTON };
    
const int PS3v2_Axes[] = {
    L_STICK_H_AXIS,  L_STICK_V_AXIS, L_TRIGGER_AXIS, R_STICK_H_AXIS, R_STICK_V_AXIS, R_TRIGGER_AXIS };

const int PS3v2_Buttons[] = {
    A_BUTTON, B_BUTTON, Y_BUTTON, X_BUTTON, L_BUTTON, R_BUTTON, INVALID_BUTTON, INVALID_BUTTON,
    SELECT_BUTTON, START_BUTTON, LOGO_BUTTON, L_STICK_BUTTON, R_STICK_BUTTON,
    DIRECTIONAL_PAD_UP_BUTTON, DIRECTIONAL_PAD_DOWN_BUTTON, DIRECTIONAL_PAD_LEFT_BUTTON, DIRECTIONAL_PAD_RIGHT_BUTTON };

const int XBOX_Axes[] = {
    L_STICK_H_AXIS, L_STICK_V_AXIS, L_TRIGGER_AXIS, R_STICK_H_AXIS, R_STICK_V_AXIS, R_TRIGGER_AXIS,
    DIRECTIONAL_PAD_H_AXIS, DIRECTIONAL_PAD_V_AXIS };

const int XBOX_Buttons[] = {
    A_BUTTON, B_BUTTON, X_BUTTON, Y_BUTTON, L_BUTTON, R_BUTTON, SELECT_BUTTON, START_BUTTON, LOGO_BUTTON,
    L_STICK_BUTTON, R_STICK_BUTTON };

const int F310_Axes[] = {
    L_STICK_H_AXIS, L_STICK_V_AXIS, L_TRIGGER_AXIS, R_STICK_H_AXIS, R_STICK_V_AXIS, R_TRIGGER_AXIS,
    DIRECTIONAL_PAD_H_AXIS, DIRECTIONAL_PAD_V_AXIS };
    
const int F310_Buttons[] = {
    A_BUTTON, B_BUTTON, X_BUTTON, Y_BUTTON, L_BUTTON, R_BUTTON, SELECT_BUTTON, START_BUTTON, LOGO_BUTTON,
    L_STICK_BUTTON, R_STICK_BUTTON };
    
struct ModelInfo {
    ModelID id;
    const int* axisMap;
    const int* buttonMap;

    /*
      Analog stick input of PS4 gamepad is -1.0 before the first operation.
      The following flag is used To correct it  to the neutral position.
    */
    bool doIgnoreInitialState;
};

const map<string, ModelID> modelIdMap = {
    { "Sony Computer Entertainment Wireless Controller", PS4 },
    { "Sony Interactive Entertainment Wireless Controller", PS4 },
    { "Sony Interactive Entertainment DUALSHOCK®4 USB Wireless Adaptor", PS4 },
    { "Sony PLAYSTATION(R)3 Controller", PS3 },
    { "Microsoft X-Box 360 pad", XBOX },
    { "Microsoft X-Box One pad", XBOX },
    { "Logitech Gamepad F310", F310 }
};

const vector<ModelInfo> modelInfos = {
    { PS4,         PS4_Axes,   PS4_Buttons,   true  },
    { PS4v2,       PS4v2_Axes, PS4v2_Buttons, true  },
    { PS3,         PS3_Axes,   PS3_Buttons,   true  },
    { PS3v2,       PS3v2_Axes, PS3v2_Buttons, true  },
    { XBOX,        XBOX_Axes,  XBOX_Buttons,  false },
    { F310,        F310_Axes,  F310_Buttons,  false },
    { UNSUPPORTED, nullptr,    nullptr,       false }
};

}

namespace cnoid {

class JoystickImpl
{
public:
    Joystick* self;
    ExtJoystick* extJoystick;
    int fd;
    ModelInfo currentModel;
    
    vector<double> axes;
    vector<bool> axisEnabled;
    Signal<void(int id, double position)> sigAxis;
    
    vector<bool> buttons;
    vector<bool> prevButtons;
    vector<chrono::system_clock::time_point> buttonDownTime;
    vector<bool> buttonHoldValid;
    Signal<void(int id, bool isPressed)> sigButton;

    vector<bool> record_init_pos;
    vector<double> initial_pos;
    vector<bool> initialized;

    string errorMessage;

    JoystickImpl(Joystick* self, const string& device);
    ~JoystickImpl();
    bool findDevice(const string& device);
    bool openDevice(const string& device);
    bool setupDevice();
    void closeDevice();
    bool readCurrentState();
    bool readEvent();
    void setAxisState(int id, double pos);
};

}


Joystick::Joystick()
{
    impl = new JoystickImpl(this, "");
}


Joystick::Joystick(const char* device)
{
    impl = new JoystickImpl(this, device);
}


JoystickImpl::JoystickImpl(Joystick* self, const string& device)
    : self(self)
{
    fd = -1;

    extJoystick = ExtJoystick::findJoystick(device);

    if(!extJoystick){
        if(!findDevice(device)){
            extJoystick = ExtJoystick::findJoystick("*");
        }
    }
}


bool JoystickImpl::findDevice(const string& device)
{
    bool found = false;
    
    if(!device.empty() && device != "*"){
        found = openDevice(device);

    } else {
        closeDevice();
        
        string filebase("/dev/input/js{}");
        regex sonyMotionSensors("^Sony.*Motion Sensors$");
        int id = 0;
        while(true){
            string file = format(filebase, id);
            if(!filesystem::exists(filesystem::path(file))){
                break;
            }
            int tmpfd = open(file.c_str(), O_RDONLY | O_NONBLOCK);
            if(tmpfd < 0){
                break;
            }

            // check the josytick model name
            char buf[1024];
            if(ioctl(tmpfd, JSIOCGNAME(sizeof(buf)), buf) < 0){
                close(tmpfd);
                break;
            }
            string identifier(buf);
            if(regex_match(identifier, sonyMotionSensors)){
                // skip the device corresponding to the motion sensors of a Sony Dualshock gamepad
                ++id;
                continue;
            }

            fd = tmpfd;
            found = setupDevice();
            break;
        }
    }

    return found;
}
        

bool JoystickImpl::openDevice(const string& device)
{
    closeDevice();

    fd = open(device.c_str(), O_RDONLY | O_NONBLOCK);

    if(fd < 0){
        errorMessage = format("Device \"{0}\": {1}", device, strerror(errno));
        return false;
    }
    errorMessage.clear();

    return setupDevice();
}

bool JoystickImpl::setupDevice()
{
    char identifier[1024];
    ioctl(fd, JSIOCGNAME(sizeof(identifier)), identifier);

    char numAxes;
    ioctl(fd, JSIOCGAXES, &numAxes);

    char numButtons;
    ioctl(fd, JSIOCGBUTTONS, &numButtons);

    ModelID id = UNSUPPORTED;
    auto iter = modelIdMap.find(identifier);
    if(iter != modelIdMap.end()){
        id = iter->second;
    }
    if(id == PS4){
        if(numButtons == 13){
            id = PS4v2;
        }
    } else if(id == PS3){
        if(numButtons == 17){
            id = PS3v2;
        }
    }

    currentModel = modelInfos[id];

    if(id != UNSUPPORTED){
        numAxes = NUM_AXES;
        numButtons = NUM_BUTTONS;
    }
    axes.resize(numAxes, 0.0);
    axisEnabled.resize(numAxes, true);
    buttons.resize(numButtons, false);
    prevButtons.resize(numButtons, false);
    buttonDownTime.resize(numButtons);
    buttonHoldValid.resize(numButtons, false);

    if(currentModel.doIgnoreInitialState){
        record_init_pos.assign(axisEnabled.size(), false);
        initial_pos.assign(axisEnabled.size(), 0.0);
        initialized.assign(axisEnabled.size(), false);
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
        if(axis < static_cast<int>(impl->axes.size())){
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
    prevButtons = buttons;
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

    int id = event.number;
    double pos = (double)event.value / MAX_VALUE_16BIT;

    if(event.type & JS_EVENT_BUTTON) { // button

        if(currentModel.id != UNSUPPORTED){
            id = currentModel.buttonMap[id];
        }
        if(id != INVALID_BUTTON){
            bool isPressed = (pos > 0.0);

            if(id < DIRECTIONAL_PAD_LEFT_BUTTON){
                buttons[id] = isPressed;
                sigButton(id, isPressed);
                if(isPressed){
                    buttonDownTime[id] = chrono::system_clock::now();
                    buttonHoldValid[id] = false;
                }
            } else {
                double p = isPressed ? 1.0 : 0.0;
                switch(id){
                case DIRECTIONAL_PAD_LEFT_BUTTON:
                    setAxisState(DIRECTIONAL_PAD_H_AXIS, -p);
                    break;
                case DIRECTIONAL_PAD_RIGHT_BUTTON:
                    setAxisState(DIRECTIONAL_PAD_H_AXIS, p);
                    break;
                case DIRECTIONAL_PAD_UP_BUTTON:
                    setAxisState(DIRECTIONAL_PAD_V_AXIS, -p);
                    break;
                case DIRECTIONAL_PAD_DOWN_BUTTON:
                    setAxisState(DIRECTIONAL_PAD_V_AXIS, p);
                    break;
                default:
                    break;
                }
            }
        }
    } else if(event.type & JS_EVENT_AXIS){ // axis
        if(currentModel.id != UNSUPPORTED){
            id = currentModel.axisMap[id];
            if(id == L_TRIGGER_AXIS || id == R_TRIGGER_AXIS){
                pos = (pos + 1.0) / 2.0;
            }
        }
        if(id != INVALID_AXIS && axisEnabled[id]){
            // normalize value (-1.0 to 1.0)
            //pos = nearbyint(pos * 10.0) / 10.0;

            if(currentModel.doIgnoreInitialState){
                if(!record_init_pos[id]){
                    initial_pos[id] = pos;
                    pos = 0.0;
                    record_init_pos[id] = true;
                    initialized[id] = false;
                } else {
                    if(!initialized[id]){
                        if(pos == initial_pos[id]){
                            pos = 0.0;
                        } else {
                            initialized[id] = true; // analog stick input changed
                        }
                    }
                }
            }

            setAxisState(id, pos);
        }
    }
    return true;
}


void JoystickImpl::setAxisState(int id, double pos)
{
    double prevPos = axes[id];
    if(pos != prevPos){
        axes[id] = pos;
        sigAxis(id, pos);
    }
}
    

double Joystick::getPosition(int axis) const
{
    double pos = 0.0;
    
    if(impl->extJoystick){
        pos = impl->extJoystick->getPosition(axis);
    } else if(axis < (int)impl->axes.size()){
        pos = impl->axes[axis];
    }

    return pos;
}


bool Joystick::getButtonState(int button) const
{
    bool state = false;
    
    if(impl->extJoystick){
        state = impl->extJoystick->getButtonState(button);
    } else if(button < (int)impl->buttons.size()){
        state = impl->buttons[button];
    }

    return state;
}


bool Joystick::getButtonDown(int button) const
{
    if(button >= (int)impl->buttons.size()){
        return false;
    }
    return getButtonState(button) && !impl->prevButtons[button];
}


bool Joystick::getButtonUp(int button) const
{
    if(button >= (int)impl->buttons.size()){
        return false;
    }
    return !getButtonState(button) && impl->prevButtons[button];
}


bool Joystick::getButtonHold(int button, int duration/*(msec)*/) const
{
    if(impl->buttonHoldValid[button]){
        return false;
    }
    if(getButtonHoldOn(button, duration)){
        impl->buttonHoldValid[button] = true;
        return true;
    }
    return false;
}


bool Joystick::getButtonHoldOn(int button, int duration/*(msec)*/) const
{
    if(button >= (int)impl->buttons.size() || !getButtonState(button)){
        return false;
    }
    auto dur = chrono::system_clock::now() - impl->buttonDownTime[button];
    if(chrono::duration_cast<chrono::milliseconds>(dur).count() > duration) return true;
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
