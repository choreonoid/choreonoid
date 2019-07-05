/**
   @author Shizuko Hattori
   @author Shin'ichiro Nakaoka

   If you have more than one joystick connected, please do the following.
   1.Launch the control panel and open the device and printer window.
   2.Right-click on one of the displayed controllers and 
   open the game controller setting.
   3.Click the detailed setting, and in the selection of the priority device, 
   select the device to use.

   If you use XBOX_ONE, please do not connect other joystick.
   The XInput function used for XBOX_ONE is because the identification number
   of the joystick can not be specified..
   When installing XInput Wrapper for DS3 (PCSX 2) to connect PS3 joystick,
   XBOX_ONE may not be detected because the virtual XBOX 360 joystick always exists.
   In this case, uninstall XInput Wrapper for DS 3 (PCSX 2).
*/

#include "Joystick.h"
#include "ExtJoystick.h"
#include <windows.h>
#include <mmsystem.h>
#include <vector>
#include <string>
#include <map>

#ifdef USE_XINPUT
#include <XInput.h>
#include <tchar.h>
#define XINPUT_GAMEPAD_GUIDE 0x400
typedef DWORD(WINAPI *XInputGetStateExFunction)(__in DWORD dwUserIndex, __out XINPUT_STATE* pState);
#endif

using namespace std;
using namespace cnoid;

namespace {

    enum ModelID {
        PS4 = 0, 
      //PS4v2,
      //PS3,   //   If the PS3 controller is connected using "XInput Wrapper for DS3(PCSX2)",
               //   the PS3 is the same as the XBOX_360.
      //PS3v2,   
        XBOX_360,
        XBOX_ONE,
        F310,
        F710,
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
        L_STICK_H_AXIS, L_STICK_V_AXIS, R_STICK_H_AXIS, R_STICK_V_AXIS, R_TRIGGER_AXIS, L_TRIGGER_AXIS,
        DIRECTIONAL_PAD_H_AXIS, DIRECTIONAL_PAD_V_AXIS };

    const int PS4_Buttons[] = {
        X_BUTTON, A_BUTTON, B_BUTTON, Y_BUTTON, L_BUTTON, R_BUTTON, INVALID_BUTTON, INVALID_BUTTON,
        SELECT_BUTTON, START_BUTTON, L_STICK_BUTTON, R_STICK_BUTTON, LOGO_BUTTON, INVALID_BUTTON };

    const int XBOX_360_Axes[] = {
        L_STICK_H_AXIS, L_STICK_V_AXIS, L_TRIGGER_AXIS, R_STICK_V_AXIS, R_STICK_H_AXIS, INVALID_AXIS,
        DIRECTIONAL_PAD_H_AXIS, DIRECTIONAL_PAD_V_AXIS };

    const int XBOX_360_Buttons[] = {
        A_BUTTON, B_BUTTON, X_BUTTON, Y_BUTTON, L_BUTTON, R_BUTTON, SELECT_BUTTON, START_BUTTON,
        L_STICK_BUTTON, R_STICK_BUTTON, INVALID_BUTTON };

    const int F310_Axes[] = {
        L_STICK_H_AXIS, L_STICK_V_AXIS, L_TRIGGER_AXIS, R_STICK_V_AXIS, R_STICK_H_AXIS, INVALID_AXIS,
        DIRECTIONAL_PAD_H_AXIS, DIRECTIONAL_PAD_V_AXIS };

    const int F310_Buttons[] = {
        A_BUTTON, B_BUTTON, X_BUTTON, Y_BUTTON, L_BUTTON, R_BUTTON, SELECT_BUTTON, START_BUTTON,
        L_STICK_BUTTON, R_STICK_BUTTON, INVALID_BUTTON };

    const int F710_Axes[] = {    // mode button OFF
        L_STICK_H_AXIS, L_STICK_V_AXIS, L_TRIGGER_AXIS, R_STICK_V_AXIS, R_STICK_H_AXIS, INVALID_AXIS,
        DIRECTIONAL_PAD_H_AXIS, DIRECTIONAL_PAD_V_AXIS };

    const int F710_Buttons[] = {
        A_BUTTON, B_BUTTON, X_BUTTON, Y_BUTTON, L_BUTTON, R_BUTTON, SELECT_BUTTON, START_BUTTON,
        L_STICK_BUTTON, R_STICK_BUTTON, INVALID_BUTTON };

    struct ModelInfo {
        ModelID id;
        const int* axisMap;
        const int* buttonMap;
    };

    const map<int, ModelID> modelIdMap = {
        { 0x054C<<16 | 0x09CC, PS4 },  // Sony Interactive Entertainment
        { 0x054C<<16 | 0x05C4, PS4 },  // Sony Computer Entertainment
        { 0x045E<<16 | 0x028E, XBOX_360 },
        { 0x045E<<16 | 0x02FF, XBOX_ONE },
        { 0x046D<<16 | 0xC21D, F310 },
        { 0x046D<<16 | 0xC21F, F710 }
    };

    map<ModelID, ModelInfo> modelInfos = {
        { PS4, { PS4,    PS4_Axes,   PS4_Buttons } },
        { XBOX_360, { XBOX_360,  XBOX_360_Axes,  XBOX_360_Buttons } },
        { XBOX_ONE,{ XBOX_ONE,  nullptr,  nullptr } },
        { F310, { F310,  F310_Axes,  F310_Buttons } }, 
        { F710, { F710,  F710_Axes,  F710_Buttons } },
        { UNSUPPORTED, { UNSUPPORTED, nullptr,    nullptr } }
    };


#ifdef USE_XINPUT
    static HMODULE xInputDll;
    static FARPROC xInputFunc;
#endif

}

namespace cnoid {

class JoystickImpl
{
public:
    Joystick* self;
    ExtJoystick* extJoystick;
    ModelInfo currentModel;
    int id;

    int numAvailableAxes;
    int numAvailableButtons;
    vector<double> axes;
    vector<bool> axisEnabled;
    vector<bool> buttons;
    string errorMessage;
    bool hasPOV;
    Signal<void(int id, bool isPressed)> sigButton;
    Signal<void(int id, double position)> sigAxis;

    JoystickImpl(const string& device);
    bool openDevice();
    bool readCurrentState();
    bool readCurrentState_joyGet();
    bool readCurrentState_xinput();
    void setJoystickModel(const int productId);
    void readLogoButton();

};

}


Joystick::Joystick()
{
    impl = new JoystickImpl("");
}


Joystick::Joystick(const char* device)
{
    impl = new JoystickImpl(device);
}


JoystickImpl::JoystickImpl(const string& device)
{
    id = -1;
    extJoystick = 0;

    extJoystick = ExtJoystick::findJoystick(device);

    if (!extJoystick) {
        if (!openDevice()){
            extJoystick = ExtJoystick::findJoystick("*");
        }
    }
}

bool JoystickImpl::openDevice()
{
    numAvailableAxes = 0;
    
    std::vector<UINT> devIds;
    JOYINFOEX info;

    info.dwSize  = sizeof(JOYINFOEX);
    info.dwFlags = JOY_RETURNALL;

    for(int i=0; i<joyGetNumDevs(); i++){
        MMRESULT result = joyGetPosEx(i, &info);
        if(result == JOYERR_PARMS){
            break;
        }
        if(result == JOYERR_NOERROR){
            devIds.push_back(i);
        }
    }

    if(!devIds.empty()){
        // Pick up the first device
        id = devIds.front();

        JOYCAPS caps;
        joyGetDevCaps(id, &caps, sizeof(caps));
        numAvailableAxes = caps.wNumAxes;
        numAvailableButtons = caps.wNumButtons;
        setJoystickModel(caps.wMid<<16 | caps.wPid);

        if ((caps.wCaps & JOYCAPS_POV4DIR) == JOYCAPS_POV4DIR){
            hasPOV = true;
        } else {
            hasPOV = false;
        }

        if(currentModel.id != UNSUPPORTED){
            axes.resize(NUM_AXES, 0.0);
            axisEnabled.resize(NUM_AXES, true);
            buttons.resize(NUM_BUTTONS, false);
        }else{
            if(hasPOV){
                axes.resize(numAvailableAxes+2, 0.0);
                axisEnabled.resize(numAvailableAxes+2, true);
            }else{
                axes.resize(numAvailableAxes, 0.0);
                axisEnabled.resize(numAvailableAxes, true);
            }
            buttons.resize(numAvailableButtons, false);
        }

        return readCurrentState();

    }

    return false;
}


void JoystickImpl::setJoystickModel(const int productId)
{
    ModelID modelId = UNSUPPORTED;
    auto iter = modelIdMap.find(productId);
    if (iter != modelIdMap.end()) {
        modelId = iter->second;
    }

    currentModel = modelInfos[modelId];

#ifdef USE_XINPUT
    if (modelId == XBOX_ONE) {
        for(int i=0; i<4; i++){
            XINPUT_STATE state;
            ZeroMemory(&state, sizeof(XINPUT_STATE));
            if (XInputGetState(i, &state) == ERROR_SUCCESS) {
                if( state.dwPacketNumber) {
                    id = i;
                    break;
                }
            }
        }
    }

    xInputDll = NULL;
    xInputFunc = NULL;
    TCHAR libpath[MAX_PATH];
    GetSystemDirectory(libpath, MAX_PATH); 
    _stprintf_s(libpath, _T("%s\\XInput1_4.dll"), libpath);
    if ((xInputDll = LoadLibrary(libpath)) != NULL)
    {
        xInputFunc = GetProcAddress(xInputDll, (LPCSTR)100); // XInputGetStateEx
    }
#endif

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
    return impl->extJoystick ? impl->extJoystick->numAxes() : impl->axes.size();
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
    
    if(currentModel.id != PS4 && currentModel.id != UNSUPPORTED){
        readLogoButton();
    }

    if (currentModel.id == XBOX_ONE) {
        return readCurrentState_xinput();
    }

    return readCurrentState_joyGet();
}
    
 
bool JoystickImpl::readCurrentState_joyGet()
{
    const float MAX_VALUE_16BIT = 32767.0f;

    JOYINFOEX info;
    info.dwSize  = sizeof(info);
    info.dwFlags = JOY_RETURNALL;

    if(joyGetPosEx(id, &info) != JOYERR_NOERROR) {
        return false;

    } else {
        // buttons
        for(size_t i=0; i < numAvailableButtons; ++i){
            int buttonId;
            if(currentModel.id==UNSUPPORTED){
                buttonId = i;
            }else{
                buttonId = currentModel.buttonMap[i];
            }
            if (buttonId != INVALID_BUTTON) {
                if(info.dwButtons & JOY_BUTTON1){
                    buttons[buttonId] = true;
                } else {
                    buttons[buttonId] = false;
                }
            }
            info.dwButtons >>= 1;
        }

        for (int i = 0; i < numAvailableAxes; i++) {
            int axisId;
            if (currentModel.id == UNSUPPORTED) {
                axisId = i;
            }
            else {
                axisId = currentModel.axisMap[i];
            }
            if ( axisId != INVALID_AXIS && axisEnabled[axisId] ) {
                if( i<6 ){
                    double value;
                    switch(i){
                    case 0: value = info.dwXpos; break;
                    case 1: value = info.dwYpos; break;
                    case 2: value = info.dwZpos; break;
                    case 3: value = info.dwRpos; break;
                    case 4: value = info.dwUpos; break;
                    case 5: value = info.dwVpos; break;
                    }

                    double value1 = (value - MAX_VALUE_16BIT) / MAX_VALUE_16BIT;
                    if ( (currentModel.id == F310 || currentModel.id == XBOX_360 || currentModel.id == F710)
                            && axisId == L_TRIGGER_AXIS) {
                        if (value1 < 0) {
                            axes[L_TRIGGER_AXIS] = 0;
                            axes[R_TRIGGER_AXIS] = -value1;
                        }
                        else {
                            axes[L_TRIGGER_AXIS] = value1;
                            axes[R_TRIGGER_AXIS] = 0;
                        }
                    } else {
                        if (axisId == L_TRIGGER_AXIS || axisId == R_TRIGGER_AXIS) {
                            axes[axisId] = (value1 + 1.0) / 2.0;
                        } else {
                            axes[axisId] = value1;
                        }
                    }
                }
            }
        }
            
        if(hasPOV){
            double value_h = info.dwPOV == JOY_POVRIGHT ? 1.0 : info.dwPOV == JOY_POVLEFT ? -1.0 : 0.0;
            double value_v = info.dwPOV == JOY_POVFORWARD ? -1.0 : info.dwPOV == JOY_POVBACKWARD ? 1.0 : 0.0;
            if (currentModel.id == UNSUPPORTED) {
                axes[numAvailableAxes] = value_h;
                axes[numAvailableAxes+1] = value_v;
            } else {
                axes[DIRECTIONAL_PAD_H_AXIS] = value_h;
                axes[DIRECTIONAL_PAD_V_AXIS] = value_v;
            }
        }
        return true;
    }

    return false;
}


bool JoystickImpl::readCurrentState_xinput()
{
#ifdef USE_XINPUT
    const float MAX_VALUE_16BIT = 32767.0f;
    const float MAX_VALUE_8BIT = 255.0f;

    XINPUT_STATE state;
    ZeroMemory(&state, sizeof(XINPUT_STATE));
    if(XInputGetState(id, &state) != ERROR_SUCCESS) {
        return false;
    }
    
    buttons[A_BUTTON] = state.Gamepad.wButtons & XINPUT_GAMEPAD_A;
    buttons[B_BUTTON] = state.Gamepad.wButtons & XINPUT_GAMEPAD_B;
    buttons[X_BUTTON] = state.Gamepad.wButtons & XINPUT_GAMEPAD_X;
    buttons[Y_BUTTON] = state.Gamepad.wButtons & XINPUT_GAMEPAD_Y;
    buttons[L_BUTTON] = state.Gamepad.wButtons & XINPUT_GAMEPAD_LEFT_SHOULDER;
    buttons[R_BUTTON] = state.Gamepad.wButtons & XINPUT_GAMEPAD_RIGHT_SHOULDER;
    buttons[SELECT_BUTTON] = state.Gamepad.wButtons & XINPUT_GAMEPAD_BACK;
    buttons[START_BUTTON] = state.Gamepad.wButtons & XINPUT_GAMEPAD_START;
    buttons[L_STICK_BUTTON] = state.Gamepad.wButtons & XINPUT_GAMEPAD_LEFT_THUMB;
    buttons[R_STICK_BUTTON] = state.Gamepad.wButtons & XINPUT_GAMEPAD_RIGHT_THUMB;

    for (int i = 0; i < 4; i++) {
        if (axisEnabled[i]) {
            double value;
            switch (i) {
            case 0: value = state.Gamepad.sThumbLX; break;
            case 1: value = - state.Gamepad.sThumbLY; break;
            case 2: value = state.Gamepad.sThumbRX; break;
            case 3: value = - state.Gamepad.sThumbRY; break;
            }
            axes[i] = value / MAX_VALUE_16BIT;
        }
    }

    axes[L_TRIGGER_AXIS] = state.Gamepad.bLeftTrigger / MAX_VALUE_8BIT;
    axes[R_TRIGGER_AXIS] = state.Gamepad.bRightTrigger / MAX_VALUE_8BIT;

    axes[DIRECTIONAL_PAD_H_AXIS] = state.Gamepad.wButtons & XINPUT_GAMEPAD_DPAD_RIGHT ? 1.0 : state.Gamepad.wButtons & XINPUT_GAMEPAD_DPAD_LEFT ? -1.0 : 0.0;
    axes[DIRECTIONAL_PAD_V_AXIS] = state.Gamepad.wButtons & XINPUT_GAMEPAD_DPAD_DOWN ? 1.0 : state.Gamepad.wButtons & XINPUT_GAMEPAD_DPAD_UP ? -1.0 : 0.0;
#endif
    return true;
}

void JoystickImpl::readLogoButton()
{
#ifdef USE_XINPUT
    if (xInputFunc) {
        XINPUT_STATE state;
        if (((XInputGetStateExFunction)(xInputFunc))(id, &state) == ERROR_SUCCESS) {
            buttons[LOGO_BUTTON] = state.Gamepad.wButtons & XINPUT_GAMEPAD_GUIDE;
        }
    }
#endif
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

