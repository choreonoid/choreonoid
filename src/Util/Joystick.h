/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_JOYSTICK_H
#define CNOID_UTIL_JOYSTICK_H

#include "JoystickInterface.h"
#include "Signal.h"
#include <string>
#include "exportdecl.h"

namespace cnoid {

class JoystickImpl;

class CNOID_EXPORT Joystick : public JoystickInterface
{
public:
    Joystick();
    Joystick(const char* device);
    virtual ~Joystick();

    std::string device() const;
    int fileDescriptor() const;

    bool makeReady();
    bool isReady() const;
    const char* errorMessage() const;

    enum AxisID {
        L_STICK_H_AXIS,
        L_STICK_V_AXIS,
        R_STICK_H_AXIS,
        R_STICK_V_AXIS,
        DIRECTIONAL_PAD_H_AXIS,
        DIRECTIONAL_PAD_V_AXIS,
        L_TRIGGER_AXIS,
        R_TRIGGER_AXIS,
        NUM_STD_AXES
    };

    enum ButtonID {
        A_BUTTON, // Cross
        B_BUTTON, // Circle
        X_BUTTON, // Square
        Y_BUTTON, // Triangle
        L_BUTTON,
        R_BUTTON,
        SELECT_BUTTON,
        START_BUTTON,
        L_STICK_BUTTON,
        R_STICK_BUTTON,
        LOGO_BUTTON,
        NUM_STD_BUTTONS
    };

    virtual int numAxes() const override;
    virtual int numButtons() const override;
    virtual bool readCurrentState() override;
    virtual double getPosition(int axis) const override;
    virtual bool getButtonState(int button) const override;
    
    void setAxisEnabled(int axis, bool on);

    //! \deprecated
    //double getDefaultPosition(int axis) const;

#ifdef __linux__
    bool getButtonDown(int button) const;
    bool getButtonUp(int button) const;
    bool getButtonHold(int button, int duration/*(msec)*/) const;
    bool getButtonHoldOn(int button, int duration/*(msec)*/) const;
#endif
    
    bool isActive() const;
    SignalProxy<void(int id, bool isPressed)> sigButton();
    SignalProxy<void(int id, double position)> sigAxis();

private:
    JoystickImpl* impl;
    friend class JoystickImpl;
};

}

#endif
