/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_SHARED_JOYSTICK_H
#define CNOID_UTIL_SHARED_JOYSTICK_H

#include <cnoid/Referenced>
#include <cnoid/Joystick>

namespace cnoid {

class SharedJoystick : public Referenced
{
public:
    //Switch mode with either button
    static const Joystick::ButtonID MODE_BUTTON1 = Joystick::LOGO_BUTTON;
    static const Joystick::ButtonID MODE_BUTTON2 = Joystick::SELECT_BUTTON;

    SharedJoystick(){
        joystick = &defaultJoystick;
        mode_ = 0;
        numModes_ = 0;
        prevButtonState = false;
    }

    void setJoystick(JoystickInterface* joystick){
        this->joystick = joystick;
    }
    
    int addMode(){
        int id = numModes_++;
        return id;
    }
    
    bool getModeButtonState() {
        joystick->readCurrentState();
        return (joystick->getButtonState(MODE_BUTTON1) || joystick->getButtonState(MODE_BUTTON2));
    }

    void updateState(int targetMode){
        if(targetMode == 0){
            bool state = getModeButtonState();
            if(!prevButtonState && state){
                ++mode_;
                if(mode_ >= numModes_){
                    mode_ = 0;
                }
            }
            prevButtonState = state;
        }
    }

    int mode() const {
        return mode_;
    }

    bool isMode(int id) const {
        return (mode_ == id);
    }

    void setMode(int id){
        mode_ = id;
    }

    double getPosition(int axis, double threshold = 0.0) const {
        double pos = joystick->getPosition(axis);
        return (fabs(pos) >= threshold) ? pos : 0.0;
    }

    double getPosition(int targetMode, int axis, double threshold = 0.0) const {
        return isMode(targetMode) ? getPosition(axis, threshold) : 0.0;
    }
    
    bool getButtonState(int button) const {
        return joystick->getButtonState(button);
    }

    bool getButtonState(int targetMode, int button) const {
        return isMode(targetMode) ? joystick->getButtonState(button) : false;
    }

private:
    JoystickInterface* joystick;
    Joystick defaultJoystick;
    int mode_;
    int numModes_;
    bool prevButtonState;
};

typedef ref_ptr<SharedJoystick> SharedJoystickPtr;

}

#endif
