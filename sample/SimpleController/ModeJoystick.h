#include <cnoid/Referenced>
#include <cnoid/Joystick>

namespace cnoid {

class ModeJoystick : public Referenced
{
public:
    static const Joystick::ButtonID MODE_BUTTON = Joystick::LOGO_BUTTON;

    ModeJoystick(){
        mode_ = 0;
        numModes_ = 0;
        prevButtonState = false;
    }
    
    int addMode(){
        int id = numModes_++;
        return id;
    }
        
    void updateState(int targetMode){
        if(targetMode == 0){
            joystick_.readCurrentState();
            bool state = joystick_.getButtonState(MODE_BUTTON);
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
        return joystick_.getPosition(axis, threshold);
    }

    double getPosition(int targetMode, int axis, double threshold = 0.0) const {
        return isMode(targetMode) ? joystick_.getPosition(axis, threshold) : joystick_.getDefaultPosition(axis);
    }
    
    bool getButtonState(int button) const {
        return joystick_.getButtonState(button);
    }

    bool getButtonState(int targetMode, int button) const {
        return isMode(targetMode) ? joystick_.getButtonState(button) : false;
    }

    Joystick* joystick() { return &joystick_; }

private:
    Joystick joystick_;
    int mode_;
    int numModes_;
    bool prevButtonState;
};

typedef ref_ptr<ModeJoystick> ModeJoystickPtr;

}
