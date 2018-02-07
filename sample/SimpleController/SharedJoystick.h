#include <cnoid/Referenced>
#include <cnoid/Joystick>
#include <map>

namespace cnoid {

class SharedJoystick : public Referenced
{
public:
    int addMode(Joystick::ButtonID modeButton){
        ModeInfo& info = modeMap[modeButton];
        int id = info.numModes;
        ++info.numModes;
        return id;
    }
        
    int currentMode(Joystick::ButtonID modeButton){
        return modeMap[modeButton].currentMode;
    }

    void updateState(){
        joystick_.readCurrentState();
        for(auto& kv : modeMap){
            int buttonID = kv.first;
            ModeInfo& info = kv.second;
            if(joystick_.getButtonState(buttonID)){
                ++info.currentMode;
                if(info.currentMode >= info.numModes){
                    info.currentMode = 0;
                }
            }
        }
    }

    double getPosition(int axis, double threshold = 0.0) const {
        return joystick_.getPosition(axis, threshold);
    }

    bool getButtonState(int button) const {
        return joystick_.getButtonState(button);
    }

    Joystick* joystick() { return &joystick_; }

private:
    Joystick joystick_;

    struct ModeInfo {
        int currentMode;
        int numModes;
        ModeInfo(){
            currentMode = 0;
            numModes = 0;
        }
    };

    std::map<Joystick::ButtonID, ModeInfo> modeMap;
};

typedef ref_ptr<SharedJoystick> SharedJoystickPtr;

}
