/*!
  @author Shin'ichiro Nakaoka
*/

#include <cnoid/Plugin>
#include <cnoid/JoystickCapture>
#include <cnoid/MessageView>

using namespace std::placeholders;
using namespace cnoid;

namespace {

class JoystickTestPlugin : public Plugin
{
    JoystickCapture joystick;
    MessageView* mv;
    
public:
    JoystickTestPlugin() : Plugin("JoystickTest") { }
    
    virtual bool initialize() {

        joystick.setDevice("/dev/input/js0");

        joystick.sigButton().connect(
            std::bind(&JoystickTestPlugin::onButtonEvent, this, _1, _2));
        joystick.sigAxis().connect(
            std::bind(&JoystickTestPlugin::onAxisEvent, this, _1, _2));

        mv = MessageView::instance();
            
        return true;
    }

    void onButtonEvent(int id, bool isPressed) {
        mv->putln(boost::format("Joystick button %1%: %2%") % id % isPressed);
    }

    void onAxisEvent(int id, double position) {
        mv->putln(boost::format("Joystick axis %1%: %2%") % id % position);
    }
};

}

CNOID_IMPLEMENT_PLUGIN_ENTRY(JoystickTestPlugin);
