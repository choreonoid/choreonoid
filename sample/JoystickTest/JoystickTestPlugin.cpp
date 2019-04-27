/*!
  @author Shin'ichiro Nakaoka
*/

#include <cnoid/Plugin>
#include <cnoid/JoystickCapture>
#include <cnoid/MessageView>
#include <fmt/format.h>

using namespace cnoid;

namespace {

class JoystickTestPlugin : public Plugin
{
    JoystickCapture joystick;
    MessageView* mv;
    
public:
    JoystickTestPlugin() : Plugin("JoystickTest") { }
    
    virtual bool initialize() {

        mv = MessageView::instance();

        joystick.setDevice("/dev/input/js0");

        joystick.sigButton().connect(
            [&](int id, bool isPressed){
                mv->putln(fmt::format("Joystick button {0}: {1}", id, isPressed));
            });

        joystick.sigAxis().connect(
            [&](int id, double position){
                mv->putln(fmt::format("Joystick axis {0}: {1}", id, position));
            });
            
        return true;
    }
};

}

CNOID_IMPLEMENT_PLUGIN_ENTRY(JoystickTestPlugin);
