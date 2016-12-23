/*!
  @author Shin'ichiro Nakaoka
*/

#include "ExtJoystick.h"
#include <map>
#include <functional>

using namespace std;
using namespace cnoid;

namespace {

typedef map<string, ExtJoystick*> JoystickMap;
JoystickMap joystickMap;

void onExtJoystickDestroyed(JoystickMap::iterator iter, ExtJoystick* joystick)
{
    if(iter->second == joystick){
        joystickMap.erase(iter);
    }
}

}


void ExtJoystick::registerJoystick(const std::string& name, ExtJoystick* joystick)
{
    pair<JoystickMap::iterator, bool> result =
        joystickMap.insert(JoystickMap::value_type(name, joystick));
    if(!result.second){
        // overwrite a pre-registered object
        result.first->second = joystick;
    }
    joystick->sigDestroyed().connect(std::bind(onExtJoystickDestroyed, result.first, joystick));
}


ExtJoystick* ExtJoystick::findJoystick(const std::string& name)
{
    if(!joystickMap.empty()){
        if(name == "*"){
            return joystickMap.begin()->second;
        }
        JoystickMap::iterator p = joystickMap.find(name);
        if(p != joystickMap.end()){
            return p->second;
        }
    }
    return 0;
}


ExtJoystick::~ExtJoystick()
{
    sigDestroyed_();
}
