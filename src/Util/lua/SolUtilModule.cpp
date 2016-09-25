/**
   @author Shin'ichiro Nakaoka
*/

#include "LuaUtil.h"
#include <cmath>
#include <iostream>

using namespace std;
using namespace cnoid;

namespace cnoid {

void exportLuaSignalTypes(sol::table& module);

}

extern "C" int luaopen_cnoid_Util(lua_State* L)
{
    sol::state_view lua(L);

    sol::table module = lua.create_table();
    
    module["sin"] = [](double x){ return sin(x); };
    module["call"] = [](sol::function func){ func(); };

    exportLuaSignalTypes(module);

    sol::stack::push(L, module);
    
    return 1;
}
