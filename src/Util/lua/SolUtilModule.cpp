/**
   @author Shin'ichiro Nakaoka
*/

#include "LuaUtil.h"
#include <cmath>
#include <iostream>
#include "exportdecl.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

void exportLuaSignalTypes(sol::table& module);
void exportLuaTaskTypes(sol::table& module);

}

extern "C" CNOID_EXPORT int luaopen_cnoid_Util(lua_State* L)
{
    sol::state_view lua(L);

    sol::table module = lua.create_table();
    
    module["sin"] = [](double x){ return sin(x); };
    module["call"] = [](sol::function func){ func(); };

    exportLuaSignalTypes(module);
    exportLuaTaskTypes(module);

    sol::stack::push(L, module);
    
    return 1;
}
