/**
   @author Shin'ichiro Nakaoka
*/

#include "LuaUtil.h"
#include "../Sleep.h"
#include "exportdecl.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

void exportLuaSignalTypes(sol::table& module);
void exportLuaEigenTypes(sol::table& module);
void exportLuaValueTree(lua_State* L, sol::table& module);
void exportLuaTaskTypes(sol::table& module);

}

extern "C" CNOID_EXPORT int luaopen_cnoid_Util(lua_State* L)
{
    sol::state_view lua(L);

    sol::table module = lua.create_table();

    /**
       \note The lua.script() function cannot be used here because it has a bug
       in which the return value cannot be obtained if the stack is not empty
       when the function is called.
    */
    sol::stack::script(
        L,
        "return function(baseClass) "
        "  local class = { } "
        "  class.new = function(...) "
        "    local obj = { } "
        "    baseobj = baseClass.new(...) "
        "    if type(baseobj) == \"table\" then "
        "      obj.cppobj = baseobj.cppobj "
        "    else "
        "      obj.cppobj = baseobj "
        "    end "
        "    obj.super = baseobj "
        "    setmetatable(obj, { __index = class }) "
        "    obj.cppobj:setDerivedLuaObject(obj) "
        "    if class.initialize then "
        "      class.initialize(obj) "
        "    end "
        "    return obj "
        "  end "
        "  setmetatable(class, { __index = baseClass }) "
        "  return class "
        "end "
        );
    sol::stack_object deriveFunction(L);
    module["derive"] = deriveFunction;
    lua_pop(L, 1);

    exportLuaSignalTypes(module);
    exportLuaEigenTypes(module);
    exportLuaValueTree(L, module);
    exportLuaTaskTypes(module);

    module["sleep"] = [](double time) { cnoid::msleep(time * 1000.0); };

    sol::stack::push(L, module);
    
    return 1;
}
