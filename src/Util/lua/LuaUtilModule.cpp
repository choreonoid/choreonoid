/**
   @author Shin'ichiro Nakaoka
*/

#include "LuaUtil.h"
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

    /**
       \note It is desriable to directly obtain the defined function,
       but the return value of the script does not seems to be pushed to the stack
       and the value is not correctly retrieved, so the global variable is currently
       used to retireve the defined function
    */
    //sol::object result =
    lua.script(
        //"return function(baseClass) "
        "createDerivedClass = function(baseClass) "
        "  local class = { } "
        "  class.new = function(...) "
        "  local obj = { } "
        "  baseobj = baseClass.new(...) "
        "  if type(baseobj) == \"table\" then "
        "    obj.cppobj = baseobj.cppobj "
        "  else "
        "    obj.cppobj = baseobj "
        "  end "
        "  obj.super = baseobj "
        "  setmetatable(obj, { __index = class }) "
        "  obj.cppobj:setDerivedLuaObject(obj) "
        "  class.initialize(obj) "
        "    return obj "
        "  end "
        "  setmetatable(class, { __index = baseClass }) "
        "  return class "
        "end "
        );

    //module["derive"] = result;
    module["derive"] = lua["createDerivedClass"];

    exportLuaSignalTypes(module);
    exportLuaTaskTypes(module);

    sol::stack::push(L, module);
    
    return 1;
}
