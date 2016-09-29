/*!
  @author Shin'ichiro Nakaoka
*/

#include "LuaUtil.h"
#include <iostream>

using namespace std;

/**
   \todo use the "print" function defined in Lua
*/
void cnoid::stackDump(lua_State* L)
{
    stackDump(L, std::cout);
}

/**
   \todo use the "print" function defined in Lua
*/
void cnoid::stackDump(lua_State* L, std::ostream& os)
{
    os << "stack: ";
    
    int top = lua_gettop(L);
    for(int i=1; i <= top; ++i){
        int t = lua_type(L, i);
        switch(t){
        case LUA_TSTRING: {
            os << "'" << lua_tostring(L, i) << "'";
            break;
        }
        case LUA_TBOOLEAN: {
            os << (lua_toboolean(L, i) ? "true" : "false");
            break;
        }
        case LUA_TNUMBER: {
            os << lua_tonumber(L, i);
            break;
        }
        default: {
            os << lua_typename(L, t);
            break;
        }
        }
        os << ", ";
    }
    os << endl;
}
