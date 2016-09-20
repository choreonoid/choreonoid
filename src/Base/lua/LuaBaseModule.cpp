/**
   @author Shin'ichiro Nakaoka
*/

#include <lua.hpp>
#include <cmath>

static int l_sin(lua_State* L)
{
    double d = luaL_checknumber(L, 1);
    lua_pushnumber(L, sin(d));
    return 1;
}

static const struct luaL_Reg base[] = {
    { "sin", l_sin },
    { NULL, NULL }
};


extern "C" int luaopen_cnoid_Base(lua_State* L)
{
#if LUA_VERSION_NUM >= 502
    luaL_newlib(L, base);
#else
    luaL_register(L, "Base", base);
#endif   
    return 1;
}
