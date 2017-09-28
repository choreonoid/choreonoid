/**
   @author Shin'ichiro Nakaoka
*/

#include "../MessageView.h"
#include <lua.hpp>
#include <cmath>

using namespace std;
using namespace cnoid;

namespace {

int messageViewMetaTable = 0;

void stackDump(lua_State* L)
{
    ostream& os = mvout();

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


int l_sin(lua_State* L)
{
    double d = luaL_checknumber(L, 1);
    lua_pushnumber(L, sin(d));
    return 1;
}

const struct luaL_Reg base[] = {
    { "sin", l_sin },
    { NULL, NULL }
};


int MessageView_putln(lua_State* L)
{
    MessageView* self = *(MessageView**)lua_touserdata(L, 1);
    const char* message = lua_tostring(L, -1);
    self->putln(message);
    return 0;
}


int MessageView_instance(lua_State* L)
{
    MessageView** udata = (MessageView**)lua_newuserdata(L, sizeof(MessageView**));
    *udata = MessageView::instance();

    //luaL_getmetatable(L, "cnoid_Base_MessageView");
    lua_rawgeti(L, LUA_REGISTRYINDEX, messageViewMetaTable);
    
    lua_setmetatable(L, -2);
    
    return 1;
}


void registerMessageView(lua_State* L)
{
    luaL_Reg messageViewRegs[] = {
        { "instance", MessageView_instance },
        { "putln", MessageView_putln },
        { NULL, NULL }
    };

    luaL_newmetatable(L, "cnoid_Base_MessageView");

    lua_pushvalue(L, -1);
    messageViewMetaTable = luaL_ref(L, LUA_REGISTRYINDEX);
    
    luaL_setfuncs(L, messageViewRegs, 0);

    lua_pushvalue(L, -1);
    lua_setfield(L, -1, "__index");
}

}
    

extern "C" int luaopen_cnoid_Base(lua_State* L)
{
    luaL_newlib(L, base);
    registerMessageView(L);
    lua_setfield(L, -2, "MessageView");
    return 1;
}
