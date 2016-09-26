/**
   @author Shin'ichiro Nakaoka
*/

#include "LuaInterpreter.h"
#include <cnoid/ExecutablePath>
#include <cnoid/FileUtil>
#include <iostream>
#include <stack>

using namespace std;
using namespace cnoid;
namespace filesystem = boost::filesystem;

namespace cnoid {

class LuaInterpreterImpl
{
public:
    lua_State* state;
    stack<ostream*> outputStack;

    LuaInterpreterImpl();
    ~LuaInterpreterImpl();
};

}

static const char* InterpreterInstanceKey = "cnoid_lua_interpreter";

static int print(lua_State* state)
{
    int result = 0;
    lua_pushstring(state, InterpreterInstanceKey);
    lua_gettable(state, LUA_REGISTRYINDEX);

    if(!lua_islightuserdata(state, -1)){
        lua_pop(state, 1);

    } else {
        LuaInterpreterImpl* instance = (LuaInterpreterImpl*)lua_touserdata(state, -1);
        ostream& os = *instance->outputStack.top();
        lua_pop(state, 1);

        int n = lua_gettop(state);  // number of arguments
        lua_getglobal(state, "tostring");
        for(int i=1; i <= n; ++i) {
            const char *s;
            size_t l;
            lua_pushvalue(state, -1); // function to be called
            lua_pushvalue(state, i);  // value to print
            lua_call(state, 1, 1);
            s = lua_tolstring(state, -1, &l); // get result
            if(s == NULL){
                return luaL_error(state, "'tostring' must return a string to 'print'");
            }
            if(i > 1){
                os << "\t";
            }
            os << s;

            lua_pop(state, 1);  // pop result
        }
        os << endl;
    }
    
    return result;
}


std::shared_ptr<LuaInterpreter> LuaInterpreter::mainInstance()
{
    static std::shared_ptr<LuaInterpreter> mainInterpreter;
    if(!mainInterpreter){
        mainInterpreter = make_shared<LuaInterpreter>();
    }
    return mainInterpreter;
}
    
        
LuaInterpreter::LuaInterpreter()
{
    impl = new LuaInterpreterImpl;
}


LuaInterpreterImpl::LuaInterpreterImpl()
{
    state = luaL_newstate();
    luaL_openlibs(state);

    // Append the Choreonoid lua module path to package.cpath
    lua_getglobal(state, "package");
    lua_pushstring(state, "cpath");
    lua_gettable(state, -2);
    string cpath(lua_tostring(state, -1));
    lua_pop(state, 1);
    filesystem::path path = filesystem::path(executableTopDirectory()) / CNOID_PLUGIN_SUBDIR / "lua" / "?.so";
    lua_pushstring(state, "cpath");
    lua_pushstring(state, (cpath + ";" + getNativePathString(path)).c_str());
    lua_settable(state, -3);
    lua_pop(state, 1);

    // for redirecting the output from Lua
    lua_register(state, "print", print);
    lua_pushstring(state, InterpreterInstanceKey);
    lua_pushlightuserdata(state, this);
    lua_settable(state, LUA_REGISTRYINDEX);

    outputStack.push(&std::cout);
}
    

LuaInterpreter::~LuaInterpreter()
{
    delete impl;
}


LuaInterpreterImpl::~LuaInterpreterImpl()
{
    lua_close(state);
}


lua_State* LuaInterpreter::state()
{
    return impl->state;
}


void LuaInterpreter::beginRedirect(std::ostream& os)
{
    impl->outputStack.push(&os);
}


void LuaInterpreter::endRedirect()
{
    if(impl->outputStack.size() > 1){
        impl->outputStack.pop();
    }
}


void LuaInterpreter::dumpStack()
{
    ostream& os = *impl->outputStack.top();
    lua_State* L = impl->state;

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
