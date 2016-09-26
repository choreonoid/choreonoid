/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_LUA_PLUGIN_LUA_INTERPRETER_H
#define CNOID_LUA_PLUGIN_LUA_INTERPRETER_H

#include <lua.hpp>
#include <iosfwd>
#include <memory>
#include "exportdecl.h"

namespace cnoid {

class LuaInterpreterImpl;

/**
   \todo Add a lock mechanism
*/
class CNOID_EXPORT LuaInterpreter
{
public:
    static std::shared_ptr<LuaInterpreter> mainInstance();
        
    LuaInterpreter();
    virtual ~LuaInterpreter();

    lua_State* state();

    //! \todo Introduce an object to change the output in a particular scope
    void beginRedirect(std::ostream& os);
    void endRedirect();

    void dumpStack();
        
private:
    LuaInterpreterImpl* impl;
};

}

#endif
