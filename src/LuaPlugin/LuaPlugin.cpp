/**
   @author Shin'ichiro Nakaoka
*/

#include "LuaScriptItem.h"
#include "LuaConsoleView.h"
#include <cnoid/Plugin>

using namespace cnoid;

namespace {

class LuaPlugin : public Plugin
{
public:
    LuaPlugin() : Plugin("Lua")
    {

    }
    
    virtual bool initialize()
    {
        LuaScriptItem::initializeClass(this);
        LuaConsoleView::initializeClass(this);
        return true;
    }
        
    virtual bool finalize()
    {
        return true;
    }
};

}

CNOID_IMPLEMENT_PLUGIN_ENTRY(LuaPlugin);
