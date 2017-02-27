/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_LUA_PLUGIN_LUA_CONSOLE_VIEW_H
#define CNOID_LUA_PLUGIN_LUA_CONSOLE_VIEW_H

#include <cnoid/View>

namespace cnoid {

class LuaConsoleViewImpl;

class CNOID_EXPORT LuaConsoleView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);
        
    LuaConsoleView();
    virtual ~LuaConsoleView();
        
private:
    LuaConsoleViewImpl* impl;
};

}

#endif
