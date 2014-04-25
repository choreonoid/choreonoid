/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_SCRIPT_BAR_H_INCLUDED
#define CNOID_BASE_SCRIPT_BAR_H_INCLUDED

#include <cnoid/ToolBar>

namespace cnoid {

class ScriptBar : public ToolBar
{
public:
    static void initialize(ExtensionManager* ext);
        
private:
    ScriptBar();
    virtual ~ScriptBar();
    void executeCheckedScriptItems();
};
}

#endif
