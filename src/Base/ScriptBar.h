/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_SCRIPT_BAR_H
#define CNOID_BASE_SCRIPT_BAR_H

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
