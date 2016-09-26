/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_PYTHON_PLUGIN_LUA_SCRIPT_ITEM_H
#define CNOID_PYTHON_PLUGIN_LUA_SCRIPT_ITEM_H

#include <cnoid/ScriptItem>
#include "exportdecl.h"

namespace cnoid {

class LuaScriptItemImpl;
        
class CNOID_EXPORT LuaScriptItem : public ScriptItem
{
public:
    static void initializeClass(ExtensionManager* ext);

    LuaScriptItem();
    LuaScriptItem(const LuaScriptItem& org);

    bool setScriptFilename(const std::string& filename);
    virtual const std::string& scriptFilename() const;
                
    virtual void setBackgroundMode(bool on) override;
    virtual bool isBackgroundMode() const;
    virtual bool isRunning() const;

    virtual bool execute();
    virtual bool executeCode(const char* code);
    virtual bool waitToFinish(double timeout = 0.0);
    virtual std::string resultString() const;
    virtual SignalProxy<void()> sigScriptFinished();
        
    virtual bool terminate();

protected:
    virtual ~LuaScriptItem();
    virtual void onDisconnectedFromRoot();
    virtual Item* doDuplicate() const;
    virtual void doPutProperties(PutPropertyFunction& putProperty);
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);
            
private:
    LuaScriptItemImpl* impl;
};

typedef ref_ptr<LuaScriptItem> LuaScriptItemPtr;

}

#endif
