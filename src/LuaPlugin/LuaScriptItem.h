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
    virtual const std::string& scriptFilename() const override;
                
    virtual void setBackgroundMode(bool on) override;
    virtual bool isBackgroundMode() const override;
    virtual bool isRunning() const override;

    virtual bool execute() override;
    virtual bool waitToFinish(double timeout = 0.0) override;
    virtual std::string resultString() const override;
    virtual SignalProxy<void()> sigScriptFinished() override;
        
    virtual bool terminate() override;

protected:
    virtual ~LuaScriptItem();
    virtual void onDisconnectedFromRoot() override;
    virtual Item* doDuplicate() const override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;
            
private:
    LuaScriptItemImpl* impl;
};

typedef ref_ptr<LuaScriptItem> LuaScriptItemPtr;

}

#endif
