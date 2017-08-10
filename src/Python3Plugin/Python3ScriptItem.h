/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_PYTHON3_PLUGIN_PYTHON3_SCRIPT_ITEM_H
#define CNOID_PYTHON3_PLUGIN_PYTHON3_SCRIPT_ITEM_H

#include <cnoid/ScriptItem>
#include "exportdecl.h"

namespace cnoid {

class Python3ScriptItemImpl;
        
class CNOID_EXPORT Python3ScriptItem : public ScriptItem
{
public:
    static void initializeClass(ExtensionManager* ext);

    Python3ScriptItem();
    Python3ScriptItem(const Python3ScriptItem& org);

    bool setScriptFilename(const std::string& filename);
    virtual const std::string& scriptFilename() const override;
                
    virtual void setBackgroundMode(bool on) override;
    virtual bool isBackgroundMode() const override;
    virtual bool isRunning() const override;

    virtual bool execute() override;
    virtual bool executeCode(const char* code) override;
    virtual bool waitToFinish(double timeout = 0.0) override;
    virtual std::string resultString() const override;
    virtual SignalProxy<void()> sigScriptFinished() override;
        
    virtual bool terminate() override;

protected:
    virtual ~Python3ScriptItem();
    virtual void onDisconnectedFromRoot() override;
    virtual Item* doDuplicate() const override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;
            
private:
    Python3ScriptItemImpl* impl;
    bool doExecutionOnLoading;
};

typedef ref_ptr<Python3ScriptItem> Python3ScriptItemPtr;

}

#endif
