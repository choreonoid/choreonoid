/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_PYTHON_PLUGIN_PYTHON_SCRIPT_ITEM_H
#define CNOID_PYTHON_PLUGIN_PYTHON_SCRIPT_ITEM_H

#include <cnoid/ScriptItem>
#include "exportdecl.h"

namespace cnoid {

class PythonScriptItemImpl;
        
class CNOID_EXPORT PythonScriptItem : public ScriptItem
{
public:
    static void initializeClass(ExtensionManager* ext);

    PythonScriptItem();
    PythonScriptItem(const PythonScriptItem& org);

    bool setScriptFilename(const std::string& filename);
    virtual const std::string& scriptFilename() const;
                
    virtual bool setBackgroundMode(bool on);
    virtual bool isBackgroundMode() const;
    virtual bool isRunning() const;

    virtual bool execute();
    virtual bool executeCode(const char* code);
    virtual bool waitToFinish(double timeout = 0.0);
    virtual std::string resultString() const;
    virtual SignalProxy<void()> sigScriptFinished();
        
    virtual bool terminate();

protected:
    virtual ~PythonScriptItem();
    virtual void onDisconnectedFromRoot();
    virtual ItemPtr doDuplicate() const;
    virtual void doPutProperties(PutPropertyFunction& putProperty);
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);
            
private:
    PythonScriptItemImpl* impl;
    bool doExecutionOnLoading;
};

typedef ref_ptr<PythonScriptItem> PythonScriptItemPtr;
}

#endif
