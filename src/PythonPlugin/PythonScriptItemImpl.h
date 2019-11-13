/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_PYTHON_PLUGIN_PYTHON_SCRIPT_ITEM_IMPL_H
#define CNOID_PYTHON_PLUGIN_PYTHON_SCRIPT_ITEM_IMPL_H

#include "PythonExecutor.h"
#include <cnoid/ScriptItem>
#include <cnoid/MessageView>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT PythonScriptItemImpl
{
public:
    PythonScriptItemImpl(ScriptItem* scriptItem);
    PythonScriptItemImpl(ScriptItem* scriptItem, const PythonScriptItemImpl& org);
    virtual ~PythonScriptItemImpl();
    ScriptItem* scriptItem() { return scriptItem_; }
    void onDisconnectedFromRoot();
    bool setScriptFilename(const std::string& filename);
    const std::string& scriptFilename() const { return scriptFilename_; }
    void setBackgroundMode(bool on);
    bool isBackgroundMode() const;
    bool isRunning() const;
    bool execute();
    bool executeCode(const char* code);
    bool waitToFinish(double timeout);
    const std::string exceptionText() const;
    Signal<void()>& sigScriptFinished() { return sigScriptFinished_; }
    bool terminate();
    void doPutProperties(PutPropertyFunction& putProperty);
    bool store(Archive& archive);
    bool restore(const Archive& archive);

private:
    void onScriptFinished();
    bool onBackgroundModeChanged(bool on);
        
    ScriptItem* scriptItem_;
    std::string scriptFilename_;
    MessageView* mv;
    PythonExecutor executor;
    Connection sigFinishedConnection;
    Signal<void()> sigScriptFinished_;
};

}

#endif
