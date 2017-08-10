/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_PYTHON3_PLUGIN_PYTHON3_SCRIPT_ITEM_IMPL_H
#define CNOID_PYTHON3_PLUGIN_PYTHON3_SCRIPT_ITEM_IMPL_H

#include "Python3Executor.h"
#include <cnoid/ScriptItem>
#include <cnoid/MessageView>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT Python3ScriptItemImpl
{
public:
    Python3ScriptItemImpl(ScriptItem* scriptItem);
    Python3ScriptItemImpl(ScriptItem* scriptItem, const Python3ScriptItemImpl& org);
    virtual ~Python3ScriptItemImpl();
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
    pybind11::object resultObject();
    const std::string resultString() const;
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
    Python3Executor executor;
    Connection sigFinishedConnection;
    Signal<void()> sigScriptFinished_;
};

}

#endif
