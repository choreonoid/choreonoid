/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_PYTHON_SIM_SCRIPT_PLUGIN_PYTHON_SIM_SCRIPT_ITEM_H
#define CNOID_PYTHON_SIM_SCRIPT_PLUGIN_PYTHON_SIM_SCRIPT_ITEM_H

#include <cnoid/SimulationScriptItem>
#include "exportdecl.h"

namespace cnoid {

class PythonScriptItemImpl;

class CNOID_EXPORT PythonSimScriptItem : public SimulationScriptItem
{
public:
    static void initialize(ExtensionManager* ext);
        
    PythonSimScriptItem();
    PythonSimScriptItem(const PythonSimScriptItem& org);
    virtual ~PythonSimScriptItem();

    bool setScriptFilename(const std::string& filename);
    virtual const std::string& scriptFilename() const;
        
    virtual bool setBackgroundMode(bool on);
    virtual bool isBackgroundMode() const;
    virtual bool isRunning() const;

    virtual bool executeAsSimulationScript();
    virtual bool executeCode(const char* code);
    virtual bool waitToFinish(double timeout = 0.0);
    virtual std::string resultString() const;
    virtual SignalProxy<void()> sigScriptFinished();

    virtual bool terminate();
        
protected:
    virtual void onDisconnectedFromRoot();
    virtual Item* doDuplicate() const;
    virtual void doPutProperties(PutPropertyFunction& putProperty);
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);
            
private:
    PythonScriptItemImpl* impl;
};

typedef ref_ptr<PythonSimScriptItem> PythonSimScriptItemPtr;
}

#endif
