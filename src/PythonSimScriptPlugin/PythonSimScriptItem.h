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
    virtual const std::string& scriptFilename() const override;
        
    virtual void setBackgroundMode(bool on) override;
    virtual bool isBackgroundMode() const override;
    virtual bool isRunning() const override;

    virtual bool executeAsSimulationScript() override;
    virtual bool executeCode(const char* code) override;
    virtual bool waitToFinish(double timeout = 0.0) override;
    virtual std::string resultString() const override;
    virtual SignalProxy<void()> sigScriptFinished() override;

    virtual bool terminate() override;
        
protected:
    virtual void onDisconnectedFromRoot() override;
    virtual Item* doDuplicate() const override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;
            
private:
    PythonScriptItemImpl* impl;
};

typedef ref_ptr<PythonSimScriptItem> PythonSimScriptItemPtr;

}

#endif
