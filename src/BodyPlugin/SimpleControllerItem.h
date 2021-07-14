/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_PLUGIN_SIMPLE_CONTROLLER_ITEM_H
#define CNOID_BODY_PLUGIN_SIMPLE_CONTROLLER_ITEM_H

#include <cnoid/ControllerItem>
#include "exportdecl.h"

namespace cnoid {

class SimpleController;

class CNOID_EXPORT SimpleControllerItem : public ControllerItem
{
public:
    static void initializeClass(ExtensionManager* ext);
    
    SimpleControllerItem();
    SimpleControllerItem(const SimpleControllerItem& org);
    virtual ~SimpleControllerItem();
        
    void setController(const std::string& name);
    SimpleController* controller();

    virtual bool checkIfSubController(ControllerItem* controllerItem) const override;
    virtual bool initialize(ControllerIO* io) override;
    virtual bool start() override;
    virtual double timeStep() const override;
    virtual void input() override;
    virtual bool control() override;
    virtual void output() override;
    virtual void stop() override;

    class Impl;

protected:
    virtual Item* doDuplicate() const override;
    virtual void onTreePathChanged() override;
    virtual void onDisconnectedFromRoot() override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;
        
private:
    Impl* impl;
};
        
typedef ref_ptr<SimpleControllerItem> SimpleControllerItemPtr;

}

#endif
