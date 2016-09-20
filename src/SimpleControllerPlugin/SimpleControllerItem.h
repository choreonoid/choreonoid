/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_SIMPLE_CONTROLLER_PLUGIN_SIMPLE_CONTROLLER_ITEM_H
#define CNOID_SIMPLE_CONTROLLER_PLUGIN_SIMPLE_CONTROLLER_ITEM_H

#include <cnoid/ControllerItem>
#include "exportdecl.h"

namespace cnoid {

class SimpleControllerItemImpl;
class SimpleController;
class MessageView;

class CNOID_EXPORT SimpleControllerItem : public ControllerItem
{
public:
    SimpleControllerItem();
    SimpleControllerItem(const SimpleControllerItem& org);
    virtual ~SimpleControllerItem();
        
    void setController(const std::string& name);

    virtual bool initialize(ControllerItemIO* io);
    virtual bool start();
    virtual double timeStep() const;
    virtual void input();
    virtual bool control();
    virtual void output();
    virtual void stop();

    SimpleController* initialize(ControllerItemIO* io, Body* sharedIoBody);

protected:
    virtual void onDisconnectedFromRoot();
    virtual Item* doDuplicate() const;
    virtual void doPutProperties(PutPropertyFunction& putProperty);
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);
        
private:
    SimpleControllerItemImpl* impl;
};
        
typedef ref_ptr<SimpleControllerItem> SimpleControllerItemPtr;

}

#endif
