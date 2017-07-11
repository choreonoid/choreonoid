/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_PLUGIN_SIMPLE_CONTROLLER_ITEM_H
#define CNOID_BODY_PLUGIN_SIMPLE_CONTROLLER_ITEM_H

#include <cnoid/ControllerItem>
#include "exportdecl.h"

namespace cnoid {

class SimpleControllerItemImpl;
class SimpleController;
class MessageView;

class CNOID_EXPORT SimpleControllerItem : public ControllerItem
{
public:
    static void initializeClass(ExtensionManager* ext);
    
    SimpleControllerItem();
    SimpleControllerItem(const SimpleControllerItem& org);
    virtual ~SimpleControllerItem();
        
    void setController(const std::string& name);
    SimpleController* controller();
    SignalProxy<void()> sigControllerChanged();

    virtual bool initialize(ControllerItemIO* io);
    virtual bool start();
    virtual double timeStep() const;
    virtual void input();
    virtual bool control();
    virtual void output();
    virtual void stop();

protected:
    virtual void onDisconnectedFromRoot();
    virtual Item* doDuplicate() const;
    virtual void doPutProperties(PutPropertyFunction& putProperty);
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);
        
private:
    SimpleControllerItemImpl* impl;
    friend class SimpleControllerItemImpl;
};
        
typedef ref_ptr<SimpleControllerItem> SimpleControllerItemPtr;

}

#endif
