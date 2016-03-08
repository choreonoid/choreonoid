/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_PLUGIN_BODY_MOTION_CONTROLLER_ITEM_H
#define CNOID_BODY_PLUGIN_BODY_MOTION_CONTROLLER_ITEM_H

#include "ControllerItem.h"
#include "exportdecl.h"

namespace cnoid {

class BodyMotionControllerItemImpl;

class CNOID_EXPORT BodyMotionControllerItem : public ControllerItem
{
public:
    static void initializeClass(ExtensionManager* ext);

    BodyMotionControllerItem();
    BodyMotionControllerItem(const BodyMotionControllerItem& org);
    virtual ~BodyMotionControllerItem();
        
    virtual bool initialize(Target* target);
    virtual bool start(ControllerItem::Target* target);
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
    friend class BodyMotionControllerItemImpl;
    BodyMotionControllerItemImpl* impl;
};
        
typedef ref_ptr<BodyMotionControllerItem> BodyMotionControllerItemPtr;

}

#endif
