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
        
    virtual bool initialize(ControllerIO* io) override;
    virtual bool start() override;
    virtual double timeStep() const override;
    virtual void input() override;
    virtual bool control() override;
    virtual void output() override;
    virtual void stop() override;

protected:
    virtual void onDisconnectedFromRoot() override;
    virtual Item* doDuplicate() const override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;
        
private:
    friend class BodyMotionControllerItemImpl;
    BodyMotionControllerItemImpl* impl;
};
        
typedef ref_ptr<BodyMotionControllerItem> BodyMotionControllerItemPtr;

}

#endif
