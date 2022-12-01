#ifndef CNOID_BODY_PLUGIN_COLLISION_DETECTION_CONTROLLER_ITEM_H
#define CNOID_BODY_PLUGIN_COLLISION_DETECTION_CONTROLLER_ITEM_H

#include "ControllerItem.h"
#include <cnoid/CollisionLinkPair>
#include <vector>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT CollisionDetectionControllerItem : public ControllerItem
{
public:
    static void initializeClass(ExtensionManager* ext);
    
    CollisionDetectionControllerItem();
    ~CollisionDetectionControllerItem();

    bool isCollisionDetectionEnabled() const;
    void setCollisionDetectionEnabled(bool on);

    bool isAttachedLinkPairExclusionMode() const;
    void setAttachedLinkPairExclusionMode(bool on);

    double collisionDepthThreshold() const;
    void setCollisionDepthThreshold(double depth);

    virtual bool initialize(ControllerIO* io) override;
    virtual bool start() override;
    virtual void input() override;
    virtual bool control() override;
    virtual void output() override;
    virtual void stop() override;

    enum BodyType { TargetBody, AttachedInCollision, AttachedOutsideCollision, NotAttached };
    int checkLinkRelationshipWithTargetBody(Link* link, Link* counterpartLink) const;

    class Impl;

protected:
    CollisionDetectionControllerItem(const CollisionDetectionControllerItem& org);
    virtual Item* doCloneItem(CloneMap* cloneMap) const override;

    // Used from a sub class
    bool isCollisionDetectionEnabledInternally() const;
    void setCollisionDetectionEnabledInternally(bool on);

    // Called from the main simulator thread in the output phase
    virtual void onCollisionsDetected(const std::vector<CollisionLinkPair>& collisions);
    
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

private:
    Impl* impl;
};

}

#endif
