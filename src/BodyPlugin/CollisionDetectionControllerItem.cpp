#include "CollisionDetectionControllerItem.h"
#include <cnoid/ItemManager>
#include <cnoid/AISTCollisionDetector>
#include <cnoid/Body>
#include <cnoid/BodyCollisionDetector>
#include <cnoid/PutPropertyFunction>
#include <cnoid/Archive>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace cnoid {

class CollisionDetectionControllerItem::Impl
{
public:
    CollisionDetectionControllerItem* self;
    BodyPtr targetBody;
    unique_ptr<BodyCollisionDetector> bodyCollisionDetector;
    
    // When a link of a non-target body is being attached to the target body, the attached
    // link pair must be treated as one link. To achive this, an extra group ID is temporarily
    // given to the attached link in the collision detection. The following map is used to
    // store the information on those temporary group IDs.
    map<LinkPtr, int> attachedLinkToGroupIdMap;
    
    vector<CollisionLinkPair> collisions;
    bool isCollisionDetectionEnabled;
    bool isCollisionDetectionEnabledInternally;

    bool isAttachedLinkPairExclusionMode;
    bool isAttachedLinkPairExclusionModeInControlThread;

    double collisionDepthThreshold;
    double collisionDepthThresholdInControlThread;

    Impl(CollisionDetectionControllerItem* self);
    Impl(CollisionDetectionControllerItem* self, Impl& org);
    void initializeCollisionDetector();
    bool initialize(ControllerIO* io);
    bool start();
    void input();
    bool control();
    void output();
    int checkBodyType(Link* link, Link* counterPartLink) const;    
    void setGroupForAttachedLink(Link* link);
};

}


void CollisionDetectionControllerItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerClass<CollisionDetectionControllerItem, ControllerItem>(
        N_("CollisionDetectionController"));
}


CollisionDetectionControllerItem::CollisionDetectionControllerItem()
{
    impl = new Impl(this);
}


CollisionDetectionControllerItem::CollisionDetectionControllerItem(const CollisionDetectionControllerItem& org)
    : ControllerItem(org)
{
    impl = new Impl(this, *org.impl);
}


CollisionDetectionControllerItem::Impl::Impl(CollisionDetectionControllerItem* self)
    : self(self)
{
    isCollisionDetectionEnabled = true;
    isCollisionDetectionEnabledInternally = true;
    isAttachedLinkPairExclusionMode = false;
    collisionDepthThreshold = 0.0;
}


CollisionDetectionControllerItem::Impl::Impl(CollisionDetectionControllerItem* self, Impl& org)
    : Impl(self)
{
    isCollisionDetectionEnabled = org.isCollisionDetectionEnabled;
    isCollisionDetectionEnabledInternally = org.isCollisionDetectionEnabledInternally;
    isAttachedLinkPairExclusionMode = org.isAttachedLinkPairExclusionMode;
    collisionDepthThreshold = org.collisionDepthThreshold;
}


CollisionDetectionControllerItem::~CollisionDetectionControllerItem()
{
    delete impl;
}


Item* CollisionDetectionControllerItem::doCloneItem(CloneMap* /* cloneMap */) const
{
    return new CollisionDetectionControllerItem(*this);
}


bool CollisionDetectionControllerItem::isCollisionDetectionEnabled() const
{
    return impl->isCollisionDetectionEnabled;
}


void CollisionDetectionControllerItem::setCollisionDetectionEnabled(bool on)
{
    impl->isCollisionDetectionEnabled = on;
}


bool CollisionDetectionControllerItem::isCollisionDetectionEnabledInternally() const
{
    return impl->isCollisionDetectionEnabledInternally;
}


void CollisionDetectionControllerItem::setCollisionDetectionEnabledInternally(bool on)
{
    impl->isCollisionDetectionEnabledInternally = on;
}


bool CollisionDetectionControllerItem::isAttachedLinkPairExclusionMode() const
{
    return impl->isAttachedLinkPairExclusionMode;
}


void CollisionDetectionControllerItem::setAttachedLinkPairExclusionMode(bool on)
{
    impl->isAttachedLinkPairExclusionMode = on;
}


double CollisionDetectionControllerItem::collisionDepthThreshold() const
{
    return impl->collisionDepthThreshold;
}


void CollisionDetectionControllerItem::setCollisionDepthThreshold(double depth)
{
    impl->collisionDepthThreshold = depth;
}


void CollisionDetectionControllerItem::Impl::initializeCollisionDetector()
{
    auto collisionDetector = new AISTCollisionDetector;
    collisionDetector->setDynamicGeometryPairChangeEnabled(true);
    bodyCollisionDetector = make_unique<BodyCollisionDetector>(collisionDetector);
    bodyCollisionDetector->setGeometryHandleMapEnabled(true);
}    


bool CollisionDetectionControllerItem::initialize(ControllerIO* io)
{
    return impl->initialize(io);
}


bool CollisionDetectionControllerItem::Impl::initialize(ControllerIO* io)
{
    targetBody = io->body();
    
    if(!bodyCollisionDetector){
        initializeCollisionDetector();
    }
    
    bodyCollisionDetector->clearBodies();

    attachedLinkToGroupIdMap.clear();

    if(isCollisionDetectionEnabled && isCollisionDetectionEnabledInternally){
        bodyCollisionDetector->addBody(targetBody, true, 0);
    }

    // Collisions between environmental objects are disabled
    bodyCollisionDetector->collisionDetector()->setGroupPairEnabled(1, 1, false);

    return true;
}


bool CollisionDetectionControllerItem::start()
{
    return impl->start();
}


bool CollisionDetectionControllerItem::Impl::start()
{
    if(isCollisionDetectionEnabled && isCollisionDetectionEnabledInternally){
        for(auto& simBody : self->simulatorItem()->simulationBodies()){
            auto body = simBody->body();
            if(body != targetBody){
                bodyCollisionDetector->addBody(body, false, 1);
            }
        }
        bodyCollisionDetector->makeReady();
    }

    isAttachedLinkPairExclusionModeInControlThread = isAttachedLinkPairExclusionMode;
    collisionDepthThresholdInControlThread = collisionDepthThreshold;

    return true;
}


void CollisionDetectionControllerItem::input()
{
    impl->input();
}


void CollisionDetectionControllerItem::Impl::input()
{
    if(isCollisionDetectionEnabled && isCollisionDetectionEnabledInternally){
        bodyCollisionDetector->updatePositions();
    }

    if(isAttachedLinkPairExclusionModeInControlThread){
        auto it = attachedLinkToGroupIdMap.begin();
        while(it != attachedLinkToGroupIdMap.end()){
            auto& link = it->first;
            if(link->body()->parentBody() == targetBody){ // still attached
                ++it;
            } else { // detached
                bodyCollisionDetector->setGroup(link, 1);
                int groupId = it->second;
                bodyCollisionDetector->collisionDetector()->setGroupPairEnabled(0, groupId, true);
                it = attachedLinkToGroupIdMap.erase(it);
            }
        }
    }
}


bool CollisionDetectionControllerItem::control()
{
    return impl->control();
}


bool CollisionDetectionControllerItem::Impl::control()
{
    collisions.clear();

    if(!isCollisionDetectionEnabled || !isCollisionDetectionEnabledInternally){
        return false;
    }
    
    bodyCollisionDetector->detectCollisions(
        [this](const CollisionPair& collisionPair){
            if(collisionDepthThresholdInControlThread > 0.0){
                bool hasDepth = false;
                for(auto& collision : collisionPair.collisions()){
                    if(collision.depth >= collisionDepthThresholdInControlThread){
                        hasDepth = true;
                        break;
                    }
                }
                if(!hasDepth){
                    return;
                }
            }
            // Note that any functions and variables of the link object must not be accessed
            // here to avoid the conflicts with other threads.
            auto link0 = static_cast<Link*>(collisionPair.object(0));
            auto link1 = static_cast<Link*>(collisionPair.object(1));
            collisions.emplace_back(link0, link1, std::move(collisionPair.collisions()));
        });
    
    return false;
}


void CollisionDetectionControllerItem::output()
{
    impl->output();
}


void CollisionDetectionControllerItem::Impl::output()
{
    if(isAttachedLinkPairExclusionModeInControlThread){
        auto it = collisions.begin();
        while(it != collisions.end()){
            auto& collision = *it;
            bool attached = false;
            for(int i=0; i < 2; ++i){
                auto link = collision.link(i);
                auto counterpartLink = collision.link(1-i);
                if(link->isRoot() && link->body()->parentBodyLink() == counterpartLink){
                    setGroupForAttachedLink(link);
                    attached = true;
                }
            }
            if(attached){
                it = collisions.erase(it);
            } else {
                ++it;
            }
        }
    }

    if(!collisions.empty()){
        self->onCollisionsDetected(collisions);
    }
}


void CollisionDetectionControllerItem::stop()
{
    impl->bodyCollisionDetector->clearBodies();
}


void CollisionDetectionControllerItem::onCollisionsDetected(const std::vector<CollisionLinkPair>& /* collisions */)
{

}


void CollisionDetectionControllerItem::Impl::setGroupForAttachedLink(Link* link)
{
    int groupId = 2;
    for(auto& kv : attachedLinkToGroupIdMap){
        int existingId = kv.second;
        if(groupId < existingId){
            break;
        }
        groupId = existingId + 1;
    }
    bodyCollisionDetector->setGroup(link, groupId);
    bodyCollisionDetector->collisionDetector()->setGroupPairEnabled(0, groupId, false);
    attachedLinkToGroupIdMap[link] = groupId;
}


int CollisionDetectionControllerItem::checkLinkRelationshipWithTargetBody(Link* link, Link* counterpartLink) const
{
    Body* body = link->body();
    Body* targetBody = impl->targetBody;

    if(body == targetBody){
        return TargetBody;
    }
    if(link->isRoot() && body->parentBody() == targetBody){
        if(body->parentBodyLink() == counterpartLink){
            return AttachedInCollision;
        } else {
            return AttachedOutsideCollision;
        }
    }
    if(targetBody->parentBody() == body){
        if(targetBody->parentBodyLink() == link){
            return AttachedInCollision;
        } else {
            return AttachedOutsideCollision;
        }
    }

    return NotAttached;
}


void CollisionDetectionControllerItem::doPutProperties(PutPropertyFunction& putProperty)
{
    ControllerItem::doPutProperties(putProperty);
}


bool CollisionDetectionControllerItem::store(Archive& archive)
{
    if(!ControllerItem::store(archive)){
        return false;
    }
    archive.write("enabled", impl->isCollisionDetectionEnabled);
    archive.write("exclude_attached_link_pairs", impl->isAttachedLinkPairExclusionMode);
    archive.write("collision_depth", impl->collisionDepthThreshold);
    return true;
}


bool CollisionDetectionControllerItem::restore(const Archive& archive)
{
    if(!ControllerItem::restore(archive)){
        return false;
    }
    archive.read("enabled", impl->isCollisionDetectionEnabled);
    archive.read("exclude_attached_link_pairs", impl->isAttachedLinkPairExclusionMode);
    archive.read("collision_depth", impl->collisionDepthThreshold);
    return true;
}
