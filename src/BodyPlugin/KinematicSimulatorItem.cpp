#include "KinematicSimulatorItem.h"
#include "BodyItem.h"
#include "WorldItem.h"
#include "IoConnectionMapItem.h"
#include <cnoid/LinkedJointHandler>
#include <cnoid/CloneMap>
#include <cnoid/HolderDevice>
#include <cnoid/AttachmentDevice>
#include <cnoid/ConveyorDevice>
#include <cnoid/IoConnectionMap>
#include <cnoid/ItemManager>
#include <cnoid/AISTCollisionDetector>
#include <cnoid/BodyCollisionDetector>
#include <cnoid/MessageView>
#include <unordered_set>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

class KinematicSimBody : public SimulationBody
{
public:
    LinkedJointHandlerPtr linkedJointHandler;
    
    KinematicSimBody(Body* body);
    virtual bool initialize(SimulatorItem* simulatorItem, BodyItem* bodyItem) override;
};

class AttachmentInfo : public Referenced
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    BodyPtr body;
    Isometry3 T_offset;
    AttachmentInfo(Body* body, const Isometry3& T_offset)
        : body(body), T_offset(T_offset) { }
};
typedef ref_ptr<AttachmentInfo> AttachmentInfoPtr;

class HolderInfo : public Referenced
{
public:
    HolderDevicePtr holder;
    vector<AttachmentInfoPtr> extraAttachments;
    vector<Link*> collidingLinks;
    bool isActive;
    
    HolderInfo(HolderDevice* holder) : holder(holder), isActive(false) { }
};
typedef ref_ptr<HolderInfo> HolderInfoPtr;

class ConveyorInfo : public Referenced
{
public:
    ConveyorDevicePtr conveyor;
    BodyPtr conveyorBody;
    unordered_set<Body*> bodiesOnConveyor;

    ConveyorInfo(ConveyorDevice* conveyor)
        : conveyor(conveyor), conveyorBody(conveyor->link()->body()) { }
};
typedef ref_ptr<ConveyorInfo> ConveyorInfoPtr;

}

namespace cnoid {

class KinematicSimulatorItem::Impl
{
public:
    KinematicSimulatorItem* self;
    vector<HolderInfoPtr> holderInfos;
    vector<HolderInfoPtr> activeHolderInfos;
    vector<ConveyorInfoPtr> conveyorInfos;
    vector<IoConnectionMapPtr> ioConnectionMaps;
    unique_ptr<BodyCollisionDetector> bodyCollisionDetector;
    bool needToUpdateBodyPositionsInCollisionDetector;
    bool needToBreakSimulation;
    double timeStep;

    Impl(KinematicSimulatorItem* self);
    Impl(KinematicSimulatorItem* self, const Impl& org);
    bool initializeSimulation(const std::vector<SimulationBody*>& simBodies);
    bool stepSimulation(const std::vector<SimulationBody*>& activeSimBodies);
    void updateBodyPositionsInCollisionDetector();
    void onHolderStateChanged(HolderInfo* info);
    void activateHolder(HolderInfo* info);
    vector<Body*> findAttachableBodies(HolderInfo* info, const Isometry3& T_holder);
    void findAttachableBodiesByCollision(HolderInfo* info, vector<Body*>& out_bodies);
    void onHolderCollisionDetected(HolderInfo* info, Link* link, const CollisionPair& collisionPair);
    void findAttachableBodiesByDistance(HolderDevice* holder, const Isometry3& T_holder, vector<Body*>& out_bodies);
    void findAttachableBodiesByName(HolderDevice* holder, const Isometry3& T_holder, vector<Body*>& out_bodies);
    void deactivateHolder(HolderInfo* info);
    void moveConveyors();
    void onConveyorCollisionDetected(ConveyorInfo* info, Link* link, const CollisionPair& collisionPair);
    void moveBodiesOnLinearConveyor(ConveyorInfo* info);
    void moveBodiesOnRotaryConveyor(ConveyorInfo* info);
};

}


void KinematicSimulatorItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerClass<KinematicSimulatorItem, SimulatorItem>(N_("KinematicSimulatorItem"));
    ext->itemManager().addCreationPanel<KinematicSimulatorItem>();
}


KinematicSimulatorItem::KinematicSimulatorItem()
{
    impl = new Impl(this);
    setName("KinematicSimulator");
    setTimeRangeMode(SimulatorItem::TR_ACTIVE_CONTROL);
}


KinematicSimulatorItem::Impl::Impl(KinematicSimulatorItem* self)
    : self(self)
{

}


KinematicSimulatorItem::KinematicSimulatorItem(const KinematicSimulatorItem& org)
    : SimulatorItem(org)
{
    impl = new Impl(this, *org.impl);
}


KinematicSimulatorItem::Impl::Impl(KinematicSimulatorItem* self, const Impl& org)
    : self(self)
{

}


KinematicSimulatorItem::~KinematicSimulatorItem()
{
    delete impl;
}


Item* KinematicSimulatorItem::doCloneItem(CloneMap* /* cloneMap */) const
{
    return new KinematicSimulatorItem(*this);
}


void KinematicSimulatorItem::clearSimulation()
{
    impl->holderInfos.clear();    
    impl->activeHolderInfos.clear();
    impl->conveyorInfos.clear();
    impl->ioConnectionMaps.clear();
    if(impl->bodyCollisionDetector){
        impl->bodyCollisionDetector->clearBodies();
    }
}


SimulationBody* KinematicSimulatorItem::createSimulationBody(Body* orgBody, CloneMap& cloneMap)
{
    return new KinematicSimBody(cloneMap.getClone(orgBody));
}


bool KinematicSimulatorItem::initializeSimulation(const std::vector<SimulationBody*>& simBodies)
{
    return impl->initializeSimulation(simBodies);
}


bool KinematicSimulatorItem::Impl::initializeSimulation(const std::vector<SimulationBody*>& simBodies)
{
    needToBreakSimulation = false;
    
    /*
      This is a temporary implementation.
      This should be implemented in IoConnectionMapItem.
      First of all, use BodyWorldAddon's sigSimulationAboutToBeStarted to detect a simulator item
      that is about to start simulation. On the slot function, set a handler for doing the initialization
      using a customization function like addPreDynamicsFunction.
      The clone ioConnectionMap object for the simulation should be managed by SimulatorItem.
      This can be achieved if SimulatorItem provides a function to accept a Referenced object to manage
      during the simulation.
    */
    for(auto& item : self->worldItem()->descendantItems<IoConnectionMapItem>()){
        item->updateIoDeviceInstances();
        auto connectionMap = self->cloneMap().getClone(item->connectionMap());
        connectionMap->establishConnections();
        ioConnectionMaps.push_back(connectionMap);
    }

    bool hasHolderDevicesWithCollisionCondition = false;
    
    for(auto& simBody : simBodies){
        auto body = simBody->body();
        for(auto& holder : body->devices<HolderDevice>()){
            auto info = new HolderInfo(holder);
            holderInfos.push_back(info);
            if(holder->numAttachments() > 0){
                info->isActive = true;
                activeHolderInfos.push_back(info);
            }
            if(holder->holdCondition() == HolderDevice::Collision){
                hasHolderDevicesWithCollisionCondition = true;
            }
            holder->sigStateChanged().connect(
                [this, info](){ onHolderStateChanged(info); });
        }
        for(auto& conveyor : body->devices<ConveyorDevice>()){
            auto info = new ConveyorInfo(conveyor);
            conveyorInfos.push_back(info);
        }
    }

    if(hasHolderDevicesWithCollisionCondition || !conveyorInfos.empty()){
        if(!bodyCollisionDetector){
            bodyCollisionDetector = make_unique<BodyCollisionDetector>(new AISTCollisionDetector);
            bodyCollisionDetector->setGeometryHandleMapEnabled(true);
            bodyCollisionDetector->setMultiplexBodySupportEnabled(true);
        }
        
        bodyCollisionDetector->clearBodies();
        for(auto& simBody : simBodies){
            auto body = simBody->body();
            auto conveyors = body->devices<ConveyorDevice>();
            if(!conveyors.empty()){
                for(auto& conveyor : conveyors){
                    bodyCollisionDetector->addLink(conveyor->link());
                }
            } else if(simBody->isActive()){
                bodyCollisionDetector->addBody(body, false);

                if(!bodyCollisionDetector->isMultiplexBodySupportEnabled()){
                    body->sigMultiplexBodyAddedOrRemoved().connect(
                        [this](Body* /* body */, bool /* isAdded */){
                            MessageView::instance()->putln(
                                _("Body multiplexing is not supported by the kinematics simulator item with "
                                  "the current collision detector that does not support the geometry removal function."),
                                MessageView::Error);
                            needToBreakSimulation = true;
                        });
                }
            }
        }
        bodyCollisionDetector->makeReady();
    }

    timeStep = self->worldTimeStep();

    return true;
}


bool KinematicSimulatorItem::stepSimulation(const std::vector<SimulationBody*>& activeSimBodies)
{
    return impl->stepSimulation(activeSimBodies);
}


bool KinematicSimulatorItem::Impl::stepSimulation(const std::vector<SimulationBody*>& activeSimBodies)
{
    needToUpdateBodyPositionsInCollisionDetector = true;
    
    if(!conveyorInfos.empty()){
        moveConveyors();
    }
    
    for(size_t i=0; i < activeSimBodies.size(); ++i){
        auto simBody = static_cast<KinematicSimBody*>(activeSimBodies[i]);
        auto body = simBody->body();
        for(auto& link : body->links()){
            if(link->actuationMode() == Link::JointDisplacement){
                link->q() = link->q_target();
            }
        }
        if(auto& linkedJointHandler = simBody->linkedJointHandler){
            if(linkedJointHandler->updateLinkedJointDisplacements()){
                linkedJointHandler->limitLinkedJointDisplacementsWithinMovableRanges();
            }
        }
        body->calcForwardKinematics();
    }

    for(auto& info : activeHolderInfos){
        auto& holder = info->holder;
        Isometry3 T_base = holder->link()->T() * holder->T_local();
        int n = holder->numAttachments();
        for(int i=0; i < n; ++i){
            auto attachment = holder->attachment(i);
            attachment->link()->T() = T_base * attachment->T_local().inverse(Eigen::Isometry);
            attachment->link()->body()->calcForwardKinematics();
        }
        for(auto& extra : info->extraAttachments){
            auto body = extra->body;
            auto link = body->rootLink();
            link->T() = T_base * extra->T_offset;
            body->calcForwardKinematics();
        }
    }

    return !needToBreakSimulation;
}


void KinematicSimulatorItem::Impl::updateBodyPositionsInCollisionDetector()
{
    if(needToUpdateBodyPositionsInCollisionDetector){
        if(bodyCollisionDetector){
            bodyCollisionDetector->makeReady();
            bodyCollisionDetector->updatePositions();
        }
        needToUpdateBodyPositionsInCollisionDetector = false;
    }
}


void KinematicSimulatorItem::Impl::onHolderStateChanged(HolderInfo* info)
{
    if(info->holder->on()){
        activateHolder(info);
    } else {
        deactivateHolder(info);
    }
    //info->on_prev = holder->on();
}


void KinematicSimulatorItem::Impl::activateHolder(HolderInfo* info)
{
    auto holder = info->holder;
    info->extraAttachments.clear();
    Isometry3 T_holder = holder->link()->T() * holder->T_local();
    for(auto& body : findAttachableBodies(info, T_holder)){
        auto rootLink = body->rootLink();
        AttachmentDevice* attachment = nullptr;
        for(auto& device : body->devices<AttachmentDevice>()){
            if(device->link() == rootLink && device->isAttachableTo(holder)){
                attachment = device;
                break;
            }
        }
        if(attachment){
            holder->addAttachment(attachment);
        } else {
            Isometry3 T_attached = rootLink->T();
            Isometry3 T_offset = T_holder.inverse(Eigen::Isometry) * T_attached;
            info->extraAttachments.push_back(new AttachmentInfo(body, T_offset));
        }
    }
    if(!info->isActive){
        activeHolderInfos.push_back(info);
    }
}


vector<Body*> KinematicSimulatorItem::Impl::findAttachableBodies(HolderInfo* info, const Isometry3& T_holder)
{
    auto holder = info->holder;
    vector<Body*> bodies;
    switch(holder->holdCondition()){
    case HolderDevice::Collision:
        findAttachableBodiesByCollision(info, bodies);
        break;
    case HolderDevice::Distance:
        findAttachableBodiesByDistance(holder, T_holder, bodies);
        break;
    case HolderDevice::Name:
        findAttachableBodiesByName(holder, T_holder, bodies);
        break;
    default:
        break;
    }
    return bodies;
}


void KinematicSimulatorItem::Impl::findAttachableBodiesByCollision
(HolderInfo* info, vector<Body*>& out_bodies)
{
    updateBodyPositionsInCollisionDetector();
    info->collidingLinks.clear();
    auto link = info->holder->link();
    bodyCollisionDetector->detectCollisions(
        link,
        [&](const CollisionPair& collisionPair){
            onHolderCollisionDetected(info, link, collisionPair);
        });
    
    for(auto& link : info->collidingLinks){
        auto body = link->body();
        auto rootLink = body->rootLink();
        if(link == rootLink && rootLink->isFreeJoint()){
            out_bodies.push_back(body);
        }
    }
}


void KinematicSimulatorItem::Impl::onHolderCollisionDetected
(HolderInfo* info, Link* link, const CollisionPair& collisionPair)
{
    Link* anotherLink = nullptr;
    for(int i=0; i < 2; ++i){
        if(collisionPair.object(i) == link){
            anotherLink = static_cast<Link*>(collisionPair.object(1 - i));
            break;
        }
    }
    if(anotherLink){
        info->collidingLinks.push_back(anotherLink);
    }
}


void KinematicSimulatorItem::Impl::findAttachableBodiesByDistance
(HolderDevice* holder, const Isometry3& T_holder, vector<Body*>& out_bodies)
{
    auto holderBody = holder->body();
    const double maxDistance = holder->maxHoldDistance();
    for(auto& simBody : self->simulationBodies()){
        Body* body = simBody->body();
        if(body != holderBody && simBody->isActive()){
            while(body){
                auto rootLink = body->rootLink();
                if(rootLink->isFreeJoint()){
                    double d = (rootLink->translation() - T_holder.translation()).norm();
                    if(d < maxDistance){
                        out_bodies.push_back(body);
                    }
                }
                body = body->nextMultiplexBody();
            }
        }
    }
}


void KinematicSimulatorItem::Impl::findAttachableBodiesByName
(HolderDevice* holder, const Isometry3& T_holder, vector<Body*>& out_bodies)
{
    const double maxDistance = holder->maxHoldDistance();
    for(auto& simBody : self->simulationBodies()){
        if(simBody->isActive()){
            auto body = simBody->body();
            if(body->name() == holder->holdTargetName()){
                while(body){
                    auto rootLink = body->rootLink();
                    if(rootLink->isFreeJoint()){
                        double d = (rootLink->translation() - T_holder.translation()).norm();
                        if(d < maxDistance){
                            out_bodies.push_back(body);
                        }
                    }
                    body = body->nextMultiplexBody();
                }
            }
        }
    }
}


void KinematicSimulatorItem::Impl::deactivateHolder(HolderInfo* info)
{
    info->holder->clearAttachments();
    info->extraAttachments.clear();

    if(info->isActive){
        activeHolderInfos.erase(
            std::find(activeHolderInfos.begin(), activeHolderInfos.end(), info),
            activeHolderInfos.end());
        info->isActive = false;
    }
}


void KinematicSimulatorItem::Impl::moveConveyors()
{
    for(auto& info : conveyorInfos){
        auto conveyor = info->conveyor;
        if(conveyor->on()){
            conveyor->setDisplacement(conveyor->displacement() + conveyor->speed() * timeStep);
            
            info->bodiesOnConveyor.clear();
            updateBodyPositionsInCollisionDetector();
            auto link = info->conveyor->link();
            bodyCollisionDetector->detectCollisions(
                link,
                [this, info, link](const CollisionPair& collisionPair){
                    onConveyorCollisionDetected(info, link, collisionPair);
                });

            if(!info->bodiesOnConveyor.empty()){
                if(conveyor->isLinearConveyor()){
                    moveBodiesOnLinearConveyor(info);
                } else if(conveyor->isRotaryConveyor()){
                    moveBodiesOnRotaryConveyor(info);
                }
            }
        }
    }
}


void KinematicSimulatorItem::Impl::onConveyorCollisionDetected
(ConveyorInfo* info, Link* link, const CollisionPair& collisionPair)
{
    Link* anotherLink = nullptr;
    for(int i=0; i < 2; ++i){
        if(collisionPair.object(i) == link){
            anotherLink = static_cast<Link*>(collisionPair.object(1 - i));
            break;
        }
    }
    if(anotherLink){
        auto body = anotherLink->body();
        if(body != info->conveyorBody){
            info->bodiesOnConveyor.insert(body);
        }
    }
}


void KinematicSimulatorItem::Impl::moveBodiesOnLinearConveyor(ConveyorInfo* info)
{
    auto conveyor = info->conveyor;
    auto link = conveyor->link();
    Vector3 direction = link->R() * conveyor->R_local() * conveyor->axis();
    Vector3 dp = timeStep * conveyor->speed() * direction;

    for(auto& body : info->bodiesOnConveyor){
        body->rootLink()->T().translation() += dp;
    }
}


void KinematicSimulatorItem::Impl::moveBodiesOnRotaryConveyor(ConveyorInfo* info)
{
    auto conveyor = info->conveyor;
    auto link = conveyor->link();
    Isometry3 Tc = link->T() * conveyor->T_local();
    Isometry3 Tci = Tc.inverse();

    for(auto& body : info->bodiesOnConveyor){
        Isometry3 T0 = Tci * body->rootLink()->T();
        Isometry3 T1 = AngleAxis(timeStep * conveyor->speed(), conveyor->axis()) * T0;
        body->rootLink()->T() = Tc * T1;
    }
}


void KinematicSimulatorItem::finalizeSimulation()
{

}
    

void KinematicSimulatorItem::doPutProperties(PutPropertyFunction& putProperty)
{
    SimulatorItem::doPutProperties(putProperty);
}


bool KinematicSimulatorItem::store(Archive& archive)
{
    return SimulatorItem::store(archive);
}


bool KinematicSimulatorItem::restore(const Archive& archive)
{
    return SimulatorItem::restore(archive);
}


KinematicSimBody::KinematicSimBody(Body* body)
    : SimulationBody(body)
{
    linkedJointHandler = LinkedJointHandler::findOrCreateLinkedJointHandler(body);
}


bool KinematicSimBody::initialize(SimulatorItem* simulatorItem, BodyItem* bodyItem)
{
    return SimulationBody::initialize(simulatorItem, bodyItem);
}
