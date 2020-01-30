#include "KinematicSimulatorItem.h"
#include "BodyItem.h"
#include "WorldItem.h"
#include "IoConnectionMapItem.h"
#include <cnoid/CloneMap>
#include <cnoid/HolderDevice>
#include <cnoid/AttachmentDevice>
#include <cnoid/IoConnectionMap>
#include <cnoid/ItemManager>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

class KinematicSimBody : public SimulationBody
{
public:
    KinematicSimBody(Body* body);
    virtual bool initialize(SimulatorItem* simulatorItem, BodyItem* bodyItem) override;
};

class AttachmentInfo : public Referenced
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    BodyPtr body;
    Position T_offset;
    AttachmentInfo(Body* body, const Position& T_offset)
        : body(body), T_offset(T_offset) { }
};
typedef ref_ptr<AttachmentInfo> AttachmentInfoPtr;

class HolderInfo : public Referenced
{
public:
    HolderDevicePtr holder;
    vector<AttachmentInfoPtr> extraAttachments;
    
    //bool on_prev;
    HolderInfo(HolderDevice* holder)
        : holder(holder) /* , on_prev(holder->on()) */ { }
    bool isActive() const { return holder->on() || !extraAttachments.empty(); }
};
typedef ref_ptr<HolderInfo> HolderInfoPtr;

}

namespace cnoid {

class KinematicSimulatorItem::Impl
{
public:
    KinematicSimulatorItem* self;
    vector<HolderInfoPtr> holders;
    vector<HolderInfoPtr> activeHolders;
    vector<IoConnectionMapPtr> ioConnectionMaps;

    Impl(KinematicSimulatorItem* self);
    Impl(KinematicSimulatorItem* self, const Impl& org);
    bool initializeSimulation(const std::vector<SimulationBody*>& simBodies);
    void onHolderStateChanged(HolderInfo* info);
    vector<Body*> findAttachableBodies(HolderDevice* holder, const Position& T_holder);
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


Item* KinematicSimulatorItem::doDuplicate() const
{
    return new KinematicSimulatorItem(*this);
}


void KinematicSimulatorItem::clearSimulation()
{
    impl->holders.clear();    
    impl->activeHolders.clear();    
    impl->ioConnectionMaps.clear();
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
    /*
      This is a temporary implementation.
      This should be implemented in IoConnectionMapItem as functions of SubSimulatorItem.
      To achieve it, SubSimulatorItem must be an interface class that does not inherit the Item class,
      or define another interface class that provides the functions similar to SubSimulatorItem.
    */
    for(auto& item : self->worldItem()->descendantItems<IoConnectionMapItem>()){
        auto connectionMap = self->cloneMap().getClone(item->connectionMap());
        connectionMap->establishConnections();
        ioConnectionMaps.push_back(connectionMap);
    }
    
    for(auto& simBody : simBodies){
        for(auto& holder : simBody->body()->devices<HolderDevice>()){
            auto info = new HolderInfo(holder);
            holders.push_back(info);
            if(holder->numAttachments() > 0){
                activeHolders.push_back(info);
            }
            holder->sigStateChanged().connect(
                [this, info](){ onHolderStateChanged(info); });
        }
    }
    return true;
}


bool KinematicSimulatorItem::stepSimulation(const std::vector<SimulationBody*>& activeSimBodies)
{
    for(size_t i=0; i < activeSimBodies.size(); ++i){
        SimulationBody* simBody = activeSimBodies[i];
        auto body = simBody->body();
        for(auto& link : body->links()){
            if(link->actuationMode() == Link::JointDisplacement){
                link->q() = link->q_target();
            }
        }
        body->calcForwardKinematics();
    }

    for(auto& info : impl->activeHolders){
        auto& holder = info->holder;
        Position T_base = holder->link()->T() * holder->T_local();
        int n = holder->numAttachments();
        if(n > 0){
            for(int i=0; i < n; ++i){
                auto attachment = holder->attachment(i);
                attachment->link()->T() = T_base * attachment->T_local().inverse(Eigen::Isometry);
                attachment->link()->body()->calcForwardKinematics();
            }
        } else {
            for(auto& extra : info->extraAttachments){
                auto body = extra->body;
                auto link = body->rootLink();
                link->T() = T_base * extra->T_offset;
                body->calcForwardKinematics();
            }
        }
    }
    
    return true;
}


void KinematicSimulatorItem::Impl::onHolderStateChanged(HolderInfo* info)
{
    auto holder = info->holder;

    if(holder->on()){
        if(info->holder->numAttachments() == 0){
            bool wasActive = !info->extraAttachments.empty();
            info->extraAttachments.clear();
            Position T_holder = holder->link()->T() * holder->T_local();
            for(auto& body : findAttachableBodies(holder, T_holder)){
                Position T_attached = body->rootLink()->T();
                Position T_offset = T_holder.inverse(Eigen::Isometry) * T_attached;
                info->extraAttachments.push_back(new AttachmentInfo(body, T_offset));
            }
            if(!wasActive && !info->extraAttachments.empty()){
                activeHolders.push_back(info);
            }
        }
    } else {
        info->extraAttachments.clear();
        if(!info->isActive()){
            activeHolders.erase(
                std::find(activeHolders.begin(), activeHolders.end(), info),
                activeHolders.end());
        }
    }
    //info->on_prev = holder->on();
}


vector<Body*> KinematicSimulatorItem::Impl::findAttachableBodies(HolderDevice* holder, const Position& T_holder)
{
    vector<Body*> bodies;
    double minDistance = std::numeric_limits<double>::max();
    Body* attachableBody = nullptr;
    for(auto& simBody : self->simulationBodies()){
        if(simBody->isActive()){
            auto body = simBody->body();
            auto rootLink = body->rootLink();
            if(rootLink->isFreeJoint()){
                double d = (rootLink->translation() - T_holder.translation()).norm();
                if(d < minDistance){
                    attachableBody = body;
                    minDistance = d;
                }
            }
        }
    }
    bodies.push_back(attachableBody);
    return bodies;
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

}


bool KinematicSimBody::initialize(SimulatorItem* simulatorItem, BodyItem* bodyItem)
{
    return SimulationBody::initialize(simulatorItem, bodyItem);
}
