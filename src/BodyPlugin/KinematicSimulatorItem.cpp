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

class HolderInfo : public Referenced
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    HolderDevicePtr holder;
    BodyPtr attachedBody;
    Position T_offset;
    bool on_prev;
    HolderInfo(HolderDevice* holder) : holder(holder), on_prev(holder->on()) { }
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
    Body* findAttachableBody(HolderDevice* holder, const Position& T_holder);
};

}


void KinematicSimulatorItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerClass<KinematicSimulatorItem>(N_("KinematicSimulatorItem"));
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
    ItemList<IoConnectionMapItem> connectionMapItems;
    if(connectionMapItems.extractSubTreeItems(self->worldItem())){
        for(auto& item : connectionMapItems){
            auto connectionMap = self->cloneMap().getClone(item->connectionMap());
            connectionMap->establishConnections();
            ioConnectionMaps.push_back(connectionMap);
        }
    }
    
    for(auto& simBody : simBodies){
        for(auto& holder : simBody->body()->devices<HolderDevice>()){
            auto info = new HolderInfo(holder);
            holders.push_back(info);
            if(holder->on() && holder->attachment()){
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
        if(auto attachment = holder->attachment()){
            attachment->link()->T() = T_base * attachment->T_local().inverse(Eigen::Isometry);
            attachment->link()->body()->calcForwardKinematics();
        } else {
            auto body = info->attachedBody;
            auto link = body->rootLink();
            link->T() = T_base * info->T_offset;
            body->calcForwardKinematics();
        }
    }
    
    return true;
}


void KinematicSimulatorItem::Impl::onHolderStateChanged(HolderInfo* info)
{
    auto holder = info->holder;

    if(holder->on()){
        Position T_holder = holder->link()->T() * holder->T_local();
        info->attachedBody = findAttachableBody(holder, T_holder);
        if(info->attachedBody){
            Position T_attached = info->attachedBody->rootLink()->T();
            info->T_offset = T_holder.inverse(Eigen::Isometry) * T_attached;
            activeHolders.push_back(info);
        }
    } else {
        activeHolders.erase(
            std::find(activeHolders.begin(), activeHolders.end(), info));
    }
    info->on_prev = holder->on();
}


Body* KinematicSimulatorItem::Impl::findAttachableBody(HolderDevice* holder, const Position& T_holder)
{
    double minDistance = std::numeric_limits<double>::max();
    Body* attachableBody = nullptr;
    for(auto& simBody : self->simulationBodies()){
        if(simBody->isActive()){
            auto body = simBody->body();
            auto rootLink = body->rootLink();
            if(rootLink->isFreeJoint()){
                double d = (rootLink->translation() - T_holder.translation()).norm();
                if(d < minDistance){
                    minDistance = d;
                    attachableBody = body;
                }
            }
        }
    }
    return attachableBody;
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
