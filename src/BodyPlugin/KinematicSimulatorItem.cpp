#include "KinematicSimulatorItem.h"
#include <cnoid/ItemManager>
#include <cnoid/BodyItem>
#include <cnoid/HolderDevice>
#include <cnoid/AttachmentDevice>
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

}

namespace cnoid {

class KinematicSimulatorItem::Impl
{
public:
    KinematicSimulatorItem* self;
    vector<HolderDevicePtr> holders;
    vector<HolderDevicePtr> activeHolders;
    BodyCloneMap cloneMap;

    Impl(KinematicSimulatorItem* self);
    Impl(KinematicSimulatorItem* self, const Impl& org);
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
    impl->cloneMap.clear();
    impl->holders.clear();    
    impl->activeHolders.clear();    
}


SimulationBody* KinematicSimulatorItem::createSimulationBody(Body* orgBody)
{
    auto body = impl->cloneMap.getClone(orgBody);
    return new KinematicSimBody(body);
}


bool KinematicSimulatorItem::initializeSimulation(const std::vector<SimulationBody*>& simBodies)
{
    impl->cloneMap.replacePendingObjects();
    
    for(auto& simBody : simBodies){
        for(auto& holder : simBody->body()->devices<HolderDevice>()){
            impl->holders.push_back(holder);
            if(holder->on() && holder->attachment()){
                impl->activeHolders.push_back(holder);
            }
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

    for(auto& holder : impl->activeHolders){
        if(auto attachment = holder->attachment()){
            Position T_base = holder->link()->T() * holder->T_local();
            attachment->link()->T() = T_base * attachment->T_local().inverse(Eigen::Isometry);
            attachment->link()->body()->calcForwardKinematics();
        }
    }
    
    return true;
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
