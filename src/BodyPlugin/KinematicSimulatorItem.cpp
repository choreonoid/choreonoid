#include "KinematicSimulatorItem.h"
#include <cnoid/ItemManager>
#include <cnoid/BodyItem>
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


SimulationBody* KinematicSimulatorItem::createSimulationBody(Body* orgBody)
{
    auto body = new Body(*orgBody);
    auto simBody = new KinematicSimBody(body);
    return simBody;
}


bool KinematicSimulatorItem::initializeSimulation(const std::vector<SimulationBody*>& simBodies)
{
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
    return true;
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
    if(SimulationBody::initialize(simulatorItem, bodyItem)){
        if(bodyItem->baseBodyItem()){
            setActive(false);
        }
        return true;
    }
    return false;
}
