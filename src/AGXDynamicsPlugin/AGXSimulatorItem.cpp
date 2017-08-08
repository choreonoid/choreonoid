#include "AGXSimulatorItem.h"
#include "AGXSimulatorItemImpl.h"
#include <cnoid/ItemManager>
#include "gettext.h"

namespace cnoid {
//using namespace std;
////using namespace cnoid;

void AGXSimulatorItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerClass<AGXSimulatorItem>("AGXSimulatorItem");
    ext->itemManager().addCreationPanel<AGXSimulatorItem>();
}

AGXSimulatorItem::AGXSimulatorItem()
{
    impl = new AGXSimulatorItemImpl(this);
}

AGXSimulatorItem::AGXSimulatorItem(const AGXSimulatorItem& org): SimulatorItem(org)
{
    impl = new AGXSimulatorItemImpl(this, *org.impl);
}

AGXSimulatorItem::~AGXSimulatorItem()
{
    delete impl;
}

bool AGXSimulatorItem::saveSimulationToAGXFile()
{
    return impl->saveSimulationToAGXFile();
}

Item* AGXSimulatorItem::doDuplicate() const
{
    return new AGXSimulatorItem(*this);
}

void AGXSimulatorItem::doPutProperties(PutPropertyFunction & putProperty){
    SimulatorItem::doPutProperties(putProperty);
    impl->doPutProperties(putProperty);
}

//bool AGXSimulatorItem::store(Archive & archive){
//    // save the common properties of SimulatorItem
//    SimulatorItem::store(archive);
//    //save the agx properties
//    return impl->store(archive);
//}
//
//bool AGXSimulatorItem::restore(const Archive & archive){
//    // restore the common properties of SimulatorItem
//    SimulatorItem::restore(archive);
//    // restore the agx properties
//    return impl->restore(archive);
//}

SimulationBody* AGXSimulatorItem::createSimulationBody(Body* orgBody)
{
    return impl->createSimulationBody(orgBody);
}

bool AGXSimulatorItem::initializeSimulation(const std::vector<SimulationBody*>& simBodies)
{
    return impl->initializeSimulation(simBodies);
}

bool AGXSimulatorItem::stepSimulation(const std::vector<SimulationBody*>& activeSimBodies)
{
    return impl->stepSimulation(activeSimBodies);
}

void AGXSimulatorItem::stopSimulation()
{
    impl->stopSimulation();
    SimulatorItem::stopSimulation();
}

void AGXSimulatorItem::pauseSimulation()
{
    impl->pauseSimulation();
    SimulatorItem::pauseSimulation();
}

void AGXSimulatorItem::restartSimulation()
{
    impl->restartSimulation();
    SimulatorItem::restartSimulation();
}




}