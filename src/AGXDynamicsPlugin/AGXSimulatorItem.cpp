#include "AGXSimulatorItem.h"
#include "AGXSimulatorItemImpl.h"
#include <cnoid/ItemManager>
#include "gettext.h"

namespace cnoid {
//using namespace std;
////using namespace cnoid;

void AGXSimulatorItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerClass<AGXSimulatorItem, SimulatorItem>("AGXSimulatorItem");
    ext->itemManager().addCreationPanel<AGXSimulatorItem>();
}

AGXSimulatorItem::AGXSimulatorItem()
{
    impl = new AGXSimulatorItemImpl(this);
    setName("AGXSimulator");
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

Vector3 AGXSimulatorItem::getGravity() const
{
    return impl->getGravity();
}

void AGXSimulatorItem::setNumThreads(unsigned int num)
{
    impl->setNumThreads(num);
}

void AGXSimulatorItem::setEnableContactReduction(bool bOn)
{
    impl->setEnableContactReduction(bOn);
}

void AGXSimulatorItem::setContactReductionBinResolution(int r)
{
    impl->setContactReductionBinResolution(r);
}

void AGXSimulatorItem::setContactReductionThreshhold(int t)
{
    impl->setContactReductionThreshhold(t);
}

void AGXSimulatorItem::setEnableContactWarmstarting(bool bOn)
{
    impl->setEnableContactWarmstarting(bOn);
}

void AGXSimulatorItem::setEnableAMOR(bool bOn)
{
    impl->setEnableAMOR(bOn);
}

Item* AGXSimulatorItem::doDuplicate() const
{
    return new AGXSimulatorItem(*this);
}

void AGXSimulatorItem::doPutProperties(PutPropertyFunction & putProperty){
    setAllLinkPositionOutputMode(true);
    SimulatorItem::doPutProperties(putProperty);
    impl->doPutProperties(putProperty);
}

bool AGXSimulatorItem::store(Archive & archive){
    SimulatorItem::store(archive);
    return impl->store(archive);
}

bool AGXSimulatorItem::restore(const Archive & archive){
    SimulatorItem::restore(archive);
    return impl->restore(archive);
}

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

}
