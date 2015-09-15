/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "SubmersibleSimulatorItem.h"
#include <cnoid/SimulatorItem>
#include <cnoid/WorldItem>
#include <cnoid/ItemManager>
#include <cnoid/MessageView>
#include <cnoid/Archive>
#include <cnoid/Body>
#include <boost/bind.hpp>

#include <iostream>

using namespace std;
using namespace cnoid;
using boost::format;


void SubmersibleSimulatorItem::initializeClass(ExtensionManager* ext)
{
    ItemManager& im = ext->itemManager();
    im.registerClass<SubmersibleSimulatorItem>("SubmersibleSimulatorItem");
    im.addCreationPanel<SubmersibleSimulatorItem>();
}


SubmersibleSimulatorItem::SubmersibleSimulatorItem()
{
    initialize();
}


SubmersibleSimulatorItem::SubmersibleSimulatorItem(const SubmersibleSimulatorItem& org)
    : SubSimulatorItem(org)
{
    initialize();
    isEnabled_ = org.isEnabled_;
}


void SubmersibleSimulatorItem::initialize()
{
    simulatorItem = 0;
    submersible = 0;
    isEnabled_ = true;
}


SubmersibleSimulatorItem::~SubmersibleSimulatorItem()
{

}


bool SubmersibleSimulatorItem::isEnabled()
{
    return isEnabled_;
}


ItemPtr SubmersibleSimulatorItem::doDuplicate() const
{
    return new SubmersibleSimulatorItem(*this);
}



bool SubmersibleSimulatorItem::initializeSimulation(SimulatorItem* simulatorItem)
{
    this->simulatorItem = simulatorItem;

    SimulationBody* submersible = simulatorItem->findSimulationBody("Submersible");
    if(submersible){
        MessageView::instance()->putln("A submersible model has been detected.");
        simulatorItem->addPostDynamicsFunction(
            boost::bind(&SubmersibleSimulatorItem::applyResistanceForce, this));
    }
    
    return true;
}


void SubmersibleSimulatorItem::applyResistanceForce()
{
    Link* root = submersible->rootLink();
}


void SubmersibleSimulatorItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty("Enabled", isEnabled_, changeProperty(isEnabled_));
}


bool SubmersibleSimulatorItem::store(Archive& archive)
{
    SubSimulatorItem::store(archive);
    return true;
}


bool SubmersibleSimulatorItem::restore(const Archive& archive)
{
    SubSimulatorItem::restore(archive);
    return true;
}
