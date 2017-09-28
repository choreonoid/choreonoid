#include "AGXSubSimulatorItem.h"
#include <cnoid/SimulatorItem>
#include <cnoid/ItemManager>
#include <cnoid/MessageView>
#include <cnoid/Archive>
#include "../AGXSimulatorItem.h"
//#include <cnoid/Body>
#include <iostream>

namespace cnoid {

void AGXSubSimulatorItem::initializeClass(ExtensionManager *ext)
{
    ItemManager &im = ext->itemManager();
    im.registerClass<AGXSubSimulatorItem>("AGXSubSimulatorItem");
    im.addCreationPanel<AGXSubSimulatorItem>();
}

AGXSubSimulatorItem::AGXSubSimulatorItem()
{
    initialize();
}

AGXSubSimulatorItem::AGXSubSimulatorItem(const AGXSubSimulatorItem &org) : SubSimulatorItem(org)
{
    initialize();
}

void AGXSubSimulatorItem::initialize()
{
    simulatorItem = nullptr;
    agxSimulatorItem = nullptr;
}

AGXSubSimulatorItem::~AGXSubSimulatorItem() {}

Item* AGXSubSimulatorItem::doDuplicate() const
{
    return new AGXSubSimulatorItem(*this);
}

bool AGXSubSimulatorItem::initializeSimulation(SimulatorItem *simulatorItem) {
    this->simulatorItem = simulatorItem;
    agxSimulatorItem = dynamic_cast<AGXSimulatorItem*>(simulatorItem);
    if(!agxSimulatorItem) return false;
    std::cout << "AGX subsimulator initialized" << std::endl;
//    SimulationBody *simSubmersible = simulatorItem->findSimulationBody("Submersible");
//    if (simSubmersible) {
//        submersible = simSubmersible->body();
//        light = submersible->findDevice<Light>("MainLight");
//        prevLightButtonState = false;
//        MessageView::instance()->putln("A submersible model has been detected.");
//        simulatorItem->addPreFunction(
//                std::bind(&SubmersibleSimulatorItem::applyResistanceForce, this));
//        joystick.reset(new Joystick());
//        joystickIntervalCounter = 0;
//    }

    return true;
}

}