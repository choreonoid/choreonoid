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

    AGXSimulatorItem::AGXSimulatorItem(const AGXSimulatorItem& org)
            : SimulatorItem(org)
    {
        impl = new AGXSimulatorItemImpl(this, *org.impl);
    }

    AGXSimulatorItem::~AGXSimulatorItem()
    {
        delete impl;
    }

    Item* AGXSimulatorItem::doDuplicate() const
    {
        return new AGXSimulatorItem(*this);
    }

    SimulationBody* AGXSimulatorItem::createSimulationBody(Body* orgBody)
    {
        return 0;
    }

    bool AGXSimulatorItem::initializeSimulation(const std::vector<SimulationBody*>& simBodies)
    {
        return false;
    }

    bool AGXSimulatorItem::stepSimulation(const std::vector<SimulationBody*>& activeSimBodies)
    {
        return false;
    }

}