#include "AGXSimulatorItem.h"
#include <cnoid/ItemManager>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

class AGXSimulatorItemImpl
{
public:
    AGXSimulatorItem* self;

    AGXSimulatorItemImpl(AGXSimulatorItem* self);
    AGXSimulatorItemImpl(AGXSimulatorItem* self, const AGXSimulatorItemImpl& org);
    ~AGXSimulatorItemImpl();
};

}


void AGXSimulatorItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerClass<AGXSimulatorItem>("AGXSimulatorItem");
    ext->itemManager().addCreationPanel<AGXSimulatorItem>();
}


AGXSimulatorItem::AGXSimulatorItem()
{
    impl = new AGXSimulatorItemImpl(this);
}


AGXSimulatorItemImpl::AGXSimulatorItemImpl(AGXSimulatorItem* self)
    : self(self)
{

}


AGXSimulatorItem::AGXSimulatorItem(const AGXSimulatorItem& org)
    : SimulatorItem(org)
{
    impl = new AGXSimulatorItemImpl(this, *org.impl);
}


AGXSimulatorItemImpl::AGXSimulatorItemImpl(AGXSimulatorItem* self, const AGXSimulatorItemImpl& org)
    : AGXSimulatorItemImpl(self)
{

}


AGXSimulatorItem::~AGXSimulatorItem()
{
    delete impl;
}


AGXSimulatorItemImpl::~AGXSimulatorItemImpl()
{

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

