#include "AGXSimulatorItemImpl.h"
#include "AGXSimulatorItem.h"

namespace cnoid {

    AGXSimulatorItemImpl::AGXSimulatorItemImpl(AGXSimulatorItem* self)
            : self(self)
    {

    }

    AGXSimulatorItemImpl::AGXSimulatorItemImpl(AGXSimulatorItem* self, const AGXSimulatorItemImpl& org)
            : AGXSimulatorItemImpl(self)
    {

    }

    AGXSimulatorItemImpl::~AGXSimulatorItemImpl()
    {
    }

    bool AGXSimulatorItemImpl::stepSimulation(const std::vector<SimulationBody*>& activeSimBodies)
    {
        std::cout << "hogehoge" << std::endl;
        return false;
    }

}