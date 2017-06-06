#ifndef CNOID_AGXDYNAMICS_PLUGIN_AGX_SIMULATOR_ITEM_IMPL_H
#define CNOID_AGXDYNAMICS_PLUGIN_AGX_SIMULATOR_ITEM_IMPL_H

#include <cnoid/SimulatorItem>
#include "AGXBody.h"
#include <iostream>
//#include<vector>

namespace cnoid {
class AGXSimulatorItem;

class AGXSimulatorItemImpl
{
public:
	AGXSimulatorItem* self;

	AGXSimulatorItemImpl(AGXSimulatorItem* self);
	AGXSimulatorItemImpl(AGXSimulatorItem* self, const AGXSimulatorItemImpl& org);
	~AGXSimulatorItemImpl();
	SimulationBody* createSimulationBody(Body* orgBody);
	/** Build AGX simulation */
	bool initializeSimulation(const std::vector<SimulationBody*>& simBodies);
	bool stepSimulation(const std::vector<SimulationBody*>& activeSimBodies);
};
}
#endif