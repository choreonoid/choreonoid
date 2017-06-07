#ifndef CNOID_AGXDYNAMICS_PLUGIN_AGX_SIMULATOR_ITEM_IMPL_H
#define CNOID_AGXDYNAMICS_PLUGIN_AGX_SIMULATOR_ITEM_IMPL_H

#include <cnoid/SimulatorItem>
#include "AGXInclude.h"
#include "AGXBody.h"
#include <iostream>
//#include<vector>

namespace cnoid {
class AGXSimulatorItem;
typedef ref_ptr<AGXSimulatorItem> AGXSimulatorItemPtr;

class AGXSimulatorItemImpl : public Referenced{
public:
	AGXSimulatorItemPtr self;

	AGXSimulatorItemImpl(AGXSimulatorItemPtr self);
	AGXSimulatorItemImpl(AGXSimulatorItemPtr self, const AGXSimulatorItemImpl& org);
	~AGXSimulatorItemImpl();
	SimulationBody* createSimulationBody(Body* orgBody);
	/** Build AGX simulation */
	bool initializeSimulation(const std::vector<SimulationBody*>& simBodies);
	bool stepSimulation(const std::vector<SimulationBody*>& activeSimBodies);
	void stopSimulation();
	void pauseSimulation();
	void restartSimulation();

	// API
	void clearAGXSimulation();
	agxSDK::SimulationRef createAGXSimulation();
	agxSDK::SimulationRef getAGXSimulation();
	bool saveAGXSimulationToFile();
	
private:
	agx::AutoInit agxInit;
	agxSDK::SimulationRef agxSimulation = nullptr;
};
}
#endif