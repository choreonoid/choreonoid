#ifndef CNOID_AGXDYNAMICS_PLUGIN_AGX_SIMULATOR_ITEM_H
#define CNOID_AGXDYNAMICS_PLUGIN_AGX_SIMULATOR_ITEM_H

#include <cnoid/SimulatorItem>
#include "exportdecl.h"

namespace cnoid {

class AGXSimulatorItemImpl;
typedef ref_ptr<AGXSimulatorItemImpl> AGXSimulatorItemImplPtr;

class CNOID_EXPORT AGXSimulatorItem : public SimulatorItem{
public:
	static void initializeClass(ExtensionManager* ext);
	AGXSimulatorItem();
	AGXSimulatorItem(const AGXSimulatorItem& org);
	virtual ~AGXSimulatorItem();

protected:
	virtual Item* doDuplicate() const;
	virtual SimulationBody* createSimulationBody(Body* orgBody);
	virtual bool initializeSimulation(const std::vector<SimulationBody*>& simBodies);
	virtual bool stepSimulation(const std::vector<SimulationBody*>& activeSimBodies);
	virtual void stopSimulation();
	virtual void pauseSimulation();
	virtual void restartSimulation();

private:
	AGXSimulatorItemImplPtr impl;
//	friend class AGXSimulatorItemImpl;
};



}

#endif
