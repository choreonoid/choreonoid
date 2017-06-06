#ifndef CNOID_AGXDYNAMICS_PLUGIN_AGX_SIMULATOR_ITEM_H
#define CNOID_AGXDYNAMICS_PLUGIN_AGX_SIMULATOR_ITEM_H

#include <cnoid/SimulatorItem>
#include "exportdecl.h"

namespace cnoid {

class AGXSimulatorItemImpl;

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

private:
	AGXSimulatorItemImpl* impl;
//	friend class AGXSimulatorItemImpl;
};

typedef ref_ptr<AGXSimulatorItem> AGXSimulatorItemPtr;

}

#endif
