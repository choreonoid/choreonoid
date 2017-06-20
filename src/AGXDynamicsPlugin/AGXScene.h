#ifndef CNOID_AGXDYNAMICS_PLUGIN_AGX_SCENE_H
#define CNOID_AGXDYNAMICS_PLUGIN_AGX_SCENE_H

#include "AGXInclude.h"

namespace cnoid{


struct AGXSimulationDesc
{
	
};

class AGXScene : public agx::Referenced{
public:
	AGXScene();
	static AGXScene* create();
	void initializeScene();
	void clearAGXScene();
	void stepAGXSimulation();
	bool saveSceneToAGXFile();
	void buildTestScene();
	agxSDK::SimulationRef getAGXSimulation();
private:
	agxSDK::SimulationRef agxSimulation;
	agxSDK::SimulationRef createAGXSimulation();
};
typedef agx::ref_ptr<AGXScene> AGXSceneRef;
}
#endif