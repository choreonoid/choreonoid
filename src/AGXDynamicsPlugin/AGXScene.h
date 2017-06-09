#ifndef CNOID_AGXDYNAMICS_PLUGIN_AGX_SCENE_H
#define CNOID_AGXDYNAMICS_PLUGIN_AGX_SCENE_H

#include "AGXInclude.h"

namespace cnoid{

class AGXScene{
public:
	AGXScene();
	static AGXScene* create();
	void initializeScene();
	void clearScene();
	void stepSimulation();
	bool saveSceneToAGXFile();
	void buildTestScene();
	agxSDK::SimulationRef getSimulation();
private:
	agxSDK::SimulationRef agxSimulation;
};
//typedef agx::ref_ptr<AGXScene> AGXSceneRef;
}
#endif