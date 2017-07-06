#ifndef CNOID_AGXDYNAMICS_PLUGIN_AGX_SCENE_H
#define CNOID_AGXDYNAMICS_PLUGIN_AGX_SCENE_H

#include "AGXInclude.h"

namespace cnoid{


struct AGXSimulationDesc
{
	AGXSimulationDesc(){}
	agx::UInt8 binResolution;
	agx::UInt  threshhold;
	agx::Vec3  gravity;
	agx::Real  timeStep;
};

class AGXScene : public agx::Referenced{
public:
	AGXScene();
	static AGXScene* create();
	void clearAGXScene();
	void stepAGXSimulation();
	bool saveSceneToAGXFile();
	void setCollisionPair(const unsigned& id1, const unsigned& id2, bool bOn);
	void setCollisionPair(const agx::Name& name1, const agx::Name& name2, bool bOn);

	void buildTestScene();
	agxSDK::SimulationRef createAGXSimulation(const AGXSimulationDesc& desc);
	agxSDK::SimulationRef getAGXSimulation();
private:
	agxSDK::SimulationRef agxSimulation;

};
typedef agx::ref_ptr<AGXScene> AGXSceneRef;
}
#endif