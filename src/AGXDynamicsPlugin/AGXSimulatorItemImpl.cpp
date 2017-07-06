#include "AGXSimulatorItemImpl.h"
#include "AGXSimulatorItem.h"
#include <assert.h>

using namespace std;

namespace cnoid {

AGXSimulatorItemImpl::AGXSimulatorItemImpl(AGXSimulatorItemPtr self) : self(self)
{
	initialize();
}
AGXSimulatorItemImpl::AGXSimulatorItemImpl(AGXSimulatorItemPtr self, const AGXSimulatorItemImpl& org) 
	: AGXSimulatorItemImpl(self) 
{
	initialize();	
}
AGXSimulatorItemImpl::~AGXSimulatorItemImpl(){}

void AGXSimulatorItemImpl::initialize()
{
	// Write defalut parameter here
	//agxScene = nullptr;
}

void AGXSimulatorItemImpl::doPutProperties(PutPropertyFunction & putProperty)
{
	//putProperty(_("Step mode"), stepMode, changeProperty(stepMode));
	//putProperty(("Step mode"), "hoge");
}

bool AGXSimulatorItemImpl::store(Archive & archive)
{
	// Write agx parameter to save
	//archive.write("friction", friction);
	return false;
}

bool AGXSimulatorItemImpl::restore(const Archive & archive)
{
	// Write agx parameter to restore
	return false;
}

SimulationBody * AGXSimulatorItemImpl::createSimulationBody(Body * orgBody)
{
	// When user click start bottom, this function will be called first.
	return new AGXBody(*orgBody);
}

#define AGX_SCENE
bool AGXSimulatorItemImpl::initializeSimulation(const std::vector<SimulationBody*>& simBodies)
{
	cout << "initializeSimulation" << endl;
	if(agxScene) agxScene->clearAGXScene();
	agxScene = AGXScene::create();
	agxScene->clearAGXScene();
	AGXSimulationDesc sd;
	sd.timeStep = self->worldTimeStep();
	agxScene->createAGXSimulation(sd);
//	agxScene->buildTestScene();

	// Create AGXLink and add to AGXsimulation
	for(size_t i=0; i < simBodies.size(); ++i){
		AGXBody* body = static_cast<AGXBody*>(simBodies[i]);
		body->createBody();
		for(int j = 0; j < body->getNumLinks(); ++j){
			agxScene->getAGXSimulation()->add(body->getAGXRigidBody(j));
			agxScene->getAGXSimulation()->add(body->getAGXConstraint(j));
		}
		// Set self collision
		if(!body->bodyItem()->isSelfCollisionDetectionEnabled()){
			agxScene->setCollisionPair(body->bodyItem()->name(), body->bodyItem()->name(), false); 
		}
		// Set external collision
		if(!body->bodyItem()->isCollisionDetectionEnabled()){
			body->setCollision(false);
		}
	}

	saveSimulationToAGXFile();

	return true;
}

bool AGXSimulatorItemImpl::stepSimulation(const std::vector<SimulationBody*>& activeSimBodies)
{
//	cout << "step" << std::endl;

	agxScene->stepAGXSimulation();

    for(size_t i=0; i < activeSimBodies.size(); ++i){
        AGXBody* agxBody = static_cast<AGXBody*>(activeSimBodies[i]);
        agxBody->synchronizeLinkStateToCnoid();

        //if(!agxBody->sensorHelper.forceSensors().empty()){
        //    agxBody->updateForceSensors();
        //}
        //if(agxBody->sensorHelper.hasGyroOrAccelerationSensors()){
        //    agxBody->sensorHelper.updateGyroAndAccelerationSensors();
        //}
    }

	return true;
}

void AGXSimulatorItemImpl::stopSimulation()
{
	cout << "stopSimulation" << endl;
}

void AGXSimulatorItemImpl::pauseSimulation()
{
	cout << "pauseSimulation" << endl;
}

void AGXSimulatorItemImpl::restartSimulation()
{
	cout << "restartSimulation" << endl;
}


//// API
//void AGXSimulatorItemImpl::clearAGXSimulation(){
//	if(getAGXSimulation()) getAGXSimulation()->cleanup(agxSDK::Simulation::CLEANUP_ALL);
//}
//
//agxSDK::SimulationRef AGXSimulatorItemImpl::createAGXSimulation(){
//	agxSimulation = new agxSDK::Simulation();
//	return agxSimulation;
//}
//
//agxSDK::SimulationRef AGXSimulatorItemImpl::getAGXSimulation(){
//	return agxSimulation;
//}

bool AGXSimulatorItemImpl::saveSimulationToAGXFile()
{
	return agxScene->saveSceneToAGXFile();
}

//AGXSceneRef AGXSimulatorItemImpl::createAGXScene()
//{
//	if(agxScene) return agxScene;
//	agxScene = new AGXScene();
//	return  agxScene;
//}

//AGXSceneRef AGXSimulatorItemImpl::getAGXScene()
//{
//	return agxScene;
//}

}