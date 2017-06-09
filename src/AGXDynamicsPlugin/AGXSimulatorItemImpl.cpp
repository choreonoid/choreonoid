#include "AGXSimulatorItemImpl.h"
#include "AGXSimulatorItem.h"
#include <assert.h>

using namespace std;

namespace cnoid {

AGXSimulatorItemImpl::AGXSimulatorItemImpl(AGXSimulatorItemPtr self) : self(self){
	initialize();
}
AGXSimulatorItemImpl::AGXSimulatorItemImpl(AGXSimulatorItemPtr self, const AGXSimulatorItemImpl& org) 
	: AGXSimulatorItemImpl(self) {
	initialize();	
}
AGXSimulatorItemImpl::~AGXSimulatorItemImpl(){}

void AGXSimulatorItemImpl::initialize(){
	// Write defalut parameter here
	//agxScene = nullptr;
}

void AGXSimulatorItemImpl::doPutProperties(PutPropertyFunction & putProperty){
	//putProperty(_("Step mode"), stepMode, changeProperty(stepMode));
	//putProperty(("Step mode"), "hoge");
}

bool AGXSimulatorItemImpl::store(Archive & archive){
	// Write agx parameter to save
	//archive.write("friction", friction);
	return false;
}

bool AGXSimulatorItemImpl::restore(const Archive & archive){
	// Write agx parameter to restore
	return false;
}

SimulationBody * AGXSimulatorItemImpl::createSimulationBody(Body * orgBody){
	// When user click start bottom, this function will be called first.
	return new AGXBody(*orgBody);
}

//#define AGX_SCENE
bool AGXSimulatorItemImpl::initializeSimulation(const std::vector<SimulationBody*>& simBodies){
	cout << "initializeSimulation" << endl;
#ifdef AGX_SCENE
	if(agxScene) agxScene->clearScene();
	agxScene = new AGXScene();
	agxScene->initializeScene();
	agxScene->buildTestScene();

	//clearAGXSimulation();
	//agxSDK::SimulationRef agxSim = createAGXSimulation();
#else
	const agx::Thread* cur = agx::Thread::getCurrentThread();
	const agx::Thread* main = agx::Thread::getMainThread();
	agxAssert(cur, main);
	cout << cur << ";" << main << endl;

	//if(agxSimulation) agxSimulation->cleanup(agxSDK::Simulation::CLEANUP_ALL);
	agxSimulation = new agxSDK::Simulation();
	//agxSimulation->setMainWorkThread(agx::Thread::getCurrentThread());

	//// boxì¬
	//agx::RigidBodyRef rigidBox = new agx::RigidBody();
	//agxCollide::GeometryRef geometryBox = new agxCollide::Geometry();
	//agxCollide::BoxRef shapeBox = new agxCollide::Box(agx::Vec3d(0.5, 0.5, 0.5));
	//geometryBox->add(shapeBox);
	//rigidBox->add(geometryBox);
	//rigidBox->setPosition(agx::Vec3d(0.0, 0.0, 5.0));
	//agxSimulation->add(rigidBox);

	//// Floor‚Ìì¬
	//agx::RigidBodyRef rigidFloor = new agx::RigidBody();
	//rigidFloor->add(new agxCollide::Geometry(new agxCollide::Box(agx::Vec3(5.0, 5.0, 0.2))));
	//rigidFloor->setMotionControl(agx::RigidBody::STATIC);
	//rigidFloor->setPosition(agx::Vec3(0, 0, -0.2));
	//agxSimulation->add(rigidFloor);
#endif

	return true;
}

bool AGXSimulatorItemImpl::stepSimulation(const std::vector<SimulationBody*>& activeSimBodies){
	cout << "step" << std::endl;
#ifdef AGX_SCENE
	cout << agxScene->getSimulation() << endl;
	if(agxScene) agxScene->stepSimulation();
#else
	agx::Thread::makeCurrentThreadMainThread();
	const agx::Thread* cur = agx::Thread::getCurrentThread();
	cout << cur << endl;
	agxSimulation->stepForward();
#endif
	return true;
}

void AGXSimulatorItemImpl::stopSimulation(){
	cout << "stopSimulation" << endl;
}

void AGXSimulatorItemImpl::pauseSimulation(){
	cout << "pauseSimulation" << endl;
}

void AGXSimulatorItemImpl::restartSimulation(){
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

bool AGXSimulatorItemImpl::saveSimulationToAGXFile(){
//	return agxScene->saveSceneToAGXFile();
//	if(!agxIO::writeFile("simulation.agx", getAGXSimulation())) return false;
	return true;
}

}