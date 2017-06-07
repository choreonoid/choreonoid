#include "AGXSimulatorItemImpl.h"
#include "AGXSimulatorItem.h"
#include <assert.h>

using namespace std;

namespace cnoid {

AGXSimulatorItemImpl::AGXSimulatorItemImpl(AGXSimulatorItemPtr self) : self(self){}
AGXSimulatorItemImpl::AGXSimulatorItemImpl(AGXSimulatorItemPtr self, const AGXSimulatorItemImpl& org) 
	: AGXSimulatorItemImpl(self) {}

AGXSimulatorItemImpl::~AGXSimulatorItemImpl(){}

SimulationBody * AGXSimulatorItemImpl::createSimulationBody(Body * orgBody){
	return new AGXBody(*orgBody);
}

bool AGXSimulatorItemImpl::initializeSimulation(const std::vector<SimulationBody*>& simBodies){
	cout << "initializeSimulation" << endl;
	clearAGXSimulation();
	agxSDK::SimulationRef agxSim = createAGXSimulation();
	if(agxSim == nullptr) return false; 
	return true;
}

bool AGXSimulatorItemImpl::stepSimulation(const std::vector<SimulationBody*>& activeSimBodies){
//	cout << "hogehoge" << std::endl;
	return false;
}

void AGXSimulatorItemImpl::stopSimulation(){
	cout << "stopSimulation" << endl;
}

void AGXSimulatorItemImpl::pauseSimulation(){
	cout << "pauseSimulation" << endl;
}

void AGXSimulatorItemImpl::restartSimulation(){
	cout << "restartSimulation" << endl;
	saveAGXSimulationToFile();
}


// API
void AGXSimulatorItemImpl::clearAGXSimulation(){
	if(getAGXSimulation()) getAGXSimulation()->cleanup(agxSDK::Simulation::CLEANUP_ALL);
}

agxSDK::SimulationRef AGXSimulatorItemImpl::createAGXSimulation(){
	agxSimulation = new agxSDK::Simulation();
	return agxSimulation;
}

agxSDK::SimulationRef AGXSimulatorItemImpl::getAGXSimulation(){
	return agxSimulation;
}

bool AGXSimulatorItemImpl::saveAGXSimulationToFile(){
	if(!agxIO::writeFile("simulation.agx", getAGXSimulation())){
		return false;
	}
	return true;
}

}