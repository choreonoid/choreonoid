#include "AGXSimulatorItemImpl.h"
#include "AGXSimulatorItem.h"

namespace cnoid {

AGXSimulatorItemImpl::AGXSimulatorItemImpl(AGXSimulatorItem* self) : self(self){}
AGXSimulatorItemImpl::AGXSimulatorItemImpl(AGXSimulatorItem* self, const AGXSimulatorItemImpl& org) 
	: AGXSimulatorItemImpl(self) {}

AGXSimulatorItemImpl::~AGXSimulatorItemImpl(){}

SimulationBody * AGXSimulatorItemImpl::createSimulationBody(Body * orgBody){
	return new AGXBody(*orgBody);
}

bool AGXSimulatorItemImpl::initializeSimulation(const std::vector<SimulationBody*>& simBodies){
	return false;
}

bool AGXSimulatorItemImpl::stepSimulation(const std::vector<SimulationBody*>& activeSimBodies){
	std::cout << "hogehoge" << std::endl;
	return false;
}

}