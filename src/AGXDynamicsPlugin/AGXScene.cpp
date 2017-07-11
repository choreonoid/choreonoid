#include "AGXScene.h"
#include <agx/Thread.h>
#include <agxIO/ReaderWriter.h>

namespace cnoid{

AGXScene::AGXScene(){}

AGXScene* AGXScene::create()
{
    return new AGXScene();
}

void AGXScene::clearAGXScene()
{ 
    if(agxSimulation) agxSimulation->cleanup(agxSDK::Simulation::CLEANUP_ALL);
}

void AGXScene::stepAGXSimulation()
{
    agx::Thread::makeCurrentThreadMainThread();
    agxSimulation->stepForward();
}

agxSDK::SimulationRef AGXScene::getAGXSimulation()
{
    return agxSimulation;
}

agxSDK::SimulationRef AGXScene::createAGXSimulation(const AGXSimulationDesc& desc)
{
    if(agxSimulation) return agxSimulation;
    agxSimulation = new agxSDK::Simulation();
    agxSimulation->setTimeStep(desc.timeStep);
    return agxSimulation;
}

bool AGXScene::saveSceneToAGXFile()
{
    if(!agxIO::writeFile("simulation.agx", agxSimulation)) return false;
    return true;
}

void AGXScene::setCollisionPair(const unsigned & id1, const unsigned & id2, bool bOn)
{
    agxSimulation->getSpace()->setEnablePair(id1, id2, bOn);
}

void AGXScene::setCollisionPair(const agx::Name & name1, const agx::Name & name2, bool bOn)
{
    agxSimulation->getSpace()->setEnablePair(name1, name2, bOn);
}

}