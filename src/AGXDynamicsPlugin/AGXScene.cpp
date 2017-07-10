#include "AGXScene.h"

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
    //agx::Thread::unregisterAsAgxThread();
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

void AGXScene::buildTestScene()
{
    // box
    agx::RigidBodyRef rigidBox = new agx::RigidBody();
    agxCollide::GeometryRef geometryBox = new agxCollide::Geometry();
    agxCollide::BoxRef shapeBox = new agxCollide::Box(agx::Vec3d(0.5, 0.5, 0.5));
    geometryBox->add(shapeBox);
    rigidBox->add(geometryBox);
    rigidBox->setPosition(agx::Vec3d(0.0, 0.0, 5.0));
    agxSimulation->add(rigidBox);

    // Floor
    agx::RigidBodyRef rigidFloor = new agx::RigidBody();
    rigidFloor->add(new agxCollide::Geometry(new agxCollide::Box(agx::Vec3(5.0, 5.0, 0.2))));
    rigidFloor->setMotionControl(agx::RigidBody::STATIC);
    rigidFloor->setPosition(agx::Vec3(0, 0, -0.2));
    agxSimulation->add(rigidFloor);
}

}