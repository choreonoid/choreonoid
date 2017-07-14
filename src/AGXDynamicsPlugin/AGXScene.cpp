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

agxSDK::SimulationRef AGXScene::createAGXSimulation(const AGXSimulationDesc& desc)
{
    if(agxSimulation) return agxSimulation;
    agxSimulation = new agxSDK::Simulation();
    agxSimulation->setTimeStep(desc.timeStep);
    return agxSimulation;
}

agxSDK::SimulationRef AGXScene::getAGXSimulation()
{
    return agxSimulation;
}

void AGXScene::createAGXMaterial(const AGXMaterialDesc& desc)
{
    agx::MaterialRef m = new agx::Material(desc.name);
    m->getBulkMaterial()->setDensity(desc.density);
    m->getBulkMaterial()->setYoungsModulus(desc.youngsModulus);
    m->getBulkMaterial()->setPoissonsRatio(desc.poissonRatio);

    // Below are overried when ContactMaterials are used.
    m->getBulkMaterial()->setViscosity(desc.viscosity);
    m->getBulkMaterial()->setDamping(desc.damping);
    m->getSurfaceMaterial()->setRoughness(desc.roughness);
    m->getSurfaceMaterial()->setViscosity(desc.surfaceViscosity);
    m->getSurfaceMaterial()->setAdhesion(desc.adhesionForce, desc.adhesivOverlap);

    agxSimulation->getMaterialManager()->add(m);
}

agx::MaterialRef AGXScene::getAGXMaterial(const agx::String& materialName)
{
    return getAGXSimulation()->getMaterialManager()->getMaterial(materialName);
}

void AGXScene::createAGXContactMaterial(const AGXContactMaterialDesc& desc)
{
    agxSDK::MaterialManagerRef mgr = agxSimulation->getMaterialManager();
    agx::MaterialRef mA = mgr->getMaterial(desc.nameA);
    agx::MaterialRef mB = mgr->getMaterial(desc.nameB);
    agx::ContactMaterialRef cm = mgr->getOrCreateContactMaterial(mA, mB);
    cm->setYoungsModulus(desc.youngsModulus);
    cm->setRestitution(desc.restitution);
    cm->setDamping(desc.damping);
    cm->setFrictionCoefficient(desc.friction);
    cm->setAdhesion(desc.adhesionForce, desc.adhesivOverlap);
    cm->setSurfaceViscosity(desc.surfaceViscosity, desc.frictionDirection);

    // Create friction model
    if(desc.frictionModelType == AGXFrictionModelType::DEFAULT) return;
    agx::FrictionModelRef fm;
    switch (desc.frictionModelType){
        case AGXFrictionModelType::BOX :
            fm = new agx::BoxFrictionModel();
            break;
        case AGXFrictionModelType::SCALE_BOX :
            fm = new agx::ScaleBoxFrictionModel();
            break;
        case AGXFrictionModelType::ITERATIVE_PROJECTED_CONE :
            fm = new agx::IterativeProjectedConeFriction();
            break;
        case AGXFrictionModelType::DEFAULT:
        default:
            return;
            break;
    }
    fm->setSolveType(desc.solveType);
    cm->setFrictionModel(fm);
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