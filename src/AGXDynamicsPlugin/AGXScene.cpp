#include "AGXScene.h"

namespace cnoid{

AGXScene::AGXScene(const AGXSceneDesc& desc)
{
    _agxSimulation = AGXObjectFactory::createSimulation(desc.simdesc);
}

AGXSceneRef AGXScene::create(const AGXSceneDesc& desc)
{
    return new AGXScene(desc);
}

void AGXScene::clear(){
    agxSDK::SimulationRef sim = getSimulation();
    if(sim) sim->cleanup(agxSDK::Simulation::CLEANUP_ALL);
}

void AGXScene::stepSimulation()
{
    agx::Thread::makeCurrentThreadMainThread();
    getSimulation()->stepForward();
}

void AGXScene::add(AGXBodyPtr agxBody) {
    // Add AGXRigidbody and constraint to AGX simulation
    for(int i = 0; i < agxBody->numAGXLinks(); ++i){
        add(agxBody->getAGXRigidBody(i));
        add(agxBody->getAGXConstraint(i));
    }

    // Add bodyparts (extrajoint, continous track)
    for(int i = 0; i < agxBody->numAGXBodyParts(); ++i){
        AGXBodyPartPtr bp = agxBody->getAGXBodyPart(i);
        for(int j = 0; j < bp->numAGXConstraints(); ++j){
            add(bp->getAGXConstraint(j));
        }
        for(int j = 0; j < bp->numAGXAssemblys(); ++j){
            add(bp->getAGXAssembly(j));
        }
        if(!bp->hasSelfCollisionGroupName()) continue;
        const std::string& scgname = bp->getSelfCollisionGroupName();
        setCollisionPair(scgname, scgname, false);
    }

    // Set self collision
    if(!agxBody->bodyItem()->isSelfCollisionDetectionEnabled()){
        const std::string& scgname = agxBody->getSelfCollisionGroupName();
        setCollisionPair(scgname, scgname, false);
    }
    // Set external collision
    if(!agxBody->bodyItem()->isCollisionDetectionEnabled()){
        agxBody->setCollision(false);
    }
}

agx::Bool AGXScene::add(agx::RigidBodyRef const rigid)
{
    return getSimulation()->add(rigid);
}

agx::Bool AGXScene::add(agx::ConstraintRef const constraint)
{
    return getSimulation()->add(constraint);
}

agx::Bool AGXScene::add(agxSDK::AssemblyRef const assembly)
{
    return getSimulation()->add(assembly);
}

agx::MaterialRef AGXScene::getMaterial(const agx::String & materialName)
{
    return getSimulation()->getMaterial(materialName);
}

agx::MaterialRef AGXScene::createMaterial(const AGXMaterialDesc& desc)
{
    agx::MaterialRef mat = AGXObjectFactory::createMaterial(desc);
    getSimulation()->add(mat);
    return mat;
}

agx::ContactMaterialRef AGXScene::createContactmaterial(agx::MaterialRef const matA, agx::MaterialRef const matB, const AGXContactMaterialDesc & desc)
{
    agx::ContactMaterialRef cm = AGXObjectFactory::createContactMaterial(matA, matB, desc);
    getSimulation()->add(cm);
    return cm;
}

agx::ContactMaterialRef AGXScene::createContactMaterial(const AGXContactMaterialDesc &desc)
{
    return AGXObjectFactory::createContactMaterial(desc, getSimulation()->getMaterialManager());
}

void AGXScene::setCollisionPair(const unsigned & id1, const unsigned & id2, bool bOn)
{
    getSimulation()->getSpace()->setEnablePair(id1, id2, bOn);
}

void AGXScene::setCollisionPair(const agx::Name & name1, const agx::Name & name2, bool bOn)
{
    getSimulation()->getSpace()->setEnablePair(name1, name2, bOn);
}

agx::Vec3 AGXScene::getGravity() const
{
    return getSimulation()->getUniformGravity();
}

void AGXScene::setGravity(const agx::Vec3 & g)
{
    getSimulation()->setUniformGravity(g);
}

bool AGXScene::getEnableAutoSleep() const
{
    return getSimulation()->getDynamicsSystem()->getAutoSleep()->getEnable();
}

void AGXScene::setEnableAutoSleep(const bool & bOn)
{
    getSimulation()->getDynamicsSystem()->getAutoSleep()->setEnable(bOn);
}

bool AGXScene::saveSceneToAGXFile()
{
    if(!agxIO::writeFile("simulation.agx", getSimulation())) return false;
    return true;
}

agxSDK::SimulationRef AGXScene::getSimulation() const
{
    return _agxSimulation;
}

}