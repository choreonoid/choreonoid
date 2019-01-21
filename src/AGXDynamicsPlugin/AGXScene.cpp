#include "AGXScene.h"
#include <agx/version.h>

namespace cnoid{

AGXScene::AGXScene(const AGXSceneDesc& desc)
{
    _agxSimulation = AGXObjectFactory::createSimulation(desc.simdesc);
}

AGXScene* AGXScene::create(const AGXSceneDesc& desc)
{
    return new AGXScene(desc);
}

void AGXScene::clear(){
    agxSDK::SimulationRef sim = getSimulation();
    if(sim) sim->cleanup(agxSDK::Simulation::CLEANUP_ALL);
}

void AGXScene::setMainWorkThread()
{
#if AGX_VERSION_GREATER_OR_EQUAL(2 ,21, 0, 0)
    agx::Thread::registerAsAgxThread();
    getSimulation()->setMainWorkThread(agx::Thread::getCurrentThread());
#else
    agx::Thread::makeCurrentThreadMainThread();
#endif
}

void AGXScene::stepSimulation()
{
    getSimulation()->stepForward();
}

agx::Bool AGXScene::add(agx::RigidBody* const rigid)
{
    return getSimulation()->add(rigid);
}

agx::Bool AGXScene::add(agx::Constraint* const constraint)
{
    return getSimulation()->add(constraint);
}

agx::Bool AGXScene::add(agxSDK::Assembly* const assembly)
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

#define PRINT_MATERIAL(field, field2) std::cout << "  " << #field << " " << field2 << std::endl
void AGXScene::printMaterials()
{
    for(auto it : getSimulation()->getMaterialManager()->getMaterials()){
        agx::Material* const mat = it.second;
    }
}

void AGXScene::printContactMaterialTable()
{
    for(auto it : getSimulation()->getMaterialManager()->getContactMaterials()){
        agx::ContactMaterial* const mat = it.second;
        std::cout << "[" << mat->getMaterial1()->getName() << " " << mat->getMaterial2()->getName() << "]" << std::endl;
        PRINT_MATERIAL(youngsModulus, mat->getYoungsModulus());
        PRINT_MATERIAL(restitution, mat->getRestitution());
        PRINT_MATERIAL(spookDamping, mat->getDamping());
        PRINT_MATERIAL(friction, mat->getFrictionCoefficient());
        PRINT_MATERIAL(secondaryFriction, mat->getFrictionCoefficient(agx::ContactMaterial::SECONDARY_DIRECTION));
        PRINT_MATERIAL(surfaceViscosity, mat->getSurfaceViscosity());
        PRINT_MATERIAL(secondarySurfaceViscosity, mat->getSurfaceViscosity(agx::ContactMaterial::SECONDARY_DIRECTION));
        PRINT_MATERIAL(enableSurfaceFriction, mat->getSurfaceFrictionEnabled());
        PRINT_MATERIAL(adhesionForce, mat->getAdhesion());
        PRINT_MATERIAL(adhesivOverLap, mat->getAdhesiveOverlap());
        PRINT_MATERIAL(contactReductionMode, mat->getContactReductionMode());
        PRINT_MATERIAL(contactReductionBinResolution, mat->getContactReductionBinResolution());
        if(!mat->getFrictionModel()) continue;
        PRINT_MATERIAL(frictionModel, mat->getFrictionModel()->getClassName());
        PRINT_MATERIAL(solveType, mat->getFrictionModel()->getSolveType());

    }
}
#undef PRINT_MATERIAL

void AGXScene::setCollision(const agx::Name& name, bool bOn)
{
    getSimulation()->getSpace()->setEnablePair(name, name, bOn);
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