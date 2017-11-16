#include "AGXSimulatorItemImpl.h"
#include "AGXSimulatorItem.h"
#include <cnoid/EigenUtil>
#include <cnoid/WorldItem>
#include <cnoid/MaterialTable>
#include "AGXConvert.h"
#include <unordered_map>
#include "gettext.h"

using namespace std;

namespace cnoid {

const std::unordered_map<std::string, agx::ContactMaterial::ContactReductionMode> agxContactReductionModeMap{
    {"", agx::ContactMaterial::ContactReductionMode::REDUCE_GEOMETRY},
    {"default", agx::ContactMaterial::ContactReductionMode::REDUCE_GEOMETRY},
    {"reduceGeometry", agx::ContactMaterial::ContactReductionMode::REDUCE_GEOMETRY},
    {"reduceALL", agx::ContactMaterial::ContactReductionMode::REDUCE_ALL},
    {"reduceNone", agx::ContactMaterial::ContactReductionMode::REDUCE_NONE}
};

const std::unordered_map<std::string, AGXFrictionModelType> agxFrictionModelTypeMap{
    {"", AGXFrictionModelType::DEFAULT},
    {"default", AGXFrictionModelType::DEFAULT},
    {"box", AGXFrictionModelType::BOX},
    {"scaledBox", AGXFrictionModelType::SCALED_BOX},
    {"orientedBox", AGXFrictionModelType::CONSTANT_NORMAL_FORCE_ORIENTED_BOX_FRICTIONMODEL},
    {"cone", AGXFrictionModelType::ITERATIVE_PROJECTED_CONE}
};

const std::unordered_map<std::string, agx::FrictionModel::SolveType> agxSolveTypeMap{
    {"", agx::FrictionModel::SolveType::SPLIT},
    {"default", agx::FrictionModel::SolveType::SPLIT},
    {"split", agx::FrictionModel::SolveType::SPLIT},
    {"direct", agx::FrictionModel::SolveType::DIRECT},
    {"iterative", agx::FrictionModel::SolveType::ITERATIVE},
    {"directAndIterative", agx::FrictionModel::SolveType::DIRECT_AND_ITERATIVE}
};

AGXSimulatorItemImpl::AGXSimulatorItemImpl(AGXSimulatorItem* self)
    : self(self)
{
    initialize();
    AGXSimulationDesc simDesc;
    const agx::Vec3& g = simDesc.gravity;
    _p_gravity = Vector3(g.x(), g.y(), g.z());
    _p_numThreads = (int)simDesc.numThreads;
    _p_enableContactReduction = simDesc.enableContactReduction;
    _p_contactReductionBinResolution = simDesc.contactReductionBinResolution;
    _p_contactReductionThreshhold = (int)simDesc.contactReductionThreshhold;
    _p_enableAutoSleep = simDesc.enableAutoSleep;
}

AGXSimulatorItemImpl::AGXSimulatorItemImpl(AGXSimulatorItem* self, const AGXSimulatorItemImpl& org)
    : AGXSimulatorItemImpl(self)
{
    (void) org;
    initialize();
}
AGXSimulatorItemImpl::~AGXSimulatorItemImpl(){}

void AGXSimulatorItemImpl::initialize(){}

void AGXSimulatorItemImpl::doPutProperties(PutPropertyFunction & putProperty)
{
    putProperty(_("Gravity"), str(_p_gravity), [&](const string& value){ return toVector3(value, _p_gravity); });
    putProperty(_("NumThreads"), _p_numThreads, changeProperty(_p_numThreads));
    putProperty(_("ContactReduction"), _p_enableContactReduction, changeProperty(_p_enableContactReduction));
    putProperty(_("ContactReductionBinResolution"), _p_contactReductionBinResolution, changeProperty(_p_contactReductionBinResolution));
    putProperty(_("ContactReductionThreshhold"), _p_contactReductionThreshhold, changeProperty(_p_contactReductionThreshhold));
    putProperty(_("AutoSleep(experimental)"), _p_enableAutoSleep, changeProperty(_p_enableAutoSleep));
}

bool AGXSimulatorItemImpl::store(Archive & archive)
{
    (void) archive;
    // Write agx parameter to save
    //archive.write("friction", friction);
    return false;
}

bool AGXSimulatorItemImpl::restore(const Archive & archive)
{
    (void) archive;
    // Write agx parameter to restore
    return false;
}

SimulationBody * AGXSimulatorItemImpl::createSimulationBody(Body * orgBody)
{
    // When user click start bottom, this function will be called first.
    return new AGXBody(*orgBody);
}

bool AGXSimulatorItemImpl::initializeSimulation(const std::vector<SimulationBody*>& simBodies)
{
    const Vector3& g = _p_gravity;
    AGXSceneDesc sd;
    sd.simdesc.timeStep = self->worldTimeStep();
    sd.simdesc.gravity = agx::Vec3(g(0), g(1), g(2));
    sd.simdesc.numThreads = _p_numThreads;
    sd.simdesc.enableContactReduction = _p_enableContactReduction;
    sd.simdesc.contactReductionBinResolution = (agx::UInt8)_p_contactReductionBinResolution;
    sd.simdesc.contactReductionThreshhold = (agx::UInt8)_p_contactReductionThreshhold;
    sd.simdesc.enableAutoSleep = _p_enableAutoSleep;
    agxScene = AGXScene::create(sd);

    createAGXMaterialTable();

    for(auto simBody : simBodies){
        AGXBody* body = static_cast<AGXBody*>(simBody);
        // Create rigidbody, geometry, constraints
        body->createBody(agxScene);
        body->setSensor(self->worldTimeStep(), g);
    }

    setAdditionalAGXMaterialParam();

    return true;
}

#define SET_AGXMATERIAL_FIELD(field) desc.field = mat->info<double>(#field, desc.field)
void AGXSimulatorItemImpl::createAGXMaterialTable()
{
    WorldItem* const worldItem = self->findOwnerItem<WorldItem>();
    if(!worldItem) return;
    MaterialTable* const matTable = worldItem->materialTable();

    // Create AGX material
    matTable->forEachMaterial(
        [&](int id, Material* mat){
            AGXMaterialDesc desc;
            desc.name = Material::name(id);
            SET_AGXMATERIAL_FIELD(density);
            SET_AGXMATERIAL_FIELD(youngsModulus);
            SET_AGXMATERIAL_FIELD(poissonRatio);
            desc.viscosity = mat->viscosity();
            SET_AGXMATERIAL_FIELD(damping);
            desc.roughness = mat->roughness();
            SET_AGXMATERIAL_FIELD(surfaceViscosity);
            SET_AGXMATERIAL_FIELD(adhesionForce);
            SET_AGXMATERIAL_FIELD(adhesivOverlap);
            getAGXScene()->createMaterial(desc);
        });

    // Create AGX contact material
    matTable->forEachMaterialPair(
        [&](int id1, int id2, ContactMaterial* mat){
            createAGXContactMaterial(id1, id2, mat);
        });

    //getAGXScene()->printContactMaterialTable();
}

void AGXSimulatorItemImpl::createAGXContactMaterial(int id1, int id2, ContactMaterial* mat)
{
    Mapping* info = mat->info();
    AGXContactMaterialDesc desc;
    desc.nameA = Material::name(id1);
    desc.nameB = Material::name(id2);
    SET_AGXMATERIAL_FIELD(youngsModulus);
    desc.restitution = mat->restitution();
    SET_AGXMATERIAL_FIELD(damping);
    desc.friction = mat->friction();
    SET_AGXMATERIAL_FIELD(secondaryFriction);
    SET_AGXMATERIAL_FIELD(surfaceViscosity);
    SET_AGXMATERIAL_FIELD(secondarySurfaceViscosity);
    SET_AGXMATERIAL_FIELD(adhesionForce);
    SET_AGXMATERIAL_FIELD(adhesivOverlap);
    auto binNode = info->find("contactReductionBinResolution");
    if(binNode->isValid()) desc.contactReductionBinResolution = (agx::UInt8)binNode->toInt();

    auto crmNode = info->find("contactReductionMode");
    agxConvert::setValue(crmNode, agxContactReductionModeMap, "", desc.contactReductionMode,
    "Illegal contactReductionMode value. Use default.");

    auto frictionModelNode = info->find("frictionModel");
    if(frictionModelNode->isValid()){
        vector<string> fmVec;
        if(agxConvert::setVector(frictionModelNode, 2, fmVec)){
            if(!agxConvert::setValue(fmVec[0], agxFrictionModelTypeMap, "", desc.frictionModelType))
                info->throwException("Illegal frictionModelType value. Use default.");
            if(!agxConvert::setValue(fmVec[1], agxSolveTypeMap, "", desc.solveType))
                info->throwException("Illegal solveType value. Use default.");
        }else{
            info->throwException("Illegal frictionModel value. Use default.");
        }
    }

    getAGXScene()->createContactMaterial(desc);
}

void AGXSimulatorItemImpl::setAdditionalAGXMaterialParam()
{
    WorldItem* const worldItem = self->findOwnerItem<WorldItem>();
    if(!worldItem) return;
    MaterialTable* const matTable = worldItem->materialTable();
    agxSDK::MaterialManager* mgr = agxScene->getSimulation()->getMaterialManager();

    // Set params of ConstantNormalForceOrientedBoxFrictionModel
    matTable->forEachMaterialPair(
        [&](int id1, int id2, ContactMaterial* mat){
            agx::Material* mat1 = mgr->getMaterial(Material::name(id1));
            agx::Material* mat2 = mgr->getMaterial(Material::name(id2));
            agx::ContactMaterial* cmat = mgr->getOrCreateContactMaterial(mat1, mat2);
            string cmatName = "[" + mat1->getName() + " " + mat2->getName() + "]";
            std::cout << "AGXDynamicsPlugin:INFO " << "contact material " << cmatName  << std::endl;
            auto cnfobfm = dynamic_cast<agx::ConstantNormalForceOrientedBoxFrictionModel*>(cmat->getFrictionModel());
            if(!cnfobfm) return;
            std::cout << "AGXDynamicsPlugin:INFO " << "cnfobfm found at " << cmatName << std::endl;

            string referenceBodyName, referenceLinkName;
            if(mat->info()->read("referenceBodyName", referenceBodyName)){}else{
                std::cout << "AGXDynamicsPlugin:WARNING " << "referenceBodyName is not set or correct at " << cmatName << std::endl;
                return;
            }
            if(mat->info()->read("referenceLinkName", referenceLinkName)){}else{
                std::cout << "AGXDynamicsPlugin:WARNING " << "referenceLinkName is not set or correct at " << cmatName << std::endl;
                return;
            }

            auto simBody = self->findSimulationBody(referenceBodyName);
            AGXBody* body = static_cast<AGXBody*>(simBody);
            if(!body){
                std::cout << "AGXDynamicsPlugin:WARNING " << "reference body " << referenceBodyName << " is not found at" << cmatName << std::endl;
                return;
            }
            agx::RigidBody* rigid = body->getAGXRigidBody(referenceLinkName);
            if(!rigid){
                std::cout << "AGXDynamicsPlugin:WARNING " << "reference rigidbody " << referenceLinkName << " is not found at " << cmatName << std::endl;
                return;
            }

            cnfobfm->setNormalForceMagnitude(agx::Real(10.0) * rigid->getMassProperties()->getMass());
            cnfobfm->setReferenceFrame(rigid->getFrame());
            Vector3 primaryDirection;
            if(agxConvert::setVector(&mat->info()->get("primaryDirection"), primaryDirection)){
                cnfobfm->setPrimaryDirection(agxConvert::toAGX(primaryDirection));
            }else{
                std::cout << "AGXDynamicsPlugin:WARNING " << "primaryDirection is not set or correct" << std::endl;
            }
            std::cout << "AGXDynamicsPlugin:INFO " << "cnfobfm modified at " << cmatName << std::endl;
        }
    );
}
#undef SET_AGXMATERIAL_FIELD

bool AGXSimulatorItemImpl::stepSimulation(const std::vector<SimulationBody*>& activeSimBodies)
{
    for(auto simBody : activeSimBodies){
        static_cast<AGXBody*>(simBody)->setControlInputToAGX();
    }

    agxScene->stepSimulation();

    for(auto simBody : activeSimBodies){
        AGXBody* agxBody = static_cast<AGXBody*>(simBody);
        agxBody->setLinkStateToCnoid();

        // Update sensors
        if(agxBody->hasForceSensors())              agxBody->updateForceSensors();
        if(agxBody->hasGyroOrAccelerationSensors()) agxBody->updateGyroAndAccelerationSensors();
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

void AGXSimulatorItemImpl::setGravity(const Vector3& g)
{
    agxScene->setGravity(agx::Vec3(g(0), g(1), g(2)));
}

Vector3 AGXSimulatorItemImpl::getGravity() const
{
    const agx::Vec3& g = agxScene->getGravity();
    return Vector3(g.x(), g.y(), g.z());
}

bool AGXSimulatorItemImpl::saveSimulationToAGXFile()
{
    return agxScene->saveSceneToAGXFile();
}

AGXScene* AGXSimulatorItemImpl::getAGXScene(){
    return agxScene;
}

}
