#include "AGXSimulatorItemImpl.h"
#include "AGXSimulatorItem.h"
#include <cnoid/EigenUtil>
#include <cnoid/WorldItem>
#include <cnoid/MaterialTable>
#include "AGXConvert.h"
#include <unordered_map>
#include <cnoid/PutPropertyFunction>
#include <cnoid/Archive>
#include <cnoid/EigenArchive>
#include "gettext.h"

using namespace std;

namespace cnoid {

const std::unordered_map<std::string, agx::ContactMaterial::ContactReductionMode> agxContactReductionModeMap{
    {"", agx::ContactMaterial::ContactReductionMode::REDUCE_GEOMETRY},
    {"default", agx::ContactMaterial::ContactReductionMode::REDUCE_GEOMETRY},
    {"reduce_geometry", agx::ContactMaterial::ContactReductionMode::REDUCE_GEOMETRY},
    {"reduce_all", agx::ContactMaterial::ContactReductionMode::REDUCE_ALL},
    {"reduce_none", agx::ContactMaterial::ContactReductionMode::REDUCE_NONE},
    // deprecated
    {"reduceGeometry", agx::ContactMaterial::ContactReductionMode::REDUCE_GEOMETRY},
    {"reduceALL", agx::ContactMaterial::ContactReductionMode::REDUCE_ALL},
    {"reduceNone", agx::ContactMaterial::ContactReductionMode::REDUCE_NONE}
};

const std::unordered_map<std::string, AGXFrictionModelType> agxFrictionModelTypeMap{
    {"", AGXFrictionModelType::DEFAULT},
    {"default", AGXFrictionModelType::DEFAULT},
    {"box", AGXFrictionModelType::BOX},
    {"scaled_box", AGXFrictionModelType::SCALED_BOX},
    {"oriented_box", AGXFrictionModelType::ORIENTED_BOX},
    {"oriented_scaled_box", AGXFrictionModelType::ORIENTED_SCALED_BOX},
    {"constant_normal_force_oriented_box", AGXFrictionModelType::CONSTANT_NORMAL_FORCE_ORIENTED_BOX},
    {"iterative", AGXFrictionModelType::ITERATIVE_PROJECTED_CONE},
    {"oriented_iterative", AGXFrictionModelType::ORIENTED_ITERATIVE_PROJECTED_CONE},
    // deprecated
    {"cone", AGXFrictionModelType::ITERATIVE_PROJECTED_CONE},
    {"scaledBox", AGXFrictionModelType::SCALED_BOX},
    {"orientedBox", AGXFrictionModelType::CONSTANT_NORMAL_FORCE_ORIENTED_BOX}
};

const std::unordered_map<std::string, agx::FrictionModel::SolveType> agxSolveTypeMap{
    {"", agx::FrictionModel::SolveType::SPLIT},
    {"default", agx::FrictionModel::SolveType::SPLIT},
    {"split", agx::FrictionModel::SolveType::SPLIT},
    {"direct", agx::FrictionModel::SolveType::DIRECT},
    {"iterative", agx::FrictionModel::SolveType::ITERATIVE},
    {"direct_and_iterative", agx::FrictionModel::SolveType::DIRECT_AND_ITERATIVE},
    // deprecated
    {"directAndIterative", agx::FrictionModel::SolveType::DIRECT_AND_ITERATIVE}
};

const std::unordered_map<std::string, agx::Notify::NotifyLevel> agxNotifyLevel{
    {"debug", agx::Notify::NOTIFY_DEBUG},
    {"info", agx::Notify::NOTIFY_INFO},
    {"warning", agx::Notify::NOTIFY_WARNING},
    {"error", agx::Notify::NOTIFY_ERROR}
};

AGXSimulatorItemImpl::AGXSimulatorItemImpl(AGXSimulatorItem* self)
    : self(self)
{
    initialize();
    AGXSimulationDesc simDesc;
    const agx::Vec3& g = simDesc.gravity;
    m_p_gravity = Vector3(g.x(), g.y(), g.z());
    m_p_numThreads = (int)simDesc.numThreads;
    m_p_enableContactReduction = simDesc.enableContactReduction;
    m_p_contactReductionBinResolution = simDesc.contactReductionBinResolution;
    m_p_contactReductionThreshhold = (int)simDesc.contactReductionThreshhold;
    m_p_enableContactWarmstarting = simDesc.enableContactWarmstarting;
    m_p_enableAMOR = simDesc.enableAMOR;
    m_p_enableAutoSleep = simDesc.enableAutoSleep;
    m_p_saveToAGXFileOnStart = false;
    m_p_debugMessageOnConsoleType = Selection(agxNotifyLevel.size(), CNOID_GETTEXT_DOMAIN_NAME);
    for(auto i : agxNotifyLevel){
        m_p_debugMessageOnConsoleType.setSymbol(i.second, N_(i.first));
    }
    m_p_debugMessageOnConsoleType.select(agx::Notify::NOTIFY_WARNING);
}

AGXSimulatorItemImpl::AGXSimulatorItemImpl(AGXSimulatorItem* self, const AGXSimulatorItemImpl& org)
    : AGXSimulatorItemImpl(self)
{
    initialize();
    m_p_gravity                       =  org.m_p_gravity                      ;
    m_p_numThreads                    =  org.m_p_numThreads                   ;
    m_p_enableContactReduction        =  org.m_p_enableContactReduction       ;
    m_p_contactReductionBinResolution =  org.m_p_contactReductionBinResolution;
    m_p_contactReductionThreshhold    =  org.m_p_contactReductionThreshhold   ;
    m_p_enableContactWarmstarting     =  org.m_p_enableContactWarmstarting    ;
    m_p_enableAMOR                    =  org.m_p_enableAMOR                   ;
    m_p_enableAutoSleep               =  org.m_p_enableAutoSleep              ;
    m_p_saveToAGXFileOnStart          =  org.m_p_saveToAGXFileOnStart         ;
    m_p_debugMessageOnConsoleType     =  org.m_p_debugMessageOnConsoleType    ;
}

AGXSimulatorItemImpl::~AGXSimulatorItemImpl(){}

void AGXSimulatorItemImpl::initialize(){}

void AGXSimulatorItemImpl::doPutProperties(PutPropertyFunction & putProperty)
{
    putProperty(_("Gravity"), str(m_p_gravity), [&](const string& value){ return toVector3(value, m_p_gravity); });
    putProperty(_("NumThreads"), m_p_numThreads, changeProperty(m_p_numThreads));
    putProperty(_("ContactReduction"), m_p_enableContactReduction, changeProperty(m_p_enableContactReduction));
    putProperty(_("ContactReductionBinResolution"), m_p_contactReductionBinResolution, changeProperty(m_p_contactReductionBinResolution));
    putProperty(_("ContactReductionThreshhold"), m_p_contactReductionThreshhold, changeProperty(m_p_contactReductionThreshhold));
    putProperty(_("ContactWarmstarting"), m_p_enableContactWarmstarting, changeProperty(m_p_enableContactWarmstarting));
    putProperty(_("AMOR"), m_p_enableAMOR, changeProperty(m_p_enableAMOR));
    putProperty(_("(deprecated)AutoSleep"), m_p_enableAutoSleep, changeProperty(m_p_enableAutoSleep));
    putProperty(_("SaveToAGXFileOnStart"), m_p_saveToAGXFileOnStart, changeProperty(m_p_saveToAGXFileOnStart));
    putProperty(_("DebugMessageOnConsole"), m_p_debugMessageOnConsoleType,
    [&](int index){ return m_p_debugMessageOnConsoleType.select(index); });
}

bool AGXSimulatorItemImpl::store(Archive & archive)
{
    write(archive, "Gravity", m_p_gravity);
    archive.write("NumThreads", m_p_numThreads);
    archive.write("ContactReduction", m_p_enableContactReduction);
    archive.write("ContactReductionBinResolution", m_p_contactReductionBinResolution);
    archive.write("ContactReductionThreshhold", m_p_contactReductionThreshhold);
    archive.write("ContactWarmstarting", m_p_enableContactWarmstarting);
    archive.write("AutoSleep", m_p_enableAutoSleep);
    archive.write("SaveToAGXFileOnStart", m_p_saveToAGXFileOnStart);
    archive.write("DebugMessageOnConsole", m_p_debugMessageOnConsoleType.selectedIndex());
    return true;
}

bool AGXSimulatorItemImpl::restore(const Archive & archive)
{
    read(archive, "Gravity", m_p_gravity);
    archive.read("NumThreads", m_p_numThreads);
    archive.read("ContactReduction", m_p_enableContactReduction);
    archive.read("ContactReductionBinResolution", m_p_contactReductionBinResolution);
    archive.read("ContactReductionThreshhold", m_p_contactReductionThreshhold);
    archive.read("ContactWarmstarting", m_p_enableContactWarmstarting);
    archive.read("AutoSleep", m_p_enableAutoSleep);
    archive.read("SaveToAGXFileOnStart", m_p_saveToAGXFileOnStart);
    int agxNL = agx::Notify::NOTIFY_WARNING;
    archive.read("DebugMessageOnConsole", agxNL);
    m_p_debugMessageOnConsoleType.select(agxNL);
    return true;
}

SimulationBody * AGXSimulatorItemImpl::createSimulationBody(Body * orgBody)
{
    // When user click start bottom, this function will be called first.
    
    return new AGXBody(orgBody->clone());
}

bool AGXSimulatorItemImpl::initializeSimulation(const std::vector<SimulationBody*>& simBodies)
{
    const Vector3& g = m_p_gravity;
    AGXSceneDesc sd;
    sd.simdesc.timeStep = self->worldTimeStep();
    sd.simdesc.gravity = agx::Vec3(g(0), g(1), g(2));
    sd.simdesc.numThreads = m_p_numThreads;
    sd.simdesc.enableContactReduction = m_p_enableContactReduction;
    sd.simdesc.contactReductionBinResolution = (agx::UInt8)m_p_contactReductionBinResolution;
    sd.simdesc.contactReductionThreshhold = (agx::UInt8)m_p_contactReductionThreshhold;
    sd.simdesc.enableAMOR = m_p_enableAMOR;
    sd.simdesc.enableContactWarmstarting = m_p_enableContactWarmstarting;
    sd.simdesc.enableAutoSleep = m_p_enableAutoSleep;
    agxScene = AGXScene::create(sd);
    const agx::Notify::NotifyLevel notifyLevel = agxNotifyLevel.at(m_p_debugMessageOnConsoleType.selectedSymbol());
    agx::Notify::instance()->setNotifyLevel(notifyLevel);
    //agx::Notify::instance()->setLogNotifyLevel(notifyLevel);
    //agx::Logger::instance()->setLogNotifyLevel(notifyLevel);

    createAGXMaterialTable();

    doUpdateLinkContactPoints = false;
    
    for(auto simBody : simBodies){
        AGXBody* agxBody = static_cast<AGXBody*>(simBody);
        // Create rigidbody, geometry, constraints
        agxBody->createBody(agxScene);
        agxBody->setSensor(self->worldTimeStep(), g);

        if(!doUpdateLinkContactPoints){
            for(auto& link : agxBody->body()->links()){
                if(link->sensingMode() & Link::LinkContactState){
                    doUpdateLinkContactPoints = true;
                    break;
                }
            }
        }
    }

    setAdditionalAGXMaterialParam();

    if(m_p_saveToAGXFileOnStart) saveSimulationToAGXFile();
    return true;
}

void AGXSimulatorItemImpl::createAGXMaterialTable()
{
    WorldItem* const worldItem = self->findOwnerItem<WorldItem>();
    if(!worldItem) return;
    MaterialTable* const matTable = worldItem->materialTable();

    // Create AGX material
    matTable->forEachMaterial(
        [&](int id, Material* mat){
            AGXMaterialDesc desc;
            desc.name = Material::nameOfId(id);

            auto info = mat->info();
            info->read("density", desc.density);
            info->read({ "youngs_modulus", "youngsModulus" }, desc.youngsModulus);
            desc.viscosity = mat->viscosity();
            info->read({ "spook_damping", "spookDamping" }, desc.spookDamping);
            desc.roughness = mat->roughness();
            info->read({ "surface_viscosity", "surfaceViscosity" }, desc.surfaceViscosity);
            info->read({ "adhesion_force", "adhesionForce" }, desc.adhesionForce);
            info->read({ "adhesiv_overlap", "adhesivOverlap" }, desc.adhesivOverlap);
            info->read({ "wire_youngs_modulus_stretch", "wireYoungsModulusStretch" }, desc.wireYoungsModulusStretch);
            info->read({ "wire_spook_damping_stretch", "wireSpookDampingStretch" }, desc.wireSpookDampingStretch);
            info->read({ "wire_youngs_modulus_bend", "wireYoungsModulusBend" }, desc.wireYoungsModulusBend);
            info->read({ "wire_spook_damping_bend", "wireSpookDampingBend" }, desc.wireSpookDampingBend);
            
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
    desc.nameA = Material::nameOfId(id1);
    desc.nameB = Material::nameOfId(id2);
    info->read({ "youngs_modulus", "youngsModulus" }, desc.youngsModulus);
    desc.restitution = mat->restitution();
    info->read({ "spook_damping", "spookDamping" }, desc.spookDamping);
    desc.friction = mat->friction();
    info->read({ "secondary_friction", "secondaryFriction" }, desc.secondaryFriction);
    info->read({ "surface_viscosity", "surfaceViscosity" }, desc.surfaceViscosity);
    info->read({ "secondary_surface_viscosity", "secondarySurfaceViscosity" }, desc.secondarySurfaceViscosity);
    info->read({ "adhesion_force", "adhesionForce" }, desc.adhesionForce);
    info->read({ "adhesiv_overlap", "adhesivOverlap" }, desc.adhesivOverlap);
    desc.contactReductionBinResolution =
        info->get({ "contact_reduction_bin_resolution", "contactReductionBinResolution" },
                  static_cast<int>(desc.contactReductionBinResolution));
    
    auto crmNode = info->find("contact_reduction_mode");
    if(!crmNode->isValid()){
        crmNode = info->find("contactReductionMode");
    }
    agxConvert::setValue(crmNode, agxContactReductionModeMap, "", desc.contactReductionMode,
                         "Illegal contactReductionMode value. Use default.");

    auto frictionModelNode = info->find("friction_model");
    if(!frictionModelNode->isValid()){
        frictionModelNode = info->find("frictionModel");
    }
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

template<class ContactModel>
static void setFrameOfOrientedFrictionModel
(AGXSimulatorItem* simulator, ContactMaterial* materialPair, const string& pairName, ContactModel* frictionModel)
{
    LOGGER_INFO() << "AGXDynamicsPlugin:INFO " << "An oriented friction model found at the material table " << pairName << LOGGER_ENDL();

    auto info = materialPair->info();
    string referenceBodyName, referenceLinkName;
    if(!info->read({ "reference_body", "referenceBodyName" } ,referenceBodyName)){
        LOGGER_WARNING() << "AGXDynamicsPlugin:WARNING "
                         << "reference_body is not set or correct at the material table " << pairName << LOGGER_ENDL();
        return;
    }
    auto simBody = simulator->findSimulationBody(referenceBodyName);
    auto agxBody = static_cast<AGXBody*>(simBody);
    if(!agxBody){
        //LOGGER_WARNING() << "AGXDynamicsPlugin:WARNING " << "reference body " << referenceBodyName
        //                 << " is not found at the material table " << pairName << LOGGER_ENDL();
        return;
    }
    if(!info->read({ "reference_link", "referenceLinkName" }, referenceLinkName)){
        LOGGER_WARNING() << "AGXDynamicsPlugin:WARNING "
                         << "reference_link is not set or correct at the material table " << pairName << LOGGER_ENDL();
        return;
    }
    agx::RigidBody* rigid = agxBody->getAGXRigidBody(referenceLinkName);
    if(!rigid){
        LOGGER_WARNING() << "AGXDynamicsPlugin:WARNING " << "reference rigidbody " << referenceLinkName
                         << " is not found at the material table " << pairName << LOGGER_ENDL();
        return;
    }
    
    frictionModel->setReferenceFrame(rigid->getFrame());
    Vector3 primaryDirection;
    if(read(info, { "primary_direction", "primaryDirection" }, primaryDirection)){
        frictionModel->setPrimaryDirection(agxConvert::toAGX(primaryDirection));
    }else{
        LOGGER_WARNING() << "AGXDynamicsPlugin:WARNING " << "primary_direction is not set or correct" << LOGGER_ENDL();
    }

    if(auto cnfModel = dynamic_cast<agx::ConstantNormalForceOrientedBoxFrictionModel*>(frictionModel)){
        double cnf;
        if(info->read("constant_normal_force", cnf)){
            cnfModel->setNormalForceMagnitude(cnf);
        } else {
            cnfModel->setNormalForceMagnitude(agx::Real(9.8) * agxBody->body()->mass() / 2.0);
        }
    }
    
    LOGGER_INFO() << "AGXDynamicsPlugin:INFO " << "The reference frame has been specified to the oriented friction model for "
                  << pairName << LOGGER_ENDL();
}

void AGXSimulatorItemImpl::setAdditionalAGXMaterialParam()
{
    WorldItem* const worldItem = self->findOwnerItem<WorldItem>();
    if(!worldItem) return;
    MaterialTable* const matTable = worldItem->materialTable();
    agxSDK::MaterialManager* mgr = agxScene->getSimulation()->getMaterialManager();

    // Extract oriented-type friction models
    matTable->forEachMaterialPair(
        [&](int id1, int id2, ContactMaterial* materialPair){
            agx::Material* mat1 = mgr->getMaterial(Material::nameOfId(id1));
            agx::Material* mat2 = mgr->getMaterial(Material::nameOfId(id2));
            if(mat1 && mat2){
                if(auto agxMaterialPair = mgr->getOrCreateContactMaterial(mat1, mat2)){
                    string pairName = "[" + mat1->getName() + " " + mat2->getName() + "]";
                    LOGGER_INFO() << "AGXDynamicsPlugin:INFO " << "contact material " << pairName << LOGGER_ENDL();
                    auto model = agxMaterialPair->getFrictionModel();
                    if(auto oriented = dynamic_cast<agx::OrientedIterativeProjectedConeFrictionModel*>(model)){
                        setFrameOfOrientedFrictionModel(self, materialPair, pairName, oriented);
                    } else if(auto oriented = dynamic_cast<agx::OrientedScaleBoxFrictionModel*>(model)){
                        setFrameOfOrientedFrictionModel(self, materialPair, pairName, oriented);
                    } else if(auto oriented = dynamic_cast<agx::ConstantNormalForceOrientedBoxFrictionModel*>(model)){
                        setFrameOfOrientedFrictionModel(self, materialPair, pairName, oriented);
                    }
                }
            }
        }
    );
}
#undef SET_AGXMATERIAL_FIELD

bool AGXSimulatorItemImpl::stepSimulation(const std::vector<SimulationBody*>& activeSimBodies)
{
    agxScene->setMainWorkThread();
    // Need to set NotifyLevel for each thread.
    agx::Notify::instance()->setNotifyLevel(agxNotifyLevel.at(m_p_debugMessageOnConsoleType.selectedSymbol()));

    for(auto simBody : activeSimBodies){
        auto const agxBody = static_cast<AGXBody*>(simBody);
        agxBody->setControlInputToAGX();
        agxBody->addForceTorqueToAGX();
    }

    agxScene->stepSimulation();

    for(auto simBody : activeSimBodies){
        auto const agxBody = static_cast<AGXBody*>(simBody);
        agxBody->setLinkStateToCnoid();

        // Update sensors
        if(agxBody->hasForceSensors())              agxBody->updateForceSensors();
        if(agxBody->hasGyroOrAccelerationSensors()) agxBody->updateGyroAndAccelerationSensors();
    }

    if(doUpdateLinkContactPoints){
        updateLinkContactPoints();
    }
    
    return true;
}

void AGXSimulatorItemImpl::updateLinkContactPoints()
{
    for(auto& contact : agxScene->getSimulation()->getSpace()->getGeometryContacts()){
        if(contact->isEnabled()){
            for(int i=0; i < 2; ++i){
                if(auto linkRigidBody = dynamic_cast<LinkRigidBody*>(contact->rigidBody(i))){
                    auto link = linkRigidBody->getLink();
                    if(link->sensingMode() & Link::LinkContactState){
                        updateLinkContactPoints(contact, link, (i == 0) ? 1.0 : -1.0);
                    }
                }
            }
        }
    }
}


void AGXSimulatorItemImpl::updateLinkContactPoints
(agxCollide::GeometryContact* contact, Link* link, double direction)
{
    auto& srcPoints = contact->points();
    auto& points = link->contactPoints();
    points.clear();
    points.reserve(srcPoints.size());
    for(auto& point : srcPoints){
        if(point.enabled()){
            Eigen::Map<Vector3> position(point.point().ptr());
            Eigen::Map<Vector3f> normal(point.normal().ptr());
            auto agxForce = point.getForce();
            Eigen::Map<Vector3> force(agxForce.ptr());
            Eigen::Map<Vector3f> velocity(point.velocity().ptr());
            points.emplace_back(
                position,
                direction * normal.cast<double>(),
                direction * force,
                direction * velocity.cast<double>(),
                point.depth());
        }
    }
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

void AGXSimulatorItemImpl::setNumThreads(unsigned int num)
{
    m_p_numThreads = num;
}

void AGXSimulatorItemImpl::setEnableContactReduction(bool bOn)
{
    m_p_enableContactReduction = bOn;
}

void AGXSimulatorItemImpl::setContactReductionBinResolution(int r)
{
    m_p_contactReductionBinResolution = r;
}

void AGXSimulatorItemImpl::setContactReductionThreshhold(int t)
{
    m_p_contactReductionThreshhold = t;
}
void AGXSimulatorItemImpl::setEnableContactWarmstarting(bool bOn)
{
    m_p_enableContactWarmstarting = bOn;
}

void AGXSimulatorItemImpl::setEnableAMOR(bool bOn)
{
    m_p_enableAMOR = bOn;
}

bool AGXSimulatorItemImpl::saveSimulationToAGXFile()
{
    return agxScene->saveSceneToAGXFile();
}

AGXScene* AGXSimulatorItemImpl::getAGXScene(){
    return agxScene;
}

}
