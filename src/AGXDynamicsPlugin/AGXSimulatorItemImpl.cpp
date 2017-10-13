#include "AGXSimulatorItemImpl.h"
#include "AGXSimulatorItem.h"
#include <assert.h>
#include <cnoid/EigenUtil>
#include "gettext.h"

using namespace std;

namespace cnoid {

AGXSimulatorItemImpl::AGXSimulatorItemImpl(AGXSimulatorItemPtr self) : self(self)
{
    initialize();
    AGXSimulationDesc simDesc;
    const agx::Vec3& g = simDesc.gravity;
    _p_gravity = Vector3(g.x(), g.y(), g.z());
    _p_numThreads = simDesc.numThreads;
    _p_enableContactReduction = simDesc.enableContactReduction;
    _p_contactReductionBinResolution = simDesc.contactReductionBinResolution;
    _p_contactReductionThreshhold = simDesc.contactReductionThreshhold;
    _p_enableAutoSleep = simDesc.enableAutoSleep;
}

AGXSimulatorItemImpl::AGXSimulatorItemImpl(AGXSimulatorItemPtr self, const AGXSimulatorItemImpl& org) 
    : AGXSimulatorItemImpl(self) 
{
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
    // Write agx parameter to save
    //archive.write("friction", friction);
    return false;
}

bool AGXSimulatorItemImpl::restore(const Archive & archive)
{
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
    sd.simdesc.contactReductionBinResolution = _p_contactReductionBinResolution;
    sd.simdesc.contactReductionThreshhold = _p_contactReductionThreshhold;
    sd.simdesc.enableAutoSleep = _p_enableAutoSleep;
    agxScene = AGXScene::create(sd);

    createMaterialTable();

    for(auto simBody : simBodies){
        AGXBody* body = static_cast<AGXBody*>(simBody);
        // Create rigidbody, geometry, constraints
        body->createBody(agxScene);
        body->setSensor(self->worldTimeStep(), g);
    }

    saveSimulationToAGXFile();

    return true;
}

void AGXSimulatorItemImpl::createMaterialTable()
{
    // Create default AGX material and contact material
    AGXMaterialDesc matDesc;
    agxScene->createMaterial(matDesc);
    AGXContactMaterialDesc cmatDesc;
    agxScene->createContactMaterial(cmatDesc);

    // Create AGX material and contact material from yaml file
}

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
};

    bool AGXSimulatorItemImpl::saveSimulationToAGXFile()
{
    return agxScene->saveSceneToAGXFile();
}

}