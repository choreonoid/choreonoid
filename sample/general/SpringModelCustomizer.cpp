/**
   \author Shin'ichiro Nakaoka
*/

#include <cnoid/BodyCustomizerInterface>

using namespace cnoid;

namespace {
BodyInterface* bodyInterface = 0;
BodyCustomizerInterface bodyCustomizerInterface;

struct Customizer
{
    double* pUpperJointPosition;
    double* pUpperJointVelocity;
    double* pUpperJointForce;
};

const char** getTargetModelNames()
{
    static const char* names[] = { "CustomizedSpringModel", 0 };
    return names;
}

BodyCustomizerHandle create(BodyHandle bodyHandle, const char* /* modelName */)
{
    Customizer* customizer = new Customizer();
    int upperJointIndex = bodyInterface->getLinkIndexFromName(bodyHandle, "UPPER");
    customizer->pUpperJointPosition = bodyInterface->getJointValuePtr(bodyHandle, upperJointIndex);
    customizer->pUpperJointVelocity = bodyInterface->getJointVelocityPtr(bodyHandle, upperJointIndex);
    customizer->pUpperJointForce = bodyInterface->getJointForcePtr(bodyHandle, upperJointIndex);
        
    return static_cast<BodyCustomizerHandle>(customizer);
}

void destroy(BodyCustomizerHandle customizerHandle)
{
    Customizer* customizer = static_cast<Customizer*>(customizerHandle);
    if(customizer){
        delete customizer;
    }
}
    
void setVirtualJointForces(BodyCustomizerHandle customizerHandle)
{
    const double KP = 2000.0;
    const double KD = 5.0;
        
    Customizer* c = static_cast<Customizer*>(customizerHandle);
    *c->pUpperJointForce = -KP * *c->pUpperJointPosition - KD * *c->pUpperJointVelocity;
}
}


CNOID_BODY_CUSTOMIZER_EXPORT
cnoid::BodyCustomizerInterface* getHrpBodyCustomizerInterface(cnoid::BodyInterface* bodyInterface_)
{
    bodyInterface = bodyInterface_;

    bodyCustomizerInterface.version = cnoid::BODY_CUSTOMIZER_INTERFACE_VERSION;
    bodyCustomizerInterface.getTargetModelNames = getTargetModelNames;
    bodyCustomizerInterface.create = create;
    bodyCustomizerInterface.destroy = destroy;
    bodyCustomizerInterface.initializeAnalyticIk = 0;
    bodyCustomizerInterface.calcAnalyticIk = 0;
    bodyCustomizerInterface.setVirtualJointForces = setVirtualJointForces;

    return &bodyCustomizerInterface;
}
