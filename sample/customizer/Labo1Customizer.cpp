
#include <cnoid/BodyCustomizerInterface>

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
#define DLL_EXPORT __declspec(dllexport)
#else 
#define DLL_EXPORT 
#endif /* Windows */

using namespace std;
using namespace cnoid;

static const bool debugMode = false;

static BodyInterface* bodyInterface = 0;

static BodyCustomizerInterface bodyCustomizerInterface;

struct JointValSet
{
    double* q_ptr;
    double* dq_ptr;
    double* u_ptr;
    double  e;
    double  p_prv;
};

struct Labo1Customizer
{
    BodyHandle bodyHandle;
    double FK; // presliding stiffness in Nm/rad
    double FB; // FK * 0.005, presliding viscosity in Nms/rad
    double FF; // friction torque in Nm
    double* q_ptr;
    double* dq_ptr;
    double* u_ptr;
    double  e;
    double  p_prv;
    bool isFirstTime;
};


static const char** getTargetModelNames()
{
    static const char* names[] = { 
        "Labo1",
        0 };
	
    return names;
}


static BodyCustomizerHandle create(BodyHandle bodyHandle, const char* modelName)
{
    Labo1Customizer* customizer = nullptr;
    
    int index = bodyInterface->getLinkIndexFromName(bodyHandle, "Valve1Handle");
    if(index >= 0){
        customizer = new Labo1Customizer;
        customizer->q_ptr = bodyInterface->getJointValuePtr(bodyHandle, index);
        customizer->dq_ptr = bodyInterface->getJointVelocityPtr(bodyHandle, index);
        customizer->u_ptr = bodyInterface->getJointForcePtr(bodyHandle, index);
        customizer->e = 0.0;
        customizer->p_prv = 0.0;
        customizer->isFirstTime = true;

        // Standard friction parameter values
        customizer->FK = 500.0;
        customizer->FB = 2.5;
        customizer->FF = 1.0;
    }

    return static_cast<BodyCustomizerHandle>(customizer);
}


static void destroy(BodyCustomizerHandle customizerHandle)
{
    Labo1Customizer* customizer = static_cast<Labo1Customizer*>(customizerHandle);
    if(customizer){
        delete customizer;
    }
}


/**
   The implementation of the following function is based on the method proposed in
   "Admittance and Impedance Representations of Friction Based on Implicit Euler Integration",
   Ryo Kikuuwe et al., IEEE Transaction on Robotics, Volume 22, Issue 6, pp.1176-1188, 2006.
*/
static void setVirtualJointForces(BodyCustomizerHandle customizerHandle, double timeStep)
{
    if(timeStep <= 0.0){
        return;
    }
        
    Labo1Customizer* customizer = static_cast<Labo1Customizer*>(customizerHandle);

    const double& FK = customizer->FK;
    const double& FB = customizer->FB;
    const double& FF = customizer->FF;
    const double& T = timeStep;

    const double v = (customizer->isFirstTime) ? 0.0 : (*(customizer->q_ptr) - customizer->p_prv) / T;
    // const double  v  = (*(customizer->dq_ptr)) ;
    double& e = customizer->e;
    const double va = v + (FK * e) / (FK * T + FB);
    const double fa = (FK * T + FB) * va ;
    const double ff = (fa > FF) ? FF : ((fa>-FF) ? fa : (-FF));
    e = (FB * e + T * ff) / (FK * T + FB);
    // ff = FK * *(customizer->q_ptr) + FB * v;
    *(customizer->u_ptr) = -ff ;
    customizer->p_prv = *(customizer->q_ptr);

    customizer->isFirstTime = false;
}


extern "C" DLL_EXPORT
BodyCustomizerInterface* getHrpBodyCustomizerInterface(BodyInterface* bodyInterface_)
{
    bodyInterface = bodyInterface_;

    bodyCustomizerInterface.version = BODY_CUSTOMIZER_INTERFACE_VERSION;
    bodyCustomizerInterface.getTargetModelNames = getTargetModelNames;
    bodyCustomizerInterface.create = create;
    bodyCustomizerInterface.destroy = destroy;
    bodyCustomizerInterface.initializeAnalyticIk = 0;
    bodyCustomizerInterface.calcAnalyticIk = 0;
    bodyCustomizerInterface.setVirtualJointForces2 = setVirtualJointForces;

    return &bodyCustomizerInterface;
}
