/**
   \file
   \brief The definitions of the body customizer interface for increasing binary compatibility
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_BODY_CUSTOMIZER_INTERFACE_H
#define CNOID_BODY_BODY_CUSTOMIZER_INTERFACE_H

#include <string>
#include <cnoid/EigenTypes>
#include <cnoid/Config>
#include "exportdecl.h"

namespace cnoid {

typedef void* BodyHandle;
typedef void* BodyCustomizerHandle;

typedef int         (*BodyGetLinkIndexFromNameFunc) (BodyHandle bodyHandle, const char* linkName);
typedef const char* (*BodyGetLinkNameFunc)          (BodyHandle bodyHandle, int linkIndex);
typedef double*     (*BodyGetLinkDoubleValuePtrFunc)(BodyHandle bodyHandle, int linkIndex);

static const int BODY_INTERFACE_VERSION = 1;

struct BodyInterface
{
    int version;
		
    BodyGetLinkIndexFromNameFunc   getLinkIndexFromName;
    BodyGetLinkNameFunc            getLinkName;
    BodyGetLinkDoubleValuePtrFunc  getJointValuePtr;
    BodyGetLinkDoubleValuePtrFunc  getJointVelocityPtr;
    BodyGetLinkDoubleValuePtrFunc  getJointForcePtr;
};
    
typedef const char** (*BodyCustomizerGetTargetModelNamesFunc)();
typedef BodyCustomizerHandle (*BodyCustomizerCreateFunc)(BodyHandle bodyHandle, const char* modelName);
	
typedef void (*BodyCustomizerDestroyFunc)              (BodyCustomizerHandle customizerHandle);
typedef int  (*BodyCustomizerInitializeAnalyticIkFunc) (BodyCustomizerHandle customizerHandle, int baseLinkIndex, int targetLinkIndex);

/*
  p and R are based on the coordinate of a base link
*/
typedef bool (*BodyCustomizerCalcAnalyticIkFunc)       (BodyCustomizerHandle customizerHandle, int ikPathId, const Vector3& p, const Matrix3& R);
	
typedef void (*BodyCustomizerSetVirtualJointForcesFunc) (BodyCustomizerHandle customizerHandle);
typedef void (*BodyCustomizerSetVirtualJointForces2Func)(BodyCustomizerHandle customizerHandle, double timeStep);
	

static const int BODY_CUSTOMIZER_INTERFACE_VERSION = 2;

struct BodyCustomizerInterface
{
    int version;

    BodyCustomizerGetTargetModelNamesFunc getTargetModelNames;
    BodyCustomizerCreateFunc create;
    BodyCustomizerDestroyFunc destroy;
    BodyCustomizerInitializeAnalyticIkFunc initializeAnalyticIk;
    BodyCustomizerCalcAnalyticIkFunc calcAnalyticIk;
    BodyCustomizerSetVirtualJointForcesFunc setVirtualJointForces;
    BodyCustomizerSetVirtualJointForces2Func setVirtualJointForces2; // since version 2

    BodyCustomizerInterface() :
        getTargetModelNames(nullptr),
        create(nullptr),
        destroy(nullptr),
        initializeAnalyticIk(nullptr),
        calcAnalyticIk(nullptr),
        setVirtualJointForces(nullptr),
        setVirtualJointForces2(nullptr)
        { }
};

typedef BodyCustomizerInterface* (*GetBodyCustomizerInterfaceFunc)(BodyInterface* bodyInterface);

CNOID_EXPORT int loadDefaultBodyCustomizers(std::ostream& os);
CNOID_EXPORT int loadBodyCustomizers(const std::string pathString, std::ostream& os);
    
CNOID_EXPORT BodyCustomizerInterface* findBodyCustomizer(std::string modelName);
}


#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
#define CNOID_BODY_CUSTOMIZER_EXPORT extern "C" __declspec(dllexport)
#else
#define CNOID_BODY_CUSTOMIZER_EXPORT extern "C"
#endif

#endif
