/*!
  @file
*/

#ifndef CNOID_BODY_VRMLBODY_H
#define CNOID_BODY_VRMLBODY_H

#include <cnoid/VRML>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT VRMLHumanoid : public VRMLNode
{
public:
    VRMLHumanoid();
    MFNode humanoidBody;
    MFNode segments;
    MFNode joints;
};

typedef ref_ptr<VRMLHumanoid> VRMLHumanoidPtr;


class CNOID_EXPORT VRMLJoint : public VRMLTransform
{
public:
    VRMLJoint();
    SFInt32 jointId;
    SFString jointType;
    SFVec3f jointAxis;
    MFFloat llimit;
    MFFloat lvlimit;
    MFFloat ulimit;
    MFFloat uvlimit;
    SFFloat gearRatio;
    SFFloat rotorInertia;
    SFFloat rotorResistor;
    SFFloat torqueConst;
    SFFloat encoderPulse;
};

typedef ref_ptr<VRMLJoint> VRMLJointPtr;


class CNOID_EXPORT VRMLSegment : public VRMLGroup
{
public:
    VRMLSegment();
    SFFloat mass;
    SFVec3f centerOfMass;
    MFFloat momentsOfInertia;
};

typedef ref_ptr<VRMLSegment> VRMLSegmentPtr;


class CNOID_EXPORT VRMLSurface : public VRMLNode
{
public:
    VRMLSurface();
    MFNode visual;
    MFNode collision;
};

typedef ref_ptr<VRMLSurface> VRMLSurfacePtr;


class CNOID_EXPORT VRMLVisionSensor : public VRMLTransform
{
public:
    VRMLVisionSensor();
    SFInt32     sensorId;
    SFString    type;
    SFInt32     width;
    SFInt32     height;
    SFFloat     frameRate;
    SFFloat     fieldOfView;
    SFFloat     frontClipDistance;
    SFFloat     backClipDistance;
};

typedef ref_ptr<VRMLVisionSensor> VRMLVisionSensorPtr;


class CNOID_EXPORT VRMLForceSensor : public VRMLTransform
{
public:
    VRMLForceSensor();
    SFInt32     sensorId;
    SFVec3f     maxForce;
    SFVec3f     maxTorque;
};

typedef ref_ptr<VRMLForceSensor> VRMLForceSensorPtr;


class CNOID_EXPORT VRMLGyro : public VRMLTransform
{
public:
    VRMLGyro();
    SFInt32     sensorId;
    SFVec3f     maxAngularVelocity;
};

typedef ref_ptr<VRMLGyro> VRMLGyroPtr;


class CNOID_EXPORT VRMLAccelerationSensor : public VRMLTransform
{
public:
    VRMLAccelerationSensor();
    SFInt32     sensorId;
    SFVec3f     maxAcceleration;
};

typedef ref_ptr<VRMLAccelerationSensor> VRMLAccelerationSensorPtr;

 
class CNOID_EXPORT VRMLRangeSensor : public VRMLTransform
{
public:
    VRMLRangeSensor();
    SFInt32     sensorId;
    SFFloat     scanAngle;
    SFFloat     scanStep;
    SFFloat     scanRate;
    SFFloat     minDistance;
    SFFloat     maxDistance;
};

typedef ref_ptr<VRMLRangeSensor> VRMLRangeSensorPtr;

}

#endif
