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

typedef boost::intrusive_ptr<VRMLHumanoid> VRMLHumanoidPtr;


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
};

typedef boost::intrusive_ptr<VRMLJoint> VRMLJointPtr;


class CNOID_EXPORT VRMLSegment : public VRMLGroup
{
public:
    VRMLSegment();
    SFFloat mass;
    SFVec3f centerOfMass;
    MFFloat momentsOfInertia;
};

typedef boost::intrusive_ptr<VRMLSegment> VRMLSegmentPtr;


class CNOID_EXPORT VRMLVisionSensor : public VRMLTransform
{
public:
    VRMLVisionSensor();
    SFInt32     sensorId;
    SFString    type;
    SFInt32     width;
    SFInt32     height;
    SFFloat     fieldOfView;
    SFFloat     frontClipDistance;
    SFFloat     backClipDistance;
};

typedef boost::intrusive_ptr<VRMLVisionSensor> VRMLVisionSensorPtr;


class CNOID_EXPORT VRMLForceSensor : public VRMLTransform
{
public:
    VRMLForceSensor();
    SFInt32     sensorId;
    SFVec3f     maxForce;
    SFVec3f     maxTorque;
};

typedef boost::intrusive_ptr<VRMLForceSensor> VRMLForceSensorPtr;

};

#endif
