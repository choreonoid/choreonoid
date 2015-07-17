/*!
  @file
*/

#include "VRMLBody.h"
#include <boost/assign/std/vector.hpp>

using namespace cnoid;
using namespace boost::assign;

VRMLHumanoid::VRMLHumanoid()
{
}


VRMLJoint::VRMLJoint() : VRMLTransform()
{
    jointId = -1;
    jointType = "";
    jointAxis << 0.0, 0.0, 1.0;
}


VRMLSegment::VRMLSegment() : VRMLGroup()
{
    mass = 0.0;
    centerOfMass << 0.0, 0.0, 0.0;
    momentsOfInertia += 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
}


VRMLVisionSensor::VRMLVisionSensor() : VRMLTransform()
{
    sensorId = -1;
    type = "NONE";
    width = 320;
    height = 240;
    fieldOfView = 0.785398;
    frontClipDistance = 0.01;
    backClipDistance = 10.0;
}


VRMLForceSensor::VRMLForceSensor() : VRMLTransform()
{
    sensorId = -1;
    maxForce << -1, -1, -1;
    maxTorque << -1, -1, -1;
}

