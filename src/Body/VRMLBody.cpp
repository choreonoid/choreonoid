/*!
  @file
*/

#include "VRMLBody.h"

using namespace cnoid;

VRMLHumanoid::VRMLHumanoid()
{
}


VRMLJoint::VRMLJoint() : VRMLTransform()
{
    jointId = -1;
    jointType = "";
    jointAxis << 0.0, 0.0, 1.0;
    gearRatio = 1;
    rotorInertia = 0;
    rotorResistor = 0;
    torqueConst = 1;
    encoderPulse = 1;
}


VRMLSegment::VRMLSegment() : VRMLGroup()
{
    mass = 0.0;
    centerOfMass << 0.0, 0.0, 0.0;
    momentsOfInertia.resize(9, 0.0);
}


VRMLSurface::VRMLSurface()
{
}


VRMLVisionSensor::VRMLVisionSensor() : VRMLTransform()
{
    sensorId = -1;
    type = "NONE";
    width = 320;
    height = 240;
    frameRate = 30;
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


VRMLGyro::VRMLGyro() : VRMLTransform()
{
    sensorId = -1;
    maxAngularVelocity << -1, -1, -1;
}


VRMLAccelerationSensor::VRMLAccelerationSensor() : VRMLTransform()
{
    sensorId = -1;
    maxAcceleration << -1, -1, -1;
}


VRMLRangeSensor::VRMLRangeSensor() : VRMLTransform()
{
    sensorId = -1;
    scanAngle = 3.14159;
    scanStep = 0.1;
    scanRate = 10;
    minDistance = 0.01;
    maxDistance = 10;
}
