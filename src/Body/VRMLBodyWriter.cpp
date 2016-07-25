/*!
  @file
*/

#include "VRMLBodyWriter.h"
#include "VRMLBody.h"

using namespace std;
using namespace cnoid;


VRMLBodyWriter::VRMLBodyWriter(std::ostream& out) : VRMLWriter(out)
{
    registerNodeMethodMap();
}


void VRMLBodyWriter::registerNodeMethodMap()
{
    VRMLWriter::registerNodeMethodMap();
    registerNodeMethod(typeid(VRMLHumanoid),
                       (VRMLWriterNodeMethod)&VRMLBodyWriter::writeHumanoidNode);
    registerNodeMethod(typeid(VRMLJoint),
                       (VRMLWriterNodeMethod)&VRMLBodyWriter::writeJointNode);
    registerNodeMethod(typeid(VRMLSegment),
                       (VRMLWriterNodeMethod)&VRMLBodyWriter::writeSegmentNode);
    registerNodeMethod(typeid(VRMLSurface),
                       (VRMLWriterNodeMethod)&VRMLBodyWriter::writeSurfaceNode);
    registerNodeMethod(typeid(VRMLVisionSensor),
                       (VRMLWriterNodeMethod)&VRMLBodyWriter::writeVisionSensorNode);
    registerNodeMethod(typeid(VRMLForceSensor),
                       (VRMLWriterNodeMethod)&VRMLBodyWriter::writeForceSensorNode);
    registerNodeMethod(typeid(VRMLGyro),
                       (VRMLWriterNodeMethod)&VRMLBodyWriter::writeGyroNode);
    registerNodeMethod(typeid(VRMLAccelerationSensor),
                       (VRMLWriterNodeMethod)&VRMLBodyWriter::writeAccelerationSensorNode);
    registerNodeMethod(typeid(VRMLRangeSensor),
                       (VRMLWriterNodeMethod)&VRMLBodyWriter::writeRangeSensorNode);
}


void VRMLBodyWriter::writeHumanoidNode(VRMLNodePtr node)
{
    VRMLHumanoidPtr humanoid = boost::static_pointer_cast<VRMLHumanoid>(node);

    beginNode("Humanoid", humanoid);
    if(!humanoid->humanoidBody.empty()){
        out << indent << "humanoidBody [\n";
        ++indent;
        for(size_t i=0; i < humanoid->humanoidBody.size(); i++){
            writeNodeIter(humanoid->humanoidBody[i]);
        }
        out << --indent << "]\n";
    }
    if(!humanoid->joints.empty()){
        out << indent << "joints [\n";
        ++indent;
        for(size_t i=0; i < humanoid->joints.size(); i++){
            out << indent << "USE " << humanoid->joints[i]->defName;
            if (i == humanoid->joints.size() - 1) {
                out << "\n";
            } else {
                out << ",\n";
            }
        }
        out << --indent << "]\n";
    }
    if(!humanoid->segments.empty()){
        out << indent << "segments [\n";
        ++indent;
        for(size_t i=0; i < humanoid->segments.size(); i++){
            out << indent << "USE " << humanoid->segments[i]->defName;
            if (i == humanoid->segments.size() - 1) {
                out << "\n";
            } else {
                out << ",\n";
            }
        }
        out << --indent << "]\n";
    }
    endNode();
}


void VRMLBodyWriter::writeJointNode(VRMLNodePtr node)
{
    VRMLJointPtr joint = boost::static_pointer_cast<VRMLJoint>(node);

    beginNode("Joint", joint);

    if (joint->jointId >= 0) {
        out << indent << "jointId " << joint->jointId << "\n";
    }
    out << indent << "jointType \"" << joint->jointType << "\"\n";
    if (joint->jointType != "free" && joint->jointType != "fixed") {
        out << indent << "jointAxis " << joint->jointAxis << "\n";
        out << indent << "llimit\n";
        writeMFValues(joint->llimit, 1);
        out << indent << "lvlimit\n";
        writeMFValues(joint->lvlimit, 1);
        out << indent << "ulimit\n";
        writeMFValues(joint->ulimit, 1);
        out << indent << "uvlimit\n";
        writeMFValues(joint->uvlimit, 1);
    }
    out << indent << "gearRatio " << joint->gearRatio << "\n";
    out << indent << "rotorInertia " << joint->rotorInertia << "\n";
    out << indent << "rotorResistor " << joint->rotorResistor << "\n";
    out << indent << "torqueConst " << joint->torqueConst << "\n";
    out << indent << "encoderPulse " << joint->encoderPulse << "\n";
    out << indent << "center " << joint->center << "\n";
    out << indent << "rotation " << joint->rotation << "\n";
    out << indent << "scale " << joint->scale << "\n";
    out << indent << "scaleOrientation " << joint->scaleOrientation << "\n";
    out << indent << "translation " << joint->translation << "\n";

    writeGroupFields(joint);

    endNode();
}


void VRMLBodyWriter::writeSegmentNode(VRMLNodePtr node)
{
    VRMLSegmentPtr segment = boost::static_pointer_cast<VRMLSegment>(node);

    beginNode("Segment", segment);

    out << indent << "mass " << segment->mass << "\n";
    out << indent << "centerOfMass " << segment->centerOfMass << "\n";
    out << indent << "momentsOfInertia\n";
    writeMFValues(segment->momentsOfInertia, 3);

    writeGroupFields(segment);

    endNode();
}


void VRMLBodyWriter::writeSurfaceNode(VRMLNodePtr node)
{
    VRMLSurfacePtr surface = boost::static_pointer_cast<VRMLSurface>(node);

    beginNode("Surface", surface);

    if(!surface->visual.empty()){
        out << indent << "visual [\n";
        ++indent;
        for(size_t i=0; i < surface->visual.size(); i++){
            writeNodeIter(surface->visual[i]);
        }
        out << --indent << "]\n";
    }

    if(!surface->collision.empty()){
        out << indent << "collision [\n";
        ++indent;
        for(size_t i=0; i < surface->collision.size(); i++){
            writeNodeIter(surface->collision[i]);
        }
        out << --indent << "]\n";
    }

    endNode();
}


void VRMLBodyWriter::writeVisionSensorNode(VRMLNodePtr node)
{
    VRMLVisionSensorPtr sensor = boost::static_pointer_cast<VRMLVisionSensor>(node);

    beginNode("VisionSensor", sensor);

    out << indent << "rotation " << sensor->rotation << "\n";
    out << indent << "translation " << sensor->translation << "\n";
    if(sensor->sensorId >= 0){
        out << indent << "sensorId " << sensor->sensorId << "\n";
    }
    out << indent << "type \"" << sensor->type << "\"\n";
    out << indent << "width " << sensor->width << "\n";
    out << indent << "height " << sensor->height << "\n";
    out << indent << "frameRate " << sensor->frameRate << "\n";
    out << indent << "fieldOfView " << sensor->fieldOfView << "\n";
    out << indent << "frontClipDistance " << sensor->frontClipDistance << "\n";
    out << indent << "backClipDistance " << sensor->backClipDistance << "\n";

    endNode();
}


void VRMLBodyWriter::writeForceSensorNode(VRMLNodePtr node)
{
    VRMLForceSensorPtr sensor = boost::static_pointer_cast<VRMLForceSensor>(node);

    beginNode("ForceSensor", sensor);

    out << indent << "rotation " << sensor->rotation << "\n";
    out << indent << "translation " << sensor->translation << "\n";
    if(sensor->sensorId >= 0){
        out << indent << "sensorId " << sensor->sensorId << "\n";
    }
    out << indent << "maxForce " << sensor->maxForce << "\n";
    out << indent << "maxTorque " << sensor->maxTorque << "\n";

    endNode();
}


void VRMLBodyWriter::writeGyroNode(VRMLNodePtr node)
{
    VRMLGyroPtr sensor = boost::static_pointer_cast<VRMLGyro>(node);

    beginNode("Gyro", sensor);

    out << indent << "rotation " << sensor->rotation << "\n";
    out << indent << "translation " << sensor->translation << "\n";
    if(sensor->sensorId >= 0){
        out << indent << "sensorId " << sensor->sensorId << "\n";
    }
    out << indent << "maxAngularVelocity " << sensor->maxAngularVelocity << "\n";

    endNode();
}


void VRMLBodyWriter::writeAccelerationSensorNode(VRMLNodePtr node)
{
    VRMLAccelerationSensorPtr sensor = boost::static_pointer_cast<VRMLAccelerationSensor>(node);

    beginNode("AccelerationSensor", sensor);

    out << indent << "rotation " << sensor->rotation << "\n";
    out << indent << "translation " << sensor->translation << "\n";
    if(sensor->sensorId >= 0){
        out << indent << "sensorId " << sensor->sensorId << "\n";
    }
    out << indent << "maxAcceleration " << sensor->maxAcceleration << "\n";

    endNode();
}


void VRMLBodyWriter::writeRangeSensorNode(VRMLNodePtr node)
{
    VRMLRangeSensorPtr sensor = boost::static_pointer_cast<VRMLRangeSensor>(node);

    beginNode("RangeSensor", sensor);

    out << indent << "rotation " << sensor->rotation << "\n";
    out << indent << "translation " << sensor->translation << "\n";
    if(sensor->sensorId >= 0){
        out << indent << "sensorId " << sensor->sensorId << "\n";
    }
    out << indent << "scanAngle " << sensor->scanAngle << "\n";
    out << indent << "scanStep " << sensor->scanStep << "\n";
    out << indent << "scanRate " << sensor->scanRate << "\n";
    out << indent << "minDistance " << sensor->minDistance << "\n";
    out << indent << "maxDistance " << sensor->maxDistance << "\n";

    endNode();
}
