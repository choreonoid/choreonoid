/*!
  @file
*/

#include "VRMLBodyWriter.h"

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


void VRMLBodyWriter::writeOpenHRPPROTOs()
{
    out << "PROTO Joint [\n";
    out << "  exposedField     SFVec3f      center              0 0 0\n";
    out << "  exposedField     MFNode       children            []\n";
    out << "  exposedField     MFFloat      llimit              []\n";
    out << "  exposedField     MFFloat      lvlimit             []\n";
    out << "  exposedField     SFRotation   limitOrientation    0 0 1 0\n";
    out << "  exposedField     SFString     name                \"\"\n";
    out << "  exposedField     SFRotation   rotation            0 0 1 0\n";
    out << "  exposedField     SFVec3f      scale               1 1 1\n";
    out << "  exposedField     SFRotation   scaleOrientation    0 0 1 0\n";
    out << "  exposedField     MFFloat      stiffness           [ 0 0 0 ]\n";
    out << "  exposedField     SFVec3f      translation         0 0 0\n";
    out << "  exposedField     MFFloat      ulimit              []\n";
    out << "  exposedField     MFFloat      uvlimit             []\n";
    out << "  exposedField     SFString     jointType           \"\"\n";
    out << "  exposedField     SFInt32      jointId             -1\n";
    out << "  exposedField     SFVec3f      jointAxis           0 0 1\n";
    out << "\n";
    out << "  exposedField     SFFloat      gearRatio           1\n";
    out << "  exposedField     SFFloat      rotorInertia        0\n";
    out << "  exposedField     SFFloat      rotorResistor       0\n";
    out << "  exposedField     SFFloat      torqueConst         1\n";
    out << "  exposedField     SFFloat      encoderPulse        1\n";
    out << "]\n";
    out << "{\n";
    out << "  Transform {\n";
    out << "    center           IS center\n";
    out << "    children         IS children\n";
    out << "    rotation         IS rotation\n";
    out << "    scale            IS scale\n";
    out << "    scaleOrientation IS scaleOrientation\n";
    out << "    translation      IS translation\n";
    out << "  }\n";
    out << "}\n";
    out << "\n";
    out << "PROTO Segment [\n";
    out << "  field           SFVec3f     bboxCenter        0 0 0\n";
    out << "  field           SFVec3f     bboxSize          -1 -1 -1\n";
    out << "  exposedField    SFVec3f     centerOutMass      0 0 0\n";
    out << "  exposedField    MFNode      children          [ ]\n";
    out << "  exposedField    SFNode      coord             NULL\n";
    out << "  exposedField    MFNode      displacers        [ ]\n";
    out << "  exposedField    SFFloat     mass              0\n";
    out << "  exposedField    MFFloat     momentsOutInertia  [ 0 0 0 0 0 0 0 0 0 ]\n";
    out << "  exposedField    SFString    name              \"\"\n";
    out << "  eventIn         MFNode      addChildren\n";
    out << "  eventIn         MFNode      removeChildren\n";
    out << "]\n";
    out << "{\n";
    out << "  Group {\n";
    out << "    addChildren    IS addChildren\n";
    out << "    bboxCenter     IS bboxCenter\n";
    out << "    bboxSize       IS bboxSize\n";
    out << "    children       IS children\n";
    out << "    removeChildren IS removeChildren\n";
    out << "  }\n";
    out << "}\n";
    out << "\n";
    out << "PROTO Humanoid [\n";
    out << "  field           SFVec3f    bboxCenter            0 0 0\n";
    out << "  field           SFVec3f    bboxSize              -1 -1 -1\n";
    out << "  exposedField    SFVec3f    center                0 0 0\n";
    out << "  exposedField    MFNode     humanoidBody          [ ]\n";
    out << "  exposedField    MFString   info                  [ ]\n";
    out << "  exposedField    MFNode     joints                [ ]\n";
    out << "  exposedField    SFString   name                  \"\"\n";
    out << "  exposedField    SFRotation rotation              0 0 1 0\n";
    out << "  exposedField    SFVec3f    scale                 1 1 1\n";
    out << "  exposedField    SFRotation scaleOrientation      0 0 1 0\n";
    out << "  exposedField    MFNode     segments              [ ]\n";
    out << "  exposedField    MFNode     sites                 [ ]\n";
    out << "  exposedField    SFVec3f    translation           0 0 0\n";
    out << "  exposedField    SFString   version               \"1.1\"\n";
    out << "  exposedField    MFNode     viewpoints            [ ]\n";
    out << "]\n";
    out << "{\n";
    out << "  Transform {\n";
    out << "    bboxCenter       IS bboxCenter\n";
    out << "    bboxSize         IS bboxSize\n";
    out << "    center           IS center\n";
    out << "    rotation         IS rotation\n";
    out << "    scale            IS scale\n";
    out << "    scaleOrientation IS scaleOrientation\n";
    out << "    translation      IS translation\n";
    out << "    children [\n";
    out << "      Group {\n";
    out << "        children IS viewpoints\n";
    out << "      }\n";
    out << "      Group {\n";
    out << "        children IS humanoidBody\n";
    out << "      }\n";
    out << "    ]\n";
    out << "  }\n";
    out << "}\n";
    out << "\n";
    out << "PROTO ExtraJoint [\n";
    out << "  exposedField SFString link1Name \"\"\n";
    out << "  exposedField SFString link2Name \"\"\n";
    out << "  exposedField SFVec3f  link1LocalPos 0 0 0\n";
    out << "  exposedField SFVec3f  link2LocalPos 0 0 0\n";
    out << "  exposedField SFString jointType \"xyz\"\n";
    out << "  exposedField SFVec3f  jointAxis 1 0 0\n";
    out << "]\n";
    out << "{\n";
    out << "}\n";
    out << "\n";
    out << "PROTO VisionSensor [\n";
    out << "  exposedField SFVec3f    translation       0 0 0\n";
    out << "  exposedField SFRotation rotation          0 0 1 0\n";
    out << "  exposedField SFFloat    fieldOutView       0.785398\n";
    out << "  exposedField SFString   name              \"\"\n";
    out << "  exposedField SFFloat    frontClipDistance 0.01\n";
    out << "  exposedField SFFloat    backClipDistance  10.0\n";
    out << "  exposedField SFString   type              \"NONE\"\n";
    out << "  exposedField SFInt32    sensorId          -1\n";
    out << "  exposedField SFInt32    width             320\n";
    out << "  exposedField SFInt32    height            240\n";
    out << "  exposedField SFFloat    frameRate         30\n";
    out << "]\n";
    out << "{\n";
    out << "  Transform {\n";
    out << "    rotation         IS rotation\n";
    out << "    translation      IS translation\n";
    out << "  }\n";
    out << "}\n";
    out << "\n";
    out << "PROTO ForceSensor [\n";
    out << "  exposedField SFVec3f maxForce -1 -1 -1\n";
    out << "  exposedField SFVec3f maxTorque -1 -1 -1\n";
    out << "  exposedField SFVec3f translation 0 0 0\n";
    out << "  exposedField SFRotation rotation 0 0 1 0\n";
    out << "  exposedField SFInt32 sensorId -1\n";
    out << "]\n";
    out << "{\n";
    out << "  Transform {\n";
    out << "    translation IS translation\n";
    out << "    rotation IS rotation\n";
    out << "  }\n";
    out << "}\n";
    out << "\n";
    out << "PROTO Gyro [\n";
    out << "  exposedField SFVec3f maxAngularVelocity -1 -1 -1\n";
    out << "  exposedField SFVec3f translation 0 0 0\n";
    out << "  exposedField SFRotation rotation 0 0 1 0\n";
    out << "  exposedField SFInt32 sensorId -1\n";
    out << "]\n";
    out << "{\n";
    out << "  Transform {\n";
    out << "    translation IS translation\n";
    out << "    rotation IS rotation\n";
    out << "  }\n";
    out << "}\n";
    out << "\n";
    out << "PROTO AccelerationSensor [\n";
    out << "  exposedField SFVec3f maxAcceleration -1 -1 -1\n";
    out << "  exposedField SFVec3f translation 0 0 0\n";
    out << "  exposedField SFRotation rotation 0 0 1 0\n";
    out << "  exposedField SFInt32 sensorId -1\n";
    out << "]\n";
    out << "{\n";
    out << "  Transform {\n";
    out << "    translation IS translation\n";
    out << "    rotation IS rotation\n";
    out << "  }\n";
    out << "}\n";
    out << "\n";
    out << "PROTO PressureSensor [\n";
    out << "  exposedField SFFloat maxPressure -1\n";
    out << "  exposedField SFVec3f translation 0 0 0\n";
    out << "  exposedField SFRotation rotation 0 0 1 0\n";
    out << "  exposedField SFInt32 sensorId -1\n";
    out << "]\n";
    out << "{\n";
    out << "  Transform {\n";
    out << "    translation IS translation\n";
    out << "    rotation IS rotation\n";
    out << "  }\n";
    out << "}\n";
    out << "\n";
    out << "PROTO PhotoInterrupter [\n";
    out << "  exposedField SFVec3f transmitter 0 0 0\n";
    out << "  exposedField SFVec3f receiver 0 0 0\n";
    out << "  exposedField SFInt32 sensorId -1\n";
    out << "]\n";
    out << "{\n";
    out << "  Transform{\n";
    out << "    children [\n";
    out << "      Transform{\n";
    out << "        translation IS transmitter\n";
    out << "      }\n";
    out << "      Transform{\n";
    out << "        translation IS receiver\n";
    out << "      }\n";
    out << "    ]\n";
    out << "  }\n";
    out << "}\n";
    out << "\n";
    out << "PROTO RangeSensor [\n";
    out << "  exposedField SFVec3f    translation       0 0 0\n";
    out << "  exposedField SFRotation rotation          0 0 1 0\n";
    out << "  exposedField MFNode     children          [ ]\n";
    out << "  exposedField SFInt32    sensorId          -1\n";
    out << "  exposedField SFFloat    scanAngle         3.14159 #[rad]\n";
    out << "  exposedField SFFloat    scanStep          0.1     #[rad]\n";
    out << "  exposedField SFFloat    scanRate          10      #[Hz]\n";
    out << "  exposedField SFFloat    minDistance       0.01\n";
    out << "  exposedField SFFloat    maxDistance       10\n";
    out << "]\n";
    out << "{\n";
    out << "  Transform {\n";
    out << "    rotation         IS rotation\n";
    out << "    translation      IS translation\n";
    out << "    children         IS children\n";
    out << "  }\n";
    out << "}\n";
    out << endl;
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
