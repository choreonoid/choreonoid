/*!
  @file
*/

#include "VRMLBodyWriter.h"

using namespace std;
using namespace boost;
using namespace cnoid;

namespace {

inline std::ostream& operator<<(std::ostream& out, VRMLWriter::TIndent& indent)
{
    return out << indent.spaces;
}


inline const char* boolstr(bool v)
{
    if(v){
        return "TRUE";
    } else {
        return "FALSE";
    }
}

ostream& operator<<(std::ostream& out, const SFVec2f& v)
{
    return out << v[0] << " " << v[1];
}
    
ostream& operator<<(std::ostream& out, const SFVec3f& v)
{
    return out << v[0] << " " << v[1] << " " << v[2];
}

ostream& operator<<(std::ostream& out, const SFColor& v)
{
    return out << v[0] << " " << v[1] << " " << v[2];
}
    
ostream& operator<<(std::ostream& out, const SFRotation& v)
{
    const SFRotation::Vector3& a = v.axis();
    return out << a[0] << " " << a[1] << " " << a[2] << " " << v.angle();
}

}


VRMLBodyWriter::VRMLBodyWriter(std::ostream& out) : VRMLWriter(out)
{
}


void VRMLBodyWriter::registerNodeMethodMap() 
{
    VRMLWriter::registerNodeMethodMap();
    registerNodeMethod(typeid(VRMLHumanoid), (VRMLWriterNodeMethod)&VRMLBodyWriter::writeHumanoidNode);
    registerNodeMethod(typeid(VRMLJoint),    (VRMLWriterNodeMethod)&VRMLBodyWriter::writeJointNode);
    registerNodeMethod(typeid(VRMLSegment),  (VRMLWriterNodeMethod)&VRMLBodyWriter::writeSegmentNode);
}


void VRMLBodyWriter::writeHumanoidNode(VRMLNodePtr node)
{
    VRMLHumanoidPtr humanoid = static_pointer_cast<VRMLHumanoid>(node);

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
    VRMLJointPtr joint = static_pointer_cast<VRMLJoint>(node);

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
    VRMLSegmentPtr segment = static_pointer_cast<VRMLSegment>(node);

    beginNode("Segment", segment);

    out << indent << "mass " << segment->mass << "\n";
    out << indent << "centerOfMass " << segment->centerOfMass << "\n";
    out << indent << "momentsOfInertia\n";
    writeMFValues(segment->momentsOfInertia, 3);

    writeGroupFields(segment);

    endNode();
}
