/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "VRMLWriter.h"

using namespace std;
using namespace boost;
using namespace cnoid;

namespace {

typedef void (VRMLWriter::*VRMLWriterNodeMethod)(VRMLNodePtr node);
typedef std::map<std::string, VRMLWriterNodeMethod> TNodeMethodMap;
typedef std::pair<std::string, VRMLWriterNodeMethod> TNodeMethodPair;

TNodeMethodMap nodeMethodMap;

inline void registerNodeMethod(const std::type_info& t, VRMLWriterNodeMethod method) {
    nodeMethodMap.insert(TNodeMethodPair(t.name(), method)); 
}

VRMLWriterNodeMethod getNodeMethod(VRMLNodePtr node) {
    TNodeMethodMap::iterator p = nodeMethodMap.find(typeid(*node).name()); 
    return (p != nodeMethodMap.end()) ? p->second : 0; 
}

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


template <class MFValues> void VRMLWriter::writeMFValues(MFValues values, int numColumn)
{
    out << ++indent << "[\n";
    ++indent;

    out << indent;
    int col = 0;
    int n = values.size();
    for(int i=0; i < n; i++){
        out << values[i] << " ";
        col++;
        if(col == numColumn){
            col = 0;
            out << "\n";
            if(i < n-1){
                out << indent;
            }
        }
    }

    out << --indent << "]\n";
    --indent;
}


void VRMLWriter::writeMFInt32SeparatedByMinusValue(MFInt32& values)
{
    out << ++indent << "[\n";
    ++indent;

    out << indent;
    int n = values.size();
    for(int i=0; i < n; i++){
        out << values[i] << " ";
        if(values[i] < 0){
            out << "\n";
            if(i < n-1){
                out << indent;
            }
        }
    }
  
    out << --indent << "]\n";
    --indent;
}


VRMLWriter::VRMLWriter(std::ostream& out) : out(out)
{
    if(nodeMethodMap.empty()){
        registerNodeMethodMap();
    }
}

void VRMLWriter::registerNodeMethodMap() 
{
    registerNodeMethod(typeid(VRMLGroup),          &VRMLWriter::writeGroupNode);
    registerNodeMethod(typeid(VRMLTransform),      &VRMLWriter::writeTransformNode);
    registerNodeMethod(typeid(VRMLShape),          &VRMLWriter::writeShapeNode);
    registerNodeMethod(typeid(VRMLIndexedFaceSet), &VRMLWriter::writeIndexedFaceSetNode);
}

void VRMLWriter::writeHeader()
{
    out << "#VRML V2.0 utf8\n";
}


bool VRMLWriter::writeNode(VRMLNodePtr node)
{
    indent.clear();
    out << "\n";
    writeNodeIter(node);
    return true;
}


void VRMLWriter::writeNodeIter(VRMLNodePtr node)
{
    VRMLWriterNodeMethod method = getNodeMethod(node);
    if(method){
        (this->*method)(node);
    }
}


void VRMLWriter::beginNode(const char* nodename, VRMLNodePtr node)
{
    out << indent;
    if(node->defName.empty()){
        out << nodename << " {\n";
    } else {
        out << "DEF " << node->defName << " " << nodename << " {\n";
    }
    ++indent;
}


void VRMLWriter::endNode()
{
    out << --indent << "}\n";
}


void VRMLWriter::writeGroupNode(VRMLNodePtr node)
{
    VRMLGroupPtr group = static_pointer_cast<VRMLGroup>(node);

    beginNode("Group", group);
    writeGroupFields(group);
    endNode();
}


void VRMLWriter::writeGroupFields(VRMLGroupPtr group)
{
    if(group->bboxSize[0] >= 0){
        out << indent << "bboxCenter " << group->bboxCenter << "\n";
        out << indent << "bboxSize " << group->bboxSize << "\n";
    }

    if(!group->children.empty()){
        out << indent << "children [\n";
        ++indent;
        for(size_t i=0; i < group->children.size(); i++){
            writeNodeIter(group->children[i]);
        }
        out << --indent << "]\n";
    }
}


void VRMLWriter::writeTransformNode(VRMLNodePtr node)
{
    VRMLTransformPtr trans = static_pointer_cast<VRMLTransform>(node);

    beginNode("Transform", trans);

    out << indent << "center " << trans->center << "\n";
    out << indent << "rotation " << trans->rotation << "\n";
    out << indent << "scale " << trans->scale << "\n";
    out << indent << "scaleOrientation " << trans->scaleOrientation << "\n";
    out << indent << "translation " << trans->translation << "\n";

    writeGroupFields(trans);

    endNode();
}


void VRMLWriter::writeShapeNode(VRMLNodePtr node)
{
    VRMLShapePtr shape = static_pointer_cast<VRMLShape>(node);

    beginNode("Shape", shape);

    if(shape->appearance){
        out << indent << "appearance\n";
        ++indent;
        writeAppearanceNode(shape->appearance);
        --indent;
    }
    if(shape->geometry){
        out << indent << "geometry\n";
        VRMLWriterNodeMethod method = getNodeMethod(shape->geometry);
        if(method){
            ++indent;
            (this->*method)(shape->geometry);
            --indent;
        }
    }

    endNode();
}


void VRMLWriter::writeAppearanceNode(VRMLAppearancePtr appearance)
{
    beginNode("Appearance", appearance);

    if(appearance->material){
        out << indent << "material\n";
        ++indent;
        writeMaterialNode(appearance->material);
        --indent;
    }

    endNode();
}


void VRMLWriter::writeMaterialNode(VRMLMaterialPtr material)
{
    beginNode("Material", material);

    out << indent << "ambientIntensity " << material->ambientIntensity << "\n";
    out << indent << "diffuseColor " << material->diffuseColor << "\n";
    out << indent << "emissiveColor " << material->emissiveColor << "\n";
    out << indent << "shininess " << material->shininess << "\n";
    out << indent << "specularColor " << material->specularColor << "\n";
    out << indent << "transparency " << material->transparency << "\n";

    endNode();
}


void VRMLWriter::writeIndexedFaceSetNode(VRMLNodePtr node)
{
    VRMLIndexedFaceSetPtr faceset = static_pointer_cast<VRMLIndexedFaceSet>(node);

    beginNode("IndexedFaceSet", faceset);

    if(faceset->coord){
        out << indent << "coord\n";
        ++indent;
        writeCoordinateNode(faceset->coord);
        --indent;
    }
    if(!faceset->coordIndex.empty()){
        out << indent << "coordIndex\n";
        writeMFInt32SeparatedByMinusValue(faceset->coordIndex);
    }

    out << indent << "ccw " << boolstr(faceset->ccw) << "\n";
    out << indent << "convex " << boolstr(faceset->convex) << "\n";
    out << indent << "creaseAngle " << faceset->creaseAngle << "\n";
    out << indent << "solid " << boolstr(faceset->solid) << "\n";

    endNode();
}


void VRMLWriter::writeCoordinateNode(VRMLCoordinatePtr coord)
{
    beginNode("Coordinate", coord);

    if(!coord->point.empty()){
        out << indent << "point\n";
        writeMFValues(coord->point, 1);
    }

    endNode();
}
