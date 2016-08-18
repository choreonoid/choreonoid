/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "VRMLWriter.h"
#include <boost/filesystem.hpp>
#include <map>
#include <iostream>

using namespace std;
using namespace boost;
using namespace cnoid;

namespace {

typedef std::map<std::string, VRMLWriterNodeMethod> TNodeMethodMap;
typedef std::pair<std::string, VRMLWriterNodeMethod> TNodeMethodPair;

TNodeMethodMap nodeMethodMap;

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


VRMLWriter::VRMLWriter(std::ostream& out) : out(out), ofname()
{
    if(nodeMethodMap.empty()){
        registerNodeMethodMap();
    }
}


void VRMLWriter::registerNodeMethod(const std::type_info& t, VRMLWriterNodeMethod method) {
    nodeMethodMap.insert(TNodeMethodPair(t.name(), method));
}


VRMLWriterNodeMethod VRMLWriter::getNodeMethod(VRMLNodePtr node) {
    TNodeMethodMap::iterator p = nodeMethodMap.find(typeid(*node).name());
    return (p != nodeMethodMap.end()) ? p->second : 0;
}


void VRMLWriter::registerNodeMethodMap()
{
    registerNodeMethod(typeid(VRMLGroup),          &VRMLWriter::writeGroupNode);
    registerNodeMethod(typeid(VRMLTransform),      &VRMLWriter::writeTransformNode);
    registerNodeMethod(typeid(VRMLInline),         &VRMLWriter::writeInlineNode);
    registerNodeMethod(typeid(VRMLShape),          &VRMLWriter::writeShapeNode);
    registerNodeMethod(typeid(VRMLIndexedFaceSet), &VRMLWriter::writeIndexedFaceSetNode);
    registerNodeMethod(typeid(VRMLBox),            &VRMLWriter::writeBoxNode);
    registerNodeMethod(typeid(VRMLCone),           &VRMLWriter::writeConeNode);
    registerNodeMethod(typeid(VRMLCylinder),       &VRMLWriter::writeCylinderNode);
    registerNodeMethod(typeid(VRMLSphere),         &VRMLWriter::writeSphereNode);
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
    } else {
        cout << "cannot find writer for " << typeid(*node).name() << " node." << endl;
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


/**
 * create relative path from absolute path
 * http://stackoverflow.com/questions/10167382/boostfilesystem-get-relative-path
 **/
std::string VRMLWriter::abstorel(std::string& fname)
{
    filesystem::path from(ofname);
    filesystem::path to(fname);
    filesystem::path::const_iterator fromIter = from.begin();
    filesystem::path::const_iterator toIter = to.begin();
    
    while(fromIter != from.end() && toIter != to.end() && (*toIter) == (*fromIter)) {
        ++toIter;
        ++fromIter;
    }
    
    if (fromIter != from.end()) ++fromIter;
    
    filesystem::path finalPath;
    while(fromIter != from.end()) {
        finalPath /= "..";
        ++fromIter;
    }
    while(toIter != to.end()) {
        finalPath /= *toIter;
        ++toIter;
    }
    return finalPath.string();
}


void VRMLWriter::writeInlineNode(VRMLNodePtr node)
{
    VRMLInlinePtr vinline = static_pointer_cast<VRMLInline>(node);

    beginNode("Inline", vinline);

    int n = vinline->urls.size();
    if (n == 1) {
        out << indent << "url \"" << abstorel(vinline->urls[0]) << "\"\n";
    } else {
        out << indent << "urls [\n";
        for(int i=0; i < n; i++){
            out << indent << "   \"" << abstorel(vinline->urls[i]) << "\"\n";
        }
        out << indent << "]\n";
    }

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


void VRMLWriter::writeBoxNode(VRMLNodePtr node)
{
    VRMLBoxPtr box = static_pointer_cast<VRMLBox>(node);

    beginNode("Box", box);

    out << indent << "size " << box->size << "\n";

    endNode();
}


void VRMLWriter::writeConeNode(VRMLNodePtr node)
{
    VRMLConePtr cone = static_pointer_cast<VRMLCone>(node);

    beginNode("Cone", cone);

    out << indent << "bottomRadius " << cone->bottomRadius << "\n";
    out << indent << "height " << cone->height << "\n";
    out << indent << "bottom " << boolstr(cone->bottom) << "\n";
    out << indent << "side " << boolstr(cone->side) << "\n";

    endNode();
}


void VRMLWriter::writeCylinderNode(VRMLNodePtr node)
{
    VRMLCylinderPtr cylinder = static_pointer_cast<VRMLCylinder>(node);

    beginNode("Cylinder", cylinder);

    out << indent << "radius " << cylinder->radius << "\n";
    out << indent << "height " << cylinder->height << "\n";
    out << indent << "top " << boolstr(cylinder->top) << "\n";
    out << indent << "bottom " << boolstr(cylinder->bottom) << "\n";
    out << indent << "side " << boolstr(cylinder->side) << "\n";

    endNode();
}


void VRMLWriter::writeSphereNode(VRMLNodePtr node)
{
    VRMLSpherePtr sphere = static_pointer_cast<VRMLSphere>(node);

    beginNode("Sphere", sphere);

    out << indent << "radius " << sphere->radius << "\n";

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
