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


void VRMLWriter::writeMFInt32(MFInt32& values, int maxColumns)
{
    out << ++indent << "[\n";
    ++indent;

    out << indent;
    int col = 0;
    int n = values.size();
    for(int i=0; i < n; i++){
        out << values[i] << " ";
        ++col;
        if(col == maxColumns){
            col = 0;
            out << "\n";
            if(i < n-1){
                out << indent;
            }
        }
    }
    if(col < maxColumns){
        cout << "\n";
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
    registerNodeMethod(typeid(VRMLSwitch),         &VRMLWriter::writeSwitchNode);
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

    if (trans->center != SFVec3f::Zero()){
        out << indent << "center " << trans->center << "\n";
    }
    if (trans->rotation.angle() != 0){
        out << indent << "rotation " << trans->rotation << "\n";
    }
    if (trans->scale != SFVec3f(1,1,1)){
        out << indent << "scale " << trans->scale << "\n";
    }
    if (trans->scaleOrientation.angle() != 0){
        out << indent << "scaleOrientation " << trans->scaleOrientation << "\n";
    }
    if (trans->translation != SFVec3f::Zero()){
        out << indent << "translation " << trans->translation << "\n";
    }

    writeGroupFields(trans);

    endNode();
}


void VRMLWriter::writeSwitchNode(VRMLNodePtr node)
{
    VRMLSwitchPtr switc = static_pointer_cast<VRMLSwitch>(node);

    beginNode("Switch", switc);

    out << indent << "whichChoice " << switc->whichChoice << "\n";

    if(!switc->choice.empty()){
        out << indent << "choice [\n";
        ++indent;
        for(size_t i=0; i < switc->choice.size(); ++i){
            writeNodeIter(switc->choice[i]);
        }
        out << --indent << "]\n";
    }

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
        VRMLWriterNodeMethod method = getNodeMethod(shape->geometry);
        if(method){
            out << indent << "geometry\n";
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

    if (material->ambientIntensity != 0.2){
        out << indent << "ambientIntensity " << material->ambientIntensity << "\n";
    }
    out << indent << "diffuseColor " << material->diffuseColor << "\n";
    if (material->emissiveColor != SFColor::Zero()){
        out << indent << "emissiveColor " << material->emissiveColor << "\n";
    }
    if (material->shininess != 0.2){
        out << indent << "shininess " << material->shininess << "\n";
    }
    if (material->specularColor != SFColor::Zero()){
        out << indent << "specularColor " << material->specularColor << "\n";
    }
    if (material->transparency != 0){
        out << indent << "transparency " << material->transparency << "\n";
    }

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
    if (!cylinder->top){
        out << indent << "top " << boolstr(cylinder->top) << "\n";
    }
    if (!cylinder->bottom){
        out << indent << "bottom " << boolstr(cylinder->bottom) << "\n";
    }
    if (!cylinder->side){
        out << indent << "side " << boolstr(cylinder->side) << "\n";
    }

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

    bool hasNormals = false;
    if(faceset->normal){
        out << indent << "normal\n";
        writeNormalNode(faceset->normal);

        if(!faceset->normalIndex.empty()){
            out << indent << "normalIndex\n";
            writeMFInt32(faceset->normalIndex);
        }
        hasNormals = true;
    }

    bool hasColors = false;
    if(faceset->color){
        out << indent << "color\n";
        writeColorNode(faceset->color);

        if(!faceset->colorIndex.empty()){
            out << indent << "colorIndex\n";
            writeMFInt32(faceset->colorIndex);
        }
        hasColors = true;
    }
        
    out << indent << "ccw " << boolstr(faceset->ccw) << "\n";
    out << indent << "convex " << boolstr(faceset->convex) << "\n";
    out << indent << "solid " << boolstr(faceset->solid) << "\n";
    if(faceset->creaseAngle > 0.0){
        out << indent << "creaseAngle " << faceset->creaseAngle << "\n";
    }
    if(hasNormals){
        out << indent << "normalPerVertex " << boolstr(faceset->normalPerVertex) << "\n";
    }
    if(hasColors){
        out << indent << "colorPerVertex " << boolstr(faceset->colorPerVertex) << "\n";
    }

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


void VRMLWriter::writeNormalNode(VRMLNormalPtr normal)
{
    beginNode("Normal", normal);

    if(!normal->vector.empty()){
        out << indent << "normal\n";
        writeMFValues(normal->vector, 1);
    }

    endNode();
}


void VRMLWriter::writeColorNode(VRMLColorPtr color)
{
    beginNode("Color", color);

    if(!color->color.empty()){
        out << indent << "color\n";
        writeMFValues(color->color, 1);
    }

    endNode();
}
