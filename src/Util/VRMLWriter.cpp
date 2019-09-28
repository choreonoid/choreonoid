/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "VRMLWriter.h"
#include <cnoid/stdx/filesystem>
#include <iostream>

using namespace std;
using namespace cnoid;

namespace {

struct TIndent {
    TIndent() { size = 2; }
    void setSize(int s) { size = s; }
    void clear() { n = 0; spaces.clear(); }
    TIndent& operator++() {
        n += size;
        updateSpaces();
        return *this;
    }
    TIndent& operator--() {
        n -= size;
        if(n < 0) { n = 0; }
        updateSpaces();
        return *this;
    }
    void updateSpaces(){
        int numTabs = n / 8;
        int numSpaces = n % 8;
        spaces.clear();
        spaces.insert(0, numTabs, '\t');
        spaces.insert(numTabs, numSpaces, ' ');
    }
    std::string spaces;
    int n;
    int size;
};

std::ostream& operator<<(std::ostream& out, TIndent& indent)
{
    return out << indent.spaces;
}

const char* boolstr(bool v)
{
    if(v){
        return "TRUE";
    } else {
        return "FALSE";
    }
}

typedef void (VRMLWriterImpl::*VRMLWriterNodeMethod)(VRMLNodePtr node);

typedef std::map<std::string, VRMLWriterNodeMethod> TNodeMethodMap;
typedef std::pair<std::string, VRMLWriterNodeMethod> TNodeMethodPair;

TNodeMethodMap nodeMethodMap;

}

namespace cnoid {

std::ostream& operator<<(std::ostream& out, const SFVec2f& v)
{
    return out << v[0] << " " << v[1];
}

std::ostream& operator<<(std::ostream& out, const SFVec3f& v)
{
    return out << v[0] << " " << v[1] << " " << v[2];
}

std::ostream& operator<<(std::ostream& out, const SFColor& v)
{
    return out << v[0] << " " << v[1] << " " << v[2];
}

std::ostream& operator<<(std::ostream& out, const cnoid::SFRotation& v)
{
    const SFRotation::Vector3& a = v.axis();
    return out << a[0] << " " << a[1] << " " << a[2] << " " << v.angle();
}

class VRMLWriterImpl
{
public:
    std::ostream& out;
    std::string ofname;
    TIndent indent;
    typedef std::map<std::string, VRMLNodePtr> NodeMap;
    NodeMap defNodeMap;
    int numOneLineScalarElements;
    int numOneLineVectorElements;
    int numOneLineFaceElements;
    
    VRMLWriterImpl(std::ostream& out);
    void registerNodeMethodMap();
    void registerNodeMethod(const std::type_info& t, VRMLWriterNodeMethod method);
    VRMLWriterNodeMethod getNodeMethod(VRMLNodePtr node);

    template <class MFValues> void writeMFValues(MFValues values, int numColumn);
    void writeMFInt32(MFInt32& values, int maxColumns = 10);
    void writeMFInt32SeparatedByMinusValue(MFInt32& values, int maxColumns);
    void writeHeader();
    bool writeNode(VRMLNode* node);
    void writeNodeIter(VRMLNode* node);
    bool beginNode(const char* nodename, VRMLNodePtr node, bool isIndependentNode);
    void endNode();
    void writeGroupNode(VRMLNodePtr node);
    void writeGroupFields(VRMLGroupPtr group);
    void writeTransformNode(VRMLNodePtr node);
    void writeSwitchNode(VRMLNodePtr node);

    std::string abstorel(std::string& fname);
    void writeInlineNode(VRMLNodePtr node);
    void writeShapeNode(VRMLNodePtr node);
    void writeAppearanceNode(VRMLAppearancePtr appearance);
    void writeMaterialNode(VRMLMaterialPtr material);
    void writeBoxNode(VRMLNodePtr node);
    void writeConeNode(VRMLNodePtr node);
    void writeCylinderNode(VRMLNodePtr node);
    void writeSphereNode(VRMLNodePtr node);
    void writeIndexedFaceSetNode(VRMLNodePtr node);
    void writeCoordinateNode(VRMLCoordinatePtr coord);
    void writeNormalNode(VRMLNormalPtr normal);
    void writeColorNode(VRMLColorPtr color);
};

}


VRMLWriter::VRMLWriter(std::ostream& out)
{
    impl = new VRMLWriterImpl(out);
}


VRMLWriterImpl::VRMLWriterImpl(std::ostream& out)
    : out(out), ofname()
{
    if(nodeMethodMap.empty()){
        registerNodeMethodMap();
    }
    numOneLineScalarElements = 10;
    numOneLineVectorElements = 1;
    numOneLineFaceElements = 1;
}


VRMLWriter::~VRMLWriter()
{
    delete impl;
}


template <class MFValues> void VRMLWriterImpl::writeMFValues(MFValues values, int numColumn)
{
    int col = 0;
    int n = values.size();
    int row = 0;
    if(n > 0){
        row = (n - 1) / numColumn + 1;
    }
    ++indent;
    if(row <= 1){
        out << "[ ";
    } else {
        out << "[\n";
        out << indent;
    }
    for(int i=0; i < n; i++){
        if(col >= numColumn){
            col = 0;
            out << "\n";
            out << indent;
            ++row;
        }
        out << values[i] << " ";
        col++;
    }
    --indent;
    if(row <= 1){
        out << "]\n";
    } else {
        out << "\n" << indent << "]\n";
    }
}


void VRMLWriterImpl::writeMFInt32(MFInt32& values, int maxColumns)
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
        out << "\n";
    }
  
    out << --indent << "]\n";
    --indent;
}


void VRMLWriterImpl::writeMFInt32SeparatedByMinusValue(MFInt32& values, int maxColumns)
{
    out << "[\n";
    
    ++indent;

    out << indent;
    int col = 0;
    int n = values.size();
    for(int i=0; i < n; i++){
        out << values[i] << " ";
        if(values[i] < 0){
            ++col;
            if(col == maxColumns){
                col = 0;
                out << "\n";
                if(i < n-1){
                    out << indent;
                }
            }
        }
    }
    if(col < maxColumns){
        out << "\n";
    }
  
    out << --indent << "]\n";
}


void VRMLWriterImpl::registerNodeMethod(const std::type_info& t, VRMLWriterNodeMethod method) {
    nodeMethodMap.insert(TNodeMethodPair(t.name(), method));
}


VRMLWriterNodeMethod VRMLWriterImpl::getNodeMethod(VRMLNodePtr node) {
    TNodeMethodMap::iterator p = nodeMethodMap.find(typeid(*node).name());
    return (p != nodeMethodMap.end()) ? p->second : 0;
}


void VRMLWriterImpl::registerNodeMethodMap()
{
    registerNodeMethod(typeid(VRMLGroup),          &VRMLWriterImpl::writeGroupNode);
    registerNodeMethod(typeid(VRMLTransform),      &VRMLWriterImpl::writeTransformNode);
    registerNodeMethod(typeid(VRMLSwitch),         &VRMLWriterImpl::writeSwitchNode);
    registerNodeMethod(typeid(VRMLInline),         &VRMLWriterImpl::writeInlineNode);
    registerNodeMethod(typeid(VRMLShape),          &VRMLWriterImpl::writeShapeNode);
    registerNodeMethod(typeid(VRMLIndexedFaceSet), &VRMLWriterImpl::writeIndexedFaceSetNode);
    registerNodeMethod(typeid(VRMLBox),            &VRMLWriterImpl::writeBoxNode);
    registerNodeMethod(typeid(VRMLCone),           &VRMLWriterImpl::writeConeNode);
    registerNodeMethod(typeid(VRMLCylinder),       &VRMLWriterImpl::writeCylinderNode);
    registerNodeMethod(typeid(VRMLSphere),         &VRMLWriterImpl::writeSphereNode);
}


void VRMLWriter::setOutFileName(const std::string& ofname)
{
    impl->ofname = ofname;
}


void VRMLWriter::setIndentSize(int s)
{
    impl->indent.setSize(s);
}


void VRMLWriter::setNumOneLineScalarElements(int n)
{
    impl->numOneLineScalarElements = n;
}


void VRMLWriter::setNumOneLineVectorElements(int n)
{
    impl->numOneLineVectorElements = n;
}


void VRMLWriter::setNumOneLineFaceElements(int n)
{
    impl->numOneLineFaceElements = n;
}


void VRMLWriter::writeHeader()
{
    impl->writeHeader();
}


void VRMLWriterImpl::writeHeader()
{
    out << "#VRML V2.0 utf8\n";
}


bool VRMLWriter::writeNode(VRMLNode* node)
{
    return impl->writeNode(node);
}


bool VRMLWriterImpl::writeNode(VRMLNode* node)
{
    indent.clear();
    defNodeMap.clear();
    out << "\n";
    writeNodeIter(node);
    defNodeMap.clear();
    return true;
}


void VRMLWriterImpl::writeNodeIter(VRMLNode* node)
{
    VRMLWriterNodeMethod method = getNodeMethod(node);
    if(method){
        (this->*method)(node);
    } else {
        cout << "cannot find writer for " << typeid(*node).name() << " node." << endl;
    }
}


bool VRMLWriterImpl::beginNode(const char* nodename, VRMLNodePtr node, bool isIndependentNode)
{
    if(isIndependentNode){
        out << indent;
    }
    
    const string& defName = node->defName;
    if(defName.empty()){
        out << nodename << " {\n";
        
    } else {
        auto p = defNodeMap.find(defName);
        if(p != defNodeMap.end() && node == p->second){
            out << "USE " << defName << "\n";
            return false;
        } else {
            out << "DEF " << node->defName << " " << nodename << " {\n";
            defNodeMap[defName] = node;
        }
    }

    ++indent;
    return true;
}


void VRMLWriterImpl::endNode()
{
    out << --indent << "}\n";
}


void VRMLWriterImpl::writeGroupNode(VRMLNodePtr node)
{
    VRMLGroupPtr group = static_pointer_cast<VRMLGroup>(node);

    if(beginNode("Group", group, true)){
        writeGroupFields(group);
        endNode();
    }
}


void VRMLWriterImpl::writeGroupFields(VRMLGroupPtr group)
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


void VRMLWriterImpl::writeTransformNode(VRMLNodePtr node)
{
    VRMLTransformPtr trans = static_pointer_cast<VRMLTransform>(node);

    if(beginNode("Transform", trans, true)){

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
}


void VRMLWriterImpl::writeSwitchNode(VRMLNodePtr node)
{
    VRMLSwitchPtr switc = static_pointer_cast<VRMLSwitch>(node);

    if(beginNode("Switch", switc, true)){

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
}


/**
 * create relative path from absolute path
 * http://stackoverflow.com/questions/10167382/boostfilesystem-get-relative-path
 **/
std::string VRMLWriterImpl::abstorel(std::string& fname)
{
    stdx::filesystem::path from(ofname);
    stdx::filesystem::path to(fname);
    stdx::filesystem::path::const_iterator fromIter = from.begin();
    stdx::filesystem::path::const_iterator toIter = to.begin();
    
    while(fromIter != from.end() && toIter != to.end() && (*toIter) == (*fromIter)) {
        ++toIter;
        ++fromIter;
    }
    
    if (fromIter != from.end()) ++fromIter;
    
    stdx::filesystem::path finalPath;
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


void VRMLWriterImpl::writeInlineNode(VRMLNodePtr node)
{
    VRMLInlinePtr vinline = static_pointer_cast<VRMLInline>(node);

    if(beginNode("Inline", vinline, true)){

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
}


void VRMLWriterImpl::writeShapeNode(VRMLNodePtr node)
{
    VRMLShapePtr shape = static_pointer_cast<VRMLShape>(node);

    if(beginNode("Shape", shape, true)){

        if(shape->appearance){
            out << indent << "appearance ";
            writeAppearanceNode(shape->appearance);
        }
        if(shape->geometry){
            VRMLWriterNodeMethod method = getNodeMethod(shape->geometry);
            if(method){
                out << indent << "geometry ";
                (this->*method)(shape->geometry);
            }
        }
        
        endNode();
    }
}


void VRMLWriterImpl::writeAppearanceNode(VRMLAppearancePtr appearance)
{
    if(beginNode("Appearance", appearance, false)){

        if(appearance->material){
            out << indent << "material ";
            writeMaterialNode(appearance->material);
        }
        
        endNode();
    }
}


void VRMLWriterImpl::writeMaterialNode(VRMLMaterialPtr material)
{
    if(beginNode("Material", material, false)){

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
}


void VRMLWriterImpl::writeBoxNode(VRMLNodePtr node)
{
    VRMLBoxPtr box = static_pointer_cast<VRMLBox>(node);

    if(beginNode("Box", box, false)){
        out << indent << "size " << box->size << "\n";
        endNode();
    }
}


void VRMLWriterImpl::writeConeNode(VRMLNodePtr node)
{
    VRMLConePtr cone = static_pointer_cast<VRMLCone>(node);

    if(beginNode("Cone", cone, false)){
        out << indent << "bottomRadius " << cone->bottomRadius << "\n";
        out << indent << "height " << cone->height << "\n";
        if(!cone->side){
            out << indent << "side " << boolstr(cone->side) << "\n";
        }
        if(!cone->bottom){
            out << indent << "bottom " << boolstr(cone->bottom) << "\n";
        }
        endNode();
    }
}


void VRMLWriterImpl::writeCylinderNode(VRMLNodePtr node)
{
    VRMLCylinderPtr cylinder = static_pointer_cast<VRMLCylinder>(node);

    if(beginNode("Cylinder", cylinder, false)){

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
}


void VRMLWriterImpl::writeSphereNode(VRMLNodePtr node)
{
    VRMLSpherePtr sphere = static_pointer_cast<VRMLSphere>(node);

    if(beginNode("Sphere", sphere, false)){
        out << indent << "radius " << sphere->radius << "\n";
        endNode();
    }
}


void VRMLWriterImpl::writeIndexedFaceSetNode(VRMLNodePtr node)
{
    VRMLIndexedFaceSetPtr faceset = static_pointer_cast<VRMLIndexedFaceSet>(node);

    if(beginNode("IndexedFaceSet", faceset, false)){

        if(faceset->coord){
            out << indent << "coord ";
            writeCoordinateNode(faceset->coord);
        }
        if(!faceset->coordIndex.empty()){
            out << indent << "coordIndex ";
            writeMFInt32SeparatedByMinusValue(faceset->coordIndex, numOneLineFaceElements);
        }
        
        bool hasNormals = false;
        if(faceset->normal){
            out << indent << "normal ";
            writeNormalNode(faceset->normal);
            
            if(!faceset->normalIndex.empty()){
                out << indent << "normalIndex ";
                if(faceset->normalPerVertex){
                    writeMFInt32SeparatedByMinusValue(faceset->normalIndex, numOneLineFaceElements);
                } else {
                    writeMFInt32(faceset->normalIndex, numOneLineScalarElements);
                }
            }
            hasNormals = true;
        }
        
        bool hasColors = false;
        if(faceset->color){
            out << indent << "color ";
            writeColorNode(faceset->color);
            
            if(!faceset->colorIndex.empty()){
                out << indent << "colorIndex ";
                if(faceset->colorPerVertex){
                    writeMFInt32SeparatedByMinusValue(faceset->colorIndex, numOneLineFaceElements);
                } else {
                    writeMFInt32(faceset->colorIndex, numOneLineScalarElements);
                }
            }
            hasColors = true;
        }

        if(!faceset->ccw){
            out << indent << "ccw " << boolstr(faceset->ccw) << "\n";
        }
        if(!faceset->convex){
            out << indent << "convex " << boolstr(faceset->convex) << "\n";
        }
        if(!faceset->solid){
            out << indent << "solid " << boolstr(faceset->solid) << "\n";
        }
        if(faceset->creaseAngle > 0.0){
            out << indent << "creaseAngle " << faceset->creaseAngle << "\n";
        }
        if(hasNormals && !faceset->normalPerVertex){
            out << indent << "normalPerVertex " << boolstr(faceset->normalPerVertex) << "\n";
        }
        if(hasColors && !faceset->colorPerVertex){
            out << indent << "colorPerVertex " << boolstr(faceset->colorPerVertex) << "\n";
        }
        
        endNode();
    }
}


void VRMLWriterImpl::writeCoordinateNode(VRMLCoordinatePtr coord)
{
    if(beginNode("Coordinate", coord, false)){
        if(!coord->point.empty()){
            out << indent << "point ";
            writeMFValues(coord->point, numOneLineVectorElements);
        }
        endNode();
    }
}


void VRMLWriterImpl::writeNormalNode(VRMLNormalPtr normal)
{
    if(beginNode("Normal", normal, false)){
        if(!normal->vector.empty()){
            out << indent << "vector ";
            writeMFValues(normal->vector, numOneLineVectorElements);
        }
        endNode();
    }
}


void VRMLWriterImpl::writeColorNode(VRMLColorPtr color)
{
    if(beginNode("Color", color, false)){
        if(!color->color.empty()){
            out << indent << "color ";
            writeMFValues(color->color, numOneLineVectorElements);
        }
        endNode();
    }
}
