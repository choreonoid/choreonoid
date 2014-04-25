/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "VRML.h"

using namespace cnoid;

const char* cnoid::labelOfVRMLfieldTypeId(const std::type_info& fieldType)
{
    if(fieldType == typeid(SFInt32)){
        return "SFInt32";
    } else if(fieldType == typeid(MFInt32)){
        return "MFInt32";
    } else if(fieldType == typeid(SFFloat)){
        return "SFFloat";
    } else if(fieldType == typeid(MFFloat)){
        return "MFFloat";
    } else if(fieldType == typeid(SFVec2f)){
        return "SFVec3f";
    } else if(fieldType == typeid(MFVec2f)){
        return "MFVec2f";
    } else if(fieldType == typeid(SFVec3f)){
        return "SFVec3f";
    } else if(fieldType == typeid(MFVec3f)){
        return "MFVec3f";
    } else if(fieldType == typeid(SFRotation)){
        return "SFRotation";
    } else if(fieldType == typeid(MFRotation)){
        return "MFRotation";
    } else if(fieldType == typeid(SFTime)){
        return "SFTime";
    } else if(fieldType == typeid(MFTime)){
        return "MFTime";
    } else if(fieldType == typeid(SFColor)){
        return "SFColor";
    } else if(fieldType == typeid(MFColor)){
        return "MFColor";
    } else if(fieldType == typeid(SFString)){
        return "SFString";
    } else if(fieldType == typeid(MFString)){
        return "MFString";
    } else if(fieldType == typeid(SFNode)){
        return "SFNode";
    } else if(fieldType == typeid(MFNode)){
        return "MFNode";
    } else if(fieldType == typeid(SFBool)){
        return "SFBool";
    } else if(fieldType == typeid(SFImage)){
        return "SFImage";
    } else {
        return "Unknown Field Type";
    }
}


VRMLNode::VRMLNode()
{
    refCounter = 0;
}


VRMLNode::~VRMLNode()
{

}


bool VRMLNode::isCategoryOf(VRMLNodeCategory category)
{
    return (category == ANY_NODE) ? true : categorySet.test(category);
}



VRMLUnsupportedNode::VRMLUnsupportedNode(const std::string& nodeTypeName) :
    nodeTypeName(nodeTypeName)
{
    categorySet.set(TOP_NODE);
    categorySet.set(CHILD_NODE);
}


VRMLViewpoint::VRMLViewpoint()
{
    categorySet.set(TOP_NODE);
    categorySet.set(BINDABLE_NODE);
    categorySet.set(CHILD_NODE);
    
    fieldOfView = 0.785398;
    jump = true;
    orientation.axis() = SFVec3f::UnitZ();
    orientation.angle() = 0.0;
    position << 0.0, 0.0, 10.0;
}


VRMLNavigationInfo::VRMLNavigationInfo() :
    avatarSize(3)
{
    categorySet.set(TOP_NODE);
    categorySet.set(BINDABLE_NODE);
    categorySet.set(CHILD_NODE);

    avatarSize[0] = 0.25;
    avatarSize[1] = 1.6;
    avatarSize[2] = 0.75;

    headlight = true;
    speed = 1.0;
    visibilityLimit = 0.0;

    type.push_back("WALK");
}


VRMLBackground::VRMLBackground()
{
    categorySet.set(TOP_NODE);
    categorySet.set(BINDABLE_NODE);
    categorySet.set(CHILD_NODE);
}


AbstractVRMLGroup::AbstractVRMLGroup()
{
    categorySet.set(TOP_NODE);
    categorySet.set(GROUPING_NODE);
    categorySet.set(CHILD_NODE);
}


void AbstractVRMLGroup::removeChild(int childIndex)
{
    replaceChild(childIndex, 0);
}


VRMLGroup::VRMLGroup()
{
    bboxCenter.setZero();
    bboxSize.fill(-1.0);
}


MFNode& VRMLGroup::getChildren()
{
    return children;
}


int VRMLGroup::countChildren()
{
    return children.size();
}


VRMLNode* VRMLGroup::getChild(int index)
{
    return children[index].get();
}


void VRMLGroup::replaceChild(int childIndex, VRMLNode* childNode)
{
    if(!childNode){
        children.erase(children.begin() + childIndex);
    } else {
        children[childIndex] = childNode;
    }
}


VRMLTransform::VRMLTransform()
{
    center.setZero();
    rotation.axis() = SFVec3f::UnitZ();
    rotation.angle() = 0.0;
    scale.setOnes();
    scaleOrientation.axis() = SFVec3f::UnitZ();
    scaleOrientation.angle() = 0.0;
    translation.setZero();
}


Eigen::Affine3d VRMLTransform::toAffine3d()
{
    const Eigen::Translation3d C(center);
    const SFRotation& SR = scaleOrientation;
    const Eigen::AlignedScaling3d S(scale);
    const SFRotation& R = rotation;
    const Eigen::Translation3d T(translation);

    return T * C * R * SR * S * SR.inverse() * C.inverse();
}


VRMLInline::VRMLInline()
{
    categorySet.set(INLINE_NODE);
}


VRMLAnotherFormatFile::VRMLAnotherFormatFile()
{
    categorySet.set(TOP_NODE);
    categorySet.set(CHILD_NODE);
}


VRMLShape::VRMLShape()
{
    categorySet.set(TOP_NODE);
    categorySet.set(CHILD_NODE);
    categorySet.set(SHAPE_NODE);
}


VRMLAppearance::VRMLAppearance()
{
    categorySet.set(APPEARANCE_NODE);
}


VRMLMaterial::VRMLMaterial()
{
    categorySet.set(MATERIAL_NODE);

    diffuseColor << 0.8f, 0.8f, 0.8f;
    emissiveColor.setZero();
    specularColor.setZero();
    ambientIntensity = 0.2;
    shininess = 0.2;
    transparency = 0.0;
}


VRMLTexture::VRMLTexture()
{
    categorySet.set(TEXTURE_NODE);
}


VRMLImageTexture::VRMLImageTexture()
{
    repeatS = true; 
    repeatT = true; 
}


VRMLTextureTransform::VRMLTextureTransform()
{
    categorySet.set(TEXTURE_TRANSFORM_NODE);

    center.setZero();
    scale.setOnes();
    translation.setZero();
    rotation = 0.0;
}


VRMLGeometry::VRMLGeometry()
{
    categorySet.set(GEOMETRY_NODE);
}


VRMLBox::VRMLBox()
{
    size.fill(2.0);
}


VRMLCone::VRMLCone()
{
    bottom = true;
    bottomRadius = 1.0;
    height = 2.0;
    side = true;
}


VRMLCylinder::VRMLCylinder()
{
    height = 2.0;
    radius = 1.0;
    bottom = true;
    side = true;
    top = true;
}


VRMLSphere::VRMLSphere()
{
    radius = 1.0; 
}


VRMLFontStyle::VRMLFontStyle()
{
    categorySet.set(FONT_STYLE_NODE);
  
    family.push_back("SERIF");
    horizontal = true;
    justify.push_back("BEGIN");
    leftToRight = true;
    size = 1.0;
    spacing = 1.0;
    style = "PLAIN";
    topToBottom = true;
}


VRMLText::VRMLText()
{
    maxExtent = 0.0;
}


VRMLIndexedLineSet::VRMLIndexedLineSet()
{
    colorPerVertex = true;
}


VRMLIndexedFaceSet::VRMLIndexedFaceSet()
{
    ccw = true;
    convex = true;
    creaseAngle = 0.0;
    normalPerVertex = true;
    solid = true;
}


VRMLColor::VRMLColor()
{
    categorySet.set(COLOR_NODE);
}


VRMLCoordinate::VRMLCoordinate()
{
    categorySet.set(COORDINATE_NODE);
}


VRMLTextureCoordinate::VRMLTextureCoordinate()
{
    categorySet.set(TEXTURE_COORDINATE_NODE);
}


VRMLNormal::VRMLNormal()
{
    categorySet.set(NORMAL_NODE);
}


VRMLCylinderSensor::VRMLCylinderSensor()
{
    categorySet.set(CHILD_NODE);
    categorySet.set(SENSOR_NODE);
  
    autoOffset = true;
    diskAngle = 0.262;
    enabled = true;
    maxAngle = -1;
    minAngle = 0;
    offset = 0;
}


VRMLPointSet::VRMLPointSet()
{
    coord = NULL;
    color = NULL;
}


VRMLPixelTexture::VRMLPixelTexture()
{
    image.width  = 0;
    image.height = 0;
    image.numComponents = 0;
    image.pixels.clear();
	
    repeatS = true;
    repeatT = true;
}


VRMLMovieTexture::VRMLMovieTexture()
{
    // url
    loop = false;
    speed = 0;
    startTime = 0.0;
    stopTime = 0.0;
    repeatS = true;
    repeatT = true;
}


VRMLElevationGrid::VRMLElevationGrid()
{
    xDimension = 0;
    zDimension = 0;
    xSpacing = 0.0;
    zSpacing = 0.0;
    // height	// MFFloat
    ccw = true;
    colorPerVertex = true;
    creaseAngle = 0.0;
    normalPerVertex = true;
    solid = true;
    color = NULL;
    normal = NULL;
    texCoord = NULL;
}


VRMLExtrusion::VRMLExtrusion()
{
    // crossSection
    // spine
    beginCap = true;
    endCap   = true;
    solid    = true;
    ccw      = true;
    convex   = true;
    creaseAngle = 0;
    orientation.push_back(SFRotation(0.0, SFVec3f::UnitZ()));
    scale.push_back(SFVec2f(1.0, 1.0));
}


VRMLSwitch::VRMLSwitch()
{
    whichChoice = -1;
}


MFNode& VRMLSwitch::getChildren()
{
    return choice;
}


int VRMLSwitch::countChildren()
{
    return choice.size();
}


VRMLNode* VRMLSwitch::getChild(int index)
{
    return choice[index].get();
}


void VRMLSwitch::replaceChild(int childIndex, VRMLNode* childNode)
{
    if(!childNode){
        choice.erase(choice.begin() + childIndex);
        if(whichChoice == childIndex){
            whichChoice = -1;
        } else if(whichChoice > childIndex){
            whichChoice -= 1;
        }
    } else {
        choice[childIndex] = childNode;
    }
}


VRMLLOD::VRMLLOD()
{
    center.setZero();
}


MFNode& VRMLLOD::getChildren()
{
    return level;
}


int VRMLLOD::countChildren()
{
    return level.size();
}


VRMLNode* VRMLLOD::getChild(int index)
{
    return level[index].get();
}


void VRMLLOD::replaceChild(int childIndex, VRMLNode* childNode)
{
    if(!childNode){
        level.erase(level.begin() + childIndex);
        if(!level.empty()){
            int rangeIndexToRemove = (childIndex > 0) ? (childIndex - 1) : 0;
            range.erase(range.begin() + rangeIndexToRemove);
        }
    } else {
        level[childIndex] = childNode;
    }
}


VRMLCollision::VRMLCollision()
{
    collide = true;
}


VRMLAnchor::VRMLAnchor()
{

}


VRMLBillboard::VRMLBillboard()
{
    axisOfRotation.setZero();
}


VRMLFog::VRMLFog()
{
    color.setZero();
    visibilityRange = 0.0;
    fogType = "LINEAR";
}


VRMLWorldInfo::VRMLWorldInfo()
{
    categorySet.set(TOP_NODE);
}


VRMLLight::VRMLLight()
{
    categorySet.set(TOP_NODE);
    categorySet.set(CHILD_NODE);
    categorySet.set(LIGHT_NODE);

    on = true;
    color.setOnes();
    intensity = 1.0;
    ambientIntensity = 0.0;
}

VRMLPointLight::VRMLPointLight()
{
    location.setZero();
    radius = 100.0;
    attenuation << 1.0, 0.0, 0.0;
}


VRMLDirectionalLight::VRMLDirectionalLight()
{
    direction << 0.0, 0.0, -1.0;
}


VRMLSpotLight::VRMLSpotLight()
{
    direction << 0.0, 0.0, -1.0;
    beamWidth = 1.570796;
    cutOffAngle = 0.785398;
}


VRMLProto::VRMLProto(const std::string& n) : protoName(n)
{
    categorySet.set(TOP_NODE);
    categorySet.set(PROTO_DEF_NODE);
}


VRMLProtoInstance::VRMLProtoInstance(VRMLProtoPtr proto0) :
    proto(proto0),
    fields(proto0->fields) 
{
    categorySet.set(TOP_NODE);
    categorySet.set(PROTO_INSTANCE_NODE);;
    categorySet.set(CHILD_NODE);
}
