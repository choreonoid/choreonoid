/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_VRML_H
#define CNOID_UTIL_VRML_H

#include "Referenced.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cnoid/stdx/variant>
#include <string>
#include <bitset>
#include <vector>
#include <map>
#include "exportdecl.h"

namespace cnoid {

typedef bool SFBool;
typedef int  SFInt32;
typedef double SFFloat;
typedef std::string SFString;

// Define SFTime as struct to clearly distinguish its type from SFFloat
struct SFTime {
    double value;
    inline SFTime() { value = 0.0; }
    inline SFTime(double time) { value = time; }
    inline double operator=(double time) { return (value = time); }
};

typedef Eigen::Vector2d SFVec2f;
typedef Eigen::Vector2f SFVec2s; // single-precision type used for texture coordinates
typedef Eigen::Vector3d SFVec3f;
typedef Eigen::Vector3f SFVec3s; // single-precision type used for vertices and normals
typedef Eigen::Vector3f SFColor;
typedef Eigen::AngleAxisd SFRotation;

typedef struct {
    int width;
    int height;
    int numComponents;
    std::vector<unsigned char> pixels;
} SFImage;

typedef std::vector<SFInt32> MFInt32;
typedef std::vector<SFFloat> MFFloat;
typedef std::vector<SFVec2f, Eigen::aligned_allocator<SFVec2f>> MFVec2f;
typedef std::vector<SFVec2s> MFVec2s; // single-precision type used for texture coordinates
typedef std::vector<SFVec3f> MFVec3f;
typedef std::vector<SFVec3s> MFVec3s; // single-precision type used for vertices and normals
typedef std::vector<SFRotation, Eigen::aligned_allocator<SFRotation>> MFRotation;
typedef std::vector<SFTime> MFTime;
typedef std::vector<SFColor> MFColor;
typedef std::vector<SFString> MFString;

// see 4.6.3 - 4.6.10 of the VRML97 specification
enum VRMLNodeCategory {

    ANY_NODE = -1,

    PROTO_DEF_NODE = 0,
    PROTO_INSTANCE_NODE,

    TOP_NODE,
    BINDABLE_NODE,
    GROUPING_NODE,
    CHILD_NODE,

    APPEARANCE_NODE,
    MATERIAL_NODE,
    TEXTURE_NODE,
    TEXTURE_TRANSFORM_NODE,

    SHAPE_NODE,
    GEOMETRY_NODE,
    COORDINATE_NODE,
    COLOR_NODE,
    NORMAL_NODE,
    TEXTURE_COORDINATE_NODE,

    LIGHT_NODE,

    FONT_STYLE_NODE,

    SENSOR_NODE,
    INLINE_NODE,

    NUM_VRML_NODE_CATEGORIES
};

class VRMLNode;

//! Abstract base class of all vrml nodes.
class CNOID_EXPORT VRMLNode : public Referenced
{
public:

    VRMLNode();
    virtual ~VRMLNode();
    virtual const char* typeName() const = 0;

    std::string defName;

    bool isCategoryOf(VRMLNodeCategory category);

protected:
    std::bitset<NUM_VRML_NODE_CATEGORIES> categorySet;
};

typedef ref_ptr<VRMLNode> VRMLNodePtr;

typedef VRMLNodePtr SFNode;
typedef std::vector<SFNode> MFNode;


class CNOID_EXPORT  VRMLUnsupportedNode : public VRMLNode
{
public:
    VRMLUnsupportedNode(const std::string& nodeTypeName);
    virtual const char* typeName() const override;
    std::string nodeTypeName;
};
typedef ref_ptr<VRMLUnsupportedNode> VRMLUnsupportedNodePtr;


//! VRML Viewpoint node
class CNOID_EXPORT  VRMLViewpoint : public VRMLNode
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
    VRMLViewpoint();
    virtual const char* typeName() const override;

    SFRotation orientation;
    SFFloat fieldOfView;
    SFBool jump;
    SFVec3f position;
    SFString description;
};
typedef ref_ptr<VRMLViewpoint> VRMLViewpointPtr;


//! VRML NavigationInfo node
class CNOID_EXPORT  VRMLNavigationInfo : public VRMLNode
{
public:
    VRMLNavigationInfo();
    virtual const char* typeName() const override;

    MFFloat avatarSize;
    SFBool headlight;
    SFFloat speed;
    MFString type;
    SFFloat visibilityLimit;
};
typedef ref_ptr<VRMLNavigationInfo> VRMLNavigationInfoPtr;


//! VRML Background node
class CNOID_EXPORT  VRMLBackground : public VRMLNode
{
public:
    VRMLBackground();
    virtual const char* typeName() const override;
      
    MFFloat groundAngle;
    MFColor groundColor;
    MFFloat skyAngle;
    MFColor skyColor;
    MFString backUrl;
    MFString bottomUrl;
    MFString frontUrl;
    MFString leftUrl;
    MFString rightUrl;
    MFString topUrl;
};
typedef ref_ptr<VRMLBackground> VRMLBackgroundPtr;


class CNOID_EXPORT AbstractVRMLGroup : public VRMLNode
{
public:
    AbstractVRMLGroup();
        
    virtual MFNode& getChildren() = 0;
    virtual int countChildren() = 0;
    virtual VRMLNode* getChild(int index) = 0;
    virtual void replaceChild(int childIndex, VRMLNode* childNode) = 0;
        
    void removeChild(int childIndex);
};
typedef ref_ptr<AbstractVRMLGroup> AbstractVRMLGroupPtr;
    
    
//! VRML Group node
class CNOID_EXPORT VRMLGroup : public AbstractVRMLGroup
{
public:
    VRMLGroup();
    virtual const char* typeName() const override;

    virtual MFNode& getChildren() override;
    virtual int countChildren() override;
    virtual VRMLNode* getChild(int index) override;
    virtual void replaceChild(int childIndex, VRMLNode* childNode) override;

    SFVec3f bboxCenter;    
    SFVec3f bboxSize;  
    MFNode children;
};
typedef ref_ptr<VRMLGroup> VRMLGroupPtr;


//! VRML Transform node
class CNOID_EXPORT  VRMLTransform : public VRMLGroup
{
public:
    VRMLTransform();
    virtual const char* typeName() const override;

    Eigen::Affine3d toAffine3d();

    SFVec3f center;
    SFRotation rotation;
    SFVec3f scale;
    SFRotation scaleOrientation;
    SFVec3f translation;
};
typedef ref_ptr<VRMLTransform> VRMLTransformPtr;

//! VRML Inline node
class CNOID_EXPORT  VRMLInline : public VRMLGroup
{
public:
    VRMLInline();
    virtual const char* typeName() const override;

    MFString urls;
};
typedef ref_ptr<VRMLInline> VRMLInlinePtr;


class CNOID_EXPORT  VRMLNonVrmlInline : public VRMLNode
{
public:
    VRMLNonVrmlInline();
    virtual const char* typeName() const override;

    SFString url; // An absolute path should be specified
};
typedef ref_ptr<VRMLNonVrmlInline> VRMLNonVrmlInlinePtr;


class VRMLAppearance;
typedef ref_ptr<VRMLAppearance> VRMLAppearancePtr;

class VRMLGeometry;
typedef ref_ptr<VRMLGeometry> VRMLGeometryPtr;


//! VRML Shape node
class CNOID_EXPORT  VRMLShape : public VRMLNode
{
public:
    VRMLShape();
    virtual const char* typeName() const override;

    VRMLAppearancePtr appearance;
    SFNode geometry;
};
typedef ref_ptr<VRMLShape> VRMLShapePtr;


class VRMLMaterial;
typedef ref_ptr<VRMLMaterial> VRMLMaterialPtr;

class VRMLTexture;
typedef ref_ptr<VRMLTexture> VRMLTexturePtr;

class VRMLTextureTransform;
typedef ref_ptr<VRMLTextureTransform> VRMLTextureTransformPtr;

//! VRML Appearance node
class CNOID_EXPORT VRMLAppearance : public VRMLNode
{
public:
    VRMLAppearance();
    virtual const char* typeName() const override;
      
    VRMLMaterialPtr material;
    VRMLTexturePtr texture;
    VRMLTextureTransformPtr textureTransform;
};


//! VRML Material node
class CNOID_EXPORT VRMLMaterial : public VRMLNode
{
public:
    VRMLMaterial();
    virtual const char* typeName() const override;

    SFFloat ambientIntensity;
    SFColor diffuseColor;
    SFColor emissiveColor;
    SFFloat shininess;
    SFColor specularColor;
    SFFloat transparency;
};


//! Base class of VRML Texture nodes
class CNOID_EXPORT VRMLTexture : public VRMLNode
{
public:
    VRMLTexture();
};

    
//! VRML ImageTexture node
class CNOID_EXPORT VRMLImageTexture : public VRMLTexture
{
public:
    VRMLImageTexture();
    virtual const char* typeName() const override;

    MFString url;
    SFBool repeatS;
    SFBool repeatT;
};
typedef ref_ptr<VRMLImageTexture> VRMLImageTexturePtr;


//! VRML TextureTransform node
class CNOID_EXPORT VRMLTextureTransform : public VRMLNode
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
    VRMLTextureTransform();
    virtual const char* typeName() const override;

    SFVec2f center;
    SFFloat rotation;
    SFVec2f scale;
    SFVec2f translation;
};

//! Base class of VRML geometry nodes
class CNOID_EXPORT VRMLGeometry : public VRMLNode
{
public:
    VRMLGeometry();
};

//! VRML Box node
class CNOID_EXPORT VRMLBox : public VRMLGeometry
{
public:
    VRMLBox();
    virtual const char* typeName() const override;

    SFVec3f size;
};
typedef ref_ptr<VRMLBox> VRMLBoxPtr;


//! VRML Cone node
class CNOID_EXPORT VRMLCone : public VRMLGeometry
{
public:
    VRMLCone();
    virtual const char* typeName() const override;

    SFBool bottom;
    SFFloat bottomRadius;
    SFFloat height;
    SFBool side;
};
typedef ref_ptr<VRMLCone> VRMLConePtr;


//! VRML Cylinder node
class CNOID_EXPORT VRMLCylinder : public VRMLGeometry
{
public:
    VRMLCylinder();
    virtual const char* typeName() const override;

    SFBool bottom;
    SFFloat height;
    SFFloat radius;
    SFBool side;
    SFBool top;
};
typedef ref_ptr<VRMLCylinder> VRMLCylinderPtr;


//! VRML Sphere node
class CNOID_EXPORT VRMLSphere : public VRMLGeometry
{
public:
    VRMLSphere();
    virtual const char* typeName() const override;

    SFFloat radius;
};
typedef ref_ptr<VRMLSphere> VRMLSpherePtr;


//! VRML FontStyle node
class CNOID_EXPORT VRMLFontStyle : public VRMLNode
{
public:
    VRMLFontStyle();
    virtual const char* typeName() const override;

    MFString family;       
    SFBool horizontal;
    MFString justify;
    SFString language;
    SFBool leftToRight;
    SFFloat size;
    SFFloat spacing;
    SFString style;
    SFBool topToBottom;
};
typedef ref_ptr<VRMLFontStyle> VRMLFontStylePtr;


//! VRML Text node
class CNOID_EXPORT VRMLText : public VRMLGeometry
{
public:
    VRMLText();
    virtual const char* typeName() const override;

    MFString fstring;
    VRMLFontStylePtr fontStyle;
    MFFloat length;
    SFFloat maxExtent;
};
typedef ref_ptr<VRMLText> VRMLTextPtr;


class VRMLColor;
typedef ref_ptr<VRMLColor> VRMLColorPtr;

class VRMLCoordinate;
typedef ref_ptr<VRMLCoordinate> VRMLCoordinatePtr;

//! VRML IndexedLineSet node
class CNOID_EXPORT VRMLIndexedLineSet : public VRMLGeometry
{
public: 
    VRMLIndexedLineSet();
    virtual const char* typeName() const override;

    VRMLColorPtr color;
    VRMLCoordinatePtr coord;
    MFInt32 colorIndex;
    SFBool colorPerVertex;
    MFInt32 coordIndex;
};
typedef ref_ptr<VRMLIndexedLineSet> VRMLIndexedLineSetPtr;


class VRMLNormal;
typedef ref_ptr<VRMLNormal> VRMLNormalPtr;

class VRMLTextureCoordinate;
typedef ref_ptr<VRMLTextureCoordinate> VRMLTextureCoordinatePtr;


//! VRML IndexedFaseSet node
class CNOID_EXPORT VRMLIndexedFaceSet : public VRMLIndexedLineSet
{
public:
    VRMLIndexedFaceSet();
    virtual const char* typeName() const override;

    VRMLNormalPtr normal;
    VRMLTextureCoordinatePtr texCoord;
    SFBool ccw;
    SFBool convex;
    SFFloat creaseAngle;
    MFInt32 normalIndex;
    SFBool normalPerVertex;
    SFBool solid;
    MFInt32 texCoordIndex;
};
typedef ref_ptr<VRMLIndexedFaceSet> VRMLIndexedFaceSetPtr;


//! VRML Color node
class CNOID_EXPORT VRMLColor : public VRMLNode
{
public:
    VRMLColor();
    virtual const char* typeName() const override;
      
    MFColor color;
};


//! VRML Coordinate node
class CNOID_EXPORT VRMLCoordinate : public VRMLNode
{
public:
    VRMLCoordinate();
    virtual const char* typeName() const override;

    MFVec3s point;
};


//! VRML TextureCoordinate node
class CNOID_EXPORT VRMLTextureCoordinate : public VRMLNode
{
public:
    VRMLTextureCoordinate();
    virtual const char* typeName() const override;

    MFVec2s point;
};


//! VRML Normal node
class CNOID_EXPORT VRMLNormal : public VRMLNode
{
public:
    VRMLNormal();
    virtual const char* typeName() const override;

    MFVec3s vector;
};


//! VRML CylinderSensor node
class CNOID_EXPORT VRMLCylinderSensor : public VRMLNode
{
public:
    VRMLCylinderSensor();
    virtual const char* typeName() const override;

    SFBool autoOffset;
    SFFloat diskAngle;
    SFBool enabled;
    SFFloat maxAngle;
    SFFloat minAngle;
    SFFloat offset;
};
typedef ref_ptr<VRMLCylinderSensor> VRMLCylinderSensorPtr;


//! VRML PointSet node
class CNOID_EXPORT VRMLPointSet : public VRMLGeometry
{
public:
    VRMLPointSet();
    virtual const char* typeName() const override;

    VRMLCoordinatePtr coord;
    VRMLColorPtr color;
};

typedef ref_ptr<VRMLPointSet> VRMLPointSetPtr;


//! VRML PixelTexture node
class CNOID_EXPORT VRMLPixelTexture : public VRMLTexture
{
public:
    VRMLPixelTexture();
    virtual const char* typeName() const override;

    SFImage image;
    SFBool repeatS;
    SFBool repeatT;
};

typedef ref_ptr<VRMLPixelTexture> VRMLPixelTexturePtr;


//! VRML MovieTexture node
class CNOID_EXPORT VRMLMovieTexture : public VRMLTexture
{
public:
    VRMLMovieTexture();
    virtual const char* typeName() const override;

    MFString url;
    SFBool loop;
    SFFloat speed;
    SFTime startTime;
    SFTime stopTime;
    SFBool repeatS;
    SFBool repeatT;
};

typedef ref_ptr<VRMLMovieTexture> VRMLMovieTexturePtr;


//! VRML ElevationGrid node
class CNOID_EXPORT VRMLElevationGrid : public VRMLGeometry
{
public:
    VRMLElevationGrid();
    virtual const char* typeName() const override;

    SFInt32 xDimension;
    SFInt32 zDimension;
    SFFloat xSpacing;
    SFFloat zSpacing;
    MFFloat height;
    SFBool ccw;
    SFBool colorPerVertex;
    SFFloat creaseAngle;
    SFBool normalPerVertex;
    SFBool solid;
    VRMLColorPtr color;
    VRMLNormalPtr normal;
    VRMLTextureCoordinatePtr texCoord;
};

typedef ref_ptr<VRMLElevationGrid> VRMLElevationGridPtr;


//! VRML Extrusion node
class CNOID_EXPORT VRMLExtrusion : public VRMLGeometry
{
public:
    VRMLExtrusion();
    virtual const char* typeName() const override;

    MFVec2f crossSection;
    MFVec3f spine;
    MFVec2f scale;
    MFRotation orientation;
    SFBool beginCap;
    SFBool endCap;
    SFBool solid;
    SFBool ccw;
    SFBool convex;
    SFFloat creaseAngle;
};

typedef ref_ptr<VRMLExtrusion> VRMLExtrusionPtr;


class CNOID_EXPORT VRMLSwitch : public AbstractVRMLGroup
{
public:
    VRMLSwitch();
    virtual const char* typeName() const override;

    virtual MFNode& getChildren() override;
    virtual int countChildren() override;
    virtual VRMLNode* getChild(int index) override;
    virtual void replaceChild(int childIndex, VRMLNode* childNode) override;

    MFNode choice;
    SFInt32 whichChoice;
};

typedef ref_ptr<VRMLSwitch> VRMLSwitchPtr;


class CNOID_EXPORT VRMLLOD : public AbstractVRMLGroup
{
public:
    VRMLLOD();
    virtual const char* typeName() const override;

    virtual MFNode& getChildren() override;
    virtual int countChildren() override;
    virtual VRMLNode* getChild(int index) override;
    virtual void replaceChild(int childIndex, VRMLNode* childNode) override;

    MFFloat range;
    SFVec3f center;
    MFNode level;
};

typedef ref_ptr<VRMLLOD> VRMLLODPtr;


class CNOID_EXPORT VRMLCollision : public VRMLGroup
{
public:
    VRMLCollision();
    virtual const char* typeName() const override;

    SFBool collide;
    SFNode proxy;
};

typedef ref_ptr<VRMLCollision> VRMLCollisionPtr;


class CNOID_EXPORT VRMLAnchor : public VRMLGroup
{
public:
    VRMLAnchor();
    virtual const char* typeName() const override;

    SFString description;
    MFString parameter;
    MFString url;
};

typedef ref_ptr<VRMLAnchor> VRMLAnchorPtr;


class CNOID_EXPORT VRMLBillboard : public VRMLGroup
{
public:
    VRMLBillboard();
    virtual const char* typeName() const override;

    SFVec3f axisOfRotation;
};

typedef ref_ptr<VRMLBillboard> VRMLBillboardPtr;


class CNOID_EXPORT VRMLFog : public VRMLNode
{
public:
    VRMLFog();
    virtual const char* typeName() const override;

    SFColor color;
    SFFloat visibilityRange;
    SFString fogType;
};

typedef ref_ptr<VRMLFog> VRMLFogPtr;


class CNOID_EXPORT  VRMLWorldInfo : public VRMLNode
{
public:
    VRMLWorldInfo();
    virtual const char* typeName() const override;

    SFString title;
    MFString info;
};

typedef ref_ptr<VRMLWorldInfo> VRMLWorldInfoPtr;


class CNOID_EXPORT VRMLLight : public VRMLNode
{
public:
    VRMLLight();

    SFBool on;
    SFColor color;
    SFFloat intensity;
    SFFloat ambientIntensity;
};

typedef ref_ptr<VRMLLight> VRMLLightPtr;


class CNOID_EXPORT VRMLPointLight : public VRMLLight
{
public:
    VRMLPointLight();
    virtual const char* typeName() const override;

    SFVec3f location;
    SFFloat radius;
    SFVec3f attenuation;
};

typedef ref_ptr<VRMLPointLight> VRMLPointLightPtr;


class CNOID_EXPORT VRMLDirectionalLight : public VRMLLight
{
public:
    VRMLDirectionalLight();
    virtual const char* typeName() const override;

    SFVec3f direction;
};

typedef ref_ptr<VRMLDirectionalLight> VRMLDirectionalLightPtr;


class CNOID_EXPORT VRMLSpotLight : public VRMLPointLight
{
public:
    VRMLSpotLight();
    virtual const char* typeName() const override;

    SFVec3f direction;
    SFFloat beamWidth;
    SFFloat cutOffAngle;
};

typedef ref_ptr<VRMLSpotLight> VRMLSpotLightPtr;

typedef stdx::variant<
    SFBool,
    SFInt32, SFFloat, SFVec2f, SFVec3f, SFRotation, SFColor, SFTime, SFString, SFNode, SFImage,
    MFInt32, MFFloat, MFVec2f, MFVec3f, MFRotation, MFColor, MFTime, MFString, MFNode>
VRMLVariantField;

enum VRMLFieldTypeId {
    SFBOOL,
    SFINT32, SFFLOAT, SFVEC2F, SFVEC3F, SFROTATION, SFCOLOR, SFTIME, SFSTRING, SFNODE, SFIMAGE,
    MFINT32, MFFLOAT, MFVEC2F, MFVEC3F, MFROTATION, MFCOLOR, MFTIME, MFSTRING, MFNODE,
    UNKNOWN_VRML_FIELD_TYPE
};
    
typedef std::map <std::string, VRMLVariantField> VRMLProtoFieldMap;
typedef std::pair<std::string, VRMLVariantField> VRMLProtoFieldPair;

CNOID_EXPORT const char* labelOfVRMLfieldTypeId(const std::type_info& fieldType);

template<typename TValue> inline const char* labelOfVRMLfieldType() {
    return labelOfVRMLfieldTypeId(typeid(TValue));
}

//! VRML Proto definition
class CNOID_EXPORT VRMLProto : public VRMLNode
{
public:
    std::string protoName;
    VRMLProtoFieldMap fields;

    VRMLProto(const std::string& n);
    virtual const char* typeName() const override;

    inline VRMLVariantField* findField(const std::string& fieldName) {
        VRMLProtoFieldMap::iterator p = fields.find(fieldName);
        return (p != fields.end()) ? &p->second : 0;
    }

    inline VRMLVariantField& field(const std::string& fieldName){
        return fields[fieldName];
    }

    /*
      inline VRMLVariantField& addField(const std::string& fieldName, VRMLFieldTypeId typeId){
      VRMLVariantField* field = &(fields[fieldName]);
      field->setType(typeId);
      return field;
      }
    */

};
typedef ref_ptr<VRMLProto> VRMLProtoPtr;


//! VRML node which is instance of VRML Prototype
class CNOID_EXPORT VRMLProtoInstance : public VRMLNode
{
public:
    VRMLProtoPtr proto;
    VRMLProtoFieldMap fields;
    VRMLNodePtr actualNode;

    VRMLProtoInstance(VRMLProtoPtr proto0);
    virtual const char* typeName() const override;

    inline VRMLVariantField* findField(const std::string& fieldName) {
        VRMLProtoFieldMap::iterator p = fields.find(fieldName);
        return (p != fields.end()) ? &p->second : 0;
    }
};
typedef ref_ptr<VRMLProtoInstance> VRMLProtoInstancePtr;

/**
   The upper cast operation that supports the situation where the original pointer
   is VRMLProtoInstance and you want to get the actual node,
   the node replaced with the pre-defined node type written in the PROTO definition.
*/
template<class VRMLNodeType>
inline ref_ptr<VRMLNodeType> dynamic_node_cast(VRMLNodePtr node) {
    VRMLProtoInstancePtr protoInstance = dynamic_pointer_cast<VRMLProtoInstance>(node);
    if(protoInstance){
        return dynamic_pointer_cast<VRMLNodeType>(protoInstance->actualNode);
    } else {
        return dynamic_pointer_cast<VRMLNodeType>(node);
    }
}

#ifdef CNOID_BACKWARD_COMPATIBILITY
typedef VRMLNodeCategory VrmlNodeCategory;
typedef VRMLNode VrmlNode;
typedef VRMLNodePtr VrmlNodePtr;
typedef VRMLUnsupportedNode VrmlUnsupportedNode;
typedef VRMLUnsupportedNodePtr VrmlUnsupportedNodePtr;
typedef VRMLViewpoint VrmlViewpoint;
typedef VRMLViewpointPtr VrmlViewpointPtr;
typedef VRMLNavigationInfo VrmlNavigationInfo;
typedef VRMLNavigationInfoPtr VrmlNavigationInfoPtr;
typedef VRMLBackground VrmlBackground;
typedef VRMLBackgroundPtr VrmlBackgroundPtr;
typedef AbstractVRMLGroup AbstractVrmlGroup;
typedef AbstractVRMLGroupPtr AbstractVrmlGroupPtr;
typedef VRMLGroup VrmlGroup;
typedef VRMLGroupPtr VrmlGroupPtr;
typedef VRMLTransform VrmlTransform;
typedef VRMLTransformPtr VrmlTransformPtr;
typedef VRMLInline VrmlInline;
typedef VRMLInlinePtr VrmlInlinePtr;
typedef VRMLShape VrmlShape;
typedef VRMLShapePtr VrmlShapePtr;
typedef VRMLAppearance VrmlAppearance;
typedef VRMLAppearancePtr VrmlAppearancePtr;
typedef VRMLMaterial VrmlMaterial;
typedef VRMLMaterialPtr VrmlMaterialPtr;
typedef VRMLTexture VrmlTexture;
typedef VRMLTexturePtr VrmlTexturePtr;
typedef VRMLImageTexture VrmlImageTexture;
typedef VRMLImageTexturePtr VrmlImageTexturePtr;
typedef VRMLTextureTransform VrmlTextureTransform;
typedef VRMLTextureTransformPtr VrmlTextureTransformPtr;
typedef VRMLGeometry VrmlGeometry;
typedef VRMLGeometryPtr VrmlGeometryPtr;
typedef VRMLBox VrmlBox;
typedef VRMLBoxPtr VrmlBoxPtr;
typedef VRMLCone VrmlCone;
typedef VRMLConePtr VrmlConePtr;
typedef VRMLCylinder VrmlCylinder;
typedef VRMLCylinderPtr VrmlCylinderPtr;
typedef VRMLSphere VrmlSphere;
typedef VRMLSpherePtr VrmlSpherePtr;
typedef VRMLFontStyle VrmlFontStyle;
typedef VRMLFontStylePtr VrmlFontStylePtr;
typedef VRMLText VrmlText;
typedef VRMLTextPtr VrmlTextPtr;
typedef VRMLIndexedLineSet VrmlIndexedLineSet;
typedef VRMLIndexedLineSetPtr VrmlIndexedLineSetPtr;
typedef VRMLIndexedFaceSet VrmlIndexedFaceSet;
typedef VRMLIndexedFaceSetPtr VrmlIndexedFaceSetPtr;
typedef VRMLColor VrmlColor;
typedef VRMLColorPtr VrmlColorPtr;
typedef VRMLCoordinate VrmlCoordinate;
typedef VRMLCoordinatePtr VrmlCoordinatePtr;
typedef VRMLTextureCoordinate VrmlTextureCoordinate;
typedef VRMLTextureCoordinatePtr VrmlTextureCoordinatePtr;
typedef VRMLNormal VrmlNormal;
typedef VRMLNormalPtr VrmlNormalPtr;
typedef VRMLCylinderSensor VrmlCylinderSensor;
typedef VRMLCylinderSensorPtr VrmlCylinderSensorPtr;
typedef VRMLPointSet VrmlPointSet;
typedef VRMLPointSetPtr VrmlPointSetPtr;
typedef VRMLPixelTexture VrmlPixelTexture;
typedef VRMLPixelTexturePtr VrmlPixelTexturePtr;
typedef VRMLMovieTexture VrmlMovieTexture;
typedef VRMLMovieTexturePtr VrmlMovieTexturePtr;
typedef VRMLElevationGrid VrmlElevationGrid;
typedef VRMLElevationGridPtr VrmlElevationGridPtr;
typedef VRMLExtrusion VrmlExtrusion;
typedef VRMLExtrusionPtr VrmlExtrusionPtr;
typedef VRMLSwitch VrmlSwitch;
typedef VRMLSwitchPtr VrmlSwitchPtr;
typedef VRMLLOD VrmlLOD;
typedef VRMLLODPtr VrmlLODPtr;
typedef VRMLCollision VrmlCollision;
typedef VRMLCollisionPtr VrmlCollisionPtr;
typedef VRMLAnchor VrmlAnchor;
typedef VRMLAnchorPtr VrmlAnchorPtr;
typedef VRMLBillboard VrmlBillboard;
typedef VRMLBillboardPtr VrmlBillboardPtr;
typedef VRMLFog VrmlFog;
typedef VRMLFogPtr VrmlFogPtr;
typedef VRMLWorldInfo VrmlWorldInfo;
typedef VRMLWorldInfoPtr VrmlWorldInfoPtr;
typedef VRMLPointLight VrmlPointLight;
typedef VRMLPointLightPtr VrmlPointLightPtr;
typedef VRMLDirectionalLight VrmlDirectionalLight;
typedef VRMLDirectionalLightPtr VrmlDirectionalLightPtr;
typedef VRMLSpotLight VrmlSpotLight;
typedef VRMLSpotLightPtr VrmlSpotLightPtr;
typedef VRMLProto VrmlProto;
typedef VRMLProtoPtr VrmlProtoPtr;
typedef VRMLProtoInstance VrmlProtoInstance;
typedef VRMLProtoInstancePtr VrmlProtoInstancePtr;
typedef VRMLVariantField VrmlVariantField;
#endif

}

#endif
