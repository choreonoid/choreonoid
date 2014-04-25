/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_VRML_H_INCLUDED
#define CNOID_UTIL_VRML_H_INCLUDED

#include <map>
#include <string>
#include <bitset>
#include <boost/variant.hpp>
#include <boost/intrusive_ptr.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
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
typedef std::vector<SFVec2f, Eigen::aligned_allocator<SFVec2f> > MFVec2f;
typedef std::vector<SFVec2s> MFVec2s; // single-precision type used for texture coordinates
typedef std::vector<SFVec3f> MFVec3f;
typedef std::vector<SFVec3s> MFVec3s; // single-precision type used for vertices and normals
typedef std::vector<SFRotation> MFRotation;
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

inline void intrusive_ptr_add_ref(VRMLNode* obj);
inline void intrusive_ptr_release(VRMLNode* obj);

//! Abstract base class of all vrml nodes.
class CNOID_EXPORT VRMLNode
{
public:

    VRMLNode();
    virtual ~VRMLNode();

    std::string defName;

    bool isCategoryOf(VRMLNodeCategory category);

protected:
    std::bitset<NUM_VRML_NODE_CATEGORIES> categorySet;

private:
    int refCounter;

    friend void intrusive_ptr_add_ref(VRMLNode* obj);
    friend void intrusive_ptr_release(VRMLNode* obj);
};

inline void intrusive_ptr_add_ref(VRMLNode* obj){
    obj->refCounter++;
}
    
inline void intrusive_ptr_release(VRMLNode* obj){
    obj->refCounter--;
    if(obj->refCounter <= 0){
        delete obj;
    }
}

typedef boost::intrusive_ptr<VRMLNode> VRMLNodePtr;

typedef VRMLNodePtr SFNode;
typedef std::vector<SFNode> MFNode;


class CNOID_EXPORT  VRMLUnsupportedNode : public VRMLNode
{
public:
    VRMLUnsupportedNode(const std::string& nodeTypeName);
    std::string nodeTypeName;
};
typedef boost::intrusive_ptr<VRMLUnsupportedNode> VRMLUnsupportedNodePtr;


//! VRML Viewpoint node
class CNOID_EXPORT  VRMLViewpoint : public VRMLNode
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        
    VRMLViewpoint();

    SFRotation orientation;
    SFFloat fieldOfView;
    SFBool jump;
    SFVec3f position;
    SFString description;
};
typedef boost::intrusive_ptr<VRMLViewpoint> VRMLViewpointPtr;


//! VRML NavigationInfo node
class CNOID_EXPORT  VRMLNavigationInfo : public VRMLNode
{
public:
    VRMLNavigationInfo();

    MFFloat avatarSize;
    SFBool headlight;
    SFFloat speed;
    MFString type;
    SFFloat visibilityLimit;
};
typedef boost::intrusive_ptr<VRMLNavigationInfo> VRMLNavigationInfoPtr;


//! VRML Background node
class CNOID_EXPORT  VRMLBackground : public VRMLNode
{
public:
    VRMLBackground();
      
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
typedef boost::intrusive_ptr<VRMLBackground> VRMLBackgroundPtr;


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
typedef boost::intrusive_ptr<AbstractVRMLGroup> AbstractVRMLGroupPtr;
    
    
//! VRML Group node
class CNOID_EXPORT VRMLGroup : public AbstractVRMLGroup
{
public:
    VRMLGroup();

    virtual MFNode& getChildren();
    virtual int countChildren();
    virtual VRMLNode* getChild(int index);
    virtual void replaceChild(int childIndex, VRMLNode* childNode);

    SFVec3f bboxCenter;    
    SFVec3f bboxSize;  
    MFNode children;
};
typedef boost::intrusive_ptr<VRMLGroup> VRMLGroupPtr;


//! VRML Transform node
class CNOID_EXPORT  VRMLTransform : public VRMLGroup
{
public:
    VRMLTransform();

    Eigen::Affine3d toAffine3d();

    SFVec3f center;
    SFRotation rotation;
    SFVec3f scale;
    SFRotation scaleOrientation;
    SFVec3f translation;
};
typedef boost::intrusive_ptr<VRMLTransform> VRMLTransformPtr;

//! VRML Inline node
class CNOID_EXPORT  VRMLInline : public VRMLGroup
{
public:
    VRMLInline();
    MFString urls;
};
typedef boost::intrusive_ptr<VRMLInline> VRMLInlinePtr;


class CNOID_EXPORT  VRMLAnotherFormatFile : public VRMLNode
{
public:
    VRMLAnotherFormatFile();
    SFString url;
    //SFString extension;
};
typedef boost::intrusive_ptr<VRMLAnotherFormatFile> VRMLAnotherFormatFilePtr;


class VRMLAppearance;
typedef boost::intrusive_ptr<VRMLAppearance> VRMLAppearancePtr;

class VRMLGeometry;
typedef boost::intrusive_ptr<VRMLGeometry> VRMLGeometryPtr;


//! VRML Shape node
class CNOID_EXPORT  VRMLShape : public VRMLNode
{
public:
    VRMLShape();
    VRMLAppearancePtr appearance;
    SFNode geometry;
};
typedef boost::intrusive_ptr<VRMLShape> VRMLShapePtr;


class VRMLMaterial;
typedef boost::intrusive_ptr<VRMLMaterial> VRMLMaterialPtr;

class VRMLTexture;
typedef boost::intrusive_ptr<VRMLTexture> VRMLTexturePtr;

class VRMLTextureTransform;
typedef boost::intrusive_ptr<VRMLTextureTransform> VRMLTextureTransformPtr;

//! VRML Appearance node
class CNOID_EXPORT VRMLAppearance : public VRMLNode
{
public:
    VRMLAppearance();
      
    VRMLMaterialPtr material;
    VRMLTexturePtr texture;
    VRMLTextureTransformPtr textureTransform;
};


//! VRML Material node
class CNOID_EXPORT VRMLMaterial : public VRMLNode
{
public:
    VRMLMaterial();

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

    MFString url;
    SFBool   repeatS;
    SFBool   repeatT;
};
typedef boost::intrusive_ptr<VRMLImageTexture> VRMLImageTexturePtr;


//! VRML TextureTransform node
class CNOID_EXPORT VRMLTextureTransform : public VRMLNode
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        
    VRMLTextureTransform();

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
    SFVec3f size;
};
typedef boost::intrusive_ptr<VRMLBox> VRMLBoxPtr;


//! VRML Cone node
class CNOID_EXPORT VRMLCone : public VRMLGeometry
{
public:
    VRMLCone();

    SFBool bottom;
    SFFloat bottomRadius;
    SFFloat height;
    SFBool side;
};
typedef boost::intrusive_ptr<VRMLCone> VRMLConePtr;


//! VRML Cylinder node
class CNOID_EXPORT VRMLCylinder : public VRMLGeometry
{
public:
    VRMLCylinder();

    SFBool bottom;
    SFFloat height;
    SFFloat radius;
    SFBool side;
    SFBool top;
};
typedef boost::intrusive_ptr<VRMLCylinder> VRMLCylinderPtr;


//! VRML Sphere node
class CNOID_EXPORT VRMLSphere : public VRMLGeometry
{
public:
    VRMLSphere();
    SFFloat radius;
};
typedef boost::intrusive_ptr<VRMLSphere> VRMLSpherePtr;


//! VRML FontStyle node
class CNOID_EXPORT VRMLFontStyle : public VRMLNode
{
public:
    VRMLFontStyle();

    MFString family;       
    SFBool   horizontal;
    MFString justify;
    SFString language;
    SFBool   leftToRight;
    SFFloat  size;
    SFFloat  spacing;
    SFString style;
    SFBool   topToBottom;
};
typedef boost::intrusive_ptr<VRMLFontStyle> VRMLFontStylePtr;


//! VRML Text node
class CNOID_EXPORT VRMLText : public VRMLGeometry
{
public:
    VRMLText();

    MFString fstring;
    VRMLFontStylePtr fontStyle;
    MFFloat length;
    SFFloat maxExtent;
};
typedef boost::intrusive_ptr<VRMLText> VRMLTextPtr;


class VRMLColor;
typedef boost::intrusive_ptr<VRMLColor> VRMLColorPtr;

class VRMLCoordinate;
typedef boost::intrusive_ptr<VRMLCoordinate> VRMLCoordinatePtr;

//! VRML IndexedLineSet node
class CNOID_EXPORT VRMLIndexedLineSet : public VRMLGeometry
{
public: 
    VRMLIndexedLineSet();

    VRMLColorPtr color;
    VRMLCoordinatePtr coord;
    MFInt32 colorIndex;
    SFBool colorPerVertex;
    MFInt32 coordIndex;
};
typedef boost::intrusive_ptr<VRMLIndexedLineSet> VRMLIndexedLineSetPtr;


class VRMLNormal;
typedef boost::intrusive_ptr<VRMLNormal> VRMLNormalPtr;

class VRMLTextureCoordinate;
typedef boost::intrusive_ptr<VRMLTextureCoordinate> VRMLTextureCoordinatePtr;


//! VRML IndexedFaseSet node
class CNOID_EXPORT VRMLIndexedFaceSet : public VRMLIndexedLineSet
{
public:
    VRMLIndexedFaceSet();

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
typedef boost::intrusive_ptr<VRMLIndexedFaceSet> VRMLIndexedFaceSetPtr;


//! VRML Color node
class CNOID_EXPORT VRMLColor : public VRMLNode
{
public:
    VRMLColor();
      
    MFColor color;
};


//! VRML Coordinate node
class CNOID_EXPORT VRMLCoordinate : public VRMLNode
{
public:
    VRMLCoordinate();
    MFVec3s point;
};


//! VRML TextureCoordinate node
class CNOID_EXPORT VRMLTextureCoordinate : public VRMLNode
{
public:
    VRMLTextureCoordinate();
    MFVec2s point;
};


//! VRML Normal node
class CNOID_EXPORT VRMLNormal : public VRMLNode
{
public:
    VRMLNormal();
    MFVec3s vector;
};


//! VRML CylinderSensor node
class CNOID_EXPORT VRMLCylinderSensor : public VRMLNode
{
public:
    VRMLCylinderSensor();

    SFBool  autoOffset;
    SFFloat diskAngle;
    SFBool  enabled;
    SFFloat maxAngle;
    SFFloat minAngle;
    SFFloat offset;
};
typedef boost::intrusive_ptr<VRMLCylinderSensor> VRMLCylinderSensorPtr;


//! VRML PointSet node
class CNOID_EXPORT VRMLPointSet : public VRMLGeometry
{
public:
    VRMLPointSet();

    VRMLCoordinatePtr	coord;
    VRMLColorPtr		color;
};

typedef boost::intrusive_ptr<VRMLPointSet> VRMLPointSetPtr;


//! VRML PixelTexture node
class CNOID_EXPORT VRMLPixelTexture : public VRMLTexture
{
public:
    VRMLPixelTexture();

    SFImage			image;
    SFBool			repeatS;
    SFBool			repeatT;
};

typedef boost::intrusive_ptr<VRMLPixelTexture> VRMLPixelTexturePtr;


//! VRML MovieTexture node
class CNOID_EXPORT VRMLMovieTexture : public VRMLTexture
{
public:
    VRMLMovieTexture();

    MFString		url;
    SFBool			loop;
    SFFloat			speed;
    SFTime			startTime;
    SFTime			stopTime;
    SFBool			repeatS;
    SFBool			repeatT;
};

typedef boost::intrusive_ptr<VRMLMovieTexture> VRMLMovieTexturePtr;


//! VRML ElevationGrid node
class CNOID_EXPORT VRMLElevationGrid : public VRMLGeometry
{
public:
    VRMLElevationGrid();

    SFInt32			xDimension;
    SFInt32			zDimension;
    SFFloat			xSpacing;
    SFFloat			zSpacing;
    MFFloat			height;
    SFBool			ccw;
    SFBool			colorPerVertex;
    SFFloat			creaseAngle;
    SFBool			normalPerVertex;
    SFBool			solid;
    VRMLColorPtr	color;
    VRMLNormalPtr	normal;
    VRMLTextureCoordinatePtr	texCoord;
};

typedef boost::intrusive_ptr<VRMLElevationGrid> VRMLElevationGridPtr;


//! VRML Extrusion node
class CNOID_EXPORT VRMLExtrusion : public VRMLGeometry
{
public:
    VRMLExtrusion();

    MFVec2f			crossSection;
    MFVec3f			spine;
    MFVec2f			scale;
    MFRotation		orientation;
    SFBool			beginCap;
    SFBool			endCap;
    SFBool			solid;
    SFBool			ccw;
    SFBool			convex;
    SFFloat			creaseAngle;
};

typedef boost::intrusive_ptr<VRMLExtrusion> VRMLExtrusionPtr;


class CNOID_EXPORT VRMLSwitch : public AbstractVRMLGroup
{
public:
    VRMLSwitch();

    virtual MFNode& getChildren();
    virtual int countChildren();
    virtual VRMLNode* getChild(int index);
    virtual void replaceChild(int childIndex, VRMLNode* childNode);

    MFNode	choice;
    SFInt32	whichChoice;
};

typedef boost::intrusive_ptr<VRMLSwitch> VRMLSwitchPtr;


class CNOID_EXPORT VRMLLOD : public AbstractVRMLGroup
{
public:
    VRMLLOD();

    virtual MFNode& getChildren();
    virtual int countChildren();
    virtual VRMLNode* getChild(int index);
    virtual void replaceChild(int childIndex, VRMLNode* childNode);

    MFFloat range;
    SFVec3f center;
    MFNode  level;
};

typedef boost::intrusive_ptr<VRMLLOD> VRMLLODPtr;


class CNOID_EXPORT VRMLCollision : public VRMLGroup
{
public:
    VRMLCollision();
    SFBool collide;
    SFNode proxy;
};

typedef boost::intrusive_ptr<VRMLCollision> VRMLCollisionPtr;


class CNOID_EXPORT VRMLAnchor : public VRMLGroup
{
public:
    VRMLAnchor();
    SFString description;
    MFString parameter;
    MFString url;
};

typedef boost::intrusive_ptr<VRMLAnchor> VRMLAnchorPtr;


class CNOID_EXPORT VRMLBillboard : public VRMLGroup
{
public:
    VRMLBillboard();
    SFVec3f axisOfRotation;
};

typedef boost::intrusive_ptr<VRMLBillboard> VRMLBillboardPtr;


class CNOID_EXPORT VRMLFog : public VRMLNode
{
public:
    VRMLFog();
    SFColor  color;
    SFFloat  visibilityRange;
    SFString fogType;
};

typedef boost::intrusive_ptr<VRMLFog> VRMLFogPtr;


class CNOID_EXPORT  VRMLWorldInfo : public VRMLNode
{
public:
    VRMLWorldInfo();
    SFString title;
    MFString info;
};

typedef boost::intrusive_ptr<VRMLWorldInfo> VRMLWorldInfoPtr;


class CNOID_EXPORT VRMLLight : public VRMLNode
{
public:
    VRMLLight();
    SFBool  on;
    SFColor color;
    SFFloat intensity;
    SFFloat ambientIntensity;
};

typedef boost::intrusive_ptr<VRMLLight> VRMLLightPtr;


class CNOID_EXPORT VRMLPointLight : public VRMLLight
{
public:
    VRMLPointLight();
    SFVec3f location;
    SFFloat radius;
    SFVec3f attenuation;
};

typedef boost::intrusive_ptr<VRMLPointLight> VRMLPointLightPtr;


class CNOID_EXPORT VRMLDirectionalLight : public VRMLLight
{
public:
    VRMLDirectionalLight();
    SFVec3f direction;
};

typedef boost::intrusive_ptr<VRMLDirectionalLight> VRMLDirectionalLightPtr;


class CNOID_EXPORT VRMLSpotLight : public VRMLPointLight
{
public:
    VRMLSpotLight();
    SFVec3f direction;
    SFFloat beamWidth;
    SFFloat cutOffAngle;
};

typedef boost::intrusive_ptr<VRMLSpotLight> VRMLSpotLightPtr;

typedef boost::variant<SFBool,
                       SFInt32, SFFloat, SFVec2f, SFVec3f, SFRotation, SFColor, SFTime, SFString, SFNode, SFImage,
                       MFInt32, MFFloat, MFVec2f, MFVec3f, MFRotation, MFColor, MFTime, MFString, MFNode> VRMLVariantField;

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
typedef boost::intrusive_ptr<VRMLProto> VRMLProtoPtr;


//! VRML node which is instance of VRML Prototype
class CNOID_EXPORT VRMLProtoInstance : public VRMLNode
{
public:
    VRMLProtoPtr proto;
    VRMLProtoFieldMap fields;
    VRMLNodePtr actualNode;

    VRMLProtoInstance(VRMLProtoPtr proto0);

    inline VRMLVariantField* findField(const std::string& fieldName) {
        VRMLProtoFieldMap::iterator p = fields.find(fieldName);
        return (p != fields.end()) ? &p->second : 0;
    }
};
typedef boost::intrusive_ptr<VRMLProtoInstance> VRMLProtoInstancePtr;

/**
   The upper cast operation that supports the situation where the original pointer
   is VRMLProtoInstance and you want to get the actual node,
   the node replaced with the pre-defined node type written in the PROTO definition.
*/
template<class VRMLNodeType>
inline boost::intrusive_ptr<VRMLNodeType> dynamic_node_cast(VRMLNodePtr node) {
    VRMLProtoInstancePtr protoInstance = boost::dynamic_pointer_cast<VRMLProtoInstance>(node);
    if(protoInstance){
        return boost::dynamic_pointer_cast<VRMLNodeType>(protoInstance->actualNode);
    } else {
        return boost::dynamic_pointer_cast<VRMLNodeType>(node);
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

};

#endif
