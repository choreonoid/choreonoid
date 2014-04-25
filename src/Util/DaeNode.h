/*!
 * @brief  Collada file is read only once like SAX in IrrXML. 
 *         It handled temporarily all the information at that time, it then creates and returns an SgObject.
 * @author Hisashi Ikari 
 * @file
 */
#ifndef CNOID_UTIL_DAE_NODE_H_INCLUDED
#define CNOID_UTIL_DAE_NODE_H_INCLUDED

#include "Referenced.h"
#include <cnoid/EigenTypes>
#include <boost/shared_ptr.hpp>

namespace cnoid {

// Introduction. SceneGraph switches the float and double in SgFloat. And the type of eigen also switched. 
// But, Link does not receive that type. To that end, I will use the type double(Maximum accuracy) in DaeNode.

typedef std::vector<double>                       DaeVectorXArray;
typedef boost::shared_ptr<DaeVectorXArray>        DaeVectorXArrayPtr;
typedef std::map<std::string, DaeVectorXArrayPtr> DaeVectorMap;

/*!
 * @brief This is the base class of the dae parser. (In order to share a pointer for all bjects)
 */
class Dae : public Referenced
{
public:
    Dae() : id("") {}

public:
    // All nodes must have a "UNIQUE" id always.
    std::string id;
}; 


/*!
 * @brief It will hold the information of SgTexture for 3D shape model.
 */
class DaeTexture : public Dae
{
public:
    DaeTexture(std::string targetName) : fileName_(targetName) {}
    std::string fileName() { return fileName_; }

protected:
    std::string fileName_;

}; 
typedef ref_ptr<DaeTexture>                  DaeTexturePtr;
typedef std::map<std::string, DaeTexturePtr> DaeTextures;
typedef std::vector<Vector3>                Vector3s;

/*!
 * @brief It will hold the information of SgTransform for 3D shape model and kinematics model.
 */
class DaeTransform : public Dae
{
public:
    DaeTransform() : translate(Vector3(0.0, 0.0, 0.0)),
                     scale    (Vector3(1.0, 1.0, 1.0)),
                     center   (Vector3(0.0, 0.0, 0.0)),
                     rotate   (AngleAxis(0.0, Vector3(0, 0, 0))),
                     affine   (NULL),
                     matrix   (NULL) {}
    ~DaeTransform()
        {
            // affine is the value using the rotation matrix from the parent node. 
            if (affine) delete affine;
            if (matrix) delete matrix;
        }

public:
    // Not compatible with the SceneGraph to Link. Therefore, I use the double type.
    Vector3    translate;
    Vector3s   translates;
    Vector3    scale;
    Vector3    center;
    AngleAxis  rotate;
    Affine3*   affine;
    Matrix4*   matrix;
#ifdef _INVALID_VERSION
    Affine3d    parents;
#endif
    Affine3d*   retention;

};
typedef ref_ptr<DaeTransform> DaeTransformPtr;


/*!
 * @brief It will hold the information of SgMaterial for 3D shape model.
 */
class DaeEffect : public Dae
{
public:
    DaeEffect() : refImageId  (""),
                  effectId    (""),
                  emission    (Vector3(0.0, 0.0, 0.0)),
                  ambient     (0.0),
                  diffuse     (Vector3(1.0, 1.0, 1.0)),
                  specular    (Vector3(0.0, 0.0, 0.0)),
                  shininess   (0.0),
                  transparency(0.0) {}
public:
    // !!! important !!!
    // I held in the variable of the prefix ref is the url attribute and the target attribute of all.
    std::string refImageId;        

    std::string effectId;
    Vector3    emission;
    double      ambient; 
    Vector3    diffuse;
    Vector3    specular; 
    double      shininess;
    double      transparency;
};
typedef ref_ptr<DaeEffect>                  DaeEffectPtr;
typedef std::map<std::string, DaeEffectPtr> DaeEffects;


/*!
 * @brief It will hold the information of SgMaterial for 3D shape model.
 */
class DaeMaterial : public Dae
{
public:
    DaeMaterial() : materialId (""),
                    refEffectId("") {}
public:
    std::string materialId;
    std::string refEffectId;
};
typedef ref_ptr<DaeMaterial>                  DaeMaterialPtr;
typedef std::map<std::string, DaeMaterialPtr> DaeMaterials;
typedef std::map<std::string, std::string>    DaeMaterialRef;


/*!
 * @brief It will hold the information of SgGeometry for 3D shape model.
 */
class DaeMesh : public Dae
{
public:
    DaeMesh() : refVerticesId (""),
                refNormalsId  (""),
                refColorId    (""),
                refTexcoordId (""),
                refMaterialId ("")
        {
            // mesh is equivalent to polylist and lines and triangles of collada.
            // This means that I will have the index and the number of vertices. 
            // I also hold such as vertex coordinates of the source tag further. (For ease of processing)
            vcount    = DaeVectorXArrayPtr(new DaeVectorXArray);
            vertices  = DaeVectorXArrayPtr(new DaeVectorXArray);
            normals   = DaeVectorXArrayPtr(new DaeVectorXArray);
            colors    = DaeVectorXArrayPtr(new DaeVectorXArray);
            texcoords = DaeVectorXArrayPtr(new DaeVectorXArray);

            verticesIndexes  = DaeVectorXArrayPtr(new DaeVectorXArray);
            normalsIndexes   = DaeVectorXArrayPtr(new DaeVectorXArray);
            colorsIndexes    = DaeVectorXArrayPtr(new DaeVectorXArray);
            texcoordsIndexes = DaeVectorXArrayPtr(new DaeVectorXArray);
        }                    

public:
    std::string refVerticesId;
    std::string refNormalsId;
    std::string refColorId;
    std::string refTexcoordId;
    std::string refMaterialId;

    DaeVectorXArrayPtr vcount;

    DaeVectorXArrayPtr vertices;
    DaeVectorXArrayPtr normals;
    DaeVectorXArrayPtr colors;
    DaeVectorXArrayPtr texcoords;

    DaeVectorXArrayPtr verticesIndexes;
    DaeVectorXArrayPtr normalsIndexes;     
    DaeVectorXArrayPtr colorsIndexes;     
    DaeVectorXArrayPtr texcoordsIndexes;     
};
typedef ref_ptr<DaeMesh>        DaeMeshPtr;
typedef std::vector<DaeMeshPtr> DaeMeshes;


/*!
 * @brief It will hold the information of SgGeometry for 3D shape model.
 */
class DaeGeometry : public Dae
{
public:
    DaeGeometry() : geometryId   (""),
                    refMaterialId("") {}
public:
    std::string geometryId;
    std::string refMaterialId;

    DaeMeshes meshes;
};
typedef ref_ptr<DaeGeometry>                  DaeGeometryPtr;
typedef std::map<std::string, DaeGeometryPtr> DaeGeometries;

class DaeNode; 
typedef ref_ptr<DaeNode>                                  DaeNodePtr;
typedef std::map<std::string, DaeNodePtr>                 DaeNodes;
typedef std::vector< std::pair<std::string, DaeNodePtr> > DaeNodeStack;


/*!
 * @brief This is the base class of the dae parser.
 */
class DaeNode : public Dae
{
public:
    DaeNode() :   refNodeId(""),
                  refName  (""),
                  parent   (NULL),
                  type     (-1) {}
public:
    std::string   name;
    std::string   refNodeId;
    std::string   refName;

    DaeNodePtr    parent;
    DaeTransform  transform; 
    DaeGeometries geometries;
    DaeNodeStack  stack;
    DaeNodes      children;
    int           type;

    DaeNodePtr clone();

protected:
    void addChild(DaeNodePtr node);
};


/*!
 * @brief It will hold the information of primitive figure for physics.
 */
class DaeShape : public DaeNode
{
public:
    DaeShape() { format(); }

    // If the graphic primitive, if specified in the rigid body, I will use that figure.
    class Box {
    public:
        bool      presence;
        Vector3 halfExtents;
    };
    class Sphere {
    public:
        bool      presence;
        double    radius;
    };
    class Cylinder {
    public:
        bool      presence;
        double    height;
        double    radius;
    };
    class Cone {
    public:
        bool      presence;
        double    height;
        Vector2 radius1;
        Vector2 radius2;
    };
    class Polygon {
    public:
        bool presence;
        DaeGeometryPtr geometry;
    };

    bool isPresence() {
        // Graphic primitive is assumed to be only one.
        return (box.presence || sphere.presence || cylinder.presence || cone.presence || polygon.presence);
    }

    void setBox(Vector3 ext) {
        format();
        box.presence      = true;
        box.halfExtents   = ext;
    }
    void setSphere(double radius) {
        format();
        sphere.presence   = true;
        sphere.radius     = radius;
    }
    void setCylinder(double height, double radius) {
        format();
        cylinder.presence = true;
        cylinder.height   = height;
        cylinder.radius   = radius;
    }
    void setCone(double height, const Vector2& radius1, const Vector2& radius2) {
        format();
        cone.presence     = true;
        cone.height       = height;
        cone.radius1      = radius1;
        cone.radius2      = radius2;
    }
    void setPolygon(DaeGeometryPtr geometry) {
        format();
        polygon.presence  = true;
        polygon.geometry  = geometry;    
    }
    void format() {
        box.presence      = false;
        sphere.presence   = false;
        cylinder.presence = false;
        cone.presence     = false;
        polygon.presence  = false;
        refMaterialId     = "";
    }

public:
    Box      box;
    Sphere   sphere;
    Cylinder cylinder;
    Cone     cone; 
    Polygon  polygon;

    std::string refMaterialId;
};
typedef ref_ptr<DaeShape>        DaeShapePtr;
typedef std::vector<DaeShapePtr> DaeShapes;


/*!
 * @brief It will hold the information of primitive figure for physics.
 */
class DaeMassFrame : public DaeNode
{
    // This is to just keep rottion and angles.
}; 
typedef ref_ptr<DaeMassFrame> DaeMassFramePtr;


/*!
 * @brief It will hold the information of primitive figure for physics.
 */
class DaeRigid : public DaeNode
{
public:
    DaeRigid() : mass(0.0),
                 inertia(Vector3(0.0, 0.0, 0.0)) {}
public:
    double   mass;
    Vector3 inertia; 

    DaeShapes       shapes;
    DaeMassFramePtr massFrame;
};
typedef ref_ptr<DaeRigid>                  DaeRigidPtr;
typedef std::map<std::string, DaeRigidPtr> DaeRigids;
typedef std::map<std::string, std::string> DaeRigidRelations;


/*!
 * @brief It will hold the information for Kinematics.
 */
class DaeRevolute : public Dae { // this is just the child of joint-value.
public:
    DaeRevolute() : axis(Vector3(0.0, 0.0, 0.0)),
                    limitMin(0),
                    limitMax(0) {}
public:
    Vector3 axis;
    double   limitMin;
    double   limitMax;
};

typedef ref_ptr<DaeRevolute>                  DaeRevolutePtr;
typedef std::map<std::string, DaeRevolutePtr> DaeRevolutes;
typedef std::vector<DaeRevolutePtr>           DaeRevoluteChildren;

typedef std::vector<std::string> DaeLinkChildren;
typedef std::vector<std::string> DaeJointChildren;


/*!
 * @brief It will hold the information for Kinematics.
 */
class DaeActuator : public DaeNode
{
public:
    DaeActuator() : assignedPowerRating(1.0),
                    maxSpeed           (0.0),
                    noLoadSpeed        (0.0),
                    nominalTorque      (0.0),
                    nominalVoltage     (0.0),
                    rotorInertia       (0.0),
                    speedConstant      (0.0),
                    speedTorqueGradient(0.0), 
                    startingCurrent    (0.0),
                    terminalResistance (0.0),
                    torqueConstant     (1.0) {}
public:
    double assignedPowerRating;
    double maxSpeed;
    double noLoadSpeed;
    double nominalTorque;
    double nominalVoltage;
    double rotorInertia;
    double speedConstant;
    double speedTorqueGradient;
    double startingCurrent;
    double terminalResistance;
    double torqueConstant;
};
typedef ref_ptr<DaeActuator>                  DaeActuatorPtr;
typedef std::map<std::string, DaeActuatorPtr> DaeActuators;
typedef std::map<std::string, std::string>    DaeActuatorRelations;


/*!
 * @brief It will hold the information for Kinematics.
 */
class DaeSensor : public DaeNode
{
public:
    DaeSensor() : type           (""),
                  focalLength    (0.0),
                  intrinsic      (VectorXd::Zero(6)),
                  imageDimensions(VectorXd::Zero(3)),
                  measurementTime(0.0) {}
public:
    std::string type;
    double      focalLength;
    VectorXd    intrinsic;
    Vector3    imageDimensions;
    double      measurementTime; 
};
typedef ref_ptr<DaeSensor>                                 DaeSensorPtr;
typedef std::map<std::string, DaeSensorPtr>                DaeSensors;
typedef std::vector<DaeSensor*>                            DaeResultSensors;
typedef boost::shared_ptr<DaeResultSensors>                DaeResultSensorsPtr;
typedef std::vector< std::pair<std::string, std::string> > DaeSensorRelations;


/*!
 * @brief It will hold the information for Kinematics.
 */
class DaeAttachActuator : public DaeNode
{
public:
    DaeAttachActuator() : instanceActuator(""),
                          bindActuator    ("") {}
public:
    std::string instanceActuator;
    std::string bindActuator; 
}; 
typedef ref_ptr<DaeAttachActuator> DaeAttachActuatorPtr;


/*!
 * @brief It will hold the information for Kinematics.
 */
class DaeAttachSensor : public DaeNode
{
public:
    DaeAttachSensor() : instanceSensor(""),
                        frameOrigin   ("") {}
public:
    std::string instanceSensor;
    std::string frameOrigin; 
}; 
typedef ref_ptr<DaeAttachSensor> DaeAttachSensorPtr;


/*!
 * @brief It will hold the information for Kinematics.
 */
class DaeLink : public DaeNode // This is a not Dae, this is a NODE value in collada.
{
public:
    DaeLink() : name("") {}
public:
    std::string name;

    // transform different with the "LINK" and "NODE" is given in collada.
    // To do this, management information as them differently.
    DaeTransform transform;
    
    // This is same as attachment_full-tag. That values are joint-id.
    DaeJointChildren children;  
};
typedef ref_ptr<DaeLink>                  DaeLinkPtr;
typedef std::map<std::string, DaeLinkPtr> DaeLinks;


/*!
 * @brief It will hold the information for Kinematics.
 */
class DaeJoint : public DaeNode // This is not a Dae, this is a NODE value in collada.
{
public:
    DaeJoint() : name("")
        {
            transform = new DaeTransform; 
        }
    
public:
    // The id value contains the name of the parent model. (ex. kinamaticmodel/thisid)
    std::string name;

    // transform different with the "JOINT" and "NODE" is given in collada.
    // To do this, management information as them differently. 
    DaeTransformPtr     transform;         
    DaeRevoluteChildren revolutes;
    DaeJointChildren    children;
}; 
typedef ref_ptr<DaeJoint>                  DaeJointPtr;
typedef std::map<std::string, DaeJointPtr> DaeJoints;


}; //end of namespace

#endif
