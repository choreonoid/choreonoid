/*!
 * @brief  Collada file is read only once like SAX in IrrXML.
 It handled temporarily all the information at that time, it then creates and returns an SgObject.
 * @author Hisashi Ikari 
 * @file
 */
#include <iostream>
#include <map>
#include <vector>
#include <string>
#include <float.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <cnoid/MeshGenerator>
#include <cnoid/MeshNormalGenerator>
#include <cnoid/MeshGenerator>
#include <cnoid/SceneGraph>
#include <cnoid/ValueTree>

#include <irrXML.h>

#include "SceneGraph.h"
#include "PolygonMeshTriangulator.h"
#include "Exception.h"
#include "DaeParser.h"
#include "DaeNode.h"
#include "FileUtil.h"

using namespace std;
using boost::lexical_cast;
using boost::format;
using namespace boost::algorithm;
using namespace boost::uuids;
using namespace irr::io;

namespace cnoid {

typedef pair<string, DaeGeometryPtr> GEOMETRY_PAIR;
typedef pair<string, DaeNodePtr>     NODES_PAIR;

typedef std::vector<std::string>           DaeOrder;
typedef std::map<std::string, int>         DaeStrides;
typedef std::map<std::string, std::string> DaeVerticesRef;

/*!
 * @brief Enumeration type of "TAG" used in Collada.
 */
enum {
    EXT_NODE,
    EXT_UNIT,
    EXT_TRANSLATE,
    EXT_ROTATE,
    EXT_SCALE,
    EXT_MATRIX,
    EXT_INSTANCE_NODE,
    EXT_INSTANCE_GEOMETRY,
    EXT_INSTANCE_MATERIAL,
    EXT_INSTANCE_EFFECT,
    EXT_GEOMETRY,
    EXT_VERTICES,
    EXT_SOURCE,
    EXT_FLOAT_ARRAY,
    EXT_ACCESSOR,
    EXT_INPUT,
    EXT_TRIANGLES,
    EXT_LINES,
    EXT_P,
    EXT_MATERIAL,
    EXT_EFFECT,
    EXT_EMISSION,
    EXT_AMBIENT,
    EXT_DIFFUSE,
    EXT_SPECULAR,
    EXT_SHININESS,
    EXT_TRANSPARENCY,
    EXT_INIT_FORM,
    EXT_IMAGE,
    EXT_POLYLIST,
    EXT_VCOUNT,

    EXT_BOX,
    EXT_SPHERE,
    EXT_CYLINDER,
    EXT_TAPERED_CYLINDER,
    EXT_KINEMATICS_MODEL, 
    EXT_LINK, 
    EXT_ATTACHMENT_FULL, 
    EXT_JOINT, 
    EXT_REVOLUTE, 
    EXT_PRISMATIC,
    EXT_AXIS, 
    EXT_MIN, 
    EXT_MAX,
    EXT_RIGID_BODY,
    EXT_SHAPE,
    EXT_INSTANCE_RIGID_BODY, 
    EXT_INSTANCE_PHYSICS_MODEL,
    EXT_MASS,
    EXT_MASS_FRAME,
    EXT_INERTIA,

    EXT_ACTUATOR,
    EXT_ASSIGNED_POWER_RATING,
    EXT_MAX_SPEED,
    EXT_NO_LOAD_SPEED,
    EXT_NOMINAL_TORQUE,
    EXT_NOMINAL_VOLTAGE,
    EXT_ROTOR_INERTIA,
    EXT_SPEED_CONSTANT,
    EXT_SPEED_TORQUE_GRADIENT,
    EXT_STARTING_CURRENT,
    EXT_TERMINAL_RESISTANCE,
    EXT_TORQUE_CONSTANT,
    EXT_INSTANCE_ACTUATOR,
    EXT_BIND_ACTUATOR,
    EXT_INSTANCE_SENSOR,
    EXT_FRAME_ORIGIN,

    EXT_SENSOR,
    EXT_FOCAL_LENGTH,
    EXT_INTRINSIC,
    EXT_IMAGE_DIMENSIONS,
    EXT_MEASUREMENT_TIME,
    
    EXT_EXTRA,
};


/*!
 * @brief These indexes are defined in polylist tag and triangles.
 */
enum {
    EXT_POSITION,
    EXT_VERTEX,
    EXT_NORMAL,
    EXT_COLOR,
    EXT_TEXCOORD,
};


/*!
 * @brief An enumeration type of the "BASE" node in the Collada.
 *        Information("CONCRETE or LIBRARY") of the entity has been stored in these nodes.
 */
enum {
    EXT_VISUAL_SCENES,
    EXT_GEOMETRIES,
    EXT_MATERIALS,
    EXT_EFFECTS,
    EXT_NODES,
    EXT_IMAGES,
    EXT_PHYSICS_MODELS,
    EXT_KINEMATICS_MODELS,
    EXT_PHYSICS_SCENES,
};


/*!
 * @brief There are two types of joint and node to node of Collada.
 *        It will remain at that type when reading the node tags.
 */
enum {
    EXT_NODE_NODE,
    EXT_NODE_JOINT
};


/*!
 * @brief Perform the processing for each node as sax.
 */
class DaeParserImpl : public Dae
{
    friend class DaeParser;
    friend class BodyLoaderImpl;
public:
    DaeParserImpl(ostream* os) : rootLinkName    (""),
                                 refMaterialId   (""),
                                 refEffectId     (""),
                                 refGeometryId   (""),
                                 refVerticesId   (""),
                                 refNormalsId    (""),
                                 refPositionId   (""),
                                 refSourceId     (""),
                                 refImageId      (""),
                                 refKinematicsId (""),
                                 refRevoluteId   (""),
                                 refRigidBodyId  (""),
                                 offsetVertex    (-1),
                                 offsetNormal    (-1),
                                 offsetColor     (-1),
                                 offsetTexcoord  (-1),
                                 offsetMaximum   (-1),
                                 accessible      (-1),
                                 figure          (-1),
                                 unit            ( 1.0),
                                 inlineParse     (false),
                                 path            (""),
                                 befFile         (""),
                                 library         (new DaeNode),
                                 instance        (new DaeNode),
                                 rootNodePresence(false),
                                 rootLinkPresence(false),
                                 reader          (NULL),
                                 os              (os) {}
public:
    /*!
     * @brief Create SceneGraph from file. This is the process of Inline only.
     */
    SgGroup* createScene(const string& fileName);

    /*!
     * @brief I will parse the file of Collada. And I get a node in the following methods.
     */
    void parse(const string& fileName, const bool inlineParse);

    DaeNode*          findNode       (const string& nodeName);
    DaeNode*          findLinkByJoint(const string& jointName); 
    DaeNode*          findJointByLink(const string& linkName); 
    DaeNode*          findRigidByLink(const string& linkName);
    DaeNode*          findActuator   (const string& jointId);
    DaeResultSensors* findSensor     (const string& linkId);

protected:
    int  symbolNode    (const string& elementName);
    int  symbolSemantic(const string& elementName);

    void split (string& value, double* point, int max, double init);
    void rotate(string& value, DaeTransform& transform);
    void array (string& value, DaeVectorXArrayPtr array);
    void index (string& value, DaeVectorXArrayPtr index, int offset, int offsetMaximum);
    void matrix(string& value, Matrix4d &matrix);

    void file  (const string& value);
    long line  ();

    bool checkConcreteNode  (DaeNodePtr node);
    bool checkNodeStack     (DaeNodePtr node);
    bool checkOrder         (DaeOrder& order, int type);
    bool checkAccessibleNode(string& name);
    bool checkSafetyNode    ();
    void createNodeStack    (DaeNodePtr node, DaeNodeStack& stack, bool omit);
    void createStructure    ();

    SgGroup* convert();
    void createEmptyMaterial   (DaeGeometryPtr geometry);
    void createLists           (DaeNodePtr node, bool omit);
    void createVertexAndNormal (int type, string& source, int offset);
    int  createStride          (string& verticesId);
    bool createVcount          (DaeVectorXArrayPtr vcount);
    void createMesh            (int line);
    void createActuatorRelation(DaeAttachActuatorPtr attach);
    void createSensorRelation  (DaeAttachSensorPtr   attach);

    DaeEffectPtr findOrCreateEffect(string& refEffectId);
    DaeEffectPtr moveToColor       (string& refEffectId);
    DaeEffectPtr moveToFloat       (string& refEffectId);
    DaeEffectPtr findEffect        (string& refMaerialId);
    DaeRigidPtr  findRigidBody     (string& refNodeId);   

    DaeNodePtr readNode                      (DaeNodePtr node);
    DaeNodePtr readUnit                      (DaeNodePtr node);
    DaeNodePtr readTranslate                 (DaeNodePtr node);
    DaeNodePtr readRotate                    (DaeNodePtr node);
    DaeNodePtr readScale                     (DaeNodePtr node);
    DaeNodePtr readMatrix                    (DaeNodePtr node);
    DaeNodePtr readInstanceNode              (DaeNodePtr node);
    DaeNodePtr readInstanceMaterial          (DaeNodePtr node);
    DaeNodePtr readInstanceGeometry          (DaeNodePtr node);
    DaeNodePtr readInstanceGeometryForPhysics(DaeNodePtr node);
    DaeNodePtr readInstanceGeometryForNodes  (DaeNodePtr node);
    DaeNodePtr readInstanceEffect            (DaeNodePtr node);
    DaeNodePtr readGeometry                  (DaeNodePtr node);
    DaeNodePtr readVertices                  (DaeNodePtr node);
    DaeNodePtr readSource                    (DaeNodePtr node);
    DaeNodePtr readFloatArray                (DaeNodePtr node);
    DaeNodePtr readAccessor                  (DaeNodePtr node);
    DaeNodePtr readInput                     (DaeNodePtr node);
    DaeNodePtr readTriangles                 (DaeNodePtr node);
    DaeNodePtr readLines                     (DaeNodePtr node);
    DaeNodePtr readP                         (DaeNodePtr node);
    DaeNodePtr readMaterial                  (DaeNodePtr node);
    DaeNodePtr readEffect                    (DaeNodePtr node);
    DaeNodePtr readEmission                  (DaeNodePtr node);
    DaeNodePtr readAmbient                   (DaeNodePtr node);
    DaeNodePtr readDiffuse                   (DaeNodePtr node);
    DaeNodePtr readSpecular                  (DaeNodePtr node);
    DaeNodePtr readShininess                 (DaeNodePtr node);
    DaeNodePtr readTransparency              (DaeNodePtr node);
    DaeNodePtr readInitFrom                  (DaeNodePtr node);
    DaeNodePtr readImage                     (DaeNodePtr node);
    DaeNodePtr readPolylist                  (DaeNodePtr node);
    DaeNodePtr readVcount                    (DaeNodePtr node);

    DaeNodePtr readBox                       (DaeNodePtr node);
    DaeNodePtr readSphere                    (DaeNodePtr node);
    DaeNodePtr readCylinder                  (DaeNodePtr node);
    DaeNodePtr readTaperedCylinder           (DaeNodePtr node);
    DaeNodePtr readKinematicsModel           (DaeNodePtr node);
    DaeNodePtr readLink                      (DaeNodePtr node);
    DaeNodePtr readAttachmentFull            (DaeNodePtr node);
    DaeNodePtr readJoint                     (DaeNodePtr node);
    DaeNodePtr readRevolute                  (DaeNodePtr node);
    DaeNodePtr readPrismatic                 (DaeNodePtr node);
    DaeNodePtr readAxis                      (DaeNodePtr node);
    DaeNodePtr readMin                       (DaeNodePtr node);
    DaeNodePtr readMax                       (DaeNodePtr node);
    DaeNodePtr readRigidBody                 (DaeNodePtr node);
    DaeNodePtr readShape                     (DaeNodePtr node);
    DaeNodePtr readInstanceRigidBody         (DaeNodePtr node);
    DaeNodePtr readInstancePhysicsModel      (DaeNodePtr node);
    DaeNodePtr readMass                      (DaeNodePtr node);
    DaeNodePtr readMassFrame                 (DaeNodePtr node);
    DaeNodePtr readInertia                   (DaeNodePtr node);
    
    DaeNodePtr readActuator                  (DaeNodePtr node);
    DaeNodePtr readAssignedPowerRating       (DaeNodePtr node);
    DaeNodePtr readMaxSpeeed                 (DaeNodePtr node);
    DaeNodePtr readNoLoadSpeed               (DaeNodePtr node);
    DaeNodePtr readNominalTorque             (DaeNodePtr node);
    DaeNodePtr readNominalVoltage            (DaeNodePtr node);
    DaeNodePtr readRotorInertia              (DaeNodePtr node);
    DaeNodePtr readSpeedConstant             (DaeNodePtr node);
    DaeNodePtr readSpeedTorqueGradient       (DaeNodePtr node);
    DaeNodePtr readStartingCurrent           (DaeNodePtr node);
    DaeNodePtr readTerminalResistance        (DaeNodePtr node);
    DaeNodePtr readTorqueConstant            (DaeNodePtr node);
    DaeNodePtr readInstanceActuator          (DaeNodePtr node);
    DaeNodePtr readBindActuator              (DaeNodePtr node);       
    DaeNodePtr readInstanceSensor            (DaeNodePtr node);
    DaeNodePtr readFrameOrigin               (DaeNodePtr node);

    DaeNodePtr readSensor                    (DaeNodePtr node);
    DaeNodePtr readFocalLength               (DaeNodePtr node);
    DaeNodePtr readIntrinsic                 (DaeNodePtr node);
    DaeNodePtr readImageDimensions           (DaeNodePtr node);
    DaeNodePtr readMeasurementTime           (DaeNodePtr node);
 
    DaeNodePtr readExtra                     (DaeNodePtr node);
    DaeNodePtr endNode                       (DaeNodePtr node);

    void setNode     (DaeNodePtr extNode, SgGroup* sg, bool createTransform);
    void setGeometry (DaeGeometryPtr extGeometry, DaeNodePtr extNode, SgGroup* sg);
    void setMesh     (DaeGeometryPtr ext, DaeMeshPtr extMesh, SgMeshBase* sg, bool polygon);

    void setVertices (DaeGeometryPtr ext, DaeMeshPtr extMesh, SgMeshBase* sg);
    void setNormals  (DaeGeometryPtr ext, DaeMeshPtr extMesh, SgMeshBase* sg);
    void setTexcoord (DaeGeometryPtr ext, DaeMeshPtr extMesh, SgMeshBase* sg);
    void setColors   (DaeGeometryPtr ext, DaeMeshPtr extMesh, SgMeshBase* sg);

    void setVIndexes (DaeGeometryPtr ext, DaeMeshPtr extMesh, SgMeshBase* sg, bool polygon);
    void setXIndexes (DaeGeometryPtr ext, DaeMeshPtr extMesh, string id, string text, int stride,
                      DaeVectorXArrayPtr  extArray,  SgIndexArray& sgArray, bool polygon);

    void setRigid    (DaeNodePtr extNode, SgGroup* sgGroup, SgGroup* sgTransParent);
    void setTexture  (string meshImageId, SgTexture*  sg);
    void setMaterial (DaeEffectPtr extEffect, SgMaterial* sg);
    void setTransform(DaeNodePtr ext, SgGroup** sgParent, SgGroup** sgChild);

    void setActuator (DaeNodePtr node, double& value, string message);

protected:
    void throwException(int line, const std::string& messsage);
    void readTaperedCylinderConcrete(std::string& name, std::string& value, double& height, double radius1[2], double radius2[2]);

protected:
    string rootLinkName;
    string refMaterialId; // Keep the processing id at now.
    string refEffectId;
    string refGeometryId;
    string refVerticesId;
    string refNormalsId;
    string refColorId;
    string refTexcoordId;
    string refPositionId;
    string refSourceId;
    string refImageId;
    string refKinematicsId;
    string refRevoluteId;
    string refRigidBodyId;

    int    offsetVertex;
    int    offsetNormal;
    int    offsetColor;
    int    offsetTexcoord;
    int    offsetMaximum;
    int    accessible;

    int    figure;      // ex. triangles, polylist
    double unit;        // It will determine the coefficients of the coordinates.
    int    nodeType;    // ex. NODE, JOINT.
    bool   inlineParse; //
 
    string path;
    string befFile;

    DaeMeshPtr refMesh;
    DaeNodePtr library;
    DaeNodePtr instance;
        
    DaeNodePtr rootNode;
    DaeLinkPtr rootLink;
    bool rootNodePresence;
    bool rootLinkPresence;
 
    irr::io::IrrXMLReader* reader;
    ostream*               os;       // for message view
    DaeNodeStack           stack;    // It will keep the node up to the node
    DaeNodeStack           lists;    // It will keep the connection of "ALL" nodes.

    DaeJointPtr            befJoint;
    DaeRevolutePtr         befRevolute;
    DaeRigidPtr            befRigid;

    // Container of the parser are managed by entities of "ALL INFORMATION"
    // Their value(entity...Ext*) has only the pointer information
    DaeNodes               nodes;
    DaeNodes               nodeNames;
    DaeGeometries          geometries;
    DaeStrides             strides;
    DaeVerticesRef         verticesRef;       
    DaeMaterialRef         materialRef;
 
    DaeLinks               links;
    DaeLinks               linkNames;
    DaeJoints              joints;

    DaeMaterials           materials;
    DaeEffects             effects;
    DaeVectorMap           sources;
    DaeVectorMap           indexes;
    DaeTextures            textures;
    DaeOrder               meshes;         
    DaeRigids              rigids;
    DaeRigids              rigidNames;
    DaeRigidRelations      rrelations;
    DaeActuators           actuators;
    DaeActuatorRelations   arelations;
    DaeSensors             sensors;
    DaeResultSensorsPtr    sensorResults;
    DaeSensorRelations     srelations;

    MeshGenerator meshGenerator;
};


int DaeParserImpl::symbolSemantic(const string& elementName)
{
    // For the small number, and does not utilize the map and hash.
    static const int    EXT_ELEMENT_MAX = 5;
    static const string EXT_ELEMENT_VALUE[EXT_ELEMENT_MAX] = { 
        "POSITION",
        "VERTEX",
        "NORMAL",
        "COLOR",
        "TEXCOORD",
    };
    for (int i = 0; i < EXT_ELEMENT_MAX; i++) {
        if (iequals(EXT_ELEMENT_VALUE[i], elementName)) {
            return i;
        }
    }
    return -1;
}


int DaeParserImpl::symbolNode(const string& elementName)
{
    static const int    EXT_ELEMENT_MAX = 73;
    static const string EXT_ELEMENT_VALUE[EXT_ELEMENT_MAX] = { 
        "node",
        "unit",
        "translate",
        "rotate",
        "scale",
        "matrix",
        "instance_node",
        "instance_geometry",
        "instance_material",
        "instance_effect",
        "geometry",
        "vertices",
        "source",
        "float_array",
        "accessor",
        "input",
        "triangles",
        "lines",
        "p",
        "material",
        "effect",
        "emission",
        "ambient",
        "diffuse",
        "specular",
        "shininess",
        "transparency",
        "init_from",
        "image",
        "polylist",
        "vcount",

        "box",
        "sphere",
        "cylinder",
        "tapered_cylinder",
        "kinematics_model",
        "link",
        "attachment_full",
        "joint",
        "revolute",
        "prismatic",
        "axis",
        "min",
        "max",
        "rigid_body",
        "shape",
        "instance_rigid_body",
        "instance_physics_model",
        "mass",
        "mass_frame",
        "inertia",

        "actuator",
        "assigned_power_rating",
        "max_speed",
        "no_load_speed",
        "nominal_torque",
        "nominal_voltage",
        "rotor_inertia",
        "speed_constant",
        "speed_torque_gradient",
        "starting_current",
        "terminal_resistance",
        "torque_constant",
        "instance_actuator",
        "bind_actuator", 
        "instance_sensor",
        "frame_origin",

        "sensor",
        "focal_length",
        "intrinsic",
        "image_dimensions",
        "measurement_time",

        "extra",
    };
    // For the small number of, and does not utilize the map and hash.
    for (int i = 0; i < EXT_ELEMENT_MAX; i++) {
        if (iequals(EXT_ELEMENT_VALUE[i], elementName)) {
            return i; 
        }
    }
    return -1;
}


long DaeParserImpl::line()
{
    return (reader ? reader->getLineNumber() : -1);
}


DaeParser::DaeParser(ostream* os)
{
    daeParserImpl = new DaeParserImpl(os);
}


DaeParser::~DaeParser()
{
    delete daeParserImpl;
}


SgGroup* DaeParser::createScene(const string& fileName)
{
    daeParserImpl->parse(fileName, true);
    SgGroup* group = daeParserImpl->convert();
    return group;
}


void DaeParser::parse(const string& fileName) 
{
    daeParserImpl->parse(fileName, false);
}


DaeNode* DaeParser::findNode(const string& nodeName)
{
    return daeParserImpl->findNode(nodeName);
}


DaeNode* DaeParser::findJointByLink(const string& linkName)
{
    return daeParserImpl->findJointByLink(linkName);
}


DaeNode* DaeParser::findLinkByJoint(const string& jointName)
{
    return daeParserImpl->findLinkByJoint(jointName);
}


DaeNode* DaeParser::findRigidByLink(const string& linkName)
{
    return daeParserImpl->findRigidByLink(linkName);
}


DaeNode* DaeParser::findActuator(const string& jointId)
{
    return daeParserImpl->findActuator(jointId);
}


DaeResultSensors* DaeParser::findSensor(const string& linkId)
{
    return daeParserImpl->findSensor(linkId);
}


DaeNode* DaeParser::findRootLink()
{
    return daeParserImpl->rootLink.get();
}


string DaeParser::findRootName()
{
    return daeParserImpl->rootLinkName;
}


void DaeParser::createNode(DaeNodePtr extNode, SgGroup* group)
{
    daeParserImpl->setNode(extNode, group, false);
} 

    
void DaeParser::createTransform(DaeNodePtr extNode, SgGroup** sgParent, SgGroup** sgChild)
{
    daeParserImpl->setTransform(extNode, sgParent, sgChild);
}


void DaeParserImpl::throwException(int line, const string& message)
{
    if (0 < message.size()) {
        *os << message << endl;
    }
    ValueNode::SyntaxException error;
    error.setMessage(message);
    error.setPosition(line, -1);
    throw error;
}


void DaeNode::addChild(DaeNodePtr node)
{ 
    children.insert(pair<string, DaeNodePtr>(node->id, node)); 
}


DaeNodePtr DaeNode::clone()
{
    // It will create a clone for instance_node.
    DaeNodePtr node = new DaeNode;

    // Node of children also clone all, I want to address a different address.        
    for (DaeNodes::iterator iter = children.begin(); iter != children.end(); iter++) {
        DaeNodePtr renode = iter->second->clone();
        node->children.insert(pair<string, DaeNodePtr>(renode->id, renode));
    }

    node->geometries.insert(geometries.begin(), geometries.end());
    node->stack = stack;

    node->id                   = id;
    node->transform.translate  = transform.translate;
    node->transform.translates = transform.translates;
    node->transform.scale      = transform.scale;
    node->transform.rotate     = transform.rotate;
    node->transform.affine     = transform.affine;
    node->transform.center     = transform.center;
    node->transform.matrix     = transform.matrix;

    return node;
}


void DaeParserImpl::file(const string& fileName)
{
    vector<string> pathes; path = "";
    string abpath = boost::algorithm::replace_all_copy(fileName, "\\", "/");
    boost::algorithm::split(pathes, abpath, boost::is_any_of("/"));
    for (unsigned int i = 0; i < pathes.size() - 1; i++) {
        path += "/" + pathes[i];
    }
}


void DaeParserImpl::parse(const string& fileName, const bool inlineParse)
{
    if (iequals(befFile, fileName)) {
        return; // use created data.
    }
    file(fileName);
    this->inlineParse = inlineParse;

    DaeNodePtr node = NULL;
    reader = createIrrXMLReader(fileName.c_str());

    while (reader && reader->read()) {
        switch (reader->getNodeType()) {
        case EXN_ELEMENT: {
            string element = reader->getNodeName();
            // It get only in the "library_ *".
            if (checkAccessibleNode(element)) {
                switch (symbolNode(element)) {
                case EXT_NODE:                   node = readNode                (node); break;
                case EXT_TRANSLATE:              node = readTranslate           (node); break;
                case EXT_ROTATE:                 node = readRotate              (node); break;    
                case EXT_SCALE:                  node = readScale               (node); break;    
                case EXT_MATRIX:                 node = readMatrix              (node); break;    
                case EXT_INSTANCE_NODE:          node = readInstanceNode        (node); break;
                case EXT_INSTANCE_GEOMETRY:      node = readInstanceGeometry    (node); break;
                case EXT_INSTANCE_MATERIAL:      node = readInstanceMaterial    (node); break;
                case EXT_INSTANCE_EFFECT:        node = readInstanceEffect      (node); break;
                case EXT_GEOMETRY:               node = readGeometry            (node); break;
                case EXT_VERTICES:               node = readVertices            (node); break;
                case EXT_SOURCE:                 node = readSource              (node); break;
                case EXT_FLOAT_ARRAY:            node = readFloatArray          (node); break;         
                case EXT_ACCESSOR:               node = readAccessor            (node); break;         
                case EXT_INPUT:                  node = readInput               (node); break;         
                case EXT_TRIANGLES:              node = readTriangles           (node); break;         
                case EXT_LINES:                  node = readLines               (node); break;         
                case EXT_P:                      node = readP                   (node); break;         
                case EXT_MATERIAL:               node = readMaterial            (node); break;         
                case EXT_EFFECT:                 node = readEffect              (node); break;         
                case EXT_EMISSION:               node = readEmission            (node); break;         
                case EXT_AMBIENT:                node = readAmbient             (node); break;         
                case EXT_DIFFUSE:                node = readDiffuse             (node); break;         
                case EXT_SPECULAR:               node = readSpecular            (node); break;         
                case EXT_SHININESS:              node = readShininess           (node); break;         
                case EXT_TRANSPARENCY:           node = readTransparency        (node); break;         
                case EXT_INIT_FORM:              node = readInitFrom            (node); break;         
                case EXT_IMAGE:                  node = readImage               (node); break;         
                case EXT_POLYLIST:               node = readPolylist            (node); break;         
                case EXT_VCOUNT:                 node = readVcount              (node); break;         

                case EXT_BOX:                    node = readBox                 (node); break;         
                case EXT_SPHERE:                 node = readSphere              (node); break;         
                case EXT_CYLINDER:               node = readCylinder            (node); break;         
                case EXT_TAPERED_CYLINDER:       node = readTaperedCylinder     (node); break;         
                case EXT_KINEMATICS_MODEL:       node = readKinematicsModel     (node); break;         
                case EXT_LINK:                   node = readLink                (node); break;         
                case EXT_ATTACHMENT_FULL:        node = readAttachmentFull      (node); break;         
                case EXT_JOINT:                  node = readJoint               (node); break;         
                case EXT_REVOLUTE:               node = readRevolute            (node); break;         
                case EXT_PRISMATIC:              node = readPrismatic           (node); break;         
                case EXT_AXIS:                   node = readAxis                (node); break;         
                case EXT_MIN:                    node = readMin                 (node); break;         
                case EXT_MAX:                    node = readMax                 (node); break;         
                case EXT_RIGID_BODY:             node = readRigidBody           (node); break;         
                case EXT_SHAPE:                  node = readShape               (node); break;         
                case EXT_INSTANCE_RIGID_BODY:    node = readInstanceRigidBody   (node); break;         
                case EXT_INSTANCE_PHYSICS_MODEL: node = readInstancePhysicsModel(node); break;         
                case EXT_MASS:                   node = readMass                (node); break;         
                case EXT_MASS_FRAME:             node = readMassFrame           (node); break;         
                case EXT_INERTIA:                node = readInertia             (node); break;         

                case EXT_ACTUATOR:               node = readActuator            (node); break;
                case EXT_ASSIGNED_POWER_RATING:  node = readAssignedPowerRating (node); break;
                case EXT_MAX_SPEED:              node = readMaxSpeeed           (node); break;
                case EXT_NO_LOAD_SPEED:          node = readNoLoadSpeed         (node); break;
                case EXT_NOMINAL_TORQUE:         node = readNominalTorque       (node); break;
                case EXT_NOMINAL_VOLTAGE:        node = readNominalVoltage      (node); break;
                case EXT_ROTOR_INERTIA:          node = readRotorInertia        (node); break;
                case EXT_SPEED_CONSTANT:         node = readSpeedConstant       (node); break;
                case EXT_SPEED_TORQUE_GRADIENT:  node = readSpeedTorqueGradient (node); break;
                case EXT_STARTING_CURRENT:       node = readStartingCurrent     (node); break;
                case EXT_TERMINAL_RESISTANCE:    node = readTerminalResistance  (node); break;
                case EXT_TORQUE_CONSTANT:        node = readTorqueConstant      (node); break;

                case EXT_SENSOR:                 node = readSensor              (node); break;
                case EXT_FOCAL_LENGTH:           node = readFocalLength         (node); break;
                case EXT_INTRINSIC:              node = readIntrinsic           (node); break;
                case EXT_IMAGE_DIMENSIONS:       node = readImageDimensions     (node); break;
                case EXT_MEASUREMENT_TIME:       node = readMeasurementTime     (node); break;
 
                }
                if (reader->isEmptyElement()) {
                    // If the empty tag(<foo/>), It will pop the stack of nodes.
                    int symbol = symbolNode(element);
                    if (symbol == EXT_NODE            || symbol == EXT_INSTANCE_NODE ||
                        symbol == EXT_ATTACHMENT_FULL || symbol == EXT_LINK) {
                        node = endNode(node);
                    }
                }
            } else {
                // It get in certain other 
                switch (symbolNode(element)) {
                case EXT_UNIT:              node = readUnit            (node); break;
                case EXT_EXTRA:             node = readExtra           (node); break; 
                case EXT_INSTANCE_ACTUATOR: node = readInstanceActuator(node); break;
                case EXT_BIND_ACTUATOR:     node = readBindActuator    (node); break;
                case EXT_INSTANCE_SENSOR:   node = readInstanceSensor  (node); break;
                case EXT_FRAME_ORIGIN:      node = readFrameOrigin     (node); break;
                }
            }
        }
            break;
        case EXN_NONE:
        case EXN_TEXT:
        case EXN_COMMENT:
        case EXN_CDATA:
        case EXN_UNKNOWN:
            break;
        case EXN_ELEMENT_END: {
            string element = reader->getNodeName();
            // It treated with end tags for the control of the node stack.
            if (checkAccessibleNode(element)) {
                switch (symbolNode(element)) {
                case EXT_NODE:            node = endNode (node); break;
                case EXT_INSTANCE_NODE:   node = endNode (node); break;
                case EXT_LINK:            node = endNode (node); break;
                case EXT_ATTACHMENT_FULL: node = endNode (node); break;
                }
            }
        }
            break;
        }
    }
    if (reader) {
        delete reader;
        reader = NULL;
    }

    createStructure();
    checkSafetyNode();
    befFile = fileName;

}

    
bool DaeParserImpl::checkAccessibleNode(string& name)
{
    if (iequals(name.substr(0, 8), "library_")) {
        // For the small number, and does not utilize the map and hash.
        static const int    EXT_ELEMENT_MAX = 9;
        static const string EXT_ELEMENT_VALUE[EXT_ELEMENT_MAX] = { 
            "library_visual_scenes",
            "library_geometries",
            "library_materials",
            "library_effects",
            "library_nodes",
            "library_images",
            "library_physics_models", 
            "library_kinematics_models",
            "library_physics_scenes", 
        };
        for (int i = 0; i < EXT_ELEMENT_MAX; i++) {
            if (iequals(EXT_ELEMENT_VALUE[i], name)) {
                accessible = i;
                return true;
            }
        }
        accessible = -1;
    }
    return (accessible < 0 ? false : true);
}


void DaeParserImpl::split(string& value, double* point, int max, double init = 0.0)
{
    BOOST_ASSERT(point && 0 < max);

    trim(value);
    for (int i = 0; i < max; i++) point[i] = init;

    // Parses a string and returns by substituting a pair of specified number of elements
    int i = 0;
    for (split_iterator<string::iterator> iter  = make_split_iterator(value, token_finder(is_space(), token_compress_on));
         iter != split_iterator<string::iterator>(); 
         iter++)
        {
            point[i++] = lexical_cast<double>(*iter);
            if (i == max) {
                return;
            }
        }
}

    
DaeNodePtr DaeParserImpl::readUnit(DaeNodePtr node)
{
    // It get the coefficients of the coordinates.
    if (reader->getAttributeValue("meter")) {
        unit = lexical_cast<double>(reader->getAttributeValue("meter"));
    }
    return node;
}


DaeNodePtr DaeParserImpl::readNode(DaeNodePtr node)
{
    // There is always a name for node.
    node       = new DaeNode;
    node->id   = (reader->getAttributeValue("id") ? reader->getAttributeValue("id") : reader->getAttributeValue("name"));
    node->name =  reader->getAttributeValue("name");
    node->type = (reader->getAttributeValue("type") 
                  ? (iequals(reader->getAttributeValue("type"), "joint") ? EXT_NODE_JOINT : EXT_NODE_NODE) 
                  :  EXT_NODE_NODE);
 
    pair<DaeNodes::iterator, bool> pib = nodes.insert(pair<string, DaeNodePtr>(node->id, node));
    if (!pib.second) {
        throwException(line(), (format("[%1%]duplicate node:%2%") % line() % node->id).str());
    }
    // for the context of the link name
    pib = nodeNames.insert(pair<string, DaeNodePtr>(node->name, node));
    if (!pib.second) {
        if (!inlineParse) {
            throwException(line(), ((format("[%1%][WARNING]duplicate node-name:%2%") % line() % node->name).str()));
        }
    }

    if (!rootNodePresence) {
        stack.clear();
    }

    // It will tell the relationship between the nodes so far.
    DaeNodePtr root = (accessible == EXT_NODES ? library : instance);
    node->parent = (stack.empty() ? root : stack.at(stack.size() - 1).second);
    node->parent->children.insert(pair<string, DaeNodePtr>(node->id, node)); 

    node->stack = stack;
    stack.push_back(pair<string, DaeNodePtr>(node->id, node));

    if (!rootNodePresence) {
        rootNode         = node;
        rootNodePresence = true;
        rootLinkName     = node->name;
    }

    return node; 
}


DaeNodePtr DaeParserImpl::readInstanceNode(DaeNodePtr node)
{
    string url = reader->getAttributeValue("url");
    url = url.substr(1);

    string uuidNode = lexical_cast<string>(random_generator()());
    DaeNodePtr refNode = new DaeNode;
    refNode->id = uuidNode;
    refNode->refNodeId = url;

    // It will tell the relationship between the nodes so far.
    DaeNodePtr root = (accessible == EXT_NODES ? library : instance);
    refNode->parent = (stack.empty() ? root : stack.at(stack.size() - 1).second);
    refNode->parent->children.insert(pair<string, DaeNodePtr>(refNode->id, refNode)); 

    refNode->stack = stack;
    stack.push_back(pair<string, DaeNodePtr>(refNode->id, refNode));

    return node;
}

    
DaeNodePtr DaeParserImpl::endNode(DaeNodePtr node)
{
    if (!stack.empty()) {
        stack.pop_back();
    }
    return node;
}


DaeNodePtr DaeParserImpl::readTranslate(DaeNodePtr node)
{
    double point[3];
    point[0] = point[1] = point[2] = 0.0;   
 
    reader->read();
    string translate = reader->getNodeData(); 
    split(translate, point, 3);
    if (point[0] != 0.0 || point[1] != 0.0 || point[2] != 0.0) {
        // Do not log about all 0.
        node->transform.translate += Vector3d(point[0], point[1], point[2]);
        // ColladaBodyLoader only uses this translates information.
        // The mass_frame tag is defined multiple translate from origin.
        // But ColladaBodyLoader need one of the last. The translates to provide that one.
        node->transform.translates.push_back(Vector3d(point[0], point[1], point[2]));
    }

    return node;
}


void DaeParserImpl::rotate(string& value, DaeTransform& transform)
{
    trim(value);
    double point[4]; int i = 0;
    point[0] = point[1] = point[2] = point[3] = 0.0;
    
    for (split_iterator<string::iterator> iter  = make_split_iterator(value, token_finder(is_space(), token_compress_on));
         iter != split_iterator<string::iterator>(); 
         iter++)
        {   
            point[i++] = lexical_cast<double>(*iter);
            if (i == 4) {
                i = 0;
                // If there is a rotate more than one, It will use all of its rotation.
                transform.rotate = (transform.rotate * AngleAxis(point[3] * M_PI / 180, Vector3(point[0], point[1], point[2])));
            }
        }
}


DaeNodePtr DaeParserImpl::readRotate(DaeNodePtr node)
{
    reader->read();
    string value = reader->getNodeData(); 
    rotate(value, node->transform); 

    return node;
}


DaeNodePtr DaeParserImpl::readScale(DaeNodePtr node)
{
    double point[3];
    point[0] = point[1] = point[2] = 0.0;    
    
    reader->read();
    string scale = reader->getNodeData(); 

    split(scale, point, 3, 1.0);
    node->transform.scale = Vector3(point[0], point[1], point[2]);

    return node;
}


void DaeParserImpl::matrix(string& value, Matrix4d& matrix)
{
    trim(value);
    double point[4][4]; int i = 0, j = 0;
    for (int z = 0; z < 4; z++) point[z][0] = point[z][1] = point[z][2] = point[z][3] = 0.0;
    
    for (split_iterator<string::iterator> iter  = make_split_iterator(value, token_finder(is_space(), token_compress_on));
         iter != split_iterator<string::iterator>(); 
         iter++)
        {   
            point[j][i++] = lexical_cast<double>(*iter);
            if (i == 4) {
                if (4 < j) {
                    throwException(line(), (format("[%1%]invalid matrix") % line()).str());
                }
                i = 0; j++;
            }
        }
    matrix << point[0][0], point[0][1], point[0][2], point[0][3],
        point[1][0], point[1][1], point[1][2], point[1][3], 
        point[2][0], point[2][1], point[2][2], point[2][3], 
        point[3][0], point[3][1], point[3][2], point[3][3];
}


DaeNodePtr DaeParserImpl::readMatrix(DaeNodePtr node)
{
    reader->read();        
    string value = reader->getNodeData();

    if (node->transform.matrix) {
        throwException(line(), (format("[%1%]duplicate matrix:%2%") % line() % node->id).str());
    }
    node->transform.matrix = new Matrix4d;
    matrix(value, *(node->transform.matrix));

    return node; 
}


DaeNodePtr DaeParserImpl::readInstanceGeometry(DaeNodePtr node)
{
    if (accessible != EXT_PHYSICS_MODELS) {
        node = readInstanceGeometryForNodes  (node);
    } else {
        node = readInstanceGeometryForPhysics(node);
    } 
 
    return node;
}


DaeNodePtr DaeParserImpl::readInstanceGeometryForNodes(DaeNodePtr node)
{
    string url = reader->getAttributeValue("url");
    url = url.substr(1);
    refGeometryId = url;

    DaeGeometries::iterator iterg = geometries.find(url);
    DaeGeometryPtr geometry = (iterg != geometries.end() ? iterg->second.get() : new DaeGeometry);
    if (iterg == geometries.end()) {
        geometry->geometryId = url;
        geometries.insert(pair<string, DaeGeometryPtr>(url, geometry));
    }

    DaeGeometries::iterator itern = node->geometries.find(url);
    if (itern != node->geometries.end()) {
        throwException(line(), (format("[%1%]duplicate geometry object found:%2%") % line() % url).str());
    } else {
        node->geometries.insert(pair<string, DaeGeometryPtr>(url, geometry));
    }

    return node;
}


DaeRigidPtr DaeParserImpl::findRigidBody(string& refNodeId)
{
    DaeRigidRelations::iterator itern = rrelations.find(refNodeId);
    if (itern == rrelations.end()) {
        throwException(line(), (format("[%1%][1]node-id not found on rigid-body:%2%") 
                                % line() % refNodeId).str());
    }

    DaeRigids::iterator iterr = rigids.find(itern->second);
    if (iterr == rigids.end()) {
        throwException(line(), (format("[%1%][2]rigid-id not found on rigid-body:%2%") % line() % itern->second).str());
    }

    return iterr->second;
}


DaeNodePtr DaeParserImpl::readInstanceGeometryForPhysics(DaeNodePtr node)
{
    if (!reader->getAttributeValue("url")) {
        throwException(line(), (format("[%1%]invalid geometry") % line()).str());
    }
    string url = reader->getAttributeValue("url");
    url = url.substr(1);
    DaeGeometries::iterator iterg = geometries.find(url);
    if (iterg == geometries.end()) {
        throwException(line(), (format("[%1%]invalid geometry-id:%2%") % line() % url).str());
    }

    DaeShape* shape = static_cast<DaeShape*>(node.get());
    shape->setPolygon(iterg->second);

    return node;
}


DaeNodePtr DaeParserImpl::readInstanceMaterial(DaeNodePtr node)
{
    if (!reader->getAttributeValue("target")) {
        throwException(line(), (format("[%1%]invalid target attribute on instance material") % line()).str());
    }
    if (!reader->getAttributeValue("symbol")) {
        throwException(line(), (format("[%1%]invalid symbol attribute on instance material") % line()).str());
    }

    string target = reader->getAttributeValue("target");
    string symbol = reader->getAttributeValue("symbol");
    target = target.substr(1);

    if (DaeShape* extShape = dynamic_cast<DaeShape*>(node.get())) {
        // keeping the reference material id only for the "rigid_body".
        extShape->refMaterialId = target;

    } else {
        // creating the relation of geometryid + symbol and material-id for "geometry".
        DaeGeometries::iterator iterg = geometries.find(refGeometryId);
        if (iterg == geometries.end()) {
            throwException(line(), (format("[%1%]invalid geometry structure found:%2%") % line() % target).str());
        }
        DaeGeometryPtr geometry = iterg->second;
        geometry->refMaterialId = target;

        DaeMaterials::iterator iterm = materials.find(geometry->refMaterialId);
        if (iterm == materials.end()) {
            DaeMaterialPtr material = new DaeMaterial;
            material->materialId = target;
            materials.insert(pair<string, DaeMaterialPtr>(target, material));
        }
       
        // Keep the symbol for target name of concrete-material. 
        materialRef.insert(pair<string, string>(refGeometryId + "/" + symbol, target)); 
    }
 
    return node; 
}


DaeNodePtr DaeParserImpl::readMaterial(DaeNodePtr node)
{
    if (reader->getAttributeValue("id")) { // ignore the empty id of material-tag in gazobo-tag.
        string materialId = reader->getAttributeValue("id");
        refMaterialId = materialId;

        DaeMaterials::iterator iterm = materials.find(materialId);
        if (iterm == materials.end()) {
            DaeMaterialPtr material = new DaeMaterial;
            material->materialId = materialId;
            materials.insert(pair<string, DaeMaterialPtr>(materialId, material));
        }
    }
        
    return node; 
}


DaeNodePtr DaeParserImpl::readInstanceEffect(DaeNodePtr node)
{
    DaeMaterials::iterator iter = materials.find(refMaterialId);
    if (iter == materials.end()) {
        throwException(line(), (format("[%1%]invalid effect structure:%2%") % line() % refMaterialId).str());
    } 

    DaeMaterialPtr material = iter->second;
    string url = reader->getAttributeValue("url");
    url = url.substr(1);
    material->refEffectId = url;

    DaeEffects::iterator itere = effects.find(url); 
    if (itere == effects.end()) {
        DaeEffectPtr effect = new DaeEffect;
        effect->effectId = url;
        effects.insert(pair<string, DaeEffectPtr>(url, effect));
    }

    return node; 
}


DaeNodePtr DaeParserImpl::readEffect(DaeNodePtr node)
{
    // It will hold the id. This ID is used in the child node.
    refEffectId = reader->getAttributeValue("id"); 
    return node; 
}


DaeEffectPtr DaeParserImpl::findOrCreateEffect(string& refEffectId)
{
    DaeEffects::iterator iter = effects.find(refEffectId);
    DaeEffectPtr effect = (iter != effects.end() ? iter->second.get() : new DaeEffect);
    if (iter == effects.end()) {
        effect->effectId = refEffectId;
        effects.insert(pair<string, DaeEffectPtr>(refEffectId, effect));
    }
    return effect;
}


DaeEffectPtr DaeParserImpl::moveToColor(string& refEffectId)
{
    DaeEffectPtr effect = findOrCreateEffect(refEffectId);
 
    // It go to the color tag
    for (int i = 0; i < 2; i++) { 
        reader->read();
    }
    if (iequals(reader->getNodeName(), "texture")) {
        // skip the processing without printing error message.
        throwException(line(), "");
    }
    if (!iequals(reader->getNodeName(), "color")) {
        throwException(line(), (format("[%1%]invalid effect-color structure:%2%") % line() % refEffectId).str());
    }
    reader->read(); // It go to the text-node

    return effect;
}


DaeEffectPtr DaeParserImpl::moveToFloat(string& refEffectId)
{
    DaeEffectPtr effect = findOrCreateEffect(refEffectId);
 
    // It go to the float tag
    for (int i = 0; i < 2; i++) { 
        reader->read();
    }
    if (!iequals(reader->getNodeName(), "float")) {
        throwException(line(), (format("[%1%]invalid effect-float structure:%2%") % line() % refEffectId).str());
    }
    reader->read(); // It go to the text-node

    return effect;
}


DaeNodePtr DaeParserImpl::readEmission(DaeNodePtr node)
{
    DaeEffectPtr effect = moveToColor(refEffectId);    

    // It read the emission/color
    string emission = reader->getNodeData();
    double point[3];
    split(emission, point, 3);
    effect->emission << point[0], point[1], point[2];

    return node; 
}


DaeNodePtr DaeParserImpl::readAmbient(DaeNodePtr node)
{
    DaeEffectPtr effect = moveToColor(refEffectId);    

    // It read the emission/color
    string ambient = reader->getNodeData();
    double point[1];
    split(ambient, point, 1);
    effect->ambient = point[0];
 
    return node; 
} 


DaeNodePtr DaeParserImpl::readDiffuse(DaeNodePtr node)
{
    DaeEffectPtr effect;
    try {
        effect = moveToColor(refEffectId);   
    } catch (...) {
        // It was texture-tag
        effect = findOrCreateEffect(refEffectId);
        effect->diffuse = Vector3(1.0, 1.0, 1.0);
        return node;
    }

    // It read the diffuse/color
    string diffuse = reader->getNodeData();
    double point[3];
    split(diffuse, point, 3);
    effect->diffuse << point[0], point[1], point[2];

    return node; 
}


DaeNodePtr DaeParserImpl::readSpecular(DaeNodePtr node)
{
    DaeEffectPtr effect = moveToColor(refEffectId);    

    // It read the emission/color
    string specular = reader->getNodeData();
    double point[3];
    split(specular, point, 3);
    effect->specular << point[0], point[1], point[2];

    return node; 
}


DaeNodePtr DaeParserImpl::readShininess(DaeNodePtr node)
{
    DaeEffectPtr effect = moveToFloat(refEffectId);    
        
    // It read the shininess/float
    string shininess = reader->getNodeData();
    double point[1];
    split(shininess, point, 1);
    effect->shininess = point[0];

    return node;
}


DaeNodePtr DaeParserImpl::readTransparency(DaeNodePtr node)
{
    DaeEffectPtr effect = moveToFloat(refEffectId);    
        
    // It read the shininess/float
    string transparency = reader->getNodeData();
    double point[1];
    split(transparency, point, 1);
    effect->transparency = point[0];

    return node;
}


DaeNodePtr DaeParserImpl::readGeometry(DaeNodePtr node)
{
    // It will hold the id. This ID is used in the child node.
    refGeometryId = reader->getAttributeValue("id");

    DaeGeometries::iterator iter = geometries.find(refGeometryId);
    if (iter == geometries.end()) {
        DaeGeometryPtr geometry = new DaeGeometry;
        geometry->geometryId = refGeometryId;
        geometries.insert(pair<string, DaeGeometryPtr>(refGeometryId, geometry));
    }
 
    meshes.clear();

    return node; 
}


DaeNodePtr DaeParserImpl::readVertices(DaeNodePtr node)
{
    // It will hold the id. This ID is used in the child node.
    refVerticesId = refGeometryId + "/" + reader->getAttributeValue("id");
    meshes.push_back("vertices");
 
    return node; 
}


DaeNodePtr DaeParserImpl::readSource(DaeNodePtr node)
{
    // It will hold the id. This ID is used in the child node.
    if (reader->getAttributeValue("id")) {
        refSourceId = reader->getAttributeValue("id");
        meshes.push_back("source");
    }
    return node; 
}


void DaeParserImpl::array(string& value, DaeVectorXArrayPtr array)
{
    trim(value);
    array->clear();
    for (split_iterator<string::iterator> iter  = make_split_iterator(value, token_finder(is_space(), token_compress_on));
         iter != split_iterator<string::iterator>(); 
         iter++)
        {
            if ((boost::copy_range<std::string>(*iter)).size() <= 0) {     
                throwException(line(), (format("[%1%]invalid value:%2%") % line() % value).str());
            }
            try {
                // It holds the number of the number of specified.
                array->push_back(lexical_cast<double>(*iter));
            } catch (...) {
                // NaN is assigned, numerical error occurs.
                array->push_back(0.0); 
            }
        }

}


DaeNodePtr DaeParserImpl::readFloatArray(DaeNodePtr node)
{
    DaeVectorMap::iterator iter = sources.find(refSourceId);
    if (iter != sources.end()) {
        throwException(line(), (format("[%1%]invalid source-float structure:%2%") % line() % refSourceId).str());
    }

    int icount = (reader->getAttributeValue("count") ? lexical_cast<int>(reader->getAttributeValue("count")) : -1);
    DaeVectorXArrayPtr source = DaeVectorXArrayPtr(new DaeVectorXArray);
    if (0 < icount) {
        source->resize(icount);
    }

    reader->read();
    string value = reader->getNodeData();
    array(value, source);
    pair<DaeVectorMap::iterator, bool> pib = sources.insert(pair<string, DaeVectorXArrayPtr>(refSourceId, source));
    if (!pib.second) {
        throwException(line(), (format("[%1%]duplicate source:%2%") % line() % refSourceId).str());
    }

    return node; 
}


DaeNodePtr DaeParserImpl::readAccessor(DaeNodePtr node)
{
    if (!reader->getAttributeValue("stride")) {
        throwException(line(), (format("[%1%]invalid stride,source-id:%2%") % line() % refSourceId).str());
    }
    int stride = lexical_cast<int>(reader->getAttributeValue("stride")); 
    pair<DaeStrides::iterator, bool> pib = strides.insert(pair<string, int>(refSourceId, stride));
    if (!pib.second) {
        throwException(line(), (format("[%1%]duplicate stride:%2%") % line() % refSourceId).str());
    }

    return node;
}


DaeNodePtr DaeParserImpl::readImage(DaeNodePtr node)
{
    // It will hold the id. This ID is used in the child node.
    refImageId = reader->getAttributeValue("id"); 
    return node; 
}


void DaeParserImpl::createMesh(int line)
{
    DaeGeometries::iterator iter = geometries.find(refGeometryId);
    if (iter == geometries.end()) {
        throwException(line, (format("[%1%]invalid mesh on geometry(geimeotry-id:%2%)") % line % refGeometryId).str());
    }
    DaeGeometryPtr geometry = iter->second;
    DaeMeshPtr mesh = new DaeMesh;
    if (reader->getAttributeValue("material")) {
        // keep the material such as MESH (triangles-tag).
        mesh->refMaterialId = reader->getAttributeValue("material");
    }
    geometry->meshes.push_back(mesh);
    refMesh = mesh;
}


DaeNodePtr DaeParserImpl::readPolylist(DaeNodePtr node)
{
    figure = EXT_POLYLIST;
    createMesh(line());
    meshes.push_back("polylist"); 
    return node;
}


DaeNodePtr DaeParserImpl::readBox(DaeNodePtr node)
{
    if (accessible != EXT_PHYSICS_MODELS) {
        throwException(line(), (format("[%1%]invalid box-tag, it not join to the physics-tag") % line()).str());
    }

    for (int i = 0; i < 3; i++) reader->read();
    string value = reader->getNodeData();
    DaeVectorXArrayPtr values = DaeVectorXArrayPtr(new DaeVectorXArray);
    array(value, values);

    DaeShape* shape = static_cast<DaeShape*>(node.get());
    if (shape->isPresence()) {
        throwException(line(), (format("[%1%]duplicate primitive-physics on shape, rigid-id:%2%") 
                                % line() % befRigid->id).str());
    }
    shape->setBox(Vector3(values->at(0), values->at(1), values->at(2)));

    return node;
}


DaeNodePtr DaeParserImpl::readSphere(DaeNodePtr node)
{
    if (accessible != EXT_PHYSICS_MODELS) {
        throwException(line(), (format("[%1%]invalid sphere-tag, it not join to the physics-tag") % line()).str());
    }
    for (int i = 0; i < 3; i++) reader->read();
    string value = reader->getNodeData();

    DaeShape* shape = static_cast<DaeShape*>(node.get());
    if (shape->isPresence()) {
        throwException(line(), (format("[%1%]duplicate primitive-physics on shape, rigid-id:%2%") 
                                % line() % befRigid->id).str());
    }
    shape->setSphere(lexical_cast<double>(value));

    return node;
}


DaeNodePtr DaeParserImpl::readCylinder(DaeNodePtr node)
{
    if (accessible != EXT_PHYSICS_MODELS) {
        throwException(line(), (format("[%1%]invalid cylinder-tag, it not join to the physics-tag") % line()).str());
    }

    std::string name = "";
    double height = 0.0, radius = 0.0;

    for (int i = 0; i < 2; i++) reader->read();
    name = reader->getNodeName();
    reader->read(); string value = reader->getNodeData();
    if (iequals("height", name)) height = lexical_cast<double>(value);
    else radius = lexical_cast<double>(value);

    for (int i = 0; i < 3; i++) reader->read();
    name = reader->getNodeName();
    reader->read(); value = reader->getNodeData();
    if (iequals("radius", name)) radius = lexical_cast<double>(value);
    else height = lexical_cast<double>(value);

    DaeShape* shape = static_cast<DaeShape*>(node.get());
    if (shape->isPresence()) {
        throwException(line(), (format("[%1%]duplicate primitive-physics on shape, rigid-id:%2%") 
                                % line() % befRigid->id).str());
    }
    shape->setCylinder(height, radius);

    return node;
}


void DaeParserImpl::readTaperedCylinderConcrete(std::string& name, std::string& value, double& height, double radius1[2], double radius2[2])
{
    if (iequals("height", name)) {
        height = lexical_cast<double>(value);
    } else if (iequals("radius1", name)) {
        DaeVectorXArrayPtr values = DaeVectorXArrayPtr(new DaeVectorXArray);
        array(value, values);
        radius1[0] = values->at(0);
        radius1[1] = values->at(1);
    } else if (iequals("radius2", name)) {
        DaeVectorXArrayPtr values = DaeVectorXArrayPtr(new DaeVectorXArray);
        array(value, values);
        radius2[0] = values->at(0);
        radius2[1] = values->at(1);
    }    
}


DaeNodePtr DaeParserImpl::readTaperedCylinder(DaeNodePtr node)
{
    if (accessible != EXT_PHYSICS_MODELS) {
        throwException(line(), (format("[%1%]invalid tapered-cylinder-tag, it not join to the physics-tag") 
                                % line()).str());
    }

    std::string name = "";
    double height = 0.0;
    double radius1[2], radius2[2];

    for (int i = 0; i < 2; i++) reader->read();
    name = reader->getNodeName();
    reader->read(); string value = reader->getNodeData();
    readTaperedCylinderConcrete(name, value, height, radius1, radius2);
        
    for (int i = 0; i < 3; i++) reader->read();
    name = reader->getNodeName();
    reader->read(); value = reader->getNodeData();
    readTaperedCylinderConcrete(name, value, height, radius1, radius2);

    for (int i = 0; i < 3; i++) reader->read();
    name = reader->getNodeName();
    reader->read(); value = reader->getNodeData();
    readTaperedCylinderConcrete(name, value, height, radius1, radius2);

    DaeShape* shape = static_cast<DaeShape*>(node.get());
    if (shape->isPresence()) {
        throwException(line(), (format("[%1%]duplicate primitive-physics on shape, rigid-id:%2%") 
                                % line() % befRigid->id).str());
    }
    shape->setCone(height, Vector2(radius1[0], radius1[1]), Vector2(radius2[0], radius2[1]));

    return node;
}


DaeNodePtr DaeParserImpl::readKinematicsModel(DaeNodePtr node)
{
    // keep the kinematics model id for children processing.
    refKinematicsId = reader->getAttributeValue("id") ? reader->getAttributeValue("id") : reader->getAttributeValue("name");
    return node;
}


DaeNodePtr DaeParserImpl::readLink(DaeNodePtr node)
{  
    // There is a clear distinction between link and node in collada. 
    if (accessible != EXT_KINEMATICS_MODELS) {
        throwException(line(), (format("[%1%]invalid link-tag, it not join to the kinematics-tag") % line()).str());
    }              
    if (!reader->getAttributeValue("sid")) {
        throwException(line(), (format("[%1%]invalid link-tag, it haven't the sid:%2%") 
                                % line() % refKinematicsId).str());
    }
    DaeLinkPtr link = new DaeLink;

    string id   = refKinematicsId + "/" + reader->getAttributeValue("sid");
    string name = reader->getAttributeValue("name");
    link ->id   = id;
    link ->name = name;

    pair<DaeLinks::iterator, bool> pib = links.insert(pair<string, DaeLinkPtr>(id, link));
    if (!pib.second) {    
        throwException(line(), (format("[%1%]duplicate link:%2%") % line() % id).str());
    }
    pib = linkNames.insert(pair<string, DaeLinkPtr>(name, link));
    if (!pib.second) {    
        throwException(line(), (format("[%1%]duplicate link-name:%2%") % line() % name).str());
    }
    if (!rootLinkPresence) {
        stack.clear();
    }

    if (0 < stack.size()) {
        DaeJoint* parentNode = static_cast<DaeJoint*>(stack.at(stack.size() - 1).second.get());
        parentNode->children.push_back(name);
    } 

    link->stack = stack;
    stack.push_back(pair<string, DaeNodePtr>(link->id, link));

    if (!rootLinkPresence) {
        rootLink = link;
        rootLinkPresence = true;
    }        

    return static_cast<DaeNodePtr>(link);
}


DaeNodePtr DaeParserImpl::readAttachmentFull(DaeNodePtr node)
{
    // There is a clear distinction between joint and node in collada. 
    // Attachment_full tags are the "INSTANCE" of definition of joint.
    if (accessible != EXT_KINEMATICS_MODELS) {
        throwException(line(), (format("[%1%]invalid attachment-tag, it not join to the kinematics-tag") % line()).str());
    }              
    if (!reader->getAttributeValue("joint")) {
        throwException(line(), (format("[%1%]invalid attachment-tag, it haven't a joint attribute") % line()).str());
    }
    string target = reader->getAttributeValue("joint");

    DaeJoints::iterator iterj = joints.find(target);
    DaeJointPtr joint = (iterj == joints.end() ? new DaeJoint : iterj->second.get());
    if (iterj == joints.end()) {
        joints.insert(pair<string, DaeJointPtr>(target, joint));
    }
    joint->id = target;
      
    if (0 < stack.size()) {
        DaeLink* parentNode = static_cast<DaeLink*>(stack.at(stack.size() - 1).second.get());
        parentNode->children.push_back(target);
    }

    joint->stack = stack;
    stack.push_back(pair<string, DaeNodePtr>(joint->id, joint));

    return static_cast<DaeNodePtr>(joint);
}


DaeNodePtr DaeParserImpl::readJoint(DaeNodePtr node)
{
    // Joint tags are the "CONCRETE(like Library)" of definition of joint.
    if (accessible != EXT_KINEMATICS_MODELS) {
        throwException(line(), (format("[%1%]invalid joint-tag, it not join to the kinematics-tag") % line()).str());
    }              
    if (!reader->getAttributeValue("sid")) {
        throwException(line(), (format("[%1%]invalid joint-tag, it haven't a sid attribute") % line()).str());
    }

    string id   = refKinematicsId + "/" + reader->getAttributeValue("sid");
    string name = reader->getAttributeValue("name");

    DaeJoints::iterator iterj = joints.find(id);
    DaeJointPtr joint = (iterj == joints.end() ? new DaeJoint : iterj->second.get());
    if (iterj == joints.end()) {
        joints.insert(pair<string, DaeJointPtr>(id, joint));
    }
    joint->id   = id;
    joint->name = name;
    befJoint    = joint;
 
    return node;
}


DaeNodePtr DaeParserImpl::readPrismatic(DaeNodePtr node)
{
    return readRevolute(node);
}


DaeNodePtr DaeParserImpl::readRevolute(DaeNodePtr node)
{
    if (accessible != EXT_KINEMATICS_MODELS) {
        throwException(line(), (format("[%1%]invalid revolute-tag, it not join to the kinematics-tag") % line()).str());
    }              
    if (!reader->getAttributeValue("sid")) {
        throwException(line(), (format("[%1%]invalid revolute-tag, it haven't a sid attribute") % line()).str());
    }
    if (!befJoint) {
        throwException(line(), (format("[%1%]invalid revolute-tag, it haven't a parent joint-tag") % line()).str());
    }

    // The structure of this hierarchy is deterministic. 
    DaeRevolutePtr revolute = new DaeRevolute;
    string id    = reader->getAttributeValue("sid"); 
    revolute->id = id;
    befRevolute  = revolute;
    befJoint->revolutes.push_back(revolute);        

    return node;
}


DaeNodePtr DaeParserImpl::readAxis(DaeNodePtr node)
{
    if (accessible == EXT_KINEMATICS_MODELS) {
        if (befJoint && befRevolute) {
            reader->read(); 
            if (!reader->getNodeData()) {
                throwException(line(), (format("[%1%]invalid axis-tag, it haven't a content") % line()).str());
            }
                     
            string content = reader->getNodeData();
            vector<string> ret;
            boost::split(ret, content, is_any_of(" "));
            if (ret.size() != 3) {
                throwException(line(), (format("[%1%]invalid axis-tag, it have a invalid content") % line()).str());
            }
            // The structure of this hierarchy is deterministic.
            befRevolute->axis = Vector3(lexical_cast<double>(ret[0]),
                                        lexical_cast<double>(ret[1]),
                                        lexical_cast<double>(ret[2]));
        }
    }

    return node;
}


DaeNodePtr DaeParserImpl::readMin(DaeNodePtr node)
{
    if (accessible == EXT_KINEMATICS_MODELS) {
        if (befJoint && befRevolute) {
            reader->read(); 
            if (!reader->getNodeData()) {
                throwException(line(), (format("[%1%]invalid min-tag, it haven't a content") % line()).str());
            }
                     
            string content = reader->getNodeData();
            befRevolute->limitMin = lexical_cast<double>(content);
        }
    }

    return node;
}


DaeNodePtr DaeParserImpl::readMax(DaeNodePtr node)
{
    if (accessible == EXT_KINEMATICS_MODELS) {
        if (befJoint && befRevolute) {
            reader->read(); 
            if (!reader->getNodeData()) {
                throwException(line(), (format("[%1%]invalid max-tag, it haven't a content") % line()).str());
            }
        
            // The structure of this hierarchy is deterministic.             
            string content = reader->getNodeData();
            befRevolute->limitMax = lexical_cast<double>(content);
        }
    }

    return node;
}


DaeNodePtr DaeParserImpl::readInstancePhysicsModel(DaeNodePtr node)
{
    rrelations.clear();
    return node;
}


DaeNodePtr DaeParserImpl::readInstanceRigidBody(DaeNodePtr node)
{
    // Rigid is related to the link.
    if (accessible != EXT_PHYSICS_SCENES) {
        throwException(line(), (format("[%1%]invalid instance-rigid-tag, it not join to the physics-scene-tag") 
                                % line()).str());
    }
    if (!reader->getAttributeValue("body") || !reader->getAttributeValue("target")) {
        throwException(line(), (format("[%1%]invalid instance-rigid-tag, it haven't a body or target") 
                                % line()).str());
    }

    string body   = reader->getAttributeValue("body");
    string target = reader->getAttributeValue("target");
    target = target.substr(1);
 
    pair<DaeRigidRelations::iterator, bool> pib = rrelations.insert(pair<string, string>(target, body)); 
    if (!pib.second) {
        throwException(line(), (format("[%1%]duplicate instance-rigid-tag:%2%(%3%)") 
                                % line() % target % body).str());
    }

    return node;
}


DaeNodePtr DaeParserImpl::readMass(DaeNodePtr node)
{
    // mass is the mass of the link.
    if (accessible != EXT_PHYSICS_MODELS) {
        throwException(line(), (format("[%1%]invalid mass-tag, it not join to the physics-tag") % line()).str());
    }

    reader->read();
    if (!reader->getNodeData()) {
        throwException(line(), (format("[%1%]invalid mass-tag, it haven't a content") % line()).str());
    }
    string value = reader->getNodeData();
    DaeRigid* rigid = static_cast<DaeRigid*>(node.get());
    rigid->mass = lexical_cast<double>(value);

    return node;
}


DaeNodePtr DaeParserImpl::readMassFrame(DaeNodePtr node)
{
    // mass_frame is translation and rotation to the center of gravity.
    if (!befRigid) {
        throwException(line(), (format("[%1%]invalid mass-frmae-tag") % line()).str());
    }
    DaeMassFramePtr massFrame = new DaeMassFrame;
    befRigid->massFrame = massFrame; // easy to access this node on next(child) node.
    return static_cast<DaeNodePtr>(massFrame);
}


DaeNodePtr DaeParserImpl::readInertia(DaeNodePtr node)
{
    // This is the inertia of the link.
    if (accessible != EXT_PHYSICS_MODELS) {
        throwException(line(), (format("[%1%]invalid inertia-tag, it not join to the physics-tag") % line()).str());
    }

    reader->read();
    if (!reader->getNodeData()) {
        throwException(line(), (format("[%1%]invalid inertia-tag, it haven't a content") % line()).str());
    }
    string value = reader->getNodeData();
    DaeVectorXArrayPtr values = DaeVectorXArrayPtr(new DaeVectorXArray);
    array(value, values);

    befRigid->inertia = Vector3(lexical_cast<double>(values->at(0)),
                                lexical_cast<double>(values->at(1)),
                                lexical_cast<double>(values->at(2)));  
    return node;
}


DaeNodePtr DaeParserImpl::readShape(DaeNodePtr node)
{
    // This is a graphic of primitive.
    if (accessible != EXT_PHYSICS_MODELS) {
        throwException(line(), (format("[%1%]invalid shape-tag, it not join to the physics-tag") % line()).str());
    }

    DaeShapePtr shape = new DaeShape;
    befRigid->shapes.push_back(shape);

    return static_cast<DaeShapePtr>(shape);
}


DaeNodePtr DaeParserImpl::readRigidBody(DaeNodePtr node)
{
    if (accessible != EXT_PHYSICS_MODELS) {
        throwException(line(), (format("[%1%]invalid rigid-body-tag, it not join to the physics-tag") % line()).str());
    }
    if (!reader->getAttributeValue("sid")) {
        throwException(line(), (format("[%1%]invalid rigid-body-tag, it haven't a sid") % line()).str());
    }

    DaeRigidPtr rigid = DaeRigidPtr(new DaeRigid);

    string  id  = reader->getAttributeValue("sid");
    string name = reader->getAttributeValue("name") ? reader->getAttributeValue("name") : reader->getAttributeValue("sid");

    refRigidBodyId = id;
    rigid->id      = id;
    rigid->name    = name;
    befRigid       = rigid;
        
    pair<DaeRigids::iterator, bool> pib = rigids.insert(pair<string, DaeRigidPtr>(id, rigid)); 
    if (!pib.second) {
        throwException(line(), (format("[%1%]duplicate rigid-body-tag:%2%") % line() % id).str());
    }
    pib = rigidNames.insert(pair<string, DaeRigidPtr>(name, rigid)); 
    if (!pib.second) {
        throwException(line(), (format("[%1%]duplicate rigid-body-tag:%2%") % line() % name).str());
    }

    return static_cast<DaeNodePtr>(rigid); 
}


DaeNodePtr DaeParserImpl::readVcount(DaeNodePtr node)
{
    // vcount is the number of vertices of the shape. (vertices count)
    if (figure != EXT_POLYLIST) {
        throwException(line(), (format("[%1%]invalid structure of vcount:%2%") % line() % refGeometryId).str());
    }
    DaeGeometries::iterator iterg = geometries.find(refGeometryId);
    if (iterg == geometries.end()) {
        throwException(line(), (format("[%1%]invalid vcount on geometry:%2%") % line() % refGeometryId).str());
    }
    DaeGeometryPtr geometry = iterg->second;
       
    reader->read();
    string value = reader->getNodeData();
    array(value, refMesh->vcount);

    return node;
}

   
bool DaeParserImpl::createVcount(DaeVectorXArrayPtr vcount)
{
    if (vcount->size() <= 0) {
        return false; // no vcount
    }
    for (DaeVectorXArray::iterator iter = vcount->begin(); iter != vcount->end(); iter++) {
        if (3 < lexical_cast<int>(*iter)) {
            return true;
        }    
    }
    return false;
}

 
DaeNodePtr DaeParserImpl::readInitFrom(DaeNodePtr node)
{
    // init_from is an image of the texture.
    if (accessible == EXT_IMAGES) {
        reader->read(); 
        string fileName = reader->getNodeData();
        boost::algorithm::replace_all(fileName, "file:///", ""); // no need "file:///"
        DaeTexturePtr texture = new DaeTexture(path + "/" + fileName);
        pair<DaeTextures::iterator, bool> pib = textures.insert(pair<string, DaeTexturePtr>(refImageId, texture));
        if (!pib.second) {
            throwException(line(), (format("[%1%]duplicate texture:%2%") % line() % refImageId).str());
        } 
 
    } else if (accessible == EXT_EFFECTS) {
        reader->read();
        string id = reader->getNodeData();
        DaeEffectPtr effect = findOrCreateEffect(refEffectId);
        effect->refImageId = id;            
 
    } else {
        throwException(line(), (format("[%1%]invalid structure of init_from") % line()).str());
    }

    return node;            
}


DaeNodePtr DaeParserImpl::readActuator(DaeNodePtr node)
{
    if (!reader->getAttributeValue("id")) {
        throwException(line(), (format("[%1%]invalid actuator") % line()).str());
    }
    string id = reader->getAttributeValue("id");
    DaeActuatorPtr actuator = new DaeActuator;
    actuator->id = id;

    pair<DaeActuators::iterator, bool> pib = actuators.insert(pair<string, DaeActuatorPtr>(id, actuator));
    if (!pib.second) {
        throwException(line(), (format("[%1%]duplicate actuator:%2%") % line() % id).str());
    }
 
    return static_cast<DaeNodePtr>(actuator);
}

    
void DaeParserImpl::setActuator(DaeNodePtr node, double& value, string message)
{
    reader->read();
    if (reader->getNodeData()) {
        string data = reader->getNodeData(); 
        value = lexical_cast<double>(data); 
    } else {
        *os << format(message.c_str()) % line() % node->id << endl;
    }
}


DaeNodePtr DaeParserImpl::readAssignedPowerRating(DaeNodePtr node)
{  
    DaeActuator* value = static_cast<DaeActuator*>(node.get());
    setActuator(node, value->assignedPowerRating, "[%1%][WARNING]assigned power rating is empty on actuator-tag:%2%");
    return node;
}


DaeNodePtr DaeParserImpl::readMaxSpeeed(DaeNodePtr node)
{
    DaeActuator* value = static_cast<DaeActuator*>(node.get());
    setActuator(node, value->maxSpeed, "[%1%][WARNING]max speed is empty on actuator-tag:%2%");
    return node;
}


DaeNodePtr DaeParserImpl::readNoLoadSpeed(DaeNodePtr node)
{
    DaeActuator* value = static_cast<DaeActuator*>(node.get());
    setActuator(node, value->noLoadSpeed, "[%1%][WARNING]no load speed is empty on actuator-tag:%2%");
    return node;
}


DaeNodePtr DaeParserImpl::readNominalTorque(DaeNodePtr node)
{
    DaeActuator* value = static_cast<DaeActuator*>(node.get());
    setActuator(node, value->nominalTorque, "[%1%][WARNING]nominal torque is empty on actuator-tag:%2%");
    return node;
}


DaeNodePtr DaeParserImpl::readNominalVoltage(DaeNodePtr node)
{
    DaeActuator* value = static_cast<DaeActuator*>(node.get());
    setActuator(node, value->nominalVoltage, "[%1%][WARNING]nominal voltage is empty on actuator-tag:%2%");
    return node;
}


DaeNodePtr DaeParserImpl::readRotorInertia(DaeNodePtr node)
{
    DaeActuator* value = static_cast<DaeActuator*>(node.get());
    setActuator(node, value->rotorInertia, "[%1%][WARNING]rotor inertia is empty on actuator-tag:%2%");
    return node;
}


DaeNodePtr DaeParserImpl::readSpeedConstant(DaeNodePtr node)
{
    DaeActuator* value = static_cast<DaeActuator*>(node.get());
    setActuator(node, value->speedConstant, "[%1%][WARNING]speed constant is empty on actuator-tag:%2%");
    return node;
}


DaeNodePtr DaeParserImpl::readSpeedTorqueGradient(DaeNodePtr node)
{
    DaeActuator* value = static_cast<DaeActuator*>(node.get());
    setActuator(node, value->speedTorqueGradient, "[%1%][WARNING]speed torque gradient is empty on actuator-tag:%2%");
    return node;
}


DaeNodePtr DaeParserImpl::readStartingCurrent(DaeNodePtr node)
{
    DaeActuator* value = static_cast<DaeActuator*>(node.get());
    setActuator(node, value->startingCurrent, "[%1%][WARNING]starting current is empty on actuator-tag:%2%");
    return node;
}


DaeNodePtr DaeParserImpl::readTerminalResistance(DaeNodePtr node)
{
    DaeActuator* value = static_cast<DaeActuator*>(node.get());
    setActuator(node, value->terminalResistance, "[%1%][WARNING]terminal resistance is empty on actuator-tag:%2%");
    return node;
}


DaeNodePtr DaeParserImpl::readTorqueConstant(DaeNodePtr node)
{
    DaeActuator* value = static_cast<DaeActuator*>(node.get());
    setActuator(node, value->torqueConstant, "[%1%][WARNING]torque constant is empty on actuator-tag:%2%");
    return node;
} 


DaeNodePtr DaeParserImpl::readInstanceActuator(DaeNodePtr node)
{
    if (DaeAttachActuator* attach = dynamic_cast<DaeAttachActuator*>(node.get())) {
        if (!reader->getAttributeValue("url")) {
            throwException(line(), (format("[%1%]invalid instance actuator-tag, url attribute not found:%2%") 
                                    % line() % node->id).str()); 
        } 
        string url = reader->getAttributeValue("url");
        url = url.substr(1);

        attach->instanceActuator = url;
        createActuatorRelation(attach); 
    }
    return node;
}
    

DaeNodePtr DaeParserImpl::readBindActuator(DaeNodePtr node)
{
    if (DaeAttachActuator* attach = dynamic_cast<DaeAttachActuator*>(node.get())) {
        if (!reader->getAttributeValue("joint")) {
            throwException(line(), (format("[%1%]invalid bind actuator-tag, joint attribute not found:%2%") 
                                    % line() % node->id).str()); 
        } 
        string joint = reader->getAttributeValue("joint");
        attach->bindActuator = joint;
        createActuatorRelation(attach); 
    }
    return node;
}


void DaeParserImpl::createActuatorRelation(DaeAttachActuatorPtr attach)
{
    if (0 < attach->instanceActuator.size() && 0 < attach->bindActuator.size()) {
        pair<DaeActuatorRelations::iterator, bool> pib = arelations.insert(pair<string, string>(attach->bindActuator, 
                                                                                                attach->instanceActuator));
        if (!pib.second) {
            throwException(line(), (format("[%1%]duplicate actuator-relation:%2%") 
                                    % line() % attach->bindActuator).str());
        }
    }
}


DaeNodePtr DaeParserImpl::readInstanceSensor(DaeNodePtr node)
{
    if (DaeAttachSensor* attach = dynamic_cast<DaeAttachSensor*>(node.get())) {
        if (!reader->getAttributeValue("url")) {
            throwException(line(), (format("[%1%]invalid instance sensor-tag, url attribute not found:%2%") 
                                    % line() % node->id).str()); 
        } 
        string url = reader->getAttributeValue("url");
        url = url.substr(1);

        attach->instanceSensor = url;
        createSensorRelation(attach); 
    }
    return node;
}


DaeNodePtr DaeParserImpl::readFrameOrigin(DaeNodePtr node)
{
    if (DaeAttachSensor* attach = dynamic_cast<DaeAttachSensor*>(node.get())) {
        if (!reader->getAttributeValue("link")) {
            throwException(line(), (format("[%1%]invalid frame-origin-tag, link attribute not found:%2%") 
                                    % line() % node->id).str()); 
        } 
        string link = reader->getAttributeValue("link");
        attach->frameOrigin = link;
        createSensorRelation(attach); 
    }
    return node;
}


void DaeParserImpl::createSensorRelation(DaeAttachSensorPtr attach)
{
    if (0 < attach->instanceSensor.size() && 0 < attach->frameOrigin.size()) {
        srelations.push_back(pair<string, string>(attach->frameOrigin, attach->instanceSensor));
    }
}
 

DaeNodePtr DaeParserImpl::readExtra(DaeNodePtr node)
{
    if (reader->getAttributeValue("type")) {
        string type = reader->getAttributeValue("type");
        if (iequals(type, "attach_actuator")) {
            // This will complement the attributes of the joint.
            DaeAttachActuatorPtr value = new DaeAttachActuator;
            return static_cast<DaeNodePtr>(value); 
        } else
            if (iequals(type, "attach_sensor")) {
                // This will complement the attributes of the joint.
                DaeAttachSensorPtr value = new DaeAttachSensor;
                return static_cast<DaeNodePtr>(value); 
            }
    }

    return node;
}


DaeNodePtr DaeParserImpl::readSensor(DaeNodePtr node)
{
    if (!reader->getAttributeValue("id")) {
        throwException(line(), (format("[%1%]invalid sensor-tag, id attribute not found") % line()).str()); 
    }
    DaeSensorPtr sensor = new DaeSensor;
    sensor->id   = reader->getAttributeValue("id");
    sensor->type = reader->getAttributeValue("type") ? reader->getAttributeValue("type") : "" ; 

    pair<DaeSensors::iterator, bool> pib = sensors.insert(pair<string, DaeSensorPtr>(sensor->id, sensor));        
    if (!pib.second) {
        throwException(line(), (format("[%1%]duplicate sensor:%2%") % line() % sensor->id).str()); 
    }

    return static_cast<DaeNodePtr>(sensor);
}


DaeNodePtr DaeParserImpl::readFocalLength(DaeNodePtr node)
{
    if (DaeSensor* sensor = dynamic_cast<DaeSensor*>(node.get())) {
        reader->read();
        sensor->focalLength = lexical_cast<double>(reader->getNodeData());
    }

    return node;
}


DaeNodePtr DaeParserImpl::readIntrinsic(DaeNodePtr node)
{
    if (DaeSensor* sensor = dynamic_cast<DaeSensor*>(node.get())) {
        reader->read();
        string value = reader->getNodeData();

        DaeVectorXArrayPtr source = DaeVectorXArrayPtr(new DaeVectorXArray);
        array(value, source);
        if (source->size() != 6) {
            throwException(line(), (format("[%1%]invalid intrinsic-tag, not 6-dimensions") % line()).str()); 
        }
        sensor->intrinsic << source->at(0), source->at(1), source->at(2),
            source->at(3), source->at(4), source->at(5);
    }

    return node;
}


DaeNodePtr DaeParserImpl::readImageDimensions(DaeNodePtr node)
{
    if (DaeSensor* sensor = dynamic_cast<DaeSensor*>(node.get())) {
        reader->read();
        string value = reader->getNodeData();

        DaeVectorXArrayPtr source = DaeVectorXArrayPtr(new DaeVectorXArray);
        array(value, source);
        if (source->size() != 3) {
            throwException(line(), (format("[%1%]invalid image-dimensions-tag, not 3-dimensions") % line()).str()); 
        }
        sensor->imageDimensions << source->at(0), source->at(1), source->at(2);
    }

    return node;
}


DaeNodePtr DaeParserImpl::readMeasurementTime(DaeNodePtr node)
{
    if (DaeSensor* sensor = dynamic_cast<DaeSensor*>(node.get())) {
        reader->read();
        sensor->measurementTime = lexical_cast<double>(reader->getNodeData());
    }

    return node;
}


bool DaeParserImpl::checkOrder(DaeOrder& order, int type = 0)
{
    DaeOrder::iterator iterv = std::find(order.begin(), order.end(), "vertices");
    DaeOrder::iterator iters = std::find(order.begin(), order.end(), "source");
    DaeOrder::iterator iterp = std::find(order.begin(), order.end(), "position");
    switch (type) {
    case 0: {
        return (iterv != order.end() && iters != order.end() && iterp != order.end());
        break;
    }
    case 1: {
        DaeOrder::iterator itert = std::find(order.begin(), order.end(), "triangles");
        DaeOrder::iterator iterm = std::find(order.begin(), order.end(), "lines");
        DaeOrder::iterator iterl = std::find(order.begin(), order.end(), "polylist");
        return (iterv != order.end() && iters != order.end() && iterp != order.end() && 
                (itert != order.end() || iterm != order.end() || iterl != order.end()));
        break;
    }
    }
    return false;
}


DaeNodePtr DaeParserImpl::readInput(DaeNodePtr node)
{
    string semantic = reader->getAttributeValue("semantic");    
    string source = reader->getAttributeValue("source");
    source = source.substr(1);

    int type = symbolSemantic(semantic);
    switch (type) {
    case EXT_POSITION: {
        meshes.push_back("position"); 
        refPositionId = source;

        pair<DaeVerticesRef::iterator, bool> pia = verticesRef.insert(pair<string, string>(refVerticesId, refPositionId));
        if (!pia.second) {
            throwException(line(), (format("[%1%]duplicate vertices-id:%2%,position-id:%3%") 
                                    % line() % refVerticesId % refPositionId).str());
        }
        break;
    }

    case EXT_VERTEX:
    case EXT_NORMAL:
    case EXT_COLOR:
    case EXT_TEXCOORD:
        int offset = -1;
        if (reader->getAttributeValue("offset")) {
            offset = lexical_cast<int>(reader->getAttributeValue("offset"));
            if (offsetMaximum < offset) {
                offsetMaximum = offset;
            }
            createVertexAndNormal(type, source, offset);
        }
        break;
    }
 
    return node; 
}


void DaeParserImpl::createVertexAndNormal(int type, string& source, int offset)
{
    if (!checkOrder(meshes)) {
        throwException(line(), (format("[%1%]invalid triangle") % line()).str());
    }
    if (!refMesh) {
        throwException(line(), (format("[%1%]invalid mesh structure. Currently, only the lines and polylist and triangles are eligible(geometry-id:%2%)") % line() % refGeometryId).str());
    }
    DaeGeometries::iterator iterv = geometries.find(refGeometryId);
    if (iterv == geometries.end()) {
        throwException(line(), (format("[%1%]invalid geometry:%2%") % line() % refGeometryId).str());
    }
    DaeGeometryPtr geometry = iterv->second;

    // check the valid mesh on geometry
    DaeMeshes::iterator iterm = std::find(geometry->meshes.begin(), geometry->meshes.end(), refMesh);
    if (iterm == geometry->meshes.end()) {
        throwException(line(), (format("[%1%]invalid mesh is invoked. Currently, only the lines and polylist and triangles are eligible(geometry-id:%2%)") % line() % refGeometryId).str());
    }

    switch (type) {
    case EXT_VERTEX: {
        if (!iequals(refVerticesId, source)) {
            source = refGeometryId + "/" + source;
            if (!iequals(refVerticesId, source)) {
                throwException(line(), (format("[%1%]invalid vertices(1):%2%") % line() % refVerticesId).str());
            }
        }
        DaeVectorMap::iterator iter = sources.find(refPositionId);
        if (iter == sources.end()) {
            throwException(line(), (format("[%1%]invalid position:%2%") % line() % refPositionId).str());
        }           
        offsetVertex = offset;
        refMesh->refVerticesId = source;
        refMesh->vertices = iter->second; 
        break;
    }
    case EXT_NORMAL: {
        refNormalsId = source;
        DaeVectorMap::iterator iter = sources.find(refNormalsId);
        if (iter == sources.end()) {
            throwException(line(), (format("[%1%]invalid normals(1):%2%") % line() % refNormalsId).str());
        }            
        offsetNormal = offset;
        refMesh->refNormalsId = source;
        refMesh->normals = iter->second;
        break;
    }
    case EXT_COLOR: {
        refColorId = source;
        DaeVectorMap::iterator iter = sources.find(refColorId);
        if (iter == sources.end()) {
            throwException(line(), (format("[%1%]invalid colors:%2%") % line() % refColorId).str());
        }            
        offsetColor = offset;
        refMesh->refColorId = source;
        refMesh->colors = iter->second;
        break;
    }
    case EXT_TEXCOORD: {
        refTexcoordId = source;
        DaeVectorMap::iterator iter = sources.find(refTexcoordId);
        if (iter == sources.end()) {
            throwException(line(), (format("[%1%]invalid texcoords:%2%") % line() % refTexcoordId).str());
        }            
        offsetTexcoord = offset;
        refMesh->refTexcoordId = source;
        refMesh->texcoords = iter->second;
        break;
    }
    }   
}


DaeNodePtr DaeParserImpl::readTriangles(DaeNodePtr node)
{
    figure = EXT_TRIANGLES;
    createMesh(line());
    meshes.push_back("triangles"); 
    return node; 
}


DaeNodePtr DaeParserImpl::readLines(DaeNodePtr node)
{
    figure = EXT_LINES;
    createMesh(line());
    meshes.push_back("lines"); 
    return node; 
}


void DaeParserImpl::index(string &value, DaeVectorXArrayPtr index, int offset, int offsetMaximum)
{
    if (offset < 0) {
        return;
    }
    trim(value);
    index->clear();
    for (split_iterator<string::iterator> iter  = make_split_iterator(value, token_finder(is_space(), token_compress_on));
         iter != split_iterator<string::iterator>(); 
         iter++)
        {
            if ((boost::copy_range<std::string>(*iter)).size() <= 0) {     
                throwException(line(), (format("[%1%]invalid value:%2%") % line() % boost::copy_range<std::string>(*iter)).str());
            }
            for (int n = 0; n < offset; n++) {
                // It will skip the offset of the first
                iter++;
                if (iter == split_iterator<string::iterator>()) {
                    *os << ((format("[%1%]invalid offset(before token):%2%") % line() % offset).str()) << endl;
                    return;
                }
            }
            index->push_back(lexical_cast<double>(*iter));
            for (int n = 0; n < (offsetMaximum - offset); n++) {
                // It will skip the rest of the offset
                iter++;
                if (iter == split_iterator<string::iterator>()) {
                    return; // it finish
                }
            }
        }

}


DaeNodePtr DaeParserImpl::readP(DaeNodePtr node)
{
    if (!checkOrder(meshes, 1)) {
        throwException(line(), (format("[%1%]invalid p structure(p order on mesh)") % line()).str());
    }
    if (!refMesh) {
        throwException(line(), (format("[%1%]invalid mesh structure. Currently, only the lines and polylist and triangles are eligible(geometry-id:%2%)") % line() % refGeometryId).str());
    }
    DaeGeometries::iterator iter = geometries.find(refGeometryId);
    if (iter == geometries.end()) {
        throwException(line(), (format("[%1%]invalid geometry(geimeotry-id:%2%)") % line() % refGeometryId).str());
    }
    DaeGeometryPtr geometry = iter->second;

    // check the valid mesh on geometry
    DaeMeshes::iterator iterm = std::find(geometry->meshes.begin(), geometry->meshes.end(), refMesh);
    if (iterm == geometry->meshes.end()) {
        throwException(line(), (format("[%1%]invalid mesh is invoked. Currently, only the lines and polylist and triangles are eligible(geometry-id:%2%)") % line() % refGeometryId).str());
    }

    reader->read();
    string value = reader->getNodeData();

    index(value, refMesh->verticesIndexes, offsetVertex, offsetMaximum);
    if (0 <= offsetNormal) {
        index(value, refMesh->normalsIndexes, offsetNormal, offsetMaximum);
    }
    if (0 <= offsetColor) {
        index(value, refMesh->colorsIndexes, offsetColor, offsetMaximum);
    }
    if (0 <= offsetTexcoord) {
        index(value, refMesh->texcoordsIndexes, offsetTexcoord, offsetMaximum);
    }

    return node; 
}


bool DaeParserImpl::checkConcreteNode(DaeNodePtr node)
{
    for (DaeNodes::iterator iter = node->children.begin(); iter != node->children.end();) {
        DaeNodePtr child = iter->second;
        if (0 < child->refNodeId.length()) {
            // It replaced with a copy of the actual node to instance_node
            DaeNodes::iterator itern = nodes.find(child->refNodeId);
            if (itern == nodes.end()) { 
                throwException(line(), (format("[-1]concrete node not found:%1%") % child->refNodeId).str());
            }
            node->children.erase(iter++);
            DaeNodePtr concrete = itern->second->clone(); 
            pair<DaeNodes::iterator, bool> pib = node->children.insert(pair<string, DaeNodePtr>(concrete->id, concrete));
            if (!pib.second) {
                throwException(line(), (format("[%1%]duplicate node:%2%") % line() % concrete->id).str());
            }
        } else {
            iter++;
        }
    }
    for (DaeNodes::iterator iter = node->children.begin(); iter != node->children.end(); iter++) {
        checkConcreteNode(iter->second);
    }
    return true;
}


void DaeParserImpl::createLists(DaeNodePtr node, bool omit = false)
{
    // It will convert to format lists from the tree structure of the node.
    if (!omit) {
        lists.push_back(pair<string, DaeNodePtr>(node->id, node));
    }
    for (DaeNodes::iterator iter = node->children.begin(); iter != node->children.end(); iter++) {
        createLists(iter->second);
    }   
}


void DaeParserImpl::createNodeStack(DaeNodePtr node, DaeNodeStack& stacks, bool omit = false)
{
    // It will create a parent-child relationship of the nodes from the tree structure of the node.
    if (!omit) {
        node->stack = stacks;
        stacks.push_back(pair<string, DaeNodePtr>(node->id, node));
    }
    for (DaeNodes::iterator iter = node->children.begin(); iter != node->children.end(); iter++) {
        createNodeStack(iter->second, stacks);
    }        
    if (!omit) {
        stacks.pop_back();
    }
}


void DaeParserImpl::createStructure()
{
    // It will assign a copy of the node to instance_node.
    checkConcreteNode(library);
    checkConcreteNode(instance);

    // It will create a parent-child relationship of the nodes from the tree structure of the node.    
    DaeNodeStack stacks;
    stacks.clear();
    createNodeStack(instance, stacks, true);

    // It will convert to format lists from the tree structure of the node.
    lists.clear();
    createLists(instance, true);

}


bool DaeParserImpl::checkSafetyNode()
{
    BOOST_FOREACH(NODES_PAIR n, lists) {
        if (0 < n.second->refNodeId.length()) {
            // If there is a instance_node, it is an error.
            throwException(line(), (format("[%1%]invalid instance_node:%2%") 
                                    % line() % n.second->id).str());
        }
        BOOST_FOREACH(GEOMETRY_PAIR g, n.second->geometries) {
        
            DaeGeometryPtr geometry = g.second;
            if (geometry->refMaterialId.length() <= 0) {
                // It will add effect and material with a value of default.
                createEmptyMaterial(geometry);
            }
            DaeEffectPtr effect = findEffect(geometry->refMaterialId);
        }
    }

    return true;
}


void DaeParserImpl::createEmptyMaterial(DaeGeometryPtr geometry)
{
    string uuidEffect   = lexical_cast<string>(random_generator()());
    string uuidMaterial = lexical_cast<string>(random_generator()());

    DaeMaterialPtr newMaterial = new DaeMaterial;
    DaeEffectPtr   newEffect   = new DaeEffect;

    newEffect  ->effectId      = uuidEffect;
    newMaterial->refEffectId   = uuidEffect;
    newMaterial->materialId    = uuidMaterial;
    geometry   ->refMaterialId = uuidMaterial; 

    // New material add
    pair<DaeMaterials::iterator, bool> pibm = materials.insert(pair<string, DaeMaterialPtr>(uuidMaterial, newMaterial));
    if (!pibm.second) {
        throwException(line(), (format("[%1%]duplicate material:%2%") % line() % uuidMaterial).str());
    }

    // New Effect add
    pair<DaeEffects::iterator, bool> pibe = effects.insert(pair<string, DaeEffectPtr>(uuidEffect, newEffect));
    if (!pibe.second) {
        throwException(line(), (format("[%1%]duplicate effect:%2%") % line() % uuidEffect).str());
    }
}


int DaeParserImpl::createStride(string& verticesId)
{
    DaeVerticesRef::iterator iterv = verticesRef.find(verticesId);
    if (iterv == verticesRef.end()) {
        throwException(line(), (format("[%1%]invalid vertices-id:%2%") 
                                % line() % verticesId).str());
    }
    DaeStrides::iterator iters = strides.find(iterv->second);
    if (iters == strides.end()) {
        throwException(line(), (format("[%1%]invalid position-id:%2%") 
                                % line() % iterv->second).str());
    }
    int stride = iters->second;
    return stride;
}


SgGroup* DaeParserImpl::convert()
{
    SgGroup* sgGroup = new SgGroup();

    if (!rootNodePresence) {
        // If you do not have a node, It will return the group empty.
        return sgGroup;
    }

    // If there is a link, and use of the joint and node and link.
    BOOST_FOREACH(NODES_PAIR n, lists) {
        setNode(n.second, sgGroup, true);
    }

    return sgGroup;
}


void DaeParserImpl::setNode(DaeNodePtr extNode, SgGroup* sgGroup, bool createTransform)
{
    // If there is a node, It will create a SgTransform always.
    SgGroup* sgTransParent = NULL; SgGroup* sgTransChild = NULL;
    if (createTransform) {
        // This is calculated by including the transform itself from the root.
        setTransform(extNode, &sgTransParent, &sgTransChild);
    } else {
        // Using the empty transform, if the BodyLoader to use this function.
        // BodyLoader computes the transform by using the setTransform.
        sgTransParent = new SgPosTransform;
        sgTransChild  = new SgPosTransform;
    }
    if (sgTransParent) {
        static_cast<SgObject*>(sgTransParent)->setName(extNode->name);
    }
    sgGroup->addChild(sgTransParent);

    BOOST_FOREACH(GEOMETRY_PAIR g, extNode->geometries) {
        setGeometry(g.second, extNode, sgTransParent);
    }

    // In the case of inline tags, rigid does not refer.
    if (!inlineParse) {
        // create a figure(physics)
        setRigid(extNode, sgGroup, sgTransParent);                    
    }

}


void DaeParserImpl::setRigid(DaeNodePtr extNode, SgGroup* sgGroup, SgGroup* sgTransParent)
{
    try {
        DaeRigidPtr rigid = findRigidBody(extNode->id);
        if (0 < rigid->shapes.size() && sgTransParent->numChildren() != rigid->shapes.size()) {
            throwException(line(), (format("[%1%]invalid number of mesh and rigid:%2%") % line() % extNode->id).str());
        }
        unsigned int i = 0;
        for (DaeShapes::iterator iters = rigid->shapes.begin(); iters != rigid->shapes.end(); iters++) {
            DaeShapePtr extShape = *iters;
            if (extShape->box.presence) {
                static_cast<SgShape*>(sgTransParent->child(i))->setMesh(
                    meshGenerator.generateBox(extShape->box.halfExtents));
            } else
                if (extShape->sphere.presence) {
                    static_cast<SgShape*>(sgTransParent->child(i))->setMesh(
                        meshGenerator.generateSphere(extShape->sphere.radius));
                } else
                    if (extShape->cylinder.presence) {
                        static_cast<SgShape*>(sgTransParent->child(i))->setMesh(
                            meshGenerator.generateCylinder(extShape->cylinder.radius, extShape->cylinder.height));
                    } else
                        if (extShape->cone.presence) {
                            *os << ((format("[%1%]no implementation yet") % line()).str()) << endl;
                            return;
                        }
            i++;
        }

#ifdef _OLD_VERSION
        for (DaeShapes::iterator iters = rigid->shapes.begin(); iters != rigid->shapes.end(); iters++) {
            DaeShapePtr extShape = *iters;
                
            if (extShape->isPresence()) {
                // !!! IMPORTANT !!!
                // I am ignoring the rotation and translation of rigid now.
                // But Please use the following method if you want to use that.
                if (0 < extShape->transform.translates.size()) {
                    extShape->transform.translate = 
                        extShape->transform.translates[extShape->transform.translates.size() - 1];
                }

                // If there is a node, It will create a SgTransform always.
                SgGroup* sgTransParent = NULL; SgGroup* sgTransChild = NULL;
                setTransform(extShape, &sgTransParent, &sgTransChild);

                if (extShape->polygon.presence) {
#ifdef _PRINT_RIGID_POLYGON
                    // In order to become the shape of the multiple, it does not do anything now.
                    setGeometry(extShape->polygon.geometry, extNode, sgTransParent);
                    for (SgGroup::iterator iterg = sgTransParent->begin(); iterg != sgTransParent->end(); iterg++) {
                        if (SgShape* temp = dynamic_cast<SgShape*>((*iterg).get())) {
                            temp->setMaterial(material); // for transparency
                        }
                    }
#endif
                } else {
                    MeshGenerator meshGenerator;
                    SgShapePtr sgShape = SgShapePtr(new SgShape);

                    if (extShape->box.presence) {
                        sgShape->setMesh(meshGenerator.generateBox(extShape->box.halfExtents));
                    } else 
                        if (extShape->sphere.presence) {
                            sgShape->setMesh(meshGenerator.generateSphere(extShape->sphere.radius));
                        } else 
                            if (extShape->cylinder.presence) {
                                sgShape->setMesh(meshGenerator.generateCylinder(extShape->cylinder.radius, 
                                                                                extShape->cylinder.height));
                            } else 
                                if (extShape->cone.presence) {
                                    *os << ((format("[%1%]no implementation yet") % line()).str()) << endl;
                                    return;
                                } else {
                                    throwException(line(), (format("[%1%]invalid figure,node-id:%2%") % line() % extNode->id).str());
                                }
                    SgMaterialPtr sgMaterial = SgMaterialPtr(new SgMaterial);
                    sgMaterial->setTransparency(1.0);
                    sgShape->setMaterial(sgMaterial);

                    sgTransParent->addChild(sgShape);
                }

                sgGroup->addChild(sgTransParent);
            }
        }
#endif

    } catch (...) {
        // In this case, the rigid-body is not handled.
    } 

}


void DaeParserImpl::setGeometry(DaeGeometryPtr extGeometry, DaeNodePtr extNode, SgGroup* sgTransParent)
{
    PolygonMeshTriangulator triangulator;
    triangulator.setDeepCopyEnabled(true);

    // If you have a shape, It will create a SgShape in SgTransform.
    for (DaeMeshes::iterator iterm = extGeometry->meshes.begin(); iterm != extGeometry->meshes.end(); iterm++) {
        // create a new SgObjects.
        SgShape* sgShape = new SgShape;
        // check the count of vertices
        bool polygon = (0 < (*iterm)->vcount->size() ? true : false);
        // create the multi meshes.
        SgMeshBasePtr sgMesh  = (polygon ? static_cast<SgMeshBase*>(new SgPolygonMesh) 
                                 : static_cast<SgMeshBase*>(new SgMesh));
        setMesh(extGeometry, *iterm, sgMesh, polygon);
        sgShape->setMesh(polygon ? triangulator.triangulate(*(static_pointer_cast<SgPolygonMesh>(sgMesh)))
                         : static_pointer_cast<SgMesh>(sgMesh));
        // create a normals
        if (!sgMesh->hasNormals()) {
            MeshNormalGenerator generator;
            generator.generateNormals(static_pointer_cast<SgMesh>(sgMesh), 0.7853981633974483);
        }
        sgMesh->updateBoundingBox();

        SgMaterial* sgMaterial = new SgMaterial;
        DaeEffectPtr tmpEffect;

        // create a new materials by TRIANGLES.
        if ((*iterm)->refMaterialId.size() <= 0) {
            throwException(line(), (format("[%1%]invalid material symbol:%2%") % line() % extGeometry->geometryId).str());
        }

        string symbol = extGeometry->geometryId + "/" + (*iterm)->refMaterialId;
        DaeMaterialRef::iterator itersr = materialRef.find(symbol); 
        if (itersr == materialRef.end()) {
            throwException(line(), (format("[%1%]invalid material symbol:%2%") % line() % symbol).str());
        }
        tmpEffect = findEffect(itersr->second);
        setMaterial(tmpEffect, sgMaterial);                   
        sgShape->setMaterial(sgMaterial);

        if (0 < tmpEffect->refImageId.size()) {
            SgTexture* sgTexture = new SgTexture;
            setTexture(tmpEffect->refImageId, sgTexture);
            sgShape->setTexture(sgTexture); 
        }
 
        sgTransParent->addChild(sgShape);

    }
}


void DaeParserImpl::setMesh(DaeGeometryPtr extGeometry, DaeMeshPtr extMesh, SgMeshBase* sgMesh, bool polygon)
{ 
    if (!sgMesh) {
        throwException(line(), (format("[%1%]invalid sg-mesh created") % line()).str());
    }

    // Read side is responsible, It will generate the recording area of the normal and vertex
    // However there is no need to generate the index
    SgVertexArrayPtr sgvecVertices = new SgVertexArray;
    SgNormalArrayPtr sgvecNormals  = new SgNormalArray;

    sgMesh->setVertices(sgvecVertices);
    sgMesh->setNormals (sgvecNormals);

    setVertices(extGeometry, extMesh, sgMesh);
    setVIndexes(extGeometry, extMesh, sgMesh, polygon);

    setNormals (extGeometry, extMesh, sgMesh);
    setXIndexes(extGeometry, extMesh, extMesh->refNormalsId, "n-index", 3, 
                extMesh->normalsIndexes, sgMesh->normalIndices(), polygon);
 
    if (0 < extMesh->colors->size()) {
        SgColorArrayPtr sgvecColors = new SgColorArray;
        sgMesh->setColors(sgvecColors);

        setColors  (extGeometry, extMesh, sgMesh);
        setXIndexes(extGeometry, extMesh, extMesh->refColorId, "c-index", 3, 
                    extMesh->colorsIndexes, sgMesh->colorIndices(), polygon);
    }

    if (0 < extMesh->texcoords->size()) {
        SgTexCoordArrayPtr sgvecTexcoords = new SgTexCoordArray;
        sgMesh->setTexCoords(sgvecTexcoords);

        setTexcoord(extGeometry, extMesh, sgMesh);
        setXIndexes(extGeometry, extMesh, extMesh->refTexcoordId, "t-index", 2, 
                    extMesh->texcoordsIndexes, sgMesh->texCoordIndices(), polygon);
    }
 
}


void DaeParserImpl::setTexture(string meshImageId, SgTexture* sg)
{
    DaeTextures::iterator iter = textures.find(meshImageId);
    if (iter == textures.end()) {
        throwException(line(), (format("[%1%]invalid image-id:%2%") % line() % meshImageId).str());
    } 
    DaeTexturePtr texture = iter->second;
    sg->getOrCreateImage()->image().load(texture->fileName());
}


void DaeParserImpl::setVertices(DaeGeometryPtr extGeometry, DaeMeshPtr extMesh, SgMeshBase* sgMesh)
{
    int t = 0;
    double value[3];

    if (extMesh->vertices) {
        int tstride = createStride(extMesh->refVerticesId);
        if (3 != tstride) { // only 3-dimension
            throwException(line(), (format("[%1%]It does not correspond to the vertices of the polygon.") % line()).str());
        }

        DaeVectorXArrayPtr vertices = extMesh->vertices;
        //sgMesh->vertices()->resize(extGeometry->vertices->size() / tstride);

        for (unsigned int i = 0; i < vertices->size(); i++) {
            value[t++] = lexical_cast<double>(vertices->at(i));
            if (t == tstride) {
                t = 0;
                // it sums the coefficients of the coordinates. 
                Vector3 result(value[0], value[1], value[2]);
                sgMesh->vertices()->push_back((unit * result).cast<float>());
            }
        }
    } else {
        throwException(line(), (format("[%1%]invalid vertices(2):%2%") % line() % extGeometry->geometryId).str());
    }
}


void DaeParserImpl::setNormals(DaeGeometryPtr extGeometry, DaeMeshPtr extMesh, SgMeshBase* sgMesh)
{
    int t = 0;
    double value[3];

    if (extMesh->normals) {
        DaeStrides::iterator iter = strides.find(extMesh->refNormalsId);
        if (iter == strides.end()) {
            return;
        }
        int tstride = iter->second;
        if (3 != tstride) { // only 3-dimension
            throwException(line(), (format("[%1%]It does not correspond to the normal of the polygon.") % line()).str());
        }

        DaeVectorXArrayPtr normals = extMesh->normals;
        //sgMesh->normals()->resize(normals->size() / tstride);

        for (unsigned int i = 0; i < normals->size(); i++) {
            value[t++] = lexical_cast<double>(normals->at(i));       
            if (t == tstride) {
                t = 0;
                Vector3f result(value[0], value[1], value[2]);
                sgMesh->normals()->push_back(result);
            }
        }
    } else {
        throwException(line(), (format("[%1%]invalid normals(2):%2%") % line() % extMesh->refNormalsId).str());
    }
}


void DaeParserImpl::setTexcoord(DaeGeometryPtr extGeometry, DaeMeshPtr extMesh, SgMeshBase* sgMesh)
{
    int t = 0;
    double value[2];
 
    if (extMesh->texcoords) {
        DaeStrides::iterator iter = strides.find(extMesh->refTexcoordId);
        if (iter == strides.end()) {
            return;
        }
        int tstride = iter->second;
        if (2 != tstride) { // only S and T
            throwException(line(), (format("[%1%]It does not correspond to the texcoord of the polygon.") 
                                    % line()).str());
        }

        DaeVectorXArrayPtr texcoords = extMesh->texcoords;
        //sgMesh->texCoord()->resize(texcoords->size() / tstride);

        for (unsigned int i = 0; i < texcoords->size(); i++) {
            value[t++] = lexical_cast<double>(texcoords->at(i));
            if (t == tstride) {
                t = 0;
                // !!! important !!!
                // Direction of y is reversed the coordinates of the image coordinates and OpenGL.
                // Then, the coordinate system of the blender is the forward. Please be careful.
                Vector2f result(value[0], value[1] * -1.0);
                sgMesh->texCoords()->push_back(result);
            }
        }
    } else {
        throwException(line(), (format("[%1%]invalid texcoord") % line()).str());
    }
}  


void DaeParserImpl::setColors(DaeGeometryPtr extGeometry, DaeMeshPtr extMesh, SgMeshBase* sgMesh)
{
    int t = 0;
    double value[3];
    if (extMesh->colors) {
        DaeStrides::iterator iter = strides.find(extMesh->refColorId);
        if (iter == strides.end()) {
            return;
        }
        int tstride = iter->second;
        if (3 != tstride) { // Three primary colors
            throwException(line(), (format("[%1%]It does not correspond to the colors of the polygon.") 
                                    % line()).str());
        }

        DaeVectorXArrayPtr colors = extMesh->colors;
        //sgMesh->colors()->resize(colors->size() / tstride);

        for (unsigned int i = 0; i < colors->size(); i++) {
            value[t++] = lexical_cast<double>(colors->at(i));
            if (t == tstride) {
                t = 0;
                Vector3f result(value[0], value[1], value[2]);
                sgMesh->colors()->push_back(result);
            }
        }
    } else {
        throwException(line(), (format("[%1%]invalid colors") % line()).str());
    }
} 


void DaeParserImpl::setVIndexes(DaeGeometryPtr extGeometry, DaeMeshPtr extMesh, SgMeshBase* sgMesh, bool polygon)
{
    unsigned int t = 0, v = 0, vcount = 0;
    vector<double> value;

    if (extMesh->verticesIndexes) {
        DaeVectorXArrayPtr vIndexes = extMesh->verticesIndexes;
        for (unsigned int i = 0; i < vIndexes->size(); i++) {
            if (t == 0) {
                vcount = (polygon ? extMesh->vcount->at(v) : 3);
                value.resize(vcount);
            }
            value[t++] = lexical_cast<double>(vIndexes->at(i));

            if (t == vcount) {
                t = 0; v++; 
                if (polygon) {
                    for (unsigned int j = 0; j < vcount; j++) {
                        static_cast<SgPolygonMesh*>(sgMesh)->polygonVertices().push_back(value[j]);
                    }
                    static_cast<SgPolygonMesh*>(sgMesh)->polygonVertices().push_back(-1);
                } else {
                    for (unsigned int j = 0; j < vcount; j++) {
                        static_cast<SgMesh*>(sgMesh)->triangleVertices().push_back(value[j]);
                    }
                }
            }
        }
    } else {
        throwException(line(), (format("[%1%]invalid v-index") % line()).str());
    }
}


void DaeParserImpl::setXIndexes(DaeGeometryPtr ext, DaeMeshPtr extMesh, string id, string text, int stride,
                                DaeVectorXArrayPtr extArray, SgIndexArray& sgArray, bool polygon)
{
    unsigned int t = 0, v = 0, vcount = 0;
    vector<double> value;

    if (extArray) {
        DaeStrides::iterator iter = strides.find(id);
        if (iter == strides.end()) {
            return;
            // no normals on dae file.
        }
        int tstride = iter->second;
        if (stride != tstride) { // only 3-dimension
            throwException(line(), (format("[%1%]It does not correspond to the %2% of the polygon.") 
                                    % line() % text).str());
        }

        DaeVectorXArrayPtr indexes = extArray;
        for (unsigned int i = 0; i < indexes->size(); i++) {
            if (t == 0) {
                vcount = (polygon ? extMesh->vcount->at(v) : 3);
                value.resize(vcount);
            }
            value[t++] = lexical_cast<double>(indexes->at(i));       
            if (t == vcount) {
                t = 0; v++;
                for (unsigned int j = 0; j < vcount; j++) {
                    sgArray.push_back(value[j]);
                }
                if (polygon) {
                    sgArray.push_back(-1);
                }
            }
        }
    } else {
        throwException(line(), (format("[%1%]invalid %2%") % line() % text).str());
    }
 
}


DaeEffectPtr DaeParserImpl::findEffect(string& geoMaterialId)
{
    DaeMaterials::iterator iterm = materials.find(geoMaterialId);
    if (iterm == materials.end()) {
        throwException(line(), (format("[-1]invalid material:%1%") % geoMaterialId).str());
    }
    DaeMaterialPtr extMaterial = iterm->second; 
    if (extMaterial->refEffectId.length() <= 0) {
        throwException(line(), (format("[-1]invalid ref-effect:%1%") % extMaterial->refEffectId).str());
    }

    DaeEffects::iterator itere = effects.find(extMaterial->refEffectId);
    if (itere == effects.end()) {
        throwException(line(), (format("[-1]invalid effect:%1%") % extMaterial->refEffectId).str());
    } 
    DaeEffectPtr extEffect = itere->second;

    return extEffect;
}


void DaeParserImpl::setMaterial(DaeEffectPtr extEffect, SgMaterial* sgMaterial)
{
    sgMaterial->setEmissiveColor   (extEffect->emission);
    sgMaterial->setAmbientIntensity(extEffect->ambient);
    sgMaterial->setDiffuseColor    (extEffect->diffuse);
    sgMaterial->setSpecularColor   (extEffect->specular);
    sgMaterial->setShininess       (extEffect->shininess);
    sgMaterial->setTransparency    (extEffect->transparency);

}


void DaeParserImpl::setTransform(DaeNodePtr extNode, SgGroup** sgParent, SgGroup** sgChild)
{
    if (extNode->transform.affine) {
        throwException(line(), (format("[-1][WARNING]duplicate transform:%1%, previous transform is used.") 
                                % extNode->id).str());
    }
 
    static const Vector3 vecZero(0.0, 0.0, 0.0);
    Translation3 C(vecZero);
    Translation3 T(extNode->transform.translate);

    // It use the unit of distance
    extNode->transform.translate *= unit;

    const AngleAxis& SR = AngleAxis(0.0, Vector3(0, 0, 0));
    const AngleAxis&  R = extNode->transform.rotate;

    Affine3 S, A, U, B, K, TS;
    S.linear() = extNode->transform.scale.asDiagonal();
    S.translation().setZero();

    // It set to transform itself, which is defined for each node.
    extNode->transform.affine = new Affine3(T * C * R * SR * S * SR.inverse() * C.inverse());        

    // If the matrix is defined, It will use that value.
    if (extNode->transform.matrix) {
        U.matrix() = *(extNode->transform.matrix);
        U.translation() *= unit;
        extNode->transform.affine->matrix() = extNode->transform.affine->matrix() * U.matrix();
    }

    // It sums of the transform from the "ALL" parent node.
    for (DaeNodeStack::iterator iter = extNode->stack.begin(); iter != extNode->stack.end(); iter++) {
        if (!iter->second->transform.affine) {
            Translation3     TT(iter->second->transform.translate);
            const AngleAxis& TR = iter->second->transform.rotate;
            TS.linear() = iter->second->transform.scale.asDiagonal();
            TS.translation().setZero();
            iter->second->transform.affine = new Affine3(TT * C * TR * SR * TS * SR.inverse() * C.inverse());        
        }
        A = ( (iter == extNode->stack.begin()) ? (     *(iter->second->transform.affine)) 
              : (A * (*(iter->second->transform.affine))) );
    }
    // It will hold the transform to their from root.
#ifdef _INVALID_VERSION
    if (0 < extNode->stack.size()) {
        extNode->transform.parents = A;
    }
#endif

    // It isolated on a scale and rotation by transformation.
    Matrix3 retTranslation, retScaling;
    B.matrix() = extNode->transform.affine->matrix();
    B.computeScalingRotation(&retTranslation, &retScaling);

    SgPosTransform* sgTransParent = new SgPosTransform;

    if ((retScaling.diagonal()[0] - 1.0) < DBL_EPSILON && 
        (retScaling.diagonal()[1] - 1.0) < DBL_EPSILON &&
        (retScaling.diagonal()[2] - 1.0) < DBL_EPSILON) {

        // It hold the transform of itself considering parent node.
        K = (0 < extNode->stack.size()) ? (A * (*(extNode->transform.affine))) 
            :      (*(extNode->transform.affine));

        // This matrix does not include the scale.
        // It hold the transform of itself considering parent node.
        sgTransParent->setTransform(K);

        *sgParent = sgTransParent;
        *sgChild  = sgTransParent;

    } else {
        // There is no direction of scale to Collada.
        SgScaleTransform* scale = new SgScaleTransform;
        scale->setScale(retScaling.diagonal());
        sgTransParent->addChild(scale);

        // It hold the transform of itself considering parent node.
        K = (0 < extNode->stack.size()) ? (A * retTranslation) 
            : (    retTranslation);

        // Returns a separate scale.
        // It hold the transform of itself considering parent node.
        sgTransParent->setTransform(K);

        // Collada has no scale-orientation and scale-center. 
        // Then SR and C has all zero.
        SgPosTransform* sgTransChild  = new SgPosTransform;
        sgTransChild->setTransform(SR.inverse() * C.inverse());
        scale->addChild(sgTransChild);    

        *sgParent = sgTransParent;
        *sgChild  = sgTransChild;
    }

}


DaeNode* DaeParserImpl::findNode(const string& nodeName)
{
    // node and joint and link to define the relationship in the attribute of the "name". 
    DaeNodes::iterator iter = nodeNames.find(nodeName);
    if (iter == nodeNames.end()) {
        throwException(line(), (format("[-1]invalid nodeName on nodes:%1%") % nodeName).str());
    }
    return iter->second.get();
}

    
DaeNode* DaeParserImpl::findLinkByJoint(const string& jointName)
{
    // node and joint and link to define the relationship in the attribute of the "name". 
    DaeLinks::iterator iter = linkNames.find(jointName);
    if (iter == linkNames.end()) {
        throwException(line(), (format("[-1]invalid jointName on links:%1%") % jointName).str());
    }
    return iter->second.get();
}


DaeNode* DaeParserImpl::findJointByLink(const string& linkName)
{
    DaeJoints::iterator iter = joints.find(linkName);
    if (iter == joints.end()) {
        throwException(line(), (format("[-1]invalid linkName on joints:%1%") % linkName).str());
    }
    return iter->second.get();
}


DaeNode* DaeParserImpl::findRigidByLink(const string& linkName)
{
    DaeRigids::iterator iter = rigidNames.find(linkName);
    if (iter == rigidNames.end()) {
        throwException(line(), (format("[-1]invalid linkName on rigids:%1%") % linkName).str());
    }
    return iter->second.get();
}


DaeNode* DaeParserImpl::findActuator(const string& jointId)
{
    DaeActuatorRelations::iterator iterr = arelations.find(jointId);
    if (iterr == arelations.end()) {
        throwException(line(), (format("[-1]invalid joint-id on actuator-relations:%1%") % jointId).str());
    }
    DaeActuators::iterator itera = actuators.find(iterr->second);
    if (itera == actuators.end()) {
        throwException(line(), (format("[-1]invalid actuator-id on actuator:%1%,%2%") % itera->second % jointId).str());
    }
    return static_cast<DaeNode*>(itera->second.get());
}


DaeResultSensors* DaeParserImpl::findSensor(const string& linkId)
{
    sensorResults = DaeResultSensorsPtr(new DaeResultSensors);

    for (DaeSensorRelations::iterator iters = srelations.begin(); iters != srelations.end(); iters++) {
        if (iequals(iters->first, linkId)) {
            DaeSensors::iterator iterr = sensors.find(iters->second);
            if (iterr != sensors.end()) {
                sensorResults->push_back(iterr->second.get());
            }
        }
    }
    return sensorResults.get();
}

};
