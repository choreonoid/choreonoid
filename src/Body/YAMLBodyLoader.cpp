/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "YAMLBodyLoader.h"
#include "Body.h"
#include "ForceSensor.h"
#include "RateGyroSensor.h"
#include "AccelerationSensor.h"
#include "Camera.h"
#include "RangeCamera.h"
#include "RangeSensor.h"
#include "PointLight.h"
#include "SpotLight.h"
#include <cnoid/SceneGraph>
#include <cnoid/SceneDrawables>
#include <cnoid/MeshGenerator>
#include <cnoid/EigenArchive>
#include <cnoid/FileUtil>
#include <cnoid/Exception>
#include <cnoid/YAMLReader>
#include <cnoid/NullOut>
#include <Eigen/StdVector>
#include <boost/dynamic_bitset.hpp>
#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;

namespace cnoid {

class YAMLBodyLoaderImpl
{
public:

    YAMLReader reader;
    Body* body;

    class LinkInfo : public Referenced
    {
    public:
        LinkPtr link;
        MappingPtr node;
        std::string parent;
        LinkInfo(LinkPtr link, Mapping* node) : link(link), node(node) { }
    };
    typedef ref_ptr<LinkInfo> LinkInfoPtr;
    
    vector<LinkInfoPtr> linkInfos;

    typedef map<string, LinkPtr> LinkMap;
    LinkMap linkMap;

    LinkPtr currentLink;
    typedef vector<Affine3, Eigen::aligned_allocator<Affine3> > Affine3Vector;
    Affine3Vector transformStack;

    struct RigidBody
    {
        Vector3 c;
        double m;
        Matrix3 I;
    };
    typedef vector<RigidBody, Eigen::aligned_allocator<RigidBody> > RigidBodyVector;
    RigidBodyVector rigidBodies;

    SgGroupPtr currentSceneGroup;

    // temporary variables for reading values
    int id;
    double value;
    string symbol;
    bool on;
    Vector3f color;
    Vector3 vec3;

    dynamic_bitset<> validJointIdSet;
    int numValidJointIds;

    MeshGenerator meshGenerator;

    int divisionNumber;
    ostream* os_;
    bool isVerbose;

    ostream& os() { return *os_; }

    YAMLBodyLoaderImpl();
    ~YAMLBodyLoaderImpl();
    bool load(Body* body, const std::string& filename);
    bool readTopNode(Body* body, Mapping* topNode);
    void setDefaultDivisionNumber(int n);
    bool readBody(Mapping* topNode);
    LinkPtr readLink(Mapping* linkNode);
    void findElements(Mapping& node, SgGroupPtr sceneGroup);
    void readElements(ValueNode& elements, SgGroupPtr sceneGroup);
    void readNode(Mapping& node, const string& type);
    void readGroup(Mapping& node);
    void readTransform(Mapping& node);
    void readRigidBody(Mapping& node);

    void addDevice(Device* device);
    void readDeviceCommonParameters(Device* device, Mapping& node);
    void readForceSensor(Mapping& node);
    void readRateGyroSensor(Mapping& node);
    void readAccelerationSensor(Mapping& node);
    void readCameraDevice(Mapping& node);
    void readRangeSensor(Mapping& node);
    void readSpotLight(Mapping& node);

    SgNodePtr readSceneShape(Mapping& node);
    
    SgMesh* readSceneGeometry(Mapping& node);
    SgMesh* readSceneBox(Mapping& node);
    SgMesh* readSceneCylinder(Mapping& node);
    SgMesh* readSceneSphere(Mapping& node);
    
    void readSceneAppearance(SgShape* shape, Mapping& node);
    void readSceneMaterial(SgShape* shape, Mapping& node);
};

}

namespace {

typedef void (YAMLBodyLoaderImpl::*NodeTypeFunction)(Mapping& node);
typedef map<string, NodeTypeFunction> NodeTypeFunctionMap;
NodeTypeFunctionMap nodeTypeFuncs;

typedef SgNodePtr (YAMLBodyLoaderImpl::*SceneNodeFunction)(Mapping& node);
typedef map<string, SceneNodeFunction> SceneNodeFunctionMap;
SceneNodeFunctionMap sceneNodeFuncs;

}


YAMLBodyLoader::YAMLBodyLoader()
{
    impl = new YAMLBodyLoaderImpl();
}


YAMLBodyLoaderImpl::YAMLBodyLoaderImpl()
{
    isVerbose = false;
    body = 0;
    os_ = &nullout();

    if(nodeTypeFuncs.empty()){
        nodeTypeFuncs["Group"] = &YAMLBodyLoaderImpl::readGroup;
        nodeTypeFuncs["Transform"] = &YAMLBodyLoaderImpl::readTransform;
        nodeTypeFuncs["RigidBody"] = &YAMLBodyLoaderImpl::readRigidBody;
        nodeTypeFuncs["ForceSensor"] = &YAMLBodyLoaderImpl::readForceSensor;
        nodeTypeFuncs["RateGyroSensor"] = &YAMLBodyLoaderImpl::readRateGyroSensor;
        nodeTypeFuncs["AccelerationSensor"] = &YAMLBodyLoaderImpl::readAccelerationSensor;
        nodeTypeFuncs["CameraDevice"] = &YAMLBodyLoaderImpl::readCameraDevice;
        nodeTypeFuncs["RangeSensor"] = &YAMLBodyLoaderImpl::readRangeSensor;
        nodeTypeFuncs["SpotLight"] = &YAMLBodyLoaderImpl::readSpotLight;

        sceneNodeFuncs["Shape"] = &YAMLBodyLoaderImpl::readSceneShape;
    }

    setDefaultDivisionNumber(20);
}


YAMLBodyLoader::~YAMLBodyLoader()
{
    delete impl;
}


YAMLBodyLoaderImpl::~YAMLBodyLoaderImpl()
{

}


const char* YAMLBodyLoader::format() const
{
    return "ChoreonoidBody";
}


void YAMLBodyLoader::setMessageSink(std::ostream& os)
{
    impl->os_ = &os;
}


void YAMLBodyLoader::setVerbose(bool on)
{
    impl->isVerbose = on;
}


void YAMLBodyLoader::enableShapeLoading(bool on)
{

}
    

void YAMLBodyLoader::setDefaultDivisionNumber(int n)
{
    impl->setDefaultDivisionNumber(n);
}


void YAMLBodyLoaderImpl::setDefaultDivisionNumber(int n)
{
    divisionNumber = n;
    meshGenerator.setDivisionNumber(divisionNumber);
}


bool YAMLBodyLoader::load(Body* body, const std::string& filename)
{
    return impl->load(body, filename);
}


bool YAMLBodyLoaderImpl::load(Body* body, const std::string& filename)
{
    bool result = false;

    MappingPtr data = 0;
    
    try {
        YAMLReader reader;
        data = reader.loadDocument(filename)->toMapping();
        if(data){
            result = readTopNode(body, data);
            if(result){
                if(body->modelName().empty()){
                    body->setModelName(getBasename(filename));
                }
            }
        }
    } catch(const ValueNode::Exception& ex){
        os() << ex.message();
    } catch(const nonexistent_key_error& error){
        if(const std::string* message = get_error_info<error_info_message>(error)){
            os() << *message << endl;
        }
    }

    os().flush();

    return result;
}


bool YAMLBodyLoader::read(Body* body, Mapping* topNode)
{
    return impl->readTopNode(body, topNode);
}


bool YAMLBodyLoaderImpl::readTopNode(Body* body, Mapping* topNode)
{
    bool result = false;
    this->body = body;
    body->clearDevices();
    body->clearExtraJoints();

    linkInfos.clear();
    linkMap.clear();
    validJointIdSet.clear();
    numValidJointIds = 0;

    try {
        result = readBody(topNode);
        
    } catch(const ValueNode::Exception& ex){
        os() << ex.message();
    } catch(const nonexistent_key_error& error){
        if(const std::string* message = get_error_info<error_info_message>(error)){
            os() << *message << endl;
        }
    }

    linkInfos.clear();
    linkMap.clear();
    validJointIdSet.clear();
    transformStack.clear();
    rigidBodies.clear();
        
    os().flush();

    return result;
}


bool YAMLBodyLoaderImpl::readBody(Mapping* topNode)
{
    double version = 1.0;
    
    ValueNode* formatNode = topNode->find("format");
    if(formatNode->isValid()){
        if(formatNode->toString() != "ChoreonoidBody"){
            formatNode->throwException(
                _("The file format cannot be loaded as a Choreonoid body model"));
        }
        ValueNode* versionNode = topNode->find("formatVersion");
        if(versionNode->isValid()){
            version = versionNode->toDouble();
        }
    }
    if(version >= 2.0){
        topNode->throwException(_("This version of the Choreonoid body format is not supported."));
    }

    if(topNode->read("name", symbol)){
        body->setModelName(symbol);
    }

    transformStack.clear();
    transformStack.push_back(Affine3::Identity());
    Listing& linkNodes = *topNode->find("links")->toListing();
    for(int i=0; i < linkNodes.size(); ++i){
        Mapping* linkNode = linkNodes[i].toMapping();
        LinkInfo* info = new LinkInfo(readLink(linkNode), linkNode);
        linkNode->read("parent", info->parent);
        linkInfos.push_back(info);
    }

    // construct a link tree
    for(size_t i=0; i < linkInfos.size(); ++i){
        LinkInfo* info = linkInfos[i];
        const std::string& parent = info->parent;
        if(!parent.empty()){
            LinkMap::iterator p = linkMap.find(parent);
            if(p != linkMap.end()){
                Link* parentLink = p->second;
                parentLink->appendChild(info->link);
            } else {
                info->node->throwException(
                    str(format(_("Parent link \"%1%\" of %2% is not defined.")) % parent % info->link->name()));
            }
        }
    }        

    ValueNode* rootLinkNode = topNode->find("rootLink");
    if(!rootLinkNode->isValid()){
        topNode->throwException(_("There is no \"rootLink\" value for specifying the root link."));
    }

    string rootLinkName = rootLinkNode->toString();
    LinkMap::iterator p = linkMap.find(rootLinkName);
    if(p == linkMap.end()){
        rootLinkNode->throwException(
            str(format(_("Link \"%1%\" specified in \"rootLink\" is not defined.")) % rootLinkName));
    }
    Link* rootLink = p->second;
    body->setRootLink(rootLink);

    // Warn empty joint ids
    if(numValidJointIds < validJointIdSet.size()){
        for(size_t i=0; i < validJointIdSet.size(); ++i){
            if(!validJointIdSet[i]){
                os() << str(format("Warning: Joint ID %1% is not specified.") % i) << endl;
            }
        }
    }
        
    body->installCustomizer();

    return true;
}


LinkPtr YAMLBodyLoaderImpl::readLink(Mapping* linkNode)
{
    LinkPtr link = body->createLink();

    ValueNode* nameNode = linkNode->find("name");
    if(nameNode->isValid()){
        link->setName(nameNode->toString());
        if(!linkMap.insert(make_pair(link->name(), link)).second){
            nameNode->throwException(
                str(format(_("Duplicated link name \"%1%\"")) % link->name()));
        }
    }

    if(read(*linkNode, "translation", vec3)){
        link->setOffsetTranslation(vec3);
    }
    Vector4 r;
    if(read(*linkNode, "rotation", r)){
        link->setOffsetRotation(AngleAxis(r[3], Vector3(r[0], r[1], r[2])));
    }
    
    if(linkNode->read("jointId", id)){
        link->setJointId(id);
    }
    
    string jointType;
    if(linkNode->read("jointType", jointType)){
        if(jointType == "revolute"){
            link->setJointType(Link::REVOLUTE_JOINT);
        } else if(jointType == "slide"){
            link->setJointType(Link::SLIDE_JOINT);
        } else if(jointType == "free"){
            link->setJointType(Link::FREE_JOINT);
        } else if(jointType == "fixed"){
            link->setJointType(Link::FIXED_JOINT);
        } else if(jointType == "crawler"){
            link->setJointType(Link::CRAWLER_JOINT);
        }
    }

    ValueNode* jointAxisNode = linkNode->find("jointAxis");
    if(jointAxisNode->isValid()){
        bool isValid = true;
        Vector3 axis;
        if(jointAxisNode->isListing()){
            read(*jointAxisNode->toListing(), axis);
        } else if(jointAxisNode->isString()){
            string label = jointAxisNode->toString();
            if(label == "X"){
                axis = Vector3::UnitX();
            } else if(label == "Y"){
                axis = Vector3::UnitY();
            } else if(label == "Z"){
                axis = Vector3::UnitZ();
            } else {
                isValid = false;
            }
        } else {
            isValid = false;
        }
        if(!isValid){
            jointAxisNode->throwException("Illegal jointAxis value");
        }
        link->setJointAxis(axis);
    }
    
    /*
      \todo A link node may contain more than one rigid bodies and
      the paremeters of them must be summed up
    */
    if(linkNode->read("mass", value)){
        link->setMass(value);
    }
    if(read(*linkNode, "centerOfMass", vec3)){
        link->setCenterOfMass(vec3);
    }

    Matrix3 I;
    if(read(*linkNode, "inertia", I)){
        link->setInertia(I);
    }

    currentLink = link;
    rigidBodies.clear();
    currentSceneGroup = 0;
    SgGroupPtr shape = new SgGroup;
    findElements(*linkNode, shape);

    if(!shape->empty()){
        link->setShape(shape);
    }

    currentLink = 0;

    return link;
}


void YAMLBodyLoaderImpl::findElements(Mapping& node, SgGroupPtr sceneGroup)
{
    ValueNode& elements = *node.find("elements");
    if(!elements.isValid()){
        return;
    }

    readElements(elements, sceneGroup);
}


void YAMLBodyLoaderImpl::readElements(ValueNode& elements, SgGroupPtr sceneGroup)
{
    SgGroupPtr parentSceneGroup = currentSceneGroup;
    currentSceneGroup = sceneGroup;
    
    if(elements.isListing()){
        Listing& listing = *elements.toListing();
        for(int i=0; i < listing.size(); ++i){
            ValueNode& elementNode = listing[i];
            if(elementNode.isListing()){
                readElements(elementNode, new SgGroup);

            } else if(elementNode.isMapping()){
                Mapping& element = *elementNode.toMapping();
                const string type = element["type"].toString();
                readNode(element, type);
            }
        }
    } else if(elements.isMapping()){
        Mapping& mapping = *elements.toMapping();
        Mapping::iterator p = mapping.begin();
        while(p != mapping.end()){
            const string& type = p->first;
            Mapping& element = *p->second->toMapping();
            ValueNode* typeNode = element.find("type");
            if(typeNode->isValid()){
                string type2 = typeNode->toString();
                if(type2 != type){
                    element.throwException(
                        str(format(_("Type node type \"%1%\" is different from the type \"%2%\" specified in the parent node"))
                            % type2 % type));
                }
            }
            readNode(element, type);
            ++p;
        }
    }

    if(parentSceneGroup && !sceneGroup->empty()){
        parentSceneGroup->addChild(sceneGroup);
    }
    currentSceneGroup = parentSceneGroup;
}


void YAMLBodyLoaderImpl::readNode(Mapping& node, const string& type)
{
    NodeTypeFunctionMap::iterator p = nodeTypeFuncs.find(type);
    if(p != nodeTypeFuncs.end()){
        NodeTypeFunction readNode = p->second;
        (this->*readNode)(node);
    } else {
        SceneNodeFunctionMap::iterator q = sceneNodeFuncs.find(type);
        if(q != sceneNodeFuncs.end()){
            SceneNodeFunction readSceneNode = q->second;
            SgNodePtr scene = (this->*readSceneNode)(node);
            if(scene){
                currentSceneGroup->addChild(scene);
            }
        } else {
            node.throwException(str(format(_("The node type \"%1%\" is not defined.")) % type));
        }
    }
}
        
   
void YAMLBodyLoaderImpl::readGroup(Mapping& node)
{
    findElements(node, new SgGroup);
}


void YAMLBodyLoaderImpl::readTransform(Mapping& node)
{
    Affine3 T = Affine3::Identity();

    if(read(node, "translation", vec3)){
        T.translation() = vec3;
    }
    Vector4 r;
    if(read(node, "rotation", r)){
        T.linear() = Matrix3(AngleAxis(r[3], Vector3(r[0], r[1], r[2])));
    }

    transformStack.push_back(transformStack.back() * T);

    findElements(node, new SgPosTransform(T));

    transformStack.pop_back();
}


void YAMLBodyLoaderImpl::readRigidBody(Mapping& node)
{
    RigidBody rbody;
    if(!read(node, "centerOfMass", rbody.c)){
        rbody.c.setZero();
    }
    if(!node.read("mass", rbody.m)){
        rbody.m = 0.0;
    }
    if(!read(node, "inertia", rbody.I)){
        rbody.I.setIdentity();
    }
    rigidBodies.push_back(rbody);

    findElements(node, new SgGroup);
}


void YAMLBodyLoaderImpl::addDevice(Device* device)
{
    device->setLink(currentLink);
    const Matrix3 RsT = currentLink->Rs();
    const Affine3& T = transformStack.back();
    device->setLocalTranslation(RsT * (T * device->localTranslation()));
    device->setLocalRotation(RsT * (T.linear() * device->localRotation()));
    body->addDevice(device);
}
    

void YAMLBodyLoaderImpl::readDeviceCommonParameters(Device* device, Mapping& node)
{
    if(node.read("name", symbol)) device->setName(symbol);
    if(node.read("id", id)) device->setId(id);
    if(read(node, "translation", vec3)) device->setLocalTranslation(vec3);

    Vector4 r;
    if(read(node, "rotation", r)){
        device->setLocalRotation(Matrix3(AngleAxis(r[3], Vector3(r[0], r[1], r[2]))));
    }
}


void YAMLBodyLoaderImpl::readForceSensor(Mapping& node)
{
    ForceSensorPtr sensor = new ForceSensor;
    readDeviceCommonParameters(sensor, node);
    if(read(node, "maxForce",  vec3)) sensor->F_max().head<3>() = vec3;
    if(read(node, "maxTorque", vec3)) sensor->F_max().tail<3>() = vec3;
    addDevice(sensor);
}


void YAMLBodyLoaderImpl::readRateGyroSensor(Mapping& node)
{
    RateGyroSensorPtr sensor = new RateGyroSensor;
    readDeviceCommonParameters(sensor, node);
    if(read(node, "maxAngularVelocity", vec3)) sensor->w_max() = vec3;
    addDevice(sensor);
}


void YAMLBodyLoaderImpl::readAccelerationSensor(Mapping& node)
{
    AccelerationSensorPtr sensor = new AccelerationSensor();
    readDeviceCommonParameters(sensor, node);
    if(read(node, "maxAngularVelocity", vec3)) sensor->dv_max() = vec3;
    addDevice(sensor);
}


void YAMLBodyLoaderImpl::readCameraDevice(Mapping& node)
{
    CameraPtr camera;
    RangeCamera* range = 0;

    string format;
    if(node.read("format", format)){
        if(format == "COLOR"){
            camera = new Camera;
            camera->setImageType(Camera::COLOR_IMAGE);
        } else if(format == "DEPTH"){
            range = new RangeCamera;
            range->setOrganized(true);
            range->setImageType(Camera::NO_IMAGE);
        } else if(format == "COLOR_DEPTH"){
            range = new RangeCamera;
            range->setOrganized(true);
            range->setImageType(Camera::COLOR_IMAGE);
        } else if(format == "POINT_CLOUD"){
            range = new RangeCamera;
            range->setOrganized(false);
            range->setImageType(Camera::NO_IMAGE);
        } else if(format == "COLOR_POINT_CLOUD"){
            range = new RangeCamera;
            range->setOrganized(false);
            range->setImageType(Camera::COLOR_IMAGE);
        }
    }

    if(!camera){
        if(range){
            camera = range;
        } else {
            camera = new Camera;
        }
    }
        
    readDeviceCommonParameters(camera, node);
    
    if(node.read("on", on)) camera->on(on);
    if(node.read("width", value)) camera->setResolutionX(value);
    if(node.read("height", value)) camera->setResolutionY(value);
    if(node.read("fieldOfView", value)) camera->setFieldOfView(value);
    if(node.read("frontClipDistance", value)) camera->setNearDistance(value);
    if(node.read("backClipDistance", value)) camera->setFarDistance(value);
    if(node.read("frameRate", value)) camera->setFrameRate(value);
    
    addDevice(camera);
}


void YAMLBodyLoaderImpl::readRangeSensor(Mapping& node)
{
    RangeSensorPtr rangeSensor = new RangeSensor;
    
    readDeviceCommonParameters(rangeSensor, node);
    
    if(node.read("on", on)) rangeSensor->on(on);
    if(node.read("scanAngle", value)) rangeSensor->setYawRange(value);
    rangeSensor->setPitchRange(0.0);
    if(node.read("scanStep", value)) rangeSensor->setYawResolution(rangeSensor->yawRange() / value);
    if(node.read("minDistance", value)) rangeSensor->setMinDistance(value);
    if(node.read("maxDistance", value)) rangeSensor->setMaxDistance(value);
    if(node.read("scanRate", value)) rangeSensor->setFrameRate(value);
    
    addDevice(rangeSensor);
}


void YAMLBodyLoaderImpl::readSpotLight(Mapping& node)
{
    SpotLightPtr light = new SpotLight();

    readDeviceCommonParameters(light, node);
    
    if(node.read("on", on)) light->on(on);
    if(read(node, "color", color)) light->setColor(color);
    if(node.read("intensity", value)) light->setIntensity(value);
    if(read(node, "direction", vec3)) light->setDirection(vec3);
    if(node.read("beamWidth", value)) light->setBeamWidth(value);
    if(node.read("cutOffAngle", value)) light->setCutOffAngle(value);
    if(read(node, "attenuation", color)){
        light->setConstantAttenuation(color[0]);
        light->setLinearAttenuation(color[1]);
        light->setQuadraticAttenuation(color[2]);
    }

    addDevice(light);
}


SgNodePtr YAMLBodyLoaderImpl::readSceneShape(Mapping& node)
{
    SgShapePtr shape = new SgShape;

    Mapping& geometry = *node.findMapping("geometry");
    if(geometry.isValid()){
        shape->setMesh(readSceneGeometry(geometry));
    }

    Mapping& appearance = *node.findMapping("appearance");
    if(appearance.isValid()){
        readSceneAppearance(shape, appearance);
    }

    return shape;
}


SgMesh* YAMLBodyLoaderImpl::readSceneGeometry(Mapping& node)
{
    SgMesh* mesh;
    ValueNode& typeNode = node["type"];
    string type = typeNode.toString();
    if(type == "Box"){
        mesh = readSceneBox(node);
    } else if(type == "Cylinder"){
        mesh = readSceneCylinder(node);
    } else if(type == "Sphere"){
        mesh = readSceneSphere(node);
    } else {
        typeNode.throwException(
            str(format(_("Unknown geometry \"%1%\"")) % type));
    }
    return mesh;
}


SgMesh* YAMLBodyLoaderImpl::readSceneBox(Mapping& node)
{
    Vector3 size;
    if(!read(node, "size", size)){
        size.setOnes(1.0);
    }
    return meshGenerator.generateBox(size);
}


SgMesh* YAMLBodyLoaderImpl::readSceneCylinder(Mapping& node)
{
    double radius = node.get("radius", 1.0);
    double height = node.get("height", 1.0);
    bool bottom = node.get("bottom", true);
    bool side = node.get("side", true);
    return meshGenerator.generateCylinder(radius, height, bottom, side);
}


SgMesh* YAMLBodyLoaderImpl::readSceneSphere(Mapping& node)
{
    return meshGenerator.generateSphere(node.get("radius", 1.0));
}


void YAMLBodyLoaderImpl::readSceneAppearance(SgShape* shape, Mapping& node)
{
    Mapping& material = *node.findMapping("material");
    if(material.isValid()){
        readSceneMaterial(shape, material);
    }
}


void YAMLBodyLoaderImpl::readSceneMaterial(SgShape* shape, Mapping& node)
{
    SgMaterialPtr material = new SgMaterial;
    Vector3 color;
    if(read(node, "diffuseColor", color)) material->setDiffuseColor(color);

    shape->setMaterial(material);
}
