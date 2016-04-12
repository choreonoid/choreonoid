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
        std::string parent;
        LinkInfo(Link* link) : link(link) { }
    };
    typedef ref_ptr<LinkInfo> LinkInfoPtr;
    
    vector<LinkInfoPtr> linkInfos;

    typedef map<string, LinkPtr> LinkMap;
    LinkMap linkMap;

    LinkPtr currentLink;
    vector<Affine3> transformStack;

    struct RigidBody
    {
        Vector3 c;
        double m;
        Matrix3 I;
    };
    vector<RigidBody> rigidBodies;

    vector<SgGroupPtr> sceneGroupStack;

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
    void setDefaultDivisionNumber(int n);
    bool readBody(Mapping* topNode);
    void readLink(Mapping& linkNode);
    void readElements(Mapping& node, SgGroup* sceneGroup = 0);
    bool readNodeOfType(const string& type, Mapping& node);
    void readGroup(Mapping& node);
    void readTransform(Mapping& node);
    void readRigidBody(Mapping& node);

    void addDevice(Device* device);
    void readDeviceCommonParameters(Device* device, Mapping& node);
    void readForceSensor(Mapping& node);
    void readRateGyroSensor(Mapping& node);
    void readAccelerationSensor(Mapping& node);
    void readCamera(Mapping& node);
    void readRangeSensor(Mapping& node);
    void readSpotLight(Mapping& node);

    SgNode* readSceneShape(Mapping& node);
    void readSceneGeometry(SgShape* shape, Mapping& node);
    void readSceneBox(SgShape* shape, Mapping& node);
    void readSceneCylinder(SgShape* shape, Mapping& node);
    void readSceneSphere(SgShape* shape, Mapping& node);
    
    SgNode* readSceneAppearance(SgShape* shape, Mapping& node);
    SgNode* readSceneMaterial(SgShape* shape, Mapping& node);
};

}

namespace {

typedef void (YAMLBodyLoaderImpl::*NodeTypeFunction)(Mapping& node);
typedef map<string, NodeTypeFunction> NodeTypeFunctionMap;
NodeTypeFunctionMap nodeTypeFuncs;

typedef SgNode* (YAMLBodyLoaderImpl::*SceneNodeFunction)(Mapping& node);
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
        nodeTypeFuncs["Camera"] = &YAMLBodyLoaderImpl::readCamera;
        nodeTypeFuncs["RangeSensor"] = &YAMLBodyLoaderImpl::readRangeSensor;
        nodeTypeFuncs["SpotLight"] = &YAMLBodyLoaderImpl::readSpotLight;

        sceneNodeFuncs["Shape"] = &YAMLBodyLoaderImpl::readSceneShape;

        //sceneNodeFuncs["Appearance"] = &YAMLBodyLoaderImpl::readSceneAppearance;
        //sceneNodeFuncs["Material"] = &YAMLBodyLoaderImpl::readSceneMaterial;
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
    return "Choreonoid-Body-Ver1.0";
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

    this->body = body;

    body->clearDevices();
    body->clearExtraJoints();

    linkInfos.clear();
    linkMap.clear();
    validJointIdSet.clear();
    numValidJointIds = 0;
    
    try {
        YAMLReader reader;
        Mapping* top = reader.loadDocument(filename)->toMapping();

        if(readBody(top)){
            if(body->modelName().empty()){
                body->setModelName(getBasename(filename));
            }
            result = true;
        }
        
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
    sceneGroupStack.clear();
        
    os().flush();

    return result;
}


bool YAMLBodyLoaderImpl::readBody(Mapping* topNode)
{
    double version = topNode->get("bodyFormatVersion", 1.0);
    if(version > 1.0){
        topNode->throwException(_("This version of the body format is not supported."));
    }

    if(topNode->read("name", symbol)){
        body->setModelName(symbol);
    }

    transformStack.clear();
    transformStack.push_back(Affine3::Identity());
    Listing& linkNodes = *topNode->find("links")->toListing();
    for(int i=0; i < linkNodes.size(); ++i){
        Mapping& linkNode = *linkNodes[i].toMapping();
        readLink(linkNode);
    }

    // construct a link tree
    for(size_t i=0; i < linkInfos.size(); ++i){
        LinkInfo* info = linkInfos[i];
        LinkMap::iterator p = linkMap.find(info->parent);
        if(p != linkMap.end()){
            Link* parentLink = p->second;
            parentLink->appendChild(info->link);
        }
    }        

    ValueNode* rootLinkNode = topNode->find("rootLink");
    if(!rootLinkNode->isValid()){
        topNode->throwException(_("There is no \"rootLink\" value for specifying the root link."));
    }

    LinkMap::iterator p = linkMap.find(rootLinkNode->toString());
    if(p == linkMap.end()){
        rootLinkNode->throwException(
            str(format(_("Link \"%1%\" specified in \"rootLink\" is not defined.")) % p->first));
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


void YAMLBodyLoaderImpl::readLink(Mapping& linkNode)
{
    Link* link = body->createLink();
    LinkInfoPtr linkInfo = new LinkInfo(link);

    ValueNode* nameNode = linkNode.find("name");
    if(nameNode->isValid()){
        link->setName(nameNode->toString());
        if(!linkMap.insert(make_pair(link->name(), link)).second){
            nameNode->throwException(
                str(format(_("Duplicated link name \"%1%\"")) % link->name()));
        }
    }
    
    string jointType;
    if(linkNode.read("jointType", jointType)){
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
    
    if(linkNode.read("jointId", id)) link->setJointId(id);
    
    /*
      \todo A link node may contain more than one rigid bodies and
      the paremeters of them must be summed up
    */
    if(linkNode.read("mass", value)) link->setMass(value);
    if(read(linkNode, "centerOfMass", vec3)) link->setCenterOfMass(vec3);

    Matrix3 I;
    if(read(linkNode, "inertia", I)) link->setInertia(I);

    currentLink = link;
    transformStack.resize(1);
    rigidBodies.clear();
    sceneGroupStack.clear();

    readElements(linkNode);

    // set extracted data here

    currentLink = 0;
    
    linkInfos.push_back(linkInfo);
}


void YAMLBodyLoaderImpl::readElements(Mapping& node, SgGroup* sceneGroup)
{
    ValueNode& elements = *node.find("elements");
    if(!elements.isValid()){
        return;
    }

    if(!sceneGroup){
        sceneGroup = new SgGroup;
    }
    if(!sceneGroupStack.empty()){
        sceneGroupStack.back()->addChild(sceneGroup);
    }
    sceneGroupStack.push_back(sceneGroup);
    
    if(elements.isListing()){
        Listing& listing = *node.toListing();
        for(int i=0; i < listing.size(); ++i){
            Mapping& element = *listing[i].toMapping();
            const string type = element["type"].toString();
            if(!readNodeOfType(type, element)){
                element.throwException(
                    str(format(_("The node type \"%1%\" is not defined.")) % type));
            }
        }
    } else if(node.isMapping()){
        Mapping& mapping = *node.toMapping();
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
            if(!readNodeOfType(type, element)){
                mapping.throwException(
                    str(format(_("The node type \"%1%\" is not defined.")) % type));
            }
            ++p;
        }
    }

    sceneGroupStack.pop_back();
}


bool YAMLBodyLoaderImpl::readNodeOfType(const string& type, Mapping& node)
{
    NodeTypeFunctionMap::iterator p = nodeTypeFuncs.find(type);
    if(p != nodeTypeFuncs.end()){
        NodeTypeFunction readNode = p->second;
        (this->*readNode)(node);
    } else {
        SceneNodeFunctionMap::iterator q = sceneNodeFuncs.find(type);
        if(q != sceneNodeFuncs.end()){
            SceneNodeFunction readSceneNode = q->second;
            SgNode* scene = (this->*readSceneNode)(node);
            if(scene){
                sceneGroupStack.back()->addChild(scene);
            }
        } else {
            // The type is not found.
            return false;
        }
    }
    return true;
}
        
   
void YAMLBodyLoaderImpl::readGroup(Mapping& node)
{
    readElements(node);
}


void YAMLBodyLoaderImpl::readTransform(Mapping& node)
{
    Affine3 T = Affine3::Identity();

    if(read(node, "translation", vec3)) T.translation() = vec3;

    Vector4 r;
    if(read(node, "rotation", r)){
        T.linear() = Matrix3(AngleAxis(r[3], Vector3(r[0], r[1], r[2])));
    }

    transformStack.push_back(transformStack.back() * T);

    readElements(node, new SgPosTransform(T));

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

    readElements(node);
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


void YAMLBodyLoaderImpl::readCamera(Mapping& node)
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


SgNode* YAMLBodyLoaderImpl::readSceneShape(Mapping& node)
{
    SgShapePtr shape = new SgShape;

    Mapping& geometry = *node.findMapping("geometry");
    if(geometry.isValid()){
        readSceneGeometry(shape, geometry);
    }

    Mapping& appearance = *node.findMapping("appearance");
    if(appearance.isValid()){
        readSceneAppearance(shape, appearance);
    }

    return shape;
}


void YAMLBodyLoaderImpl::readSceneGeometry(SgShape* shape, Mapping& node)
{
    ValueNode& typeNode = node["type"];
    string type = typeNode.toString();
    if(type == "Box"){
        readSceneBox(shape, node);
    } else if(type == "Cylinder"){
        readSceneCylinder(shape, node);
    } else if(type == "Sphere"){
        readSceneSphere(shape, node);
    } else {
        typeNode.throwException(
            str(format(_("Unknown geometry \"%1%\"")) % type));
    }
}


void YAMLBodyLoaderImpl::readSceneBox(SgShape* shape, Mapping& node)
{
    Vector3 size;
    if(!read(node, "size", size)){
        size.setOnes(1.0);
    }
    shape->setMesh(meshGenerator.generateBox(size));
}


void YAMLBodyLoaderImpl::readSceneCylinder(SgShape* shape, Mapping& node)
{
    double radius = node.get("radius", 1.0);
    double height = node.get("height", 1.0);
    bool bottom = node.get("bottom", true);
    bool side = node.get("side", true);
    shape->setMesh(meshGenerator.generateCylinder(radius, height, bottom, side));
}


void YAMLBodyLoaderImpl::readSceneSphere(SgShape* shape, Mapping& node)
{
    shape->setMesh(meshGenerator.generateSphere(node.get("radius", 1.0)));
}


SgNode* YAMLBodyLoaderImpl::readSceneAppearance(SgShape* shape, Mapping& node)
{
    return 0;
}


SgNode* YAMLBodyLoaderImpl::readSceneMaterial(SgShape* shape, Mapping& node)
{
    return 0;
}
