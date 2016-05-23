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
#include <cnoid/VRMLParser>
#include <cnoid/EasyScanner>
#include <cnoid/VRMLToSGConverter>
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
    filesystem::path directoryPath;
    
    Body* body;

    struct LinkInfo : public Referenced
    {
        LinkPtr link;
        MappingPtr node;
        std::string parent;
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
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
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
    Vector3 v;
    Matrix3 M;

    dynamic_bitset<> validJointIdSet;
    int numValidJointIds;

    MeshGenerator meshGenerator;

    SgMaterialPtr defaultMaterial;

    int divisionNumber;
    ostream* os_;
    bool isVerbose;

    VRMLParser vrmlParser;
    VRMLToSGConverter sgConverter;

    ostream& os() { return *os_; }

    YAMLBodyLoaderImpl();
    ~YAMLBodyLoaderImpl();
    bool load(Body* body, const std::string& filename);
    bool readTopNode(Body* body, Mapping* topNode);
    void setDefaultDivisionNumber(int n);
    bool readBody(Mapping* topNode);
    LinkPtr readLink(Mapping* linkNode);
    void setMassParameters(Link* link);
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
    void setDefaultMaterial(SgShape* shape);
};

}

namespace {

typedef void (YAMLBodyLoaderImpl::*NodeTypeFunction)(Mapping& node);
typedef map<string, NodeTypeFunction> NodeTypeFunctionMap;
NodeTypeFunctionMap nodeTypeFuncs;

typedef SgNodePtr (YAMLBodyLoaderImpl::*SceneNodeFunction)(Mapping& node);
typedef map<string, SceneNodeFunction> SceneNodeFunctionMap;
SceneNodeFunctionMap sceneNodeFuncs;


template<typename ValueType>
bool extract(Mapping* mapping, const char* key, ValueType& out_value)
{
    ValueNodePtr node = mapping->extract(key);
    if(node){
        out_value = node->to<ValueType>();
        return true;
    }
    return false;
}


template<typename Derived>
bool extractEigen(Mapping* mapping, const char* key, Eigen::MatrixBase<Derived>& x)
{
    ListingPtr listing = dynamic_pointer_cast<Listing>(mapping->extract(key));
    if(listing){
        read(*listing, x);
        return true;
    }
    return false;
}


// for debug
void putLinkInfoValues(Body* body, ostream& os)
{
    for(int i=0; i < body->numLinks(); ++i){
        Link* link = body->link(i);
        os << "link \"" << link->name() << "\"\n";
        Mapping* info = link->info();
        Mapping::iterator p = info->begin();
        while(p != info->end()){
            if(p->second->isScalar()){
                os << " " << p->first << ": " << p->second->toString() << "\n";
            }
            ++p;
        }
        os.flush();
    }
}

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
    sgConverter.setDivisionNumber(divisionNumber);
}


bool YAMLBodyLoader::load(Body* body, const std::string& filename)
{
    return impl->load(body, filename);
}


bool YAMLBodyLoaderImpl::load(Body* body, const std::string& filename)
{
    filesystem::path filepath(filename);
    directoryPath = filepath.parent_path();

    bool result = false;

    MappingPtr data;
    
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
    } catch(EasyScanner::Exception & ex){
        os() << ex.getFullMessage();
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
    defaultMaterial = 0;

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

    if(false){ // for debug
        putLinkInfoValues(body, os());
    }

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
        LinkInfo* info = new LinkInfo;
        extract(linkNode, "parent", info->parent);
        info->link = readLink(linkNode);
        info->node = linkNode;
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
    MappingPtr info = static_cast<Mapping*>(linkNode->clone());
    
    LinkPtr link = body->createLink();

    ValueNodePtr nameNode = info->extract("name");
    if(nameNode){
        link->setName(nameNode->toString());
        if(!linkMap.insert(make_pair(link->name(), link)).second){
            nameNode->throwException(
                str(format(_("Duplicated link name \"%1%\"")) % link->name()));
        }
    }

    if(extractEigen(info, "translation", v)){
        link->setOffsetTranslation(v);
    }
    Vector4 r;
    if(extractEigen(info, "rotation", r)){
        link->setOffsetRotation(AngleAxis(r[3], Vector3(r[0], r[1], r[2])));
    }
    
    if(extract(info, "jointId", id)){
        link->setJointId(id);
        if(id >= 0){
            if(id >= validJointIdSet.size()){
                validJointIdSet.resize(id + 1);
            }
            if(!validJointIdSet[id]){
                ++numValidJointIds;
                validJointIdSet.set(id);
            } else {
                os() << str(format("Warning: Joint ID %1% of %2% is duplicated.")
                            % id % link->name()) << endl;
            }
        }
    }
    
    string jointType;
    if(extract(info, "jointType", jointType)){
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

    ValueNodePtr jointAxisNode = info->find("jointAxis");
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
    
    currentLink = link;
    rigidBodies.clear();
    currentSceneGroup = 0;
    //SgGroupPtr shape = new SgGroup;
    SgGroupPtr shape = new SgInvariantGroup;
    ValueNodePtr elements = linkNode->extract("elements");
    if(elements){
        readElements(*elements, shape);
    }

    RigidBody rbody;
    if(!extractEigen(info, "centerOfMass", rbody.c)){
        rbody.c.setZero();
    }
    if(!extract(info, "mass", rbody.m)){
        rbody.m = 0.0;
    }
    if(!extractEigen(info, "inertia", rbody.I)){
        rbody.I.setZero();
    }
    rigidBodies.push_back(rbody);
    setMassParameters(link);
    
    if(extract(linkNode, "shapeFile", symbol)){
        filesystem::path filepath(symbol);
        if(!checkAbsolute(filepath)){
            filepath = directoryPath / filepath;
            filepath.normalize();
        }
        vrmlParser.load(getAbsolutePathString(filepath));
        while(VRMLNodePtr vrmlNode = vrmlParser.readNode()){
            SgNodePtr node = sgConverter.convert(vrmlNode);
            if(node){
                shape->addChild(node);
            }
        }
    }

    // \todo remove redundant group nodes here

    if(!shape->empty()){
        link->setShape(shape);
    }

    ValueNode* import = linkNode->find("import");
    if(import->isValid()){
        if(import->isMapping()){
            info->insert(import->toMapping());
        } else if(import->isListing()){
            Listing& importList = *import->toListing();
            for(int i=importList.size() - 1; i >= 0; --i){
                info->insert(importList[i].toMapping());
            }
        }
    }
    link->resetInfo(info);

    currentLink = 0;

    return link;
}


void YAMLBodyLoaderImpl::setMassParameters(Link* link)
{
    /*
      Mass = Sigma mass 
      C = (Sigma mass * T * c) / Mass 
      I = Sigma(R * I * Rt + m * G)       
      R = Rotation matrix part of T   
      G = y*y+z*z, -x*y, -x*z, -y*x, z*z+x*x, -y*z, -z*x, -z*y, x*x+y*y    
      (x, y, z ) = T * c - C
    */
    
    Vector3 c = Vector3::Zero();
    double m = 0.0;
    Matrix3 I = Matrix3::Zero();

    for(size_t i=0; i < rigidBodies.size(); ++i){
        const RigidBody& r = rigidBodies[i];
        m += r.m;
        c += r.m * r.c;
    }
    if(m > 0.0){
        c = c / m;

        for(size_t i=0; i < rigidBodies.size(); ++i){
            const RigidBody& r = rigidBodies[i];
            I += r.I;
            const Vector3 o = c - r.c;
            const double x = o.x();
            const double y = o.y();
            const double z = o.z();
            const double m = r.m;
            I(0,0) +=  m * (y * y + z * z);
            I(0,1) += -m * (x * y);
            I(0,2) += -m * (x * z);
            I(1,0) += -m * (y * x);
            I(1,1) +=  m * (z * z + x * x);
            I(1,2) += -m * (y * z);
            I(2,0) += -m * (z * x);
            I(2,1) += -m * (z * y);
            I(2,2) +=  m * (x * x + y * y);
        }
    }

    link->setCenterOfMass(c);
    link->setMass(m);
    link->setInertia(I);
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
                        str(format(_("The node type \"%1%\" is different from the type \"%2%\" specified in the parent node"))
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

    if(read(node, "translation", v)){
        T.translation() = v;
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
    const Affine3& T = transformStack.back();

    if(read(node, "centerOfMass", v)){
        rbody.c = T.linear() * v + T.translation();
    } else {
        rbody.c.setZero();
    }
    if(!node.read("mass", rbody.m)){
        rbody.m = 0.0;
    }
    if(read(node, "inertia", M)){
        rbody.I = T.linear() * M * T.linear().transpose();
    } else {
        rbody.I.setZero();
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
    if(read(node, "translation", v)) device->setLocalTranslation(v);

    Vector4 r;
    if(read(node, "rotation", r)){
        device->setLocalRotation(Matrix3(AngleAxis(r[3], Vector3(r[0], r[1], r[2]))));
    }
}


void YAMLBodyLoaderImpl::readForceSensor(Mapping& node)
{
    ForceSensorPtr sensor = new ForceSensor;
    readDeviceCommonParameters(sensor, node);
    if(read(node, "maxForce",  v)) sensor->F_max().head<3>() = v;
    if(read(node, "maxTorque", v)) sensor->F_max().tail<3>() = v;
    addDevice(sensor);
}


void YAMLBodyLoaderImpl::readRateGyroSensor(Mapping& node)
{
    RateGyroSensorPtr sensor = new RateGyroSensor;
    readDeviceCommonParameters(sensor, node);
    if(read(node, "maxAngularVelocity", v)) sensor->w_max() = v;
    addDevice(sensor);
}


void YAMLBodyLoaderImpl::readAccelerationSensor(Mapping& node)
{
    AccelerationSensorPtr sensor = new AccelerationSensor();
    readDeviceCommonParameters(sensor, node);
    if(read(node, "maxAngularVelocity", v)) sensor->dv_max() = v;
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
    if(read(node, "direction", v)) light->setDirection(v);
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
    } else {
        setDefaultMaterial(shape);
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
    } else {
        setDefaultMaterial(shape);
    }
}


void YAMLBodyLoaderImpl::readSceneMaterial(SgShape* shape, Mapping& node)
{
    SgMaterialPtr material = new SgMaterial;

    material->setAmbientIntensity(node.get("ambientIntensity", 0.2));
    if(read(node, "diffuseColor", color)){
        material->setDiffuseColor(color);
    } else {
        material->setDiffuseColor(Vector3f(0.8f, 0.8f, 0.8f));
    }
    if(read(node, "emissiveColor", color)) material->setEmissiveColor(color);
    material->setShininess(node.get("shininess", 0.2));
    if(read(node, "specularColor", color)) material->setSpecularColor(color);
    if(node.read("transparency", value)) material->setTransparency(value);

    shape->setMaterial(material);
}


void YAMLBodyLoaderImpl::setDefaultMaterial(SgShape* shape)
{
    if(!defaultMaterial){
        defaultMaterial = new SgMaterial;
        defaultMaterial->setDiffuseColor(Vector3f(0.8f, 0.8f, 0.8f));
        defaultMaterial->setAmbientIntensity(0.2f);
        defaultMaterial->setShininess(0.2f);
    }
    shape->setMaterial(defaultMaterial);
}

