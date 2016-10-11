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

namespace {

typedef bool (YAMLBodyLoaderImpl::*NodeFunction)(Mapping& node);

struct NodeFunctionInfo
{
    NodeFunction function;
    bool isTransformDerived;
    void set(NodeFunction function, bool isTransformDerived){
        this->function = function;
        this->isTransformDerived = isTransformDerived;
    }
};

typedef map<string, NodeFunctionInfo> NodeFunctionMap;
NodeFunctionMap nodeFunctionMap;

typedef SgNodePtr (YAMLBodyLoaderImpl::*SceneNodeFunction)(Mapping& node);
typedef map<string, SceneNodeFunction> SceneNodeFunctionMap;
SceneNodeFunctionMap sceneNodeFunctionMap;

}

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
    vector<string> nameStack;
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

    bool isDegreeMode;
    bool isVerbose;
    int divisionNumber;
    ostream* os_;

    dynamic_bitset<> validJointIdSet;
    int numValidJointIds;

    MeshGenerator meshGenerator;

    SgMaterialPtr defaultMaterial;

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
    //! \return true if any scene nodes other than Group and Transform are added in the sub tree
    bool readElements(ValueNode& elements, SgGroupPtr& sceneGroup);
    bool readNode(Mapping& node, const string& type);
    bool readContainerNode(Mapping& node, SgGroupPtr& group, NodeFunction nodeFunction);
    bool readTransformNode(Mapping& node, NodeFunction nodeFunction);
    bool readGroup(Mapping& node);
    bool readTransform(Mapping& node);
    bool readRigidBody(Mapping& node);
    bool readDevice(Device* device, Mapping& node);
    bool readForceSensor(Mapping& node);
    bool readRateGyroSensor(Mapping& node);
    bool readAccelerationSensor(Mapping& node);
    bool readCamera(Mapping& node);
    bool readRangeSensor(Mapping& node);
    bool readSpotLight(Mapping& node);
    SgNodePtr readSceneShape(Mapping& node);
    SgMesh* readSceneGeometry(Mapping& node);
    SgMesh* readSceneBox(Mapping& node);
    SgMesh* readSceneSphere(Mapping& node);
    SgMesh* readSceneCylinder(Mapping& node);
    SgMesh* readSceneCone(Mapping& node);
    SgMesh* readSceneExtrusion(Mapping& node);
    void readSceneAppearance(SgShape* shape, Mapping& node);
    void readSceneMaterial(SgShape* shape, Mapping& node);
    void setDefaultMaterial(SgShape* shape);

    double toRadian(double angle){
        return isDegreeMode ? radian(angle) : angle;
    }

    bool readAngle(Mapping& node, const char* key, double& angle){
        if(node.read(key, angle)){
            angle = toRadian(angle);
            return true;
        }
        return false;
    }

    bool readAngles(Mapping& node, const char* key, Vector3& angles){
        if(read(node, key, angles)){
            if(isDegreeMode){
                for(int i=0; i < 3; ++i){
                    angles[i] = radian(angles[i]);
                }
            }
            return true;
        }
        return false;
    }

    bool readRotation(Mapping& node, Matrix3& out_R, bool doExtract){
        ValueNodePtr value;
        if(doExtract){
            value = node.extract("rotation");
        } else {
            value = node.find("rotation");
        }
        if(!value || !value->isValid()){
            return false;
        }
        const Listing& rotations = *value->toListing();
        if(!rotations.empty()){
            if(rotations[0].isListing()){
                out_R = Matrix3::Identity();
                for(int i=0; i < rotations.size(); ++i){
                    const Listing& rotation = *rotations[i].toListing();
                    Vector4 r;
                    cnoid::read(rotation, r);
                    out_R = out_R * AngleAxis(toRadian(r[3]), Vector3(r[0], r[1], r[2]));
                }
            } else {
                Vector4 r;
                cnoid::read(rotations, r);
                out_R = AngleAxis(toRadian(r[3]), Vector3(r[0], r[1], r[2]));
            }
        }
        return true;
    }
};

}

namespace {

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

    if(nodeFunctionMap.empty()){
        nodeFunctionMap["Group"].set(&YAMLBodyLoaderImpl::readGroup, false);
        nodeFunctionMap["Transform"].set(&YAMLBodyLoaderImpl::readTransform, false);
        nodeFunctionMap["RigidBody"].set(&YAMLBodyLoaderImpl::readRigidBody, true);
        nodeFunctionMap["ForceSensor"].set(&YAMLBodyLoaderImpl::readForceSensor, true);
        nodeFunctionMap["RateGyroSensor"].set(&YAMLBodyLoaderImpl::readRateGyroSensor, true);
        nodeFunctionMap["AccelerationSensor"].set(&YAMLBodyLoaderImpl::readAccelerationSensor, true);
        nodeFunctionMap["Camera"].set(&YAMLBodyLoaderImpl::readCamera, true);
        nodeFunctionMap["CameraDevice"].set(&YAMLBodyLoaderImpl::readCamera, true);
        nodeFunctionMap["RangeSensor"].set(&YAMLBodyLoaderImpl::readRangeSensor, true);
        nodeFunctionMap["SpotLight"].set(&YAMLBodyLoaderImpl::readSpotLight, true);

        sceneNodeFunctionMap["Shape"] = &YAMLBodyLoaderImpl::readSceneShape;
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
    nameStack.clear();
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
    
    ValueNodePtr formatNode = topNode->extract("format");
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
        topNode->throwException(_("This version of the Choreonoid body format is not supported"));
    }

    isDegreeMode = false;
    ValueNodePtr angleUnitNode = topNode->extract("angleUnit");
    if(angleUnitNode){
        string unit = angleUnitNode->toString();
        if(unit == "radian"){
            isDegreeMode = false;
        } else if(unit == "degree"){
            isDegreeMode = true;
        } else {
            angleUnitNode->throwException(_("The \"angleUnit\" value must be either \"radian\" or \"degree\""));
        }
    }

    if(extract(topNode, "name", symbol)){
        body->setModelName(symbol);
    }

    transformStack.clear();
    transformStack.push_back(Affine3::Identity());
    ValueNodePtr linksNode = topNode->extract("links");
    if(!linksNode){
        topNode->throwException(_("There is no \"links\" values for defining the links in the body"));
    } else {
        Listing& linkNodes = *linksNode->toListing();
        for(int i=0; i < linkNodes.size(); ++i){
            Mapping* linkNode = linkNodes[i].toMapping();
            LinkInfo* info = new LinkInfo;
            extract(linkNode, "parent", info->parent);
            info->link = readLink(linkNode);
            info->node = linkNode;
            linkInfos.push_back(info);
        }
    }

    // construct a link tree
    for(size_t i=0; i < linkInfos.size(); ++i){
        LinkInfo* info = linkInfos[i];
        const std::string& parent = info->parent;
        if(!parent.empty()){
            LinkMap::iterator p = linkMap.find(parent);
            if(p != linkMap.end()){
                Link* parentLink = p->second;
                Link* link = info->link;
                link->setOffsetTranslation(parentLink->Rs() * link->offsetTranslation());
                link->setAccumulatedSegmentRotation(parentLink->Rs() * link->offsetRotation());
                parentLink->appendChild(link);
            } else {
                info->node->throwException(
                    str(format(_("Parent link \"%1%\" of %2% is not defined")) % parent % info->link->name()));
            }
        }
    }        

    ValueNodePtr rootLinkNode = topNode->extract("rootLink");
    if(!rootLinkNode){
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

    ValueNodePtr initDNode = topNode->extract("initialJointDisplacement");
    if(initDNode){
        Listing& initd = *initDNode->toListing();
        const int n = std::min(initd.size(), body->numLinks());
        for(int i=0; i < n; i++){
            body->link(i)->initialJointDisplacement() = toRadian(initd[i].toDouble());
        }
    }

    body->resetInfo(topNode);
        
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
    Matrix3 R;
    if(readRotation(*info, R, true)){
        link->setOffsetRotation(R);
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

    ValueNodePtr jointTypeNode = info->find("jointType");
    if(jointTypeNode->isValid()){
        string jointType = jointTypeNode->toString();
        if(jointType == "revolute"){
            link->setJointType(Link::REVOLUTE_JOINT);
        } else if(jointType == "slide"){
            link->setJointType(Link::SLIDE_JOINT);
        } else if(jointType == "free"){
            link->setJointType(Link::FREE_JOINT);
        } else if(jointType == "fixed"){
            link->setJointType(Link::FIXED_JOINT);
        } else if(jointType == "pseudoContinuousTrack"){
            link->setJointType(Link::PSEUDO_CONTINUOUS_TRACK);
        } else  if(jointType == "agx_crawler"){
            link->setJointType(Link::AGX_CRAWLER_JOINT);
        } else {
            jointTypeNode->throwException("Illegal jointType value");
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
    bool hasShape = false;
    SgGroupPtr shape = new SgInvariantGroup;
    ValueNodePtr elements = linkNode->extract("elements");
    if(elements){
        if(readElements(*elements, shape)){
            shape->setName(link->name());
            hasShape = true;
        }
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
                hasShape = true;
            }
        }
    }

    if(hasShape){
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


bool YAMLBodyLoaderImpl::readElements(ValueNode& elements, SgGroupPtr& sceneGroup)
{
    bool isSceneNodeAdded = false;
    
    SgGroupPtr parentSceneGroup = currentSceneGroup;
    currentSceneGroup = sceneGroup;
    
    if(elements.isListing()){
        Listing& listing = *elements.toListing();
        for(int i=0; i < listing.size(); ++i){
            Mapping& element = *listing[i].toMapping();
            const string type = element["type"].toString();
            if(readNode(element, type)){
                isSceneNodeAdded = true;
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
            if(readNode(element, type)){
                isSceneNodeAdded = true;
            }
            ++p;
        }
    }

    if(parentSceneGroup && isSceneNodeAdded){
        parentSceneGroup->addChild(sceneGroup);
    }
    currentSceneGroup = parentSceneGroup;

    return isSceneNodeAdded;
}


bool YAMLBodyLoaderImpl::readNode(Mapping& node, const string& type)
{
    bool isSceneNodeAdded = false;
    
    nameStack.push_back(string());
    node.read("name", nameStack.back());

    NodeFunctionMap::iterator p = nodeFunctionMap.find(type);
    if(p != nodeFunctionMap.end()){
        NodeFunctionInfo& info = p->second;
        if(info.isTransformDerived){
            if(readTransformNode(node, info.function)){
                isSceneNodeAdded = true;
            }
        } else {
            if((this->*info.function)(node)){
                isSceneNodeAdded = true;
            }
        }
    } else {
        SceneNodeFunctionMap::iterator q = sceneNodeFunctionMap.find(type);
        if(q != sceneNodeFunctionMap.end()){
            SceneNodeFunction readSceneNode = q->second;
            SgNodePtr scene = (this->*readSceneNode)(node);
            if(scene){
                currentSceneGroup->addChild(scene);
                isSceneNodeAdded = true;
            }
        } else {
            node.throwException(str(format(_("The node type \"%1%\" is not defined.")) % type));
        }
    }

    nameStack.pop_back();
    
    return isSceneNodeAdded;
}


bool YAMLBodyLoaderImpl::readContainerNode(Mapping& node, SgGroupPtr& group, NodeFunction nodeFunction)
{
    bool isSceneNodeAdded = false;

    group->setName(nameStack.back());

    if(nodeFunction){
        if((this->*nodeFunction)(node)){
            isSceneNodeAdded = true;
        }
    }

    ValueNode& elements = *node.find("elements");
    if(elements.isValid()){
        if(readElements(elements, group)){
            isSceneNodeAdded = true;
        }
    }

    return isSceneNodeAdded;
}


bool YAMLBodyLoaderImpl::readTransformNode(Mapping& node, NodeFunction nodeFunction)
{
    bool isSceneNodeAdded = false;

    Affine3 T = Affine3::Identity();
    bool isIdentity = true;
    
    if(read(node, "translation", v)){
        T.translation() = v;
        isIdentity = false;
    }
    Matrix3 R;
    if(readRotation(node, R, false)){
        T.linear() = R;
        isIdentity = false;
    }

    if(!isIdentity){
        transformStack.push_back(transformStack.back() * T);
    }

    SgGroupPtr group;
    if(isIdentity){
        group = new SgGroup;
    } else {
        group = new SgPosTransform(T);
    }
    if(readContainerNode(node, group, nodeFunction)){
        isSceneNodeAdded = true;
    }

    if(!isIdentity){
        transformStack.pop_back();
    }

    return isSceneNodeAdded;
}


bool YAMLBodyLoaderImpl::readGroup(Mapping& node)
{
    SgGroupPtr group = new SgGroup;
    return readContainerNode(node, group, 0);
}


bool YAMLBodyLoaderImpl::readTransform(Mapping& node)
{
    return readTransformNode(node, 0);
}


bool YAMLBodyLoaderImpl::readRigidBody(Mapping& node)
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

    return false;
}


bool YAMLBodyLoaderImpl::readDevice(Device* device, Mapping& node)
{
    device->setName(nameStack.back());

    if(node.read("id", id)) device->setId(id);

    const Affine3& T = transformStack.back();
    device->setLocalTranslation(currentLink->Rs() * T.translation());
    device->setLocalRotation(currentLink->Rs() * T.linear());
    device->setLink(currentLink);
    body->addDevice(device);

    return false;
}


bool YAMLBodyLoaderImpl::readForceSensor(Mapping& node)
{
    ForceSensorPtr sensor = new ForceSensor;
    if(read(node, "maxForce",  v)) sensor->F_max().head<3>() = v;
    if(read(node, "maxTorque", v)) sensor->F_max().tail<3>() = v;
    readDevice(sensor, node);
    return true;
}


bool YAMLBodyLoaderImpl::readRateGyroSensor(Mapping& node)
{
    RateGyroSensorPtr sensor = new RateGyroSensor;
    if(readAngles(node, "maxAngularVelocity", v)) sensor->w_max() = v;
    return readDevice(sensor, node);
}


bool YAMLBodyLoaderImpl::readAccelerationSensor(Mapping& node)
{
    AccelerationSensorPtr sensor = new AccelerationSensor();
    if(read(node, "maxAcceleration", v)) sensor->dv_max() = v;
    return readDevice(sensor, node);
}


bool YAMLBodyLoaderImpl::readCamera(Mapping& node)
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
        
    if(node.read("on", on)) camera->on(on);
    if(node.read("width", value)) camera->setResolutionX(value);
    if(node.read("height", value)) camera->setResolutionY(value);
    if(readAngle(node, "fieldOfView", value)) camera->setFieldOfView(value);
    if(node.read("nearClipDistance", value)) camera->setNearClipDistance(value);
    if(node.read("farClipDistance", value)) camera->setFarClipDistance(value);
    if(node.read("frameRate", value)) camera->setFrameRate(value);
    
    return readDevice(camera, node);
}


bool YAMLBodyLoaderImpl::readRangeSensor(Mapping& node)
{
    RangeSensorPtr rangeSensor = new RangeSensor;
    
    if(node.read("on", on)) rangeSensor->on(on);
    if(readAngle(node, "scanAngle", value)) rangeSensor->setYawRange(value);
    rangeSensor->setPitchRange(0.0);
    if(readAngle(node, "scanStep", value)) rangeSensor->setYawResolution(rangeSensor->yawRange() / value);
    if(node.read("minDistance", value)) rangeSensor->setMinDistance(value);
    if(node.read("maxDistance", value)) rangeSensor->setMaxDistance(value);
    if(node.read("scanRate", value)) rangeSensor->setFrameRate(value);
    
    return readDevice(rangeSensor, node);
}


bool YAMLBodyLoaderImpl::readSpotLight(Mapping& node)
{
    SpotLightPtr light = new SpotLight();

    if(node.read("on", on)) light->on(on);
    if(read(node, "color", color)) light->setColor(color);
    if(node.read("intensity", value)) light->setIntensity(value);
    if(read(node, "direction", v)) light->setDirection(v);
    if(readAngle(node, "beamWidth", value)) light->setBeamWidth(value);
    if(readAngle(node, "cutOffAngle", value)) light->setCutOffAngle(value);
    if(node.read("cutOffExponent", value)) light->setCutOffExponent(value);
    if(read(node, "attenuation", color)){
        light->setConstantAttenuation(color[0]);
        light->setLinearAttenuation(color[1]);
        light->setQuadraticAttenuation(color[2]);
    }

    return readDevice(light, node);
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
    } else if(type == "Sphere"){
        mesh = readSceneSphere(node);
    } else if(type == "Cylinder"){
        mesh = readSceneCylinder(node);
    } else if(type == "Cone"){
        mesh = readSceneCone(node);
    } else if(type == "Extrusion"){
        mesh = readSceneExtrusion(node);
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


SgMesh* YAMLBodyLoaderImpl::readSceneSphere(Mapping& node)
{
    return meshGenerator.generateSphere(node.get("radius", 1.0));
}


SgMesh* YAMLBodyLoaderImpl::readSceneCylinder(Mapping& node)
{
    double radius = node.get("radius", 1.0);
    double height = node.get("height", 1.0);
    bool bottom = node.get("bottom", true);
    bool side = node.get("side", true);
    return meshGenerator.generateCylinder(radius, height, bottom, side);
}


SgMesh* YAMLBodyLoaderImpl::readSceneCone(Mapping& node)
{
    double radius = node.get("radius", 1.0);
    double height = node.get("height", 1.0);
    bool bottom = node.get("bottom", true);
    bool side = node.get("side", true);
    return meshGenerator.generateCone(radius, height, bottom, side);
}


SgMesh* YAMLBodyLoaderImpl::readSceneExtrusion(Mapping& node)
{
    MeshGenerator::Extrusion extrusion;

    Listing& crossSectionNode = *node.findListing("crossSection");
    if(crossSectionNode.isValid()){
        const int n = crossSectionNode.size() / 2;
        MeshGenerator::Vector2Array& crossSection = extrusion.crossSection;
        crossSection.resize(n);
        for(int i=0; i < n; ++i){
            Vector2& s = crossSection[i];
            for(int j=0; j < 2; ++j){
                s[j] = crossSectionNode[i*2+j].toDouble();
            }
        }
    }

    Listing& spineNode = *node.findListing("spine");
    if(spineNode.isValid()){
        const int n = spineNode.size() / 3;
        MeshGenerator::Vector3Array& spine = extrusion.spine;
        spine.resize(n);
        for(int i=0; i < n; ++i){
            Vector3& s = spine[i];
            for(int j=0; j < 3; ++j){
                s[j] = spineNode[i*3+j].toDouble();
            }
        }
    }

    Listing& orientationNode = *node.findListing("orientation");
    if(orientationNode.isValid()){
        const int n = orientationNode.size() / 4;
        MeshGenerator::AngleAxisArray& orientation = extrusion.orientation;
        orientation.resize(n);
        for(int i=0; i < n; ++i){
            AngleAxis& aa = orientation[i];
            Vector3& axis = aa.axis();
            for(int j=0; j < 4; ++j){
                axis[j] = orientationNode[i*4+j].toDouble();
            }
            aa.angle() = toRadian(orientationNode[i*4+3].toDouble());
        }
    }
    
    Listing& scaleNode = *node.findListing("scale");
    if(scaleNode.isValid()){
        const int n = scaleNode.size() / 2;
        MeshGenerator::Vector2Array& scale = extrusion.scale;
        scale.resize(n);
        for(int i=0; i < n; ++i){
            Vector2& s = scale[i];
            for(int j=0; j < 2; ++j){
                s[j] = scaleNode[i*2+j].toDouble();
            }
        }
    }

    readAngle(node, "creaseAngle", extrusion.creaseAngle);
    node.read("beginCap", extrusion.beginCap);
    node.read("endCap", extrusion.endCap);

    return meshGenerator.generateExtrusion(extrusion);
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
