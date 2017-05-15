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
#include <cnoid/YAMLSceneReader>
#include <cnoid/EigenArchive>
#include <cnoid/EigenUtil>
#include <cnoid/FileUtil>
#include <cnoid/Exception>
#include <cnoid/YAMLReader>
#include <cnoid/SceneLoader>
#include <cnoid/NullOut>
#include <Eigen/StdVector>
#include <unordered_map>
#include <mutex>
#include "gettext.h"

using namespace std;
using namespace cnoid;
namespace filesystem = boost::filesystem;
using boost::format;

namespace {

std::mutex customNodeFunctionMutex;
typedef function<bool(YAMLBodyLoader& loader, Mapping& node)> CustomNodeFunction;
typedef map<string, CustomNodeFunction> CustomNodeFunctionMap;
CustomNodeFunctionMap customNodeFunctions;

}

namespace cnoid {

void YAMLBodyLoader::addNodeType
(const std::string& typeName,  std::function<bool(YAMLBodyLoader& loader, Mapping& node)> readFunction)
{
    std::lock_guard<std::mutex> guard(customNodeFunctionMutex);
    customNodeFunctions[typeName] = readFunction;
}

class YAMLBodyLoaderImpl
{
public:
    YAMLBodyLoader* self;
    YAMLReader reader;
    YAMLSceneReader sceneReader;
    SceneLoader sceneLoader;
    filesystem::path directoryPath;

    typedef function<bool(Mapping& node)> NodeFunction;

    struct NodeFunctionInfo {
        NodeFunction function;
        bool isTransformDerived;
        bool hasElements;
        void set(NodeFunction f) { function = f; isTransformDerived = false; hasElements = false; }
        void setT(NodeFunction f) { function = f; isTransformDerived = true; hasElements = false; }
        void setTE(NodeFunction f) { function = f; isTransformDerived = true; hasElements = true; }
    };
    typedef unordered_map<string, NodeFunctionInfo> NodeFunctionMap;
    
    NodeFunctionMap nodeFunctions;
    int numCustomNodeFunctions;
    
    Body* body;
    Link* rootLink;

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
    typedef vector<Affine3, Eigen::aligned_allocator<Affine3>> Affine3Vector;
    Affine3Vector transformStack;

    struct RigidBody
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        Vector3 c;
        double m;
        Matrix3 I;
    };
    typedef vector<RigidBody, Eigen::aligned_allocator<RigidBody>> RigidBodyVector;
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

    ostream* os_;

    vector<bool> validJointIdSet;
    int numValidJointIds;

    struct NodeInfo
    {
        SgGroupPtr parent;
        SgNodePtr node;
        Matrix3 R;
        bool isScaled;
    };

    typedef unordered_map<string, NodeInfo> NodeMap;
    
    struct ResourceInfo : public Referenced
    {
        SgNodePtr rootNode;
        unique_ptr<NodeMap> nodeMap;
    };
    typedef ref_ptr<ResourceInfo> ResourceInfoPtr;
        
    map<string, ResourceInfoPtr> resourceInfoMap;
    

    ostream& os() { return *os_; }

    YAMLBodyLoaderImpl(YAMLBodyLoader* self);
    ~YAMLBodyLoaderImpl();
    void updateCustomNodeFunctions();
    bool clear();
    bool load(Body* body, const std::string& filename);
    bool readTopNode(Body* body, Mapping* topNode);
    bool readBody(Mapping* topNode);
    void readLinkNode(Mapping* linkNode);
    LinkPtr readLink(Mapping* linkNode);
    vector<LinkInfoPtr> readContinuousTrack(ValueNodePtr crawlerNode, bool isDegreeMode);
    vector<LinkInfoPtr> generateCrawlerNode(Mapping* linkNode, bool isDegreeMode);
    void setMassParameters(Link* link);
    //! \return true if any scene nodes other than Group and Transform are added in the sub tree
    bool readElements(ValueNode& elements, SgGroupPtr& sceneGroup);
    bool readNode(Mapping& node, const string& type);
    bool readContainerNode(Mapping& node, SgGroupPtr& group, NodeFunction nodeFunction);
    bool readTransformNode(Mapping& node, NodeFunction nodeFunction, bool hasElements);
    bool readGroup(Mapping& node);
    bool readTransform(Mapping& node);
    bool readResource(Mapping& node);
    bool importModel(SgGroup* group, const std::string& uri);
    bool importModel(SgGroup* group, const string& uri, const string& nodeName);
    ResourceInfo* getOrCreateResourceInfo(const string& uri);
    void adjustNodeCoordinate(NodeInfo& info);
    void makeNodeMap(ResourceInfo* info);
    void makeNodeMapSub(const NodeInfo& nodeInfo, NodeMap& nodeMap);
    bool readRigidBody(Mapping& node);
    bool readDevice(Device* device, Mapping& node);
    bool readForceSensor(Mapping& node);
    bool readRateGyroSensor(Mapping& node);
    bool readAccelerationSensor(Mapping& node);
    bool readCamera(Mapping& node);
    bool readRangeSensor(Mapping& node);
    bool readSpotLight(Mapping& node);

    bool isDegreeMode() const {
        return sceneReader.isDegreeMode();
    }
    
    double toRadian(double angle) const {
        return sceneReader.toRadian(angle);
    }

    double readLimitValue(ValueNode& node, bool isUpper){
        double value;
        if(node.read(value)){
            return value;
        }
        std::string symbol;
        if(node.read(symbol)){
            if(symbol == "unlimited"){
                if(isUpper){
                    return std::numeric_limits<double>::max();
                } else {
                    return -std::numeric_limits<double>::max();
                }
            } else {
                node.throwException(_("Unknown symbol is used as a jointRange value"));
            }
        }
        node.throwException(_("Invalid type value is used as a jointRange value"));
    }

    bool readAngle(Mapping& node, const char* key, double& angle){
        return sceneReader.readAngle(node, key, angle);
    }

    bool readRotation(Mapping& node, Matrix3& out_R, bool doExtract){
        return sceneReader.readRotation(node, out_R, doExtract);
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


void readInertia(Listing& inertia, Matrix3& I)
{
    if(inertia.size() == 9){
        for(int i=0; i < 3; ++i){
            for(int j=0; j < 3; ++j){
                I(i, j) = inertia[i * 3 + j].toDouble();
            }
        }
    } else if(inertia.size() == 6){
        I(0, 0) = inertia[0].toDouble();
        I(0, 1) = inertia[1].toDouble();
        I(0, 2) = inertia[2].toDouble();
        I(1, 0) = I(0, 1);
        I(1, 1) = inertia[3].toDouble();
        I(1, 2) = inertia[4].toDouble();
        I(2, 0) = I(0, 2);
        I(2, 1) = I(1, 2);
        I(2, 2) = inertia[5].toDouble();
    } else {
        inertia.throwException(_("The number of elements specified as an inertia value must be six or nine."));
    }
}
    
bool extractInertia(Mapping* mapping, const char* key, Matrix3& I)
{
    ListingPtr listing = dynamic_pointer_cast<Listing>(mapping->extract(key));
    if(listing){
        readInertia(*listing, I);
        return true;
    }
    return false;
}


bool readInertia(Mapping& node, const char* key, Matrix3& I)
{
    Listing* inertia = node.findListing(key);
    if(inertia->isValid()){
        readInertia(*inertia, I);
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
    impl = new YAMLBodyLoaderImpl(this);
    setDefaultDivisionNumber(20);
}


YAMLBodyLoaderImpl::YAMLBodyLoaderImpl(YAMLBodyLoader* self)
    : self(self)
{
    nodeFunctions["Group"].set([&](Mapping& node){ return readGroup(node); });
    nodeFunctions["Transform"].set([&](Mapping& node){ return readTransform(node); });
    nodeFunctions["Resource"].setT([&](Mapping& node){ return readResource(node); });
    nodeFunctions["RigidBody"].setTE([&](Mapping& node){ return readRigidBody(node); });
    nodeFunctions["ForceSensor"].setTE([&](Mapping& node){ return readForceSensor(node); });
    nodeFunctions["RateGyroSensor"].setTE([&](Mapping& node){ return readRateGyroSensor(node); });
    nodeFunctions["AccelerationSensor"].setTE([&](Mapping& node){ return readAccelerationSensor(node); });
    nodeFunctions["Camera"].setTE([&](Mapping& node){ return readCamera(node); });
    nodeFunctions["CameraDevice"].setTE([&](Mapping& node){ return readCamera(node); });
    nodeFunctions["RangeSensor"].setTE([&](Mapping& node){ return readRangeSensor(node); });
    nodeFunctions["SpotLight"].setTE([&](Mapping& node){ return readSpotLight(node); });

    numCustomNodeFunctions = 0;

    body = 0;
    os_ = &nullout();
}


YAMLBodyLoader::~YAMLBodyLoader()
{
    delete impl;
}


YAMLBodyLoaderImpl::~YAMLBodyLoaderImpl()
{

}


void YAMLBodyLoader::setMessageSink(std::ostream& os)
{
    impl->os_ = &os;
    impl->sceneLoader.setMessageSink(os);
}


void YAMLBodyLoader::setDefaultDivisionNumber(int n)
{
    impl->sceneReader.setDefaultDivisionNumber(n);
    impl->sceneLoader.setDefaultDivisionNumber(n);
}


void YAMLBodyLoaderImpl::updateCustomNodeFunctions()
{
    std::lock_guard<std::mutex> guard(customNodeFunctionMutex);
    if(customNodeFunctions.size() > numCustomNodeFunctions){
        for(auto& p : customNodeFunctions){
            CustomNodeFunction& func = p.second;
            nodeFunctions[p.first].setTE([&, func](Mapping& node){ return func(*self, node); });
        }
        numCustomNodeFunctions = customNodeFunctions.size();
    }
}


bool YAMLBodyLoader::isDegreeMode() const
{
    return impl->isDegreeMode();
}


double YAMLBodyLoader::toRadian(double angle) const
{
    return impl->toRadian(angle);
}


bool YAMLBodyLoader::readAngle(Mapping& node, const char* key, double& angle)
{
    return impl->readAngle(node, key, angle);
}


bool YAMLBodyLoader::readRotation(Mapping& node, Matrix3& out_R)
{
    return impl->readRotation(node, out_R, false);
}


bool YAMLBodyLoaderImpl::clear()
{
    rootLink = nullptr;
    linkInfos.clear();
    linkMap.clear();
    validJointIdSet.clear();
    numValidJointIds = 0;
    nameStack.clear();
    transformStack.clear();
    rigidBodies.clear();
    sceneReader.clear();
    resourceInfoMap.clear();
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
    clear();
    
    updateCustomNodeFunctions();
    
    bool result = false;
    this->body = body;
    body->clearDevices();
    body->clearExtraJoints();

    try {
        result = readBody(topNode);
        
    } catch(const ValueNode::Exception& ex){
        os() << ex.message();
    } catch(const nonexistent_key_error& error){
        if(const std::string* message = boost::get_error_info<error_info_message>(error)){
            os() << *message << endl;
        }
    } catch(...){
        clear();
        throw;
    }

    clear();

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

    ValueNodePtr angleUnitNode = topNode->extract("angleUnit");
    if(angleUnitNode){
        string unit = angleUnitNode->toString();
        if(unit == "radian"){
            sceneReader.setAngleUnit(YAMLSceneReader::RADIAN);
        } else if(unit == "degree"){
            sceneReader.setAngleUnit(YAMLSceneReader::DEGREE);
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
        if(linksNode->isListing()){
            Listing& linkNodes = *linksNode->toListing();
            if(linkNodes.empty()){
                linkNodes.throwException(_("No link is contained in the \"links\" listing"));
            }
            for(int i=0; i < linkNodes.size(); ++i){
                readLinkNode(linkNodes[i].toMapping());
            }
        } else if(linksNode->isMapping()){
            Mapping* links = linksNode->toMapping();
            Mapping::iterator p = links->begin();
            while(p != links->end()){
                const string& type = p->first;
                if(type == "Link"){
                    readLinkNode(p->second->toMapping());
                } else {
                    linksNode->throwException(
                        str(format(_("A %1% node cannot be specified in links")) % type));
                }
                ++p;
            }
        } else {
            linksNode->throwException(_("Invalid value specified in the \"links\" key."));
        }
    }

    ValueNodePtr crawlerNode = topNode->extract("ContinuousTrack");
    if(crawlerNode){
        vector<LinkInfoPtr> crawlerInfos = readContinuousTrack(crawlerNode, isDegreeMode());
        linkInfos.insert(linkInfos.end(), crawlerInfos.begin(), crawlerInfos.end());
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
                parentLink->appendChild(link);
            } else {
                info->node->throwException(
                    str(format(_("Parent link \"%1%\" of %2% is not defined")) % parent % info->link->name()));
            }
        }
    }        

    ValueNodePtr rootLinkNode = topNode->extract("rootLink");
    if(rootLinkNode){
        string rootLinkName = rootLinkNode->toString();
        LinkMap::iterator p = linkMap.find(rootLinkName);
        if(p == linkMap.end()){
            rootLinkNode->throwException(
                str(format(_("Link \"%1%\" specified in \"rootLink\" is not defined.")) % rootLinkName));
        }
        rootLink = p->second;
    }

    if(!rootLink){
        topNode->throwException(_("There is no link defined."));
    }

    body->setRootLink(rootLink);
    body->expandLinkOffsetRotations();

    // Warn empty joint ids
    if(numValidJointIds < validJointIdSet.size()){
        for(size_t i=0; i < validJointIdSet.size(); ++i){
            if(!validJointIdSet[i]){
                os() << str(format("Warning: Joint ID %1% is not specified.") % i) << endl;
            }
        }
    }

    //! \todo Remove this later
    ValueNodePtr initDNode = topNode->extract("initialJointDisplacement");
    if(initDNode){
        Listing& initd = *initDNode->toListing();
        const int n = std::min(initd.size(), body->numLinks());
        for(int i=0; i < n; i++){
            body->link(i)->setInitialJointDisplacement(toRadian(initd[i].toDouble()));
        }
    }

    body->resetInfo(topNode);
        
    body->installCustomizer();

    return true;
}


void YAMLBodyLoaderImpl::readLinkNode(Mapping* linkNode)
{
    string type;
    if(extract(linkNode, "type", type)){
        if(type != "Link"){
            linkNode->throwException(
                str(format(_("A %1% node cannot be specified in links")) % type));
        }
    }
    LinkInfoPtr info = new LinkInfo;
    extract(linkNode, "parent", info->parent);
    Link* link = readLink(linkNode);
    info->link = link;
    info->node = linkNode;
    linkInfos.push_back(info);
    if(!rootLink){
        rootLink = link;
    }
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
                validJointIdSet[id] = true;
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
        } else if(jointType == "prismatic"){
            link->setJointType(Link::SLIDE_JOINT);
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
            string symbol = jointAxisNode->toString();
            std::transform(symbol.cbegin(), symbol.cend(), symbol.begin(), ::toupper);
            if(symbol == "X"){
                axis = Vector3::UnitX();
            } else if(symbol == "-X"){
                axis = -Vector3::UnitX();
            } else if(symbol == "Y"){
                axis = Vector3::UnitY();
            } else if(symbol == "-Y"){
                axis = -Vector3::UnitY();
            } else if(symbol == "Z"){
                axis = Vector3::UnitZ();
            } else if(symbol == "-Z"){
                axis = -Vector3::UnitZ();
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

    ValueNodePtr jointAngleNode = info->extract("jointAngle");
    if(jointAngleNode){
        link->setInitialJointDisplacement(toRadian(jointAngleNode->toDouble()));
    }
    ValueNodePtr jointDisplacementNode = info->extract("jointDisplacement");
    if(jointDisplacementNode){
        link->setInitialJointDisplacement(jointDisplacementNode->toDouble());
    }

    double lower = -std::numeric_limits<double>::max();
    double upper =  std::numeric_limits<double>::max();
    
    ValueNode& jointRangeNode = *info->find("jointRange");
    if(jointRangeNode.isValid()){
        if(jointRangeNode.isScalar()){
            upper = readLimitValue(jointRangeNode, true);
            lower = -upper;
        } else if(jointRangeNode.isListing()){
            Listing& jointRange = *jointRangeNode.toListing();
            if(jointRange.size() != 2){
                jointRangeNode.throwException(_("jointRange must have two elements"));
            }
            lower = readLimitValue(jointRange[0], false);
            upper = readLimitValue(jointRange[1], true);
        } else {
            jointRangeNode.throwException(_("Invalid type value is specefied as a jointRange"));
        }
    }
    if(link->jointType() == Link::REVOLUTE_JOINT && isDegreeMode()){
        link->setJointRange(
            lower == -std::numeric_limits<double>::max() ? lower : radian(lower),
            upper ==  std::numeric_limits<double>::max() ? upper : radian(upper));
    } else {
        link->setJointRange(lower, upper);
    }

    ValueNodePtr maxVelocityNode = info->find("maxJointVelocity");
    if(maxVelocityNode->isValid()){
        double maxVelocity = maxVelocityNode->toDouble();
        if(link->jointType() == Link::REVOLUTE_JOINT){
            link->setJointVelocityRange(toRadian(-maxVelocity), toRadian(maxVelocity));
        } else {
            link->setJointVelocityRange(-maxVelocity, maxVelocity);
        }
    }

    ValueNodePtr velocityRangeNode = info->find("jointVelocityRange");
    if(velocityRangeNode->isValid()){
        Listing& velocityRange = *velocityRangeNode->toListing();
        if(velocityRange.size() != 2){
            velocityRangeNode->throwException(_("jointVelocityRange must have two elements"));
        }
        if(link->jointType() == Link::REVOLUTE_JOINT){
            link->setJointVelocityRange(toRadian(velocityRange[0].toDouble()), toRadian(velocityRange[1].toDouble()));
        } else {
            link->setJointVelocityRange(velocityRange[0].toDouble(), velocityRange[1].toDouble());
        }
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
    if(!extractInertia(info, "inertia", rbody.I)){
        rbody.I.setZero();
    }
    rigidBodies.push_back(rbody);
    setMassParameters(link);

    // The uri parameter should be used in a Resource node
    /*
    if(extract(linkNode, "uri", symbol)){
        if(importModel(shape, symbol)){
            hasShape = true;
        }
    }
    */

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

vector<YAMLBodyLoaderImpl::LinkInfoPtr> YAMLBodyLoaderImpl::readContinuousTrack(ValueNodePtr crawlerNode, bool isDegreeMode){
    vector<LinkInfoPtr> allCrawlerInfos;
    allCrawlerInfos.clear();
    if(crawlerNode->isListing()){
        Listing& crawlerNodes = *crawlerNode->toListing();
        for(int i=0; i < crawlerNodes.size(); ++i){
            vector<LinkInfoPtr> crawlerInfo = generateCrawlerNode(crawlerNodes[i].toMapping(), isDegreeMode);
            allCrawlerInfos.insert(allCrawlerInfos.end(), crawlerInfo.begin(), crawlerInfo.end());
        }
    } else if(crawlerNode->isMapping()){
        allCrawlerInfos = generateCrawlerNode(crawlerNode->toMapping(), isDegreeMode);
    }
    return allCrawlerInfos;
}


vector<YAMLBodyLoaderImpl::LinkInfoPtr> YAMLBodyLoaderImpl::generateCrawlerNode(Mapping* linkNode, bool isDegreeMode)
{
    MappingPtr info = static_cast<Mapping*>(linkNode->clone());
    string parent;
    if(!extract(info, "parent", parent)){
        info->throwException("parent must be specified.");
    }

    string crawlername;
    if(!extract(info, "name", crawlername)){
        info->throwException("name must be specified.");
    }

    int numTracks;
    if(!extract(info, "numTracks", numTracks)){
        info->throwException("numTracks must be specified.");
    }

    bool hasShape = false;
    SgGroupPtr shape = new SgInvariantGroup;
    ValueNodePtr elements = info->extract("elements");
    if(elements){
        if(readElements(*elements, shape)){
            hasShape = true;
        }
    }
//    if(extract(info, "uri", symbol)){
//        filesystem::path filepath(symbol);
//        if(!checkAbsolute(filepath)){
//            filepath = directoryPath / filepath;
//            filepath.normalize();
//        }
//        vrmlParser.load(getAbsolutePathString(filepath));
//        while(VRMLNodePtr vrmlNode = vrmlParser.readNode()){
//            SgNodePtr node = sgConverter.convert(vrmlNode);
//            if(node){
//                shape->addChild(node);
//                hasShape = true;
//            }
//        }
//    }

    bool hasRootOffsetTranslation = false;
    if(extractEigen(info, "translation", v)){
        hasRootOffsetTranslation = true;
    }
    bool hasRootOffsetRotation = false;
    Matrix3 R;
    if(readRotation(*info, R, true)){
        hasRootOffsetRotation = true;
    }

    Vector3 step;
    if(!extractEigen(info, "stepTranslation", step)){
        info->throwException("stepTranslation must be specified.");
    }

    Vector3 axis;
    ValueNodePtr jointAxisNode = info->find("jointAxis");
    if(jointAxisNode->isValid()){
        bool isValid = true;
        if(jointAxisNode->isListing()){
            read(*jointAxisNode->toListing(), axis);
        } else if(jointAxisNode->isString()){
            string symbol = jointAxisNode->toString();
            std::transform(symbol.cbegin(), symbol.cend(), symbol.begin(), ::toupper);
            if(symbol == "X"){
                axis = Vector3::UnitX();
            } else if(symbol == "-X"){
                axis = -Vector3::UnitX();
            } else if(symbol == "Y"){
                axis = Vector3::UnitY();
            } else if(symbol == "-Y"){
                axis = -Vector3::UnitY();
            } else if(symbol == "Z"){
                axis = Vector3::UnitZ();
            } else if(symbol == "-Z"){
                axis = -Vector3::UnitZ();
            } else {
                isValid = false;
            }
        } else {
            isValid = false;
        }
        if(!isValid){
            jointAxisNode->throwException("Illegal jointAxis value");
        }
    }

    RigidBodyVector rigidBodies;
    rigidBodies.clear();
    RigidBody rbody;
    if(!extractEigen(info, "centerOfMass", rbody.c)){
        rbody.c.setZero();
    }
    if(!extract(info, "mass", rbody.m)){
        rbody.m = 0.0;
    }
    if(!extractInertia(info, "inertia", rbody.I)){
        rbody.I.setZero();
    }
    rigidBodies.push_back(rbody);

    vector<double> bendingAngles(numTracks, 0.0);
    ValueNodePtr initAnglesNode = info->extract("bendingAngles");
    if(initAnglesNode){
        Listing& initAngles = *initAnglesNode->toListing();
        const int n = std::min(initAngles.size(), numTracks);
        for(int i=0; i < n; i++){
            bendingAngles[i] = isDegreeMode ? radian(initAngles[i].toDouble()) : initAngles[i].toDouble();
        }
    }

    vector<YAMLBodyLoaderImpl::LinkInfoPtr> crawlerlinks;
    crawlerlinks.clear();
    for(int i = 0; i < numTracks; ++i){
        LinkInfoPtr crawler_info = new LinkInfo;
        crawler_info->parent = parent;
        LinkPtr link = body->createLink();
        if(hasShape){
            link->setShape(shape);
        }
        link->setName(str(format("%1%%2%") % crawlername % i));
        if(!linkMap.insert(make_pair(link->name(), link)).second){
            info->throwException(str(format(_("Duplicated link name \"%1\%\"")) % link->name()));
        }
        link->setJointId(-1);
        link->setJointAxis(axis);
        //link->setJointType(Link::AGX_CRAWLER_JOINT);
        if(i==0){
            link->setJointType(Link::AGX_CRAWLER_JOINT);
            if(hasRootOffsetTranslation){
                link->setOffsetTranslation(v);
            }
            if(hasRootOffsetRotation){
                link->setOffsetRotation(R);
            }
        } else {
            link->setJointType(Link::REVOLUTE_JOINT);
            link->setOffsetTranslation(step);
        }
        setMassParameters(link);
        link->setInitialJointDisplacement(bendingAngles[i]);
        crawler_info->link = link;
        crawler_info->node = linkNode;
        crawlerlinks.push_back(crawler_info);

        parent = crawler_info->link->name();
    }
    rigidBodies.clear();

    return crawlerlinks;

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
        /*
          Process the case like
          elements:
            -
              type: Transform
              translation: [ 1, 0, 0 ]
        */
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

        /* Check the case like
           elements:
             type: Transform
             translation: [ 1, 0, 0 ]
        */
        ValueNode* typeNode = mapping.find("type");
        if(typeNode->isValid()){
            const string type = typeNode->toString();
            if(readNode(mapping, type)){
                isSceneNodeAdded = true;
            }
        } else {
            /*
               Process the case like
               elements:
                 Transform:
                   translation: [ 1, 0, 0 ]
            */
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
    
    auto p = nodeFunctions.find(type);
    if(p != nodeFunctions.end()){
        NodeFunctionInfo& info = p->second;

        nameStack.push_back(string());
        node.read("name", nameStack.back());
        
        if(info.isTransformDerived){
            if(readTransformNode(node, info.function, info.hasElements)){
                isSceneNodeAdded = true;
            }
        } else {
            if(info.function(node)){
                isSceneNodeAdded = true;
            }
        }

        nameStack.pop_back();
        
    } else {
        SgNodePtr scene = sceneReader.readNode(node, type);
        if(scene){
            currentSceneGroup->addChild(scene);
            isSceneNodeAdded = true;
        }
    }

    return isSceneNodeAdded;
}


bool YAMLBodyLoaderImpl::readContainerNode(Mapping& node, SgGroupPtr& group, NodeFunction nodeFunction)
{
    bool isSceneNodeAdded = false;

    group->setName(nameStack.back());

    if(nodeFunction){
        if(nodeFunction(node)){
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


bool YAMLBodyLoaderImpl::readTransformNode(Mapping& node, NodeFunction nodeFunction, bool hasElements)
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
    Vector3 scale;
    bool hasScale = read(node, "scale", scale);
    
    if(!isIdentity){
        transformStack.push_back(transformStack.back() * T);
    }

    SgGroupPtr group;
    if(isIdentity){
        if(hasScale){
            group = new SgScaleTransform(scale);
        } else {
            group = new SgGroup;
        }
    } else {
        group = new SgPosTransform(T);
    }
    if(hasElements){
        if(readContainerNode(node, group, nodeFunction)){
            isSceneNodeAdded = true;
        }
    } else {
        SgGroupPtr parentSceneGroup = currentSceneGroup;
        currentSceneGroup = group;
        if(nodeFunction(node)){
            isSceneNodeAdded = true;
            parentSceneGroup->addChild(group);
        }
        currentSceneGroup = parentSceneGroup;
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
    return readTransformNode(node, 0, true);
}


bool YAMLBodyLoaderImpl::readResource(Mapping& node)
{
    bool isSceneNodeAdded = false;
    if(node.read("uri", symbol)){
        string nodeName;
        if(node.read("node", nodeName)){
            isSceneNodeAdded = importModel(currentSceneGroup, symbol, nodeName);
        } else {
            isSceneNodeAdded = importModel(currentSceneGroup, symbol);
        }
    }
    return isSceneNodeAdded;
}


bool YAMLBodyLoaderImpl::importModel(SgGroup* group, const string& uri)
{
    ResourceInfo* resourceInfo = getOrCreateResourceInfo(uri);
    if(resourceInfo){
        group->addChild(resourceInfo->rootNode);
        return true;
    }
    return false;
}


bool YAMLBodyLoaderImpl::importModel(SgGroup* group, const string& uri, const string& nodeName)
{
    ResourceInfo* resourceInfo = getOrCreateResourceInfo(uri);

    SgNodePtr model;
    if(resourceInfo){
        if(nodeName.empty()){
            model = resourceInfo->rootNode;
        } else {
            unique_ptr<NodeMap>& nodeMap = resourceInfo->nodeMap;
            if(!nodeMap){
                makeNodeMap(resourceInfo);
            }
            auto iter = nodeMap->find(nodeName);
            if(iter == nodeMap->end()){
                os() << str(format("Warning: Node \"%1%\" is not found in \"%2%\".") % nodeName % uri) << endl;
            } else {
                NodeInfo& nodeInfo = iter->second;
                if(nodeInfo.parent){
                    nodeInfo.parent->removeChild(nodeInfo.node);
                    nodeInfo.parent = 0;
                    adjustNodeCoordinate(nodeInfo);
                }
                model = nodeInfo.node;
            }
        }
    }

    if(model){
        group->addChild(model);
        return true;
    }
    return false;
}


YAMLBodyLoaderImpl::ResourceInfo* YAMLBodyLoaderImpl::getOrCreateResourceInfo(const string& uri)
{
    ResourceInfo* info = 0;
    
    auto iter = resourceInfoMap.find(uri);
    if(iter != resourceInfoMap.end()){
        info = iter->second;
    } else {
        filesystem::path filepath(uri);
        if(!checkAbsolute(filepath)){
            filepath = directoryPath / filepath;
            filepath.normalize();
        }
        SgNodePtr rootNode = sceneLoader.load(getAbsolutePathString(filepath));
        if(rootNode){
            info = new ResourceInfo;
            if(auto transform = dynamic_cast<SgPosTransform*>(rootNode.get())){
                transform->translation().setZero();
            } else if(auto transform = dynamic_cast<SgAffineTransform*>(rootNode.get())){
                transform->translation().setZero();
            }
            info->rootNode = rootNode;
        }
        resourceInfoMap[uri] = info;
    }

    return info;
}


void YAMLBodyLoaderImpl::adjustNodeCoordinate(NodeInfo& info)
{
    if(auto pos = dynamic_cast<SgPosTransform*>(info.node.get())){
        if(info.isScaled){
            auto affine = new SgAffineTransform;
            affine->setLinear(info.R * pos->rotation());
            affine->translation().setZero();
            pos->moveChildrenTo(affine);
            info.node = affine;
        } else {
            pos->setRotation(info.R * pos->rotation());
            pos->translation().setZero();
        }
            
    } else if(auto affine = dynamic_cast<SgAffineTransform*>(info.node.get())){
        affine->setLinear(info.R * affine->linear());
        affine->translation().setZero();
            
    } else {
        if(info.isScaled){
            auto transform = new SgAffineTransform;
            transform->setLinear(info.R);
            transform->translation().setZero();
            transform->addChild(info.node);
            info.node = transform;
        } else if(!info.R.isApprox(Matrix3::Identity())){
            auto transform = new SgPosTransform;
            transform->setRotation(info.R);
            transform->translation().setZero();
            transform->addChild(info.node);
            info.node = transform;
        }
    }
}
        

void YAMLBodyLoaderImpl::makeNodeMap(ResourceInfo* info)
{
    info->nodeMap.reset(new NodeMap);
    NodeInfo nodeInfo;
    nodeInfo.parent = 0;
    nodeInfo.node = info->rootNode;
    nodeInfo.R = Matrix3::Identity();
    nodeInfo.isScaled = false;
    makeNodeMapSub(nodeInfo, *info->nodeMap);
}


void YAMLBodyLoaderImpl::makeNodeMapSub(const NodeInfo& nodeInfo, NodeMap& nodeMap)
{
    const string& name = nodeInfo.node->name();
    bool wasProcessed = false;
    if(!name.empty()){
        wasProcessed = !nodeMap.insert(make_pair(name, nodeInfo)).second;
    }
    if(!wasProcessed){
        SgGroup* group = dynamic_cast<SgGroup*>(nodeInfo.node.get());
        if(group){
            NodeInfo childInfo;
            childInfo.parent = group;
            childInfo.isScaled = nodeInfo.isScaled;
            SgTransform* transform = dynamic_cast<SgTransform*>(group);
            if(!transform){
                childInfo.R = nodeInfo.R;
            } else if(auto pos = dynamic_cast<SgPosTransform*>(transform)){
                childInfo.R = nodeInfo.R * pos->rotation();
            } else {
                Affine3 T;
                transform->getTransform(T);
                childInfo.R = nodeInfo.R * T.linear();
                childInfo.isScaled = true;
            }
            for(SgNode* child : *group){
                childInfo.node = child;
                makeNodeMapSub(childInfo, nodeMap);
            }
        }
    }
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
    if(readInertia(node, "inertia", M)){
        rbody.I = T.linear() * M * T.linear().transpose();
    } else {
        rbody.I.setZero();
    }
    rigidBodies.push_back(rbody);

    return false;
}


bool YAMLBodyLoader::readDevice(Device* device, Mapping& node)
{
    return impl->readDevice(device, node);
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
    if(read(node, "maxAngularVelocity", v)){
        if(isDegreeMode()){
            for(int i=0; i < 3; ++i){
                v[i] = radian(v[i]);
            }
        }
        sensor->w_max() = v;
    }
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
