#include "StdBodyLoader.h"
#include "BodyLoader.h"
#include "BodyHandlerManager.h"
#include "Body.h"
#include <cnoid/StdSceneReader>
#include <cnoid/EigenArchive>
#include <cnoid/YAMLReader>
#include <cnoid/CloneMap>
#include <cnoid/NullOut>
#include <cnoid/UTF8>
#include <cnoid/Format>
#include <cnoid/stdx/filesystem>
#include <unordered_map>
#include <mutex>
#include <stdexcept>
#include "gettext.h"
#include <iostream>

using namespace std;
using namespace cnoid;
namespace filesystem = cnoid::stdx::filesystem;

namespace {

std::mutex customNodeFunctionMutex;
typedef function<bool(StdBodyLoader* loader, Mapping* node)> CustomNodeFunction;
typedef map<string, CustomNodeFunction> CustomNodeFunctionMap;
CustomNodeFunctionMap customNodeFunctions;

}

namespace cnoid {

class StdBodyLoader::Impl
{
public:
    StdBodyLoader* self;

    unique_ptr<BodyLoader> bodyLoader;
    
    YAMLReader reader;
    StdSceneReader sceneReader;
    filesystem::path mainFilePath;

    typedef function<bool(Mapping* node)> NodeFunction;

    struct NodeFunctionInfo {
        NodeFunction function;
        bool isTransformDerived;
        bool hasElements;
        void set(NodeFunction f, bool isTransformDerived, bool hasElements, bool isExternalFunction){
            function = f;
            this->isTransformDerived = isTransformDerived;
            this->hasElements = hasElements;
        }
        void set(NodeFunction f) { set(f, false, false, false); }
        void setT(NodeFunction f) { set(f, true, false, false); }
        void setTE(NodeFunction f) { set(f, true, true, false); }
    };
    typedef unordered_map<string, NodeFunctionInfo> NodeFunctionMap;
    
    NodeFunctionMap nodeFunctions;
    size_t numCustomNodeFunctions;
    
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

    set<string> jointNameSet;

    LinkPtr currentLink;
    vector<string> nameStack;
    typedef vector<Affine3, Eigen::aligned_allocator<Affine3>> Affine3Vector;
    Affine3Vector transformStack;
    
    CloneMap deviceInfoCloneMap;

    struct RigidBody
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        Vector3 c;
        double m;
        Matrix3 I;
    };
    typedef vector<RigidBody, Eigen::aligned_allocator<RigidBody>> RigidBodyVector;
    RigidBodyVector rigidBodies;

    struct SceneGroupSet
    {
        SgGroupPtr visual;
        SgGroupPtr collision;
        bool hasVisualChild;
        bool hasCollisionChild;

        SceneGroupSet(){
            hasVisualChild = false;
            hasCollisionChild = false;
        }
        template<class NodeType> void newGroup(){
            visual = new NodeType;
            collision = new NodeType;
        }
        template<class NodeType, class ParameterType> void newGroup(const ParameterType& param){
            visual = new NodeType(param);
            collision = new NodeType(param);
        }
        void setName(const std::string& name){
            visual->setName(name);
            collision->setName(name);
            // Group node that has name is kept even if it does not have contents
            hasVisualChild = true;
        }
    };

    vector<SceneGroupSet> sceneGroupSetStack;

    enum ModelType { VISUAL_AND_COLLISION, VISUAL, COLLISION } currentModelType;
    bool hasVisualOrCollisionNodes;

    SceneGroupSet& currentSceneGroupSet(){ return sceneGroupSetStack.back(); }

    void addNodeContents(SgGroup* parent, SgGroup* child){
        if(child->isTransformNode()){
            parent->addChild(child);
        } else {
            child->copyChildrenTo(parent);
        }
    }
    
    void addScene(SgNode* node){
        auto& current = sceneGroupSetStack.back();
        switch(currentModelType){
        case VISUAL_AND_COLLISION:
            current.visual->addChild(node);
            current.collision->addChild(node);
            current.hasVisualChild = true;
            current.hasCollisionChild = true;
            break;
        case VISUAL:
            current.visual->addChild(node);
            current.hasVisualChild = true;
            break;
        case COLLISION:
            current.collision->addChild(node);
            current.hasCollisionChild = true;
            break;
        }
    }
    void addCurrentSceneGroupToParentSceneGroup(){
        auto& parent = sceneGroupSetStack[sceneGroupSetStack.size() - 2];
        auto& current = sceneGroupSetStack.back();
        switch(currentModelType){
        case VISUAL_AND_COLLISION:
            if(current.hasVisualChild){
                addNodeContents(parent.visual, current.visual);
                parent.hasVisualChild = true;
            }
            if(current.hasCollisionChild){
                addNodeContents(parent.collision, current.collision);
                parent.hasCollisionChild = true;
            }
            break;
        case VISUAL:
            if(current.hasVisualChild){
                addNodeContents(parent.visual, current.visual);
                parent.hasVisualChild = true;
            }
            break;
        case COLLISION:
            if(current.hasCollisionChild){
                addNodeContents(parent.collision, current.collision);
                parent.hasCollisionChild = true;
            }
            break;
        }
    }

    // temporary variables for reading values
    int id;
    double value;
    string symbol;
    bool on;
    Vector3f color;
    Vector3 v;
    Matrix3 M;

    vector<bool> validJointIdSet;
    size_t numValidJointIds;

    unique_ptr<StdBodyLoader> subLoader;
    map<string, BodyPtr> subBodyMap;
    vector<BodyPtr> subBodies;
    bool isSubLoader;

    ostream* os_;
    ostream& os() { return *os_; }
    int defaultDivisionNumber;
    double defaultCreaseAngle;
    bool isVerbose;
    bool isShapeLoadingEnabled;

    BodyHandlerManager bodyHandlerManager;

    Impl(StdBodyLoader* self);
    ~Impl();
    void updateCustomNodeFunctions();
    bool clear();
    bool load(Body* body, const std::string& filename);
    bool readTopNode(Body* body, Mapping* topNode);
    bool checkFormat(Mapping* topNode);
    bool loadAnotherFormatBodyFile(Mapping* topNode);
    bool readBody(Mapping* topNode);
    void readNodeInLinks(Mapping* linkNode, const string& nodeType);
    void readLinkNode(Mapping* linkNode);
    void setLinkName(Link* link, const string& name, ValueNode* node);
    void setJointName(Link* link, const string& jointName, ValueNode* node);
    LinkPtr readLinkContents(Mapping* linkNode, LinkPtr link = nullptr);
    void setJointId(Link* link, int id);
    void readJointContents(Link* link, Mapping* node);
    void readAxis(ValueNode* node, Vector3& out_axis);
    static bool readJointDisplacementRange(Mapping* node, Link* link, bool isDegree, bool doExtract);
    static bool readJointVelocityRange(Mapping* node, Link* link, bool isDegree, bool doExtract);
    static bool readJointEffortRange(Mapping* node, Link* link, bool doExtract);
    void setJointParameters(Link* link, string jointType, ValueNode* node);
    void setMassParameters(Link* link);

    /*
      The following functions return true if any scene nodes other than Group and Transform
      are added in the sub tree.
    */
    bool readElements(Mapping* node);
    bool readElementContents(ValueNode* elements);
    bool readNode(Mapping* node, const string& type);
    bool readSkipNode(Mapping* node);
    bool readContainerNode(Mapping* node, NodeFunction nodeFunction);
    bool readTransformContents(Mapping* node, NodeFunction nodeFunction, bool hasElements);
    bool readGroup(Mapping* node);
    bool readTransform(Mapping* node);
    bool readRigidBody(Mapping* node);
    bool readVisualOrCollision(Mapping* node, bool isVisual);
    bool readVisualOrCollisionContents(Mapping* node);
    bool readResource(Mapping* node);
    bool readDevice(Device* device, const Mapping* node);
    void readContinuousTrackNode(Mapping* linkNode);
    void addTrackLink(int index, LinkPtr link, Mapping* node, string& io_parent, double initialAngle);
    void readSubBodyNode(Mapping* linkNode);
    void addSubBodyLinks(const Body* subBody, Mapping* node);
    void readExtraJoints(Mapping* topNode);
    void readExtraJoint(Mapping* node);
    void readBodyHandlers(ValueNode* node);
    
    bool isDegreeMode() const {
        return sceneReader.isDegreeMode();
    }
    
    double toRadian(double angle) const {
        return sceneReader.toRadian(angle);
    }

    static double readLimitValue(ValueNode& node, bool isUpper){
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
        return 0.0;
    }

    bool readAngle(const Mapping* node, const char* key, double& angle) const {
        return sceneReader.readAngle(node, key, angle);
    }
    bool readAngle(const Mapping* node, std::initializer_list<const char*> keys, double& angle) const {
        return sceneReader.readAngle(node, keys, angle);
    }
    bool readRotation(const Mapping* node, Matrix3& out_R) const {
        return sceneReader.readRotation(node, out_R);
    }
    bool readRotation(const Mapping* node, const char* key, Matrix3& out_R) const {
        return sceneReader.readRotation(node, key, out_R);
    }
    bool extractRotation(Mapping* node, Matrix3& out_R) const {
        return sceneReader.extractRotation(node, out_R);
    }
    bool extractRotation(Mapping* node, const char* key, Matrix3& out_R) const {
        return sceneReader.extractRotation(node, key, out_R);
    }
    bool readTranslation(const Mapping* node, Vector3& out_p) const {
        return sceneReader.readTranslation(node, out_p);
    }
    bool readTranslation(const Mapping* node, const char* key, Vector3& out_p) const {
        return sceneReader.readTranslation(node, key, out_p);
    }
    bool extractTranslation(Mapping* node, Vector3& out_p) const {
        return sceneReader.extractTranslation(node, out_p);
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


template<typename ValueType>
bool extract(Mapping* mapping, std::initializer_list<const char*> keys, ValueType& out_value)
{
    ValueNodePtr node = mapping->extract(keys);
    if(node){
        out_value = node->to<ValueType>();
        return true;
    }
    return false;
}


template<typename Derived>
bool extractEigen(Mapping* mapping, std::initializer_list<const char*> keys, Eigen::MatrixBase<Derived>& x)
{
    if(auto node = mapping->extract(keys)){
        readEx(node->toListing(), x);
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
        inertia.throwException(_("The number of elements specified as an inertia value must be six or nine"));
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


bool readInertia(Mapping* node, const char* key, Matrix3& I)
{
    Listing* inertia = node->findListing(key);
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


StdBodyLoader::StdBodyLoader()
{
    impl = new Impl(this);
}


StdBodyLoader::Impl::Impl(StdBodyLoader* self)
    : self(self)
{
    sceneReader.setGroupOptimizationEnabled(true);
    sceneReader.setYAMLReader(&reader);
    
    nodeFunctions["Skip"].set([&](Mapping* node){ return readSkipNode(node); });
    nodeFunctions["Group"].set([&](Mapping* node){ return readGroup(node); });
    nodeFunctions["Transform"].set([&](Mapping* node){ return readTransform(node); });
    nodeFunctions["RigidBody"].setTE([&](Mapping* node){ return readRigidBody(node); });
    nodeFunctions["Visual"].set([&](Mapping* node){ return readVisualOrCollision(node, true); });
    nodeFunctions["Collision"].set([&](Mapping* node){ return readVisualOrCollision(node, false); });
    nodeFunctions["Resource"].set([&](Mapping* node){ return readResource(node); });

    numCustomNodeFunctions = 0;

    body = nullptr;
    isSubLoader = false;
    os_ = &cout;
    isVerbose = false;
    isShapeLoadingEnabled = true;
    defaultDivisionNumber = -1;
    defaultCreaseAngle = -1.0;
}


StdBodyLoader::~StdBodyLoader()
{
    delete impl;
}


StdBodyLoader::Impl::~Impl()
{

}


void StdBodyLoader::setMessageSink(std::ostream& os)
{
    impl->os_ = &os;
    impl->sceneReader.setMessageSink(os);
    impl->bodyHandlerManager.setMessageSink(os);
}


void StdBodyLoader::setVerbose(bool on)
{
    impl->isVerbose = on;
}


void StdBodyLoader::setShapeLoadingEnabled(bool on)
{
    impl->isShapeLoadingEnabled = on;
}


void StdBodyLoader::setDefaultDivisionNumber(int n)
{
    impl->defaultDivisionNumber = n;
}


void StdBodyLoader::setDefaultCreaseAngle(double theta)
{
    impl->defaultCreaseAngle = theta;
}


void StdBodyLoader::Impl::updateCustomNodeFunctions()
{
    std::lock_guard<std::mutex> guard(customNodeFunctionMutex);
    if(customNodeFunctions.size() > numCustomNodeFunctions){
        for(auto& p : customNodeFunctions){
            CustomNodeFunction& func = p.second;
            nodeFunctions[p.first].setTE([&, func](Mapping* node){ return func(self, node); });
        }
        numCustomNodeFunctions = customNodeFunctions.size();
    }
}


StdSceneReader* StdBodyLoader::sceneReader()
{
    return &impl->sceneReader;
}


const StdSceneReader* StdBodyLoader::sceneReader() const
{
    return &impl->sceneReader;
}


bool StdBodyLoader::isDegreeMode() const
{
    return impl->isDegreeMode();
}


double StdBodyLoader::toRadian(double angle) const
{
    return impl->toRadian(angle);
}


bool StdBodyLoader::readAngle(const Mapping* node, const char* key, double& angle) const
{
    return impl->readAngle(node, key, angle);
}


bool StdBodyLoader::readAngle(const Mapping& node, const char* key, double& angle) const
{
    return impl->readAngle(&node, key, angle);
}


bool StdBodyLoader::readRotation(const Mapping* node, Matrix3& out_R) const
{
    return impl->readRotation(node, out_R);
}


bool StdBodyLoader::readRotation(const Mapping& node, Matrix3& out_R) const
{
    return impl->readRotation(&node, out_R);
}


bool StdBodyLoader::readRotation(const Mapping* node, const char* key, Matrix3& out_R) const
{
    return impl->readRotation(node, key, out_R);
}


bool StdBodyLoader::readRotation
(const Mapping* node, std::initializer_list<const char*> keys, Matrix3& out_R) const
{
    return impl->sceneReader.readRotation(node, keys, out_R);
}


bool StdBodyLoader::readRotation(const Mapping& node, const char* key, Matrix3& out_R) const
{
    return impl->readRotation(&node, key, out_R);
}


bool StdBodyLoader::Impl::clear()
{
    rootLink = nullptr;
    linkInfos.clear();
    linkMap.clear();
    jointNameSet.clear();
    nameStack.clear();
    transformStack.clear();
    rigidBodies.clear();
    sceneGroupSetStack.clear();
    sceneReader.clear();
    validJointIdSet.clear();
    numValidJointIds = 0;
    subBodyMap.clear();
    subBodies.clear();
    deviceInfoCloneMap.clear();
    return true;
}    


bool StdBodyLoader::load(Body* body, const std::string& filename)
{
    return impl->load(body, filename);
}


bool StdBodyLoader::Impl::load(Body* body, const std::string& filename)
{
    mainFilePath = filesystem::absolute(fromUTF8(filename));
    
    bool result = false;

    try {
        MappingPtr data = reader.loadDocument(filename)->toMapping();
        if(data){
            if(body->modelName().empty()){
                // This is the default model name
                body->setModelName(
                    toUTF8(filesystem::path(fromUTF8(filename)).stem().string()));
            }
            result = readTopNode(body, data);
        }
    } catch(const ValueNode::Exception& ex){
        os() << ex.message();
    }

    os().flush();

    reader.clearDocuments();
    
    return result;
}


bool StdBodyLoader::read(Body* body, Mapping* topNode)
{
    return impl->readTopNode(body, topNode);
}


bool StdBodyLoader::Impl::readTopNode(Body* body, Mapping* topNode)
{
    clear();

    updateCustomNodeFunctions();
    
    bool result = false;
    this->body = body;
    body->clearDevices();
    body->clearExtraJoints();

    try {
        if(checkFormat(topNode)){
            result = readBody(topNode);
        } else {
            result = loadAnotherFormatBodyFile(topNode);
        }
        if(result){
            for(auto& subBody : subBodies){
                topNode->insert(subBody->info());
            }

            auto bodyHandlers = topNode->extract("body_handlers");

            if(!isDegreeMode()){
                topNode->setForcedRadianMode();
            }
            body->resetInfo(topNode);

            if(bodyHandlers){
                readBodyHandlers(bodyHandlers);
            }
        }
        
    } catch(const ValueNode::Exception& ex){
        os() << ex.message();
    } catch(const std::exception& ex){
        os() << ex.what() << endl;
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


bool StdBodyLoader::Impl::checkFormat(Mapping* topNode)
{
    auto formatNode = topNode->extract("format");
    if(formatNode){
        if(formatNode->toString() == "ChoreonoidBody"){
            return true;
        }
        formatNode->throwException(
            _("The file format cannot be loaded as a Choreonoid body model"));
    }

    return false;
}


bool StdBodyLoader::Impl::loadAnotherFormatBodyFile(Mapping* topNode)
{
    auto modelFileNode = topNode->extract({ "model_file", "modelFile" });
    if(!modelFileNode){
        topNode->throwException(_("Neither format nor modelFile are specified"));
    }

    filesystem::path path(fromUTF8(modelFileNode->toString()));
    if(!path.has_root_path()){
        path = mainFilePath.parent_path() / path;
    }
    if(!bodyLoader){
        bodyLoader.reset(new BodyLoader);
    }

    bodyLoader->setMessageSink(os());
    bodyLoader->setVerbose(isVerbose);
    bodyLoader->setShapeLoadingEnabled(isShapeLoadingEnabled);
    bodyLoader->setDefaultCreaseAngle(defaultCreaseAngle);

    int dn = defaultDivisionNumber;
    auto geometryNode = topNode->extract("geometry");
    if(geometryNode){
        geometryNode->toMapping()->read("divisionNumber", dn);
    }
    bodyLoader->setDefaultDivisionNumber(dn);

    bool loaded = bodyLoader->load(body, toUTF8(path.string()));

    if(loaded){
        auto linkInfo = topNode->findMapping("linkInfo");
        if(linkInfo->isValid()){
            auto p = linkInfo->begin();
            while(p != linkInfo->end()){
                auto& linkName = p->first;
                auto node = p->second;
                if(node->isMapping()){
                    auto link = body->link(linkName);
                    if(link){
                        link->info()->insert(node->toMapping());
                    }
                }
                ++p;
            }
        }
    }
    
    return loaded;
}
            

bool StdBodyLoader::Impl::readBody(Mapping* topNode)
{
    auto versionNode = topNode->extract({ "format_version", "formatVersion" });
    if(!versionNode){
        topNode->throwException(_("The version of the Choreonoid body file format is not specified"));
    }
    double version = versionNode->toDouble();
    if(version >= 2.1){
        topNode->throwException(
            formatR(_("Version {0} of the Choreonoid body format is not supported"), version));
    }

    sceneReader.setBaseDirectory(toUTF8(mainFilePath.parent_path().string()));
    sceneReader.setDefaultDivisionNumber(defaultDivisionNumber);
    sceneReader.readHeader(topNode, version);

    if(extract(topNode, "name", symbol)){
        body->setModelName(symbol);
    }

    transformStack.clear();
    transformStack.push_back(Affine3::Identity());
    auto links = topNode->extract("links");
    string unspecifiedType;
    if(!links){
        topNode->throwException(_("There is no \"links\" values for defining the links in the body"));
    } else {
        if(links->isListing()){
            Listing& linkList = *links->toListing();
            if(linkList.empty()){
                linkList.throwException(_("No link is contained in the \"links\" listing"));
            }
            for(int i=0; i < linkList.size(); ++i){
                readNodeInLinks(linkList[i].toMapping(), unspecifiedType);
            }
        } else if(links->isMapping()){
            auto linksMap = links->toMapping();
            auto p = linksMap->begin();
            while(p != linksMap->end()){
                const string& type = p->first;
                auto node = p->second->toMapping();
                readNodeInLinks(node, type);
                ++p;
            }
        } else {
            links->throwException(_("Invalid value specified in the \"links\" key"));
        }
    }

    if(linkInfos.empty()){
        topNode->throwException(_("There is no link defined"));
    }

    auto rootLinkNode = topNode->extract({ "root_link", "rootLink" });
    if(rootLinkNode){
        string rootLinkName = rootLinkNode->toString();
        auto p = linkMap.find(rootLinkName);
        if(p == linkMap.end()){
            rootLinkNode->throwException(
                formatR(_("Link \"{}\" specified in \"rootLink\" is not defined"), rootLinkName));
        }
        rootLink = p->second;

    } else {
        rootLink = linkInfos[0]->link;
    }

    // construct a link tree
    for(size_t i=0; i < linkInfos.size(); ++i){
        LinkInfo* info = linkInfos[i];
        Link* link = info->link;
        const std::string& parent = info->parent;
        if(parent.empty()){
            if(info->link != rootLink){
                info->node->throwException(
                    formatR(_("The parent of {} is not specified"), link->name()));
            }
        } else {
            auto p = linkMap.find(parent);
            if(p == linkMap.end()){
                info->node->throwException(
                    formatR(_("Parent link \"{0}\" of {1} is not defined"), parent, link->name()));
            } else {
                Link* parentLink = p->second;
                if(link->isOwnerOf(parentLink)){
                    info->node->throwException(
                        formatR(_("Adding \"{0}\" to link \"{1}\" will result in a cyclic reference"),
                                link->name(), parent));
                }
                parentLink->appendChild(link);
            }
        }
    }        

    body->setRootLink(rootLink);

    if(!isSubLoader){
        // Warn empty joint ids
        if(numValidJointIds < validJointIdSet.size()){
            for(size_t i=0; i < validJointIdSet.size(); ++i){
                if(!validJointIdSet[i]){
                    os() << formatR(_("Warning: Joint ID {} is not specified."), i) << endl;
                }
            }
        }
        body->sortDevicesByLinkOrder();
    }

    readExtraJoints(topNode);

    body->installCustomizer(); // deprecated

    return true;
}


void StdBodyLoader::Impl::readNodeInLinks(Mapping* node, const string& nodeType)
{
    const string* pNodeType = nullptr;
    auto typeNode = node->extract("type");
    if(!typeNode){
        pNodeType = &nodeType;
    } else {
        auto& typeNodeValue = typeNode->toString();
        if(!nodeType.empty() && typeNodeValue != nodeType){
            node->throwException(
                formatR(_("The node type \"{0}\" is different from the type \"{1}\" specified in the parent node"),
                        typeNodeValue, nodeType));
        }
        pNodeType = &typeNodeValue;
    }

    const auto& type = *pNodeType;
    if(type.empty() || type == "Link"){
        readLinkNode(node);
    } else if(type == "ContinuousTrack"){
        readContinuousTrackNode(node);
    } else if(type == "SubBody"){
        readSubBodyNode(node);
    } else if(type == "Skip"){

    } else {
        node->throwException(
            formatR(_("A {} node cannot be specified in links"), type));
    }
}


void StdBodyLoader::Impl::readLinkNode(Mapping* linkNode)
{
    LinkInfoPtr info = new LinkInfo;
    extract(linkNode, "parent", info->parent);
    info->link = readLinkContents(linkNode);
    info->node = linkNode;
    linkInfos.push_back(info);
}


void StdBodyLoader::Impl::setLinkName(Link* link, const string& name, ValueNode* node)
{
    link->setName(name);
    
    if(!linkMap.insert(make_pair(name, link)).second){
        node->throwException(formatR(_("Duplicated link name \"{}\""), name));
    }
}


void StdBodyLoader::Impl::setJointName(Link* link, const string& jointName, ValueNode* node)
{
    link->setJointName(jointName);
    
    if(!jointNameSet.insert(jointName).second){
        node->throwException(formatR(_("Duplicated joint name \"{}\""), jointName));
    }
}


LinkPtr StdBodyLoader::Impl::readLinkContents(Mapping* node, LinkPtr link)
{
    bool isSubBodyNode = (link != nullptr);
    
    if(!isSubBodyNode){
        link = body->createLink();
        auto nameNode = node->extract("name");
        if(nameNode){
            setLinkName(link, nameNode->toString(), nameNode);
        }
        auto jointNameNode = node->extract("joint_name");
        if(jointNameNode){
            setJointName(link, jointNameNode->toString(), jointNameNode);
        }
    }

    if(extractTranslation(node, v)){
        link->setOffsetTranslation(v);
    }
    Matrix3 R;
    if(extractRotation(node, R)){
        link->setOffsetRotation(R);
    }

    readJointContents(link, node);

    if(node->read("material", symbol)){
        link->setMaterial(symbol);
    }
    
    currentLink = link;
    rigidBodies.clear();

    ValueNodePtr elementsNode = node->extract("elements");
    if(elementsNode){
        currentModelType = VISUAL_AND_COLLISION;
        hasVisualOrCollisionNodes = false;

        sceneGroupSetStack.push_back(SceneGroupSet());
        currentSceneGroupSet().newGroup<SgGroup>();
        
        if(readElementContents(elementsNode)){
            SceneGroupSet& sgs = currentSceneGroupSet();
            sgs.setName(link->name());
            if(hasVisualOrCollisionNodes){
                for(auto& node : *sgs.visual){
                    link->addVisualShapeNode(node);
                }
                for(auto& node : *sgs.collision){
                    link->addCollisionShapeNode(node);
                }
            } else {
                for(auto& node : *sgs.visual){
                    link->addShapeNode(node);
                }
            }
        }

        sceneGroupSetStack.pop_back();
    }

    if(!isSubBodyNode){
        RigidBody rbody;
        bool hasCoM = extractEigen(node, { "center_of_mass", "centerOfMass" }, rbody.c);
        bool hasMass = extract(node, "mass", rbody.m);
        bool hasInertia = extractInertia(node, "inertia", rbody.I);
        if(hasCoM || hasMass || hasInertia){
            if(rigidBodies.empty()){
                if(!hasCoM) rbody.c = link->c();
                if(!hasMass) rbody.m = link->m();
                if(!hasInertia) rbody.I = link->I();
            } else {
                if(!hasCoM) rbody.c.setZero();
                if(!hasMass) rbody.m = 0.0;
                if(!hasInertia) rbody.I.setZero();
            }
            rigidBodies.push_back(rbody);
        }
        if(!rigidBodies.empty()){
            setMassParameters(link);
        }

        ValueNode* import = node->find("import");
        if(import->isValid()){
            if(import->isMapping()){
                node->insert(import->toMapping());
            } else if(import->isListing()){
                Listing& importList = *import->toListing();
                for(int i=importList.size() - 1; i >= 0; --i){
                    node->insert(importList[i].toMapping());
                }
            }

        }

        if(!isDegreeMode()){
            node->setForcedRadianMode();
        }
        link->resetInfo(node);
    }

    currentLink = nullptr;

    return link;
}


void StdBodyLoader::Impl::setJointId(Link* link, int id)
{
    link->setJointId(id);
    if(id >= 0){
        if(id >= static_cast<int>(validJointIdSet.size())){
            validJointIdSet.resize(id + 1);
        }
        if(!validJointIdSet[id]){
            ++numValidJointIds;
            validJointIdSet[id] = true;
        } else {
            os() << formatR(_("Warning: Joint ID {0} of {1} is duplicated."),
                            id, link->name()) << endl;
        }
    }
}


void StdBodyLoader::Impl::readJointContents(Link* link, Mapping* node)
{
    if(auto jointIdNode = node->extract({ "joint_id", "jointId" })){
        setJointId(link, jointIdNode->toInt());
    }

    auto jointTypeNode = node->extract({ "joint_type", "jointType" });
    if(jointTypeNode){
        auto& jointType = jointTypeNode->toString();
        if(jointType == "revolute"){
            link->setJointType(Link::RevoluteJoint);
        } else if(jointType == "prismatic"){
            link->setJointType(Link::PrismaticJoint);
        } else if(jointType == "slide"){
            link->setJointType(Link::PrismaticJoint);
        } else if(jointType == "free"){
            link->setJointType(Link::FreeJoint);
        } else if(jointType == "fixed"){
            link->setJointType(Link::FixedJoint);
        } else if(jointType == "pseudo_continuous_track" || jointType == "pseudoContinuousTrack"){
            link->setJointType(Link::PseudoContinuousTrackJoint);
            link->setActuationMode(Link::JointVelocity);
        } else {
            jointTypeNode->throwException(
                formatR(_("Illegal jointType value \"{0}\""), jointType));
        }
    }

    if(auto axisNode = node->extract({ "joint_axis", "jointAxis" })){
        readAxis(axisNode, v);
        link->setJointAxis(v);
    }

    auto actuationModeNode = node->extract({ "actuation_mode", "actuationMode" });
    if(actuationModeNode){
        string mode = actuationModeNode->toString();

        //! \todo Use a map bewteen mode string and actuation mode id
        if(mode == "joint_effort" || mode == "jointEffort" || mode == "jointTorque" || mode == "jointForce"){
            link->setActuationMode(Link::JointEffort);

        } else if(mode == "joint_displacement" || mode == "jointDisplacement" || mode == "jointAngle"){
            link->setActuationMode(Link::JointDisplacement);

        } else if(mode == "joint_velocity" || mode == "jointVelocity"){
            link->setActuationMode(Link::JointVelocity);

        } else if(mode == "jointSurfaceVelocity"){ // deprecated
            if(jointTypeNode &&
               (link->jointType() != Link::PseudoContinuousTrackJoint || link->jointType() != Link::FixedJoint)){
                os() << formatR(_("Warning: Actuation mode \"jointSurfaceVelocity\" is specified in {0}. "
                                  "The mode is deprecated and the joint type should be the pseudo continuous track in this case."),
                                link->name()) << endl;
            }
            link->setJointType(Link::PseudoContinuousTrackJoint);
            link->setActuationMode(Link::JointVelocity);
            
        } else if(mode == "link_position" || mode == "linkPosition"){
            link->setActuationMode(Link::LinkPosition);

        } else {
            actuationModeNode->throwException("Illegal actuationMode value");
        }
    }

    auto jointAngleNode = node->extract({"joint_angle", "jointAngle"});
    if(jointAngleNode){
        link->setInitialJointDisplacement(toRadian(jointAngleNode->toDouble()));
    }
    auto jointDisplacementNode = node->extract({ "joint_displacement", "jointDisplacement" });
    if(jointDisplacementNode){
        if(link->isRevoluteJoint()){
            link->setInitialJointDisplacement(toRadian(jointDisplacementNode->toDouble()));
        } else {
            link->setInitialJointDisplacement(jointDisplacementNode->toDouble());
        }
    }

    readJointDisplacementRange(node, link, isDegreeMode(), true);
    readJointVelocityRange(node, link, isDegreeMode(), true);
    readJointEffortRange(node, link, true);

    double Ir = 0.0;
    auto jointAxisInertiaNode = node->extract("joint_axis_inertia");
    if(jointAxisInertiaNode){
        Ir = jointAxisInertiaNode->toDouble();
    } else {
        Ir = node->get({"rotor_inertia", "rotorInertia"}, 0.0);
        double gearRatio;
        if(node->read({"gear_ratio", "gearRatio"}, gearRatio)){
            Ir = gearRatio * gearRatio * Ir;
        }
    }
    link->setEquivalentRotorInertia(Ir);
}


void StdBodyLoader::Impl::readAxis(ValueNode* node, Vector3& out_axis)
{
    if(node->isListing()){
        readEx(node->toListing(), out_axis);

    } else if(node->isString()){
        string symbol = node->toString();
        std::transform(symbol.cbegin(), symbol.cend(), symbol.begin(), ::toupper);
        if(symbol == "X"){
            out_axis = Vector3::UnitX();
        } else if(symbol == "-X"){
            out_axis = -Vector3::UnitX();
        } else if(symbol == "Y"){
            out_axis = Vector3::UnitY();
        } else if(symbol == "-Y"){
            out_axis = -Vector3::UnitY();
        } else if(symbol == "Z"){
            out_axis = Vector3::UnitZ();
        } else if(symbol == "-Z"){
            out_axis = -Vector3::UnitZ();
        } else {
            node->throwException("Illegal axis value");
        }
    }
}


bool StdBodyLoader::readJointDisplacementRange(const Mapping* node, Link* link)
{
    return Impl::readJointDisplacementRange(const_cast<Mapping*>(node), link, true, false);
}


bool StdBodyLoader::Impl::readJointDisplacementRange(Mapping* node, Link* link, bool isDegree, bool doExtract)
{
    bool found = false;
    
    double lower = -std::numeric_limits<double>::max();
    double upper =  std::numeric_limits<double>::max();

    ValueNodePtr rangeNode;
    if(doExtract){
        rangeNode = node->extract({ "joint_range", "jointRange" });
    } else {
        rangeNode = node->find({ "joint_range", "jointRange" });
        if(!rangeNode->isValid()){
            rangeNode.reset();
        }
    }
    if(rangeNode){
        if(rangeNode->isScalar()){
            upper = readLimitValue(*rangeNode, true);
            lower = -upper;
        } else if(rangeNode->isListing()){
            Listing& rangeList = *rangeNode->toListing();
            if(rangeList.size() != 2){
                rangeList.throwException(_("jointRange must have two elements"));
            }
            lower = readLimitValue(rangeList[0], false);
            upper = readLimitValue(rangeList[1], true);
        } else {
            rangeNode->throwException(_("Invalid joint range value"));
        }
        found = true;
    }
    if(link->isRevoluteJoint() && isDegree){
        link->setJointRange(
            lower == -std::numeric_limits<double>::max() ? lower : radian(lower),
            upper ==  std::numeric_limits<double>::max() ? upper : radian(upper));
    } else {
        link->setJointRange(lower, upper);
    }

    return found;
}


bool StdBodyLoader::readJointVelocityRange(const Mapping* node, Link* link)
{
    return Impl::readJointVelocityRange(const_cast<Mapping*>(node), link, true, false);
}


bool StdBodyLoader::Impl::readJointVelocityRange(Mapping* node, Link* link, bool isDegree, bool doExtract)
{
    bool found = false;

    double lower = -std::numeric_limits<double>::max();
    double upper =  std::numeric_limits<double>::max();
    ValueNodePtr rangeNode;
    
    if(doExtract){
        rangeNode = node->extract({"max_joint_velocity", "maxJointVelocity"});
    } else {
        rangeNode = node->find({"max_joint_velocity", "maxJointVelocity"});
        if(!rangeNode->isValid()){
            rangeNode.reset();
        }
    }
    if(rangeNode){
        upper = readLimitValue(*rangeNode, true);
        lower = -upper;
        found = true;
    }

    if(!found){
        if(doExtract){
            rangeNode = node->extract({ "joint_velocity_range", "jointVelocityRange" });
        } else {
            rangeNode = node->find({ "joint_velocity_range", "jointVelocityRange" });
            if(!rangeNode->isValid()){
                rangeNode.reset();
            }
        }
        if(rangeNode){
            Listing& rangeList = *rangeNode->toListing();
            if(rangeList.size() != 2){
                rangeList.throwException(_("The joint velocity range node must have two elements"));
            }
            lower = readLimitValue(rangeList[0], false);
            upper = readLimitValue(rangeList[1], true);
            found = true;
        }
    }

    if(link->isRevoluteJoint() && isDegree){
        link->setJointVelocityRange(
            lower == -std::numeric_limits<double>::max() ? lower : radian(lower),
            upper ==  std::numeric_limits<double>::max() ? upper : radian(upper));
    } else {
        link->setJointVelocityRange(lower, upper);
    }

    return found;
}


bool StdBodyLoader::readJointEffortRange(const Mapping* node, Link* link)
{
    return Impl::readJointEffortRange(const_cast<Mapping*>(node), link, false);
}


bool StdBodyLoader::Impl::readJointEffortRange(Mapping* node, Link* link, bool doExtract)
{
    bool found = false;
    ValueNodePtr maxNode;
    if(doExtract){
        maxNode = node->extract("max_joint_effort");
    } else {
        maxNode = node->find("max_joint_effort");
        if(!maxNode->isValid()){
            maxNode.reset();
        }
    }
    if(maxNode){
        double effort = readLimitValue(*maxNode, true);
        link->setJointEffortRange(-effort, effort);
        found = true;
    }
    return found;
}


void StdBodyLoader::Impl::setMassParameters(Link* link)
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


bool StdBodyLoader::Impl::readElements(Mapping* node)
{
    bool isSceneNodeAdded = false;
    auto elementsNode = node->find("elements");
    if(elementsNode->isValid()){
        isSceneNodeAdded = readElementContents(elementsNode);
    }
    return isSceneNodeAdded;
}


bool StdBodyLoader::Impl::readElementContents(ValueNode* elementsNode)
{
    bool isSceneNodeAdded = false;

    if(elementsNode->isListing()){
        /*
          Process the case like
          elements:
            -
              type: Transform
              translation: [ 1, 0, 0 ]
        */
        for(auto& node : *elementsNode->toListing()){
            auto element = node->toMapping();
            auto& type = (*element)["type"].toString();
            if(readNode(element, type)){
                isSceneNodeAdded = true;
            }
        }
    } else if(elementsNode->isMapping()){
        auto mapping = elementsNode->toMapping();

        /* Check the case like
           elements:
             type: Transform
             translation: [ 1, 0, 0 ]
        */
        ValueNode* typeNode = mapping->find("type");
        if(typeNode->isValid()){
            auto& type = typeNode->toString();
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
            for(auto& kv : *mapping){
                const string& type =kv.first;
                auto element = kv.second->toMapping();
                ValueNode* typeNode = element->find("type");
                if(typeNode->isValid()){
                    auto& type2 = typeNode->toString();
                    if(type2 != type){
                        element->throwException(
                            formatR(_("The node type \"{0}\" is different from the type \"{1}\" specified in the parent node"),
                                    type2, type));
                    }
                }
                if(readNode(element, type)){
                    isSceneNodeAdded = true;
                }
            }
        }
    } else {
        elementsNode->throwException(_("A value of the \"elements\" key must be a sequence or a mapping"));
    }

    return isSceneNodeAdded;
}


bool StdBodyLoader::Impl::readNode(Mapping* node, const string& type)
{
    bool isSceneNodeAdded = false;
    
    auto p = nodeFunctions.find(type);
    if(p != nodeFunctions.end()){
        NodeFunctionInfo& info = p->second;

        nameStack.emplace_back();
        node->read("name", nameStack.back());
        
        if(!isDegreeMode()){
            node->setForcedRadianMode();
        }

        if(info.isTransformDerived){
            if(readTransformContents(node, info.function, info.hasElements)){
                isSceneNodeAdded = true;
            }
        } else {
            if(info.function(node)){
                isSceneNodeAdded = true;
            }
        }

        nameStack.pop_back();
        
    } else if(isShapeLoadingEnabled){
        if(auto scene = sceneReader.readNode(node, type)){
            addScene(scene);
            isSceneNodeAdded = true;
        }
    }

    return isSceneNodeAdded;
}


bool StdBodyLoader::Impl::readSkipNode(Mapping* /* node */)
{
    return false;
}


bool StdBodyLoader::Impl::readContainerNode(Mapping* node, NodeFunction nodeFunction)
{
    bool isSceneNodeAdded = false;

    if(nodeFunction){
        if(nodeFunction(node)){
            isSceneNodeAdded = true;
        }
    }

    auto elementsNode = node->find("elements");
    if(elementsNode->isValid()){
        if(readElementContents(elementsNode)){
            isSceneNodeAdded = true;
        }
    }

    return isSceneNodeAdded;
}


bool StdBodyLoader::Impl::readTransformContents(Mapping* node, NodeFunction nodeFunction, bool hasElements)
{
    Affine3 T = Affine3::Identity();
    bool hasPosTransform = false;
    bool hasScale = false;
    bool isSceneNodeAdded = false;
    
    if(readTranslation(node, v)){
        T.translation() = v;
        hasPosTransform = true;
    }
    if(readRotation(node, M)){
        T.linear() = M;
        hasPosTransform = true;
    }

    Affine3 Ts(T);
    Vector3 scale;
    if(cnoid::read(node, "scale", scale)){
        Ts.linear() *= scale.asDiagonal();
        hasScale = true;
    }
    
    if(hasPosTransform || hasScale){
        transformStack.push_back(transformStack.back() * Ts);
    }

    sceneGroupSetStack.push_back(SceneGroupSet());
    if(hasPosTransform){
        currentSceneGroupSet().newGroup<SgPosTransform>(Isometry3(T.matrix()));
    } else if(hasScale){
        currentSceneGroupSet().newGroup<SgScaleTransform>(scale);
    } else {
        currentSceneGroupSet().newGroup<SgGroup>();
    }
    currentSceneGroupSet().setName(nameStack.back());
    
    if(hasPosTransform && hasScale){
        sceneGroupSetStack.push_back(SceneGroupSet());
        currentSceneGroupSet().newGroup<SgScaleTransform>(scale);
        currentSceneGroupSet().setName(nameStack.back());
    }
    
    if(hasElements){
        isSceneNodeAdded = readContainerNode(node, nodeFunction);
    } else {
        isSceneNodeAdded = nodeFunction(node);
    }

    if(isSceneNodeAdded){
        addCurrentSceneGroupToParentSceneGroup();
    }

    if(hasPosTransform && hasScale){
        sceneGroupSetStack.pop_back();
        if(isSceneNodeAdded){
            addCurrentSceneGroupToParentSceneGroup();
        }
    }
    sceneGroupSetStack.pop_back();

    if(hasPosTransform || hasScale){
        transformStack.pop_back();
    }

    return isSceneNodeAdded;
}


bool StdBodyLoader::Impl::readGroup(Mapping* node)
{
    sceneGroupSetStack.push_back(SceneGroupSet());
    currentSceneGroupSet().newGroup<SgGroup>();
    currentSceneGroupSet().setName(nameStack.back());

    bool isSceneNodeAdded = readContainerNode(node, 0);
    if(isSceneNodeAdded){
        addCurrentSceneGroupToParentSceneGroup();
    }
    sceneGroupSetStack.pop_back();
    return isSceneNodeAdded;
}


bool StdBodyLoader::Impl::readTransform(Mapping* node)
{
    return readTransformContents(node, 0, true);
}


bool StdBodyLoader::Impl::readRigidBody(Mapping* node)
{
    RigidBody rbody;
    const Affine3& T = transformStack.back();

    if(!cnoid::read(node, { "center_of_mass", "centerOfMass" }, v)){
        v.setZero();
    }
    rbody.c = T.linear() * v + T.translation();

    if(!node->read("mass", rbody.m)){
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


bool StdBodyLoader::Impl::readVisualOrCollision(Mapping* node, bool isVisual)
{
    ModelType prevModelType = currentModelType;

    if(isVisual){
        if(currentModelType == COLLISION){
            node->throwException(
                _("The visual node is conflicting with the Collision node defined at the higher level"));
        }
        currentModelType = VISUAL;
    } else {
        if(currentModelType == VISUAL){
            node->throwException(
                _("The collision node is conflicting with the Visual node defined at the higher level"));
        }
        currentModelType = COLLISION;
    }

    bool isSceneNodeAdded = readTransformContents(
        node, [this](Mapping* node){ return readVisualOrCollisionContents(node); }, true);

    if(isSceneNodeAdded){
        hasVisualOrCollisionNodes = true;
    }

    currentModelType = prevModelType;

    return isSceneNodeAdded;
}


bool StdBodyLoader::Impl::readVisualOrCollisionContents(Mapping* node)
{
    bool isSceneNodeAdded = false;
    
    if(isShapeLoadingEnabled){
        auto resourceNode = node->findMapping("resource");
        if(resourceNode->isValid()){
            isSceneNodeAdded = readResource(resourceNode);
        }
        auto shape = node->findMapping("shape");
        if(shape->isValid()){
            if(auto scene = sceneReader.readNode(shape, "Shape")){
                addScene(scene);
                isSceneNodeAdded = true;
            }
        }
    }

    return isSceneNodeAdded;
}
 
    
bool StdBodyLoader::Impl::readResource(Mapping* node)
{
    bool isSceneNodeAdded = false;
    
    auto resource = sceneReader.readResourceNode(node);

    if(resource.scene){
        addScene(resource.scene);
        isSceneNodeAdded = true;

    } else if(resource.info){
        string orgBaseDirectory = sceneReader.baseDirectory();
        sceneReader.setBaseDirectory(resource.directory);

        //isSceneNodeAdded = readElementContents(*resource.node);
        ValueNodePtr resourceNode = resource.info;
        isSceneNodeAdded = readTransformContents(
            node,
            [this, resourceNode](Mapping*){ return readElementContents(resourceNode); },
            false);

        sceneReader.setBaseDirectory(orgBaseDirectory);
    }

    return isSceneNodeAdded;
}
        

bool StdBodyLoader::readDevice(Device* device, const Mapping* info)
{
    return impl->readDevice(device, info);
}


bool StdBodyLoader::readDevice(Device* device, const Mapping& info)
{
    return impl->readDevice(device, &info);
}


bool StdBodyLoader::Impl::readDevice(Device* device, const Mapping* info)
{
    device->setName(nameStack.back());

    if(info->read("id", id)) device->setId(id);
    if(info->read("on", on)) device->on(on);

    const Affine3& T = transformStack.back();
    device->setLocalTranslation(T.translation());
    device->setLocalRotation(T.linear());
    body->addDevice(device, currentLink);

    device->resetInfo(info->clone(deviceInfoCloneMap));

    return false;
}


void StdBodyLoader::Impl::readContinuousTrackNode(Mapping* node)
{
    string parent;
    if(!extract(node, "parent", parent)){
        node->throwException("parent must be specified");
    }

    int numJoints = 0;
    if(!extract(node, "numJoints", numJoints)){
        node->throwException("numJoints must be specified");
    }
    if(numJoints < 3){
        node->throwException("numJoints must be more than 2");
    }

    Vector3 jointOffset;
    if(!extractEigen(node, { "joint_offset", "jointOffset", }, jointOffset)){
        node->throwException("jointOffset must be specified");
    }

    const int numOpenJoints = numJoints - 1;
    vector<double> initialAngles(numOpenJoints, 0.0);
    ValueNodePtr initAnglesNode = node->extract("initialJointAngles");
    if(initAnglesNode){
        Listing& initAngles = *initAnglesNode->toListing();
        const int n = std::min(initAngles.size(), numOpenJoints);
        for(int i=0; i < n; i++){
            initialAngles[i] = toRadian(initAngles[i].toDouble());
        }
    }

    LinkPtr firstLink = readLinkContents(node);
    LinkPtr subsequentLink = firstLink->clone();
    subsequentLink->resetInfo(subsequentLink->info()->clone());
    
    firstLink->setJointType(Link::FreeJoint);
    firstLink->setInfo("isContinuousTrack", true);
    addTrackLink(0, firstLink, node, parent, 0.0);

    subsequentLink->setJointType(Link::RevoluteJoint);
    subsequentLink->setOffsetTranslation(jointOffset);
    for(int i=0; i < numOpenJoints; ++i){
        addTrackLink(i + 1, subsequentLink->clone(), node, parent, initialAngles[i]);
    }
}


void StdBodyLoader::Impl::addTrackLink(int index, LinkPtr link, Mapping* node, string& io_parent, double initialAngle)
{
    setLinkName(link, formatC("{0}{1}", link->name(), index), node);

    link->setInitialJointAngle(initialAngle);

    LinkInfoPtr linkInfo = new LinkInfo;
    linkInfo->link = link;
    linkInfo->node = node;
    linkInfo->parent = io_parent;
    linkInfos.push_back(linkInfo);
    
    io_parent = link->name();
}


void StdBodyLoader::Impl::readSubBodyNode(Mapping* node)
{
    string uri;
    if(!node->read("uri", uri)){
        node->throwException("uri must be specified");
    }
    
    filesystem::path filepath(fromUTF8(uri));
    if(filepath.is_relative()){
        filepath = filesystem::lexically_normal(sceneReader.baseDirPath() / filepath);
    }
    if(filesystem::equivalent(mainFilePath, filepath)){
        node->throwException("recursive sub-body is prohibited");
    }

    BodyPtr subBody;
    string filename = toUTF8(filepath.string());
    auto iter = subBodyMap.find(filename);
    if(iter != subBodyMap.end()){
        subBody = iter->second;
    } else {
        try {
            if(!subLoader){
                subLoader.reset(new StdBodyLoader);
                subLoader->setMessageSink(*os_);
                subLoader->impl->isSubLoader = true;
            }
            subLoader->setDefaultDivisionNumber(sceneReader.defaultDivisionNumber());
                
            subBody = new Body;
            if(subLoader->load(subBody, filename)){
                subBodyMap[filename] = subBody;
            } else {
                os() << formatR(_("SubBody specified by uri \"{}\" cannot be loaded."), uri) << endl;
                subBody.reset();
            }
        } catch(const ValueNode::Exception& ex){
            os() << ex.message();
        }
    }

    if(subBody){
        addSubBodyLinks(subBody, node);
        subBodies.push_back(subBody);
    }
}


void StdBodyLoader::Impl::addSubBodyLinks(const Body* subBody, Mapping* node)
{
    string prefix;
    node->read("prefix", prefix);
    string suffix;
    node->read("suffix", suffix);
    string devicePrefix;
    if(!node->read({ "device_prefix", "devicePrefix" }, devicePrefix)){
        devicePrefix = prefix;
    }

    int jointIdOffset = node->get({ "joint_id_offset", "jointIdOffset" }, 0);

    BodyPtr subBodyClone = subBody->clone();

    for(int i=0; i < subBodyClone->numLinks(); ++i){
        Link* link = subBodyClone->link(i);
        setLinkName(link, prefix + link->name() + suffix, node);
        if(link->jointId() >= 0){
            setJointId(link, link->jointId() + jointIdOffset);
        }
        if(!link->jointSpecificName().empty()){
            setJointName(link, prefix + link->jointSpecificName() + suffix, node);
        }
    }

    const int numExtraJoints = subBodyClone->numExtraJoints();
    for(int i=0; i < numExtraJoints; ++i){
        body->addExtraJoint(subBodyClone->extraJoint(i));
    }

    struct DeviceInfo {
        DevicePtr device;
        LinkPtr link;
        DeviceInfo(Device* device, Link* link)
            : device(device), link(link) { }
    };
    vector<DeviceInfo> subBodyDevices;

    const int numDevices = subBodyClone->numDevices();
    subBodyDevices.reserve(numDevices);
    for(int i=0; i < numDevices; ++i){
        auto device = subBodyClone->device(i);
        device->setName(devicePrefix + device->name());
        subBodyDevices.emplace_back(device, device->link());
    }

    LinkPtr rootLink = subBodyClone->rootLink();
    
    LinkInfoPtr linkInfo = new LinkInfo;
    linkInfo->link = rootLink;
    linkInfo->node = node;
    node->read("parent", linkInfo->parent);
    linkInfos.push_back(linkInfo);

    // Delete the sub body clone to relase its links and devices
    // so that the devices can be added to the main body.
    subBodyClone.reset();

    for(auto& info : subBodyDevices){
        body->addDevice(info.device, info.link);
    }

    // Read the link contents written in the main body file.
    // The contents overwrite the sub body link contents.
    readLinkContents(node, rootLink);
}


void StdBodyLoader::Impl::readExtraJoints(Mapping* topNode)
{
    auto node = topNode->extract({ "extra_joints", "extraJoints" });
    if(node){
        if(node->isListing()){
            Listing& extraJoints = *node->toListing();
            for(int i=0; i < extraJoints.size(); ++i){
                readExtraJoint(extraJoints[i].toMapping());
            }
        }
    }
}


void StdBodyLoader::Impl::readExtraJoint(Mapping* info)
{
    ExtraJointPtr joint = new ExtraJoint;

    string link1Name;
    string link2Name;

    if(!extract(info, { "link1_name", "link1Name" }, link1Name)){
        info->throwException(_("link1_name is not specified"));
    }
    if(!extract(info, { "link2_name", "link2Name" }, link2Name)){
        info->throwException(_("link2_name is not specified"));
    }

    joint->setLink(0, body->link(link1Name));
    joint->setLink(1, body->link(link2Name));

    for(int i=0; i < 2; ++i){
        if(!joint->link(i)){
            info->throwException(
                formatR(_("The link specified in \"link{}_name\" is not found"), (i + 1)));
        }
    }

    string jointType;
    if(!extract(info, { "joint_type", "jointType" }, jointType)){
        info->throwException(_("The joint type must be specified with the \"joint_type\" key"));
    }
    if(jointType == "fixed"){
        joint->setType(ExtraJoint::Fixed);
    } else if(jointType == "hinge"){
        joint->setType(ExtraJoint::Hinge);
    } else if(jointType == "ball"){
        joint->setType(ExtraJoint::Ball);
    } else if(jointType == "piston"){
        joint->setType(ExtraJoint::Piston);
    } else {
        info->throwException(formatR(_("Joint type \"{}\" is not available"), jointType));
    }

    if(joint->type() == ExtraJoint::Hinge || joint->type() == ExtraJoint::Piston){
        if(extractEigen(info, { "axis_in_link1", "axis", "jointAxis" }, v)){
            joint->setAxis(0, v);
        }
        if(extractEigen(info, { "axis_in_link2" }, v)){
            joint->setAxis(1, v);
        }
    }

    Matrix3 R;
    if(extractRotation(info, "rotation_in_link1", R)){
        joint->setLocalRotation(0, R);
    }
    if(extractRotation(info, "rotation_in_link2", R)){
        joint->setLocalRotation(1, R);
    }
    if(extractEigen(info, { "translation_in_link1", "link1_local_pos", "link1LocalPos" }, v)){
        joint->setPoint(0, v);
    }
    if(extractEigen(info, { "translation_in_link2", "link2_local_pos", "link2LocalPos" }, v)){
        joint->setPoint(1, v);
    }
    
    joint->resetInfo(info);

    body->addExtraJoint(joint);
}


void StdBodyLoader::Impl::readBodyHandlers(ValueNode* node)
{
    if(node){
        if(node->isString()){
            bodyHandlerManager.loadBodyHandler(body, node->toString());
        } else if(node->isListing()){
            for(auto& handlerNode : *node->toListing()){
                bodyHandlerManager.loadBodyHandler(body, handlerNode->toString());
            }
        }
    }
}


void StdBodyLoader::registerNodeType
(const char* typeName, std::function<bool(StdBodyLoader* loader, const Mapping* info)> readFunction)
{
    std::lock_guard<std::mutex> guard(customNodeFunctionMutex);
    customNodeFunctions[typeName] = readFunction;
}


void StdBodyLoader::addNodeType
(const char* typeName, std::function<bool(StdBodyLoader& loader, const Mapping& info)> readFunction)
{
    registerNodeType(
        typeName,
        [readFunction](StdBodyLoader* loader, const Mapping* info){
            return readFunction(*loader, *info);
        });
}


StdBodyLoader::NodeTypeRegistration::NodeTypeRegistration
(const char* typeName, std::function<bool(StdBodyLoader& loader, const Mapping& info)> readFunction)
{
    registerNodeType(
        typeName,
        [readFunction](StdBodyLoader* loader, const Mapping* info){
            return readFunction(*loader, *info);
        });
}
