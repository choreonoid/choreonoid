/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "YAMLSceneReader.h"
#include <cnoid/SceneDrawables>
#include <cnoid/MeshGenerator>
#include <cnoid/SceneLoader>
#include <cnoid/EigenArchive>
#include <cnoid/FileUtil>
#include <cnoid/NullOut>
#include <boost/format.hpp>
#include <unordered_map>
#include <mutex>
#include <regex>
#include "gettext.h"

using namespace std;
using namespace cnoid;
namespace filesystem = boost::filesystem;
using boost::format;

namespace {

typedef SgNode* (YAMLSceneReaderImpl::*NodeFunction)(Mapping& node);
typedef unordered_map<string, NodeFunction> NodeFunctionMap;
NodeFunctionMap nodeFunctionMap;

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

unordered_map<string, YAMLSceneReader::UriSchemeHandler> uriSchemeHandlerMap;
std::mutex uriSchemeHandlerMutex;

}

namespace cnoid {

class YAMLSceneReaderImpl
{
public:
    YAMLSceneReader* self;

    ostream* os_;
    ostream& os() { return *os_; }

    // temporary variables for reading values
    double value;
    string symbol;
    Vector3f color;
    Vector3 v;
    
    MeshGenerator meshGenerator;
    SgMaterialPtr defaultMaterial;

    map<string, ResourceInfoPtr> resourceInfoMap;
    SceneLoader sceneLoader;
    filesystem::path baseDirectory;
    std::regex uriSchemeRegex;
    bool isUriSchemeRegexReady;
    
    YAMLSceneReaderImpl(YAMLSceneReader* self);
    ~YAMLSceneReaderImpl();
    SgNode* readNode(Mapping& node, const string& type);
    SgNode* readGroup(Mapping& node);
    bool readElements(Mapping& node, SgGroup* group);
    SgNode* readTransform(Mapping& node);
    SgNode* readTransformParameters(Mapping& node, SgNode* scene);
    SgNode* readShape(Mapping& node);
    SgMesh* readGeometry(Mapping& node);
    SgMesh* readBox(Mapping& node);
    SgMesh* readSphere(Mapping& node);
    SgMesh* readCylinder(Mapping& node);
    SgMesh* readCone(Mapping& node);
    SgMesh* readCapsule(Mapping& node);
    SgMesh* readExtrusion(Mapping& node);
    SgMesh* readElevationGrid(Mapping& node);
    SgMesh* readResourceAsGeometry(Mapping& node);
    void readAppearance(SgShape* shape, Mapping& node);
    void readMaterial(SgShape* shape, Mapping& node);
    void setDefaultMaterial(SgShape* shape);
    SgNode* readResource(Mapping& node);
    SgNode* loadResource(const string& uri);
    SgNode* loadResource(const string& uri, const string& nodeName);
    ResourceInfo* getOrCreateResourceInfo(const string& uri);
    filesystem::path findFileInPackage(const string& file);
    void adjustNodeCoordinate(NodeInfo& info);
    void makeNodeMap(ResourceInfo* info);
    void makeNodeMapSub(const NodeInfo& nodeInfo, NodeMap& nodeMap);
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

}


void YAMLSceneReader::registerUriSchemeHandler(const std::string& scheme, UriSchemeHandler handler)
{
    std::lock_guard<std::mutex> guard(uriSchemeHandlerMutex);
    uriSchemeHandlerMap[scheme] = handler;
}
                                                 

YAMLSceneReader::YAMLSceneReader()
{
    impl = new YAMLSceneReaderImpl(this);
    clear();
}


YAMLSceneReaderImpl::YAMLSceneReaderImpl(YAMLSceneReader* self)
    : self(self)
{
    if(nodeFunctionMap.empty()){
        nodeFunctionMap["Group"] = &YAMLSceneReaderImpl::readGroup;
        nodeFunctionMap["Transform"] = &YAMLSceneReaderImpl::readTransform;
        nodeFunctionMap["Shape"] = &YAMLSceneReaderImpl::readShape;
        nodeFunctionMap["Resource"] = &YAMLSceneReaderImpl::readResource;
    }
    os_ = &nullout();
    isUriSchemeRegexReady = false;
}


YAMLSceneReader::~YAMLSceneReader()
{
    delete impl;
}


YAMLSceneReaderImpl::~YAMLSceneReaderImpl()
{

}


void YAMLSceneReader::setMessageSink(std::ostream& os)
{
    impl->os_ = &os;
    impl->sceneLoader.setMessageSink(os);
}


void YAMLSceneReader::setDefaultDivisionNumber(int n)
{
    impl->meshGenerator.setDivisionNumber(n);
    impl->sceneLoader.setDefaultDivisionNumber(n);
}


void YAMLSceneReader::setBaseDirectory(const std::string& directory)
{
    impl->baseDirectory = directory;
}


void YAMLSceneReader::clear()
{
    isDegreeMode_ = true;
    impl->defaultMaterial = 0;
    impl->resourceInfoMap.clear();
}


void YAMLSceneReader::setAngleUnit(AngleUnit unit)
{
    isDegreeMode_ = (unit == DEGREE);
}


bool YAMLSceneReader::readAngle(Mapping& node, const char* key, double& angle)
{
    if(node.read(key, angle)){
        angle = toRadian(angle);
        return true;
    }
    return false;
}


AngleAxis YAMLSceneReader::readAngleAxis(const Listing& rotation)
{
    Vector4 r;
    cnoid::read(rotation, r);
    Vector3 axis(r[0], r[1], r[2]);
    double size = axis.norm();
    if(size < 1.0e-6){
        rotation.throwException("Rotation axis is the zero vector");
    }
    axis /= size; // normalize
    return AngleAxis(toRadian(r[3]), axis);
}

        
bool YAMLSceneReader::readRotation(Mapping& node, Matrix3& out_R, bool doExtract)
{
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
                out_R = out_R * readAngleAxis(*rotations[i].toListing());
            }
        } else {
            out_R = readAngleAxis(rotations);
        }
    }
    return true;
}


SgNode* YAMLSceneReader::readNode(Mapping& node)
{
    const string type = node["type"].toString();
    return impl->readNode(node, type);
}


SgNode* YAMLSceneReader::readNode(Mapping& node, const std::string& type)
{
    return impl->readNode(node, type);
}


SgNode* YAMLSceneReaderImpl::readNode(Mapping& node, const string& type)
{
    NodeFunctionMap::iterator q = nodeFunctionMap.find(type);
    if(q == nodeFunctionMap.end()){
        node.throwException(str(format(_("The node type \"%1%\" is not defined.")) % type));
    }

    NodeFunction readNode = q->second;
    SgNode* scene = (this->*readNode)(node);
    if(node.read("name", symbol)){
        scene->setName(symbol);
    }
    return scene;
}


SgNode* YAMLSceneReaderImpl::readGroup(Mapping& node)
{
    SgGroupPtr group = new SgGroup;
    if(!readElements(node, group)){
        group = 0; // clear group if empty
    }
    return group.retn();
}


bool YAMLSceneReaderImpl::readElements(Mapping& node, SgGroup* group)
{
    ValueNode& elements = *node.find("elements");
    if(elements.isValid()){
        if(elements.isListing()){
            Listing& listing = *elements.toListing();
            for(int i=0; i < listing.size(); ++i){
                Mapping& element = *listing[i].toMapping();
                const string type = element["type"].toString();
                SgNodePtr scene = readNode(element, type);
                if(scene){
                    group->addChild(scene);
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
                SgNodePtr scene = readNode(element, type);
                if(scene){
                    group->addChild(scene);
                }
                ++p;
            }
        }
    }
 
    return !group->empty();
}


SgNode* YAMLSceneReaderImpl::readTransform(Mapping& node)
{
    SgPosTransformPtr transform = new SgPosTransform;

    if(!readElements(node, transform)){
        transform = 0; // clear group if empty
    } else {
        if(read(node, "translation", v)){
            transform->setTranslation(v);
        }
        Matrix3 R;
        if(self->readRotation(node, R, false)){
            transform->setRotation(R);
        }
    }
    return transform.retn();
}


SgNode* YAMLSceneReaderImpl::readTransformParameters(Mapping& node, SgNode* scene)
{
    Matrix3 R;
    bool isRotated = self->readRotation(node, R, false);
    Vector3 p;
    bool isTranslated = read(node, "translation", p);
    if(isRotated || isTranslated){
        SgPosTransform* transform = new SgPosTransform;
        if(isRotated){
            transform->setRotation(R);
        }
        if(isTranslated){
            transform->setTranslation(p);
        }
        transform->addChild(scene);
        return transform;
    }
    return scene;
}


SgNode* YAMLSceneReaderImpl::readShape(Mapping& node)
{
    SgNode* scene = 0;

    Mapping& geometry = *node.findMapping("geometry");
    if(geometry.isValid()){
        SgShapePtr shape = new SgShape;
        shape->setMesh(readGeometry(geometry));

        Mapping& appearance = *node.findMapping("appearance");
        if(appearance.isValid()){
            readAppearance(shape, appearance);
        } else {
            setDefaultMaterial(shape);
        }

        scene = readTransformParameters(node, shape);

        if(scene == shape){
            return shape.retn();
        }
    }

    return scene;
}


SgMesh* YAMLSceneReaderImpl::readGeometry(Mapping& node)
{
    SgMesh* mesh;
    ValueNode& typeNode = node["type"];
    string type = typeNode.toString();
    if(type == "Box"){
        mesh = readBox(node);
    } else if(type == "Sphere"){
        mesh = readSphere(node);
    } else if(type == "Cylinder"){
        mesh = readCylinder(node);
    } else if(type == "Cone"){
        mesh = readCone(node);
    } else if(type == "Capsule"){
        mesh = readCapsule(node);
    } else if(type == "Extrusion"){
        mesh = readExtrusion(node);
    } else if(type == "ElevationGrid"){
        mesh = readElevationGrid(node);
    } else if(type == "Resource"){
        mesh = readResourceAsGeometry(node);
    } else {
        typeNode.throwException(
            str(format(_("Unknown geometry \"%1%\"")) % type));
    }
    return mesh;
}


SgMesh* YAMLSceneReaderImpl::readBox(Mapping& node)
{
    Vector3 size;
    if(!read(node, "size", size)){
        size.setOnes(1.0);
    }
    return meshGenerator.generateBox(size);
}


SgMesh* YAMLSceneReaderImpl::readSphere(Mapping& node)
{
    return meshGenerator.generateSphere(node.get("radius", 1.0));
}


SgMesh* YAMLSceneReaderImpl::readCylinder(Mapping& node)
{
    double radius = node.get("radius", 1.0);
    double height = node.get("height", 1.0);
    bool bottom = node.get("bottom", true);
    bool side = node.get("side", true);
    return meshGenerator.generateCylinder(radius, height, bottom, side);
}


SgMesh* YAMLSceneReaderImpl::readCone(Mapping& node)
{
    double radius = node.get("radius", 1.0);
    double height = node.get("height", 1.0);
    bool bottom = node.get("bottom", true);
    bool side = node.get("side", true);
    return meshGenerator.generateCone(radius, height, bottom, side);
}


SgMesh* YAMLSceneReaderImpl::readCapsule(Mapping& node)
{
    double radius = node.get("radius", 1.0);
    double height = node.get("height", 1.0);
    return meshGenerator.generateCapsule(radius, height);
}


SgMesh* YAMLSceneReaderImpl::readExtrusion(Mapping& node)
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
            aa.angle() = self->toRadian(orientationNode[i*4+3].toDouble());
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

    self->readAngle(node, "creaseAngle", extrusion.creaseAngle);
    node.read("beginCap", extrusion.beginCap);
    node.read("endCap", extrusion.endCap);

    return meshGenerator.generateExtrusion(extrusion);
}


SgMesh* YAMLSceneReaderImpl::readElevationGrid(Mapping& node)
{
    MeshGenerator::ElevationGrid grid;

    node.read("xDimension", grid.xDimension);
    node.read("zDimension", grid.zDimension);
    node.read("xSpacing", grid.xSpacing);
    node.read("zSpacing", grid.zSpacing);
    node.read("ccw", grid.ccw);
    self->readAngle(node, "creaseAngle", grid.creaseAngle);

    Listing& heightNode = *node.findListing("height");
    if(heightNode.isValid()){
        for(int i=0; i < heightNode.size(); i++){
            grid.height.push_back(heightNode[i].toDouble());
        }
    }

    return meshGenerator.generateElevationGrid(grid);
}


SgMesh* YAMLSceneReaderImpl::readResourceAsGeometry(Mapping& node)
{
    SgNode* resource = readResource(node);
    if(resource){
        SgShape* shape = dynamic_cast<SgShape*>(resource);
        if(!shape){
            node.throwException(_("A resouce specified as a geometry must be a single mesh"));
        }
        return shape->mesh();
    }
    return 0;
}


void YAMLSceneReaderImpl::readAppearance(SgShape* shape, Mapping& node)
{
    Mapping& material = *node.findMapping("material");
    if(material.isValid()){
        readMaterial(shape, material);
    } else {
        setDefaultMaterial(shape);
    }
}


void YAMLSceneReaderImpl::readMaterial(SgShape* shape, Mapping& node)
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


void YAMLSceneReaderImpl::setDefaultMaterial(SgShape* shape)
{
    if(!defaultMaterial){
        defaultMaterial = new SgMaterial;
        defaultMaterial->setDiffuseColor(Vector3f(0.8f, 0.8f, 0.8f));
        defaultMaterial->setAmbientIntensity(0.2f);
        defaultMaterial->setShininess(0.2f);
    }
    shape->setMaterial(defaultMaterial);
}


SgNode* YAMLSceneReaderImpl::readResource(Mapping& node)
{
    SgNode* scene = 0;

    string& uri = symbol;
    if(node.read("uri", uri)){
        SgNodePtr resource;
        string nodeName;
        if(node.read("node", nodeName)){
            resource = loadResource(uri, nodeName);
        } else {
            resource = loadResource(uri);
        }
        scene = readTransformParameters(node, resource);
        if(scene == resource){
            return resource.retn();
        }
    }
    
    return scene;
}


SgNode* YAMLSceneReaderImpl::loadResource(const string& uri)
{
    ResourceInfo* resourceInfo = getOrCreateResourceInfo(uri);
    if(resourceInfo){
        return resourceInfo->rootNode;
    }
    return 0;
}


SgNode* YAMLSceneReaderImpl::loadResource(const string& uri, const string& nodeName)
{
    SgNode* scene = 0;
    ResourceInfo* resourceInfo = getOrCreateResourceInfo(uri);
    if(resourceInfo){
        if(nodeName.empty()){
            scene = resourceInfo->rootNode;
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
                scene = nodeInfo.node;
            }
        }
    }
    return scene;
}


ResourceInfo* YAMLSceneReaderImpl::getOrCreateResourceInfo(const string& uri)
{
    auto iter = resourceInfoMap.find(uri);

    if(iter != resourceInfoMap.end()){
        return iter->second;
    }

    filesystem::path filepath;
        
    if(!isUriSchemeRegexReady){
        uriSchemeRegex.assign("^(.+)://(.+)$");
        isUriSchemeRegexReady = true;
    }
    std::smatch match;
    bool hasScheme = false;
        
    if(std::regex_match(uri, match, uriSchemeRegex)){
        hasScheme = true;
        if(match.size() == 3){
            const string scheme = match.str(1);
            if(scheme == "file"){
                filepath = match.str(2);
            } else {
                std::lock_guard<std::mutex> guard(uriSchemeHandlerMutex);
                auto iter = uriSchemeHandlerMap.find(scheme);
                if(iter == uriSchemeHandlerMap.end()){
                    os() << str(format("Warning: The \"%1%\" scheme of \"%2%\" is not available.") % scheme % uri) << endl;
                } else {
                    auto& handler = iter->second;
                    filepath = handler(match.str(2), os());
                }
            }
        }
    }

    if(!hasScheme){
        filepath = uri;
        if(!checkAbsolute(filepath)){
            filepath = baseDirectory / filepath;
            filepath.normalize();
        }
    }
    
    ResourceInfo* info = 0;
    
    if(filepath.empty()){
        resourceInfoMap[uri] = info;
        
    } else {
        SgNodePtr rootNode = sceneLoader.load(getAbsolutePathString(filepath));
        if(rootNode){
            info = new ResourceInfo;
            info->rootNode = rootNode;
        }
        resourceInfoMap[uri] = info;
    }

    return info;
}


void YAMLSceneReaderImpl::adjustNodeCoordinate(NodeInfo& info)
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
        

void YAMLSceneReaderImpl::makeNodeMap(ResourceInfo* info)
{
    info->nodeMap.reset(new NodeMap);
    NodeInfo nodeInfo;
    nodeInfo.parent = 0;
    nodeInfo.node = info->rootNode;
    nodeInfo.R = Matrix3::Identity();
    nodeInfo.isScaled = false;
    makeNodeMapSub(nodeInfo, *info->nodeMap);
}


void YAMLSceneReaderImpl::makeNodeMapSub(const NodeInfo& nodeInfo, NodeMap& nodeMap)
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
