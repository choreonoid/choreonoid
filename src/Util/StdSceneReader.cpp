/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "StdSceneReader.h"
#include "SceneDrawables.h"
#include "SceneLights.h"
#include "SceneCameras.h"
#include "SceneEffects.h"
#include "MeshGenerator.h"
#include "PolygonMeshTriangulator.h"
#include "MeshFilter.h"
#include "SceneLoader.h"
#include "YAMLReader.h"
#include "EigenArchive.h"
#include "FilePathVariableProcessor.h"
#include "NullOut.h"
#include "ImageIO.h"
#include "UTF8.h"
#include <cnoid/stdx/filesystem>
#include <cnoid/Config>
#include <fmt/format.h>
#include <unordered_map>
#include <mutex>
#include <regex>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;
namespace filesystem = stdx::filesystem;

namespace cnoid {

class StdSceneReader::Impl
{
public:
    StdSceneReader* self;

    ostream* os_;
    ostream& os() { return *os_; }

    YAMLReader* mainYamlReader;

    // temporary variables for reading values
    double value;
    string symbol;
    Vector3f color;
    Vector3 v;
    Vector2 v2;
    bool on;
    
    MeshGenerator meshGenerator;
    PolygonMeshTriangulator polygonMeshTriangulator;
    MeshFilter meshFilter;
    SgMaterialPtr defaultMaterial;
    ImageIO imageIO;

    struct SceneNodeInfo
    {
        SgGroupPtr parent;
        SgNodePtr node;
        Matrix3 R;
        bool isScaled;
    };

    typedef unordered_map<string, SceneNodeInfo> SceneNodeMap;

    struct ResourceInfo : public Referenced
    {
        SgNodePtr scene;
        unique_ptr<SceneNodeMap> sceneNodeMap;
        unique_ptr<YAMLReader> yamlReader;
        string directory;
    };
    typedef ref_ptr<ResourceInfo> ResourceInfoPtr;

    map<string, ResourceInfoPtr> resourceInfoMap;
    
    SceneLoader sceneLoader;
    FilePathVariableProcessorPtr pathVariableProcessor;
    regex uriSchemeRegex;
    bool isUriSchemeRegexReady;
    typedef map<string, SgImagePtr> ImagePathToSgImageMap;
    ImagePathToSgImageMap imagePathToSgImageMap;

    typedef SgNode* (Impl::*NodeFunction)(Mapping* info);
    typedef unordered_map<string, NodeFunction> NodeFunctionMap;

    static NodeFunctionMap nodeFunctionMap;
    static std::mutex nodeFunctionMapMutex;
    static unordered_map<string, UriSchemeHandler> uriSchemeHandlerMap;
    static std::mutex uriSchemeHandlerMutex;

    Impl(StdSceneReader* self);
    ~Impl();

    bool readAngle(const Mapping* info, const char* key, double& angle) const;
    bool readAngle(const Mapping* info, std::initializer_list<const char*> keys, double& angle) const;
    AngleAxis readAngleAxis(const Listing* rotation) const;
    bool readRotation(const ValueNode* info, Matrix3& out_R) const;
    bool readTranslation(const ValueNode* info, Vector3& out_p) const;

    FilePathVariableProcessor* getOrCreatePathVariableProcessor();
    SgNode* readNode(Mapping* info);
    SgNode* readNode(Mapping* info, const string& type);
    SgNode* readNodeNode(Mapping* info);
    SgNode* readGroup(Mapping* info);
    void readElements(Mapping* info, SgGroup* group);
    void readNodeList(ValueNode* elements, SgGroup* group);
    SgNode* readTransform(Mapping* info);
    SgNode* readTransformParameters(Mapping* info, SgNode* scene);
    SgNode* readShape(Mapping* info);
    SgMesh* readGeometry(Mapping* info, int meshOptions);
    void readDivisionNumbers(Mapping* info, SgMesh* mesh);
    SgMesh* readBox(Mapping* info, int meshOptions);
    SgMesh* readSphere(Mapping* info, int meshOptions);
    SgMesh* readCylinder(Mapping* info, int meshOptions);
    SgMesh* readCone(Mapping* info, int meshOptions);
    SgMesh* readCapsule(Mapping* info, int meshOptions);
    SgMesh* readExtrusion(Mapping* info, int meshOptions);
    SgMesh* readElevationGrid(Mapping* info, int meshOptions);
    SgMesh* readMesh(Mapping* info, bool isTriangleMesh, int meshOptions);
    SgMesh* readResourceAsGeometry(Mapping* info, int meshOptions);
    void readAppearance(SgShape* shape, Mapping* info);
    void readMaterial(SgShape* shape, Mapping* info);
    void readTexture(SgShape* shape, Mapping* info);
    SgTextureTransform* readTextureTransform(Mapping* info);
    void readLightCommon(Mapping* info, SgLight* light);
    SgNode* readDirectionalLight(Mapping* info);
    SgNode* readPointLight(Mapping* info);
    void readPointLightParameters(Mapping* info, SgPointLight* light);
    SgNode* readSpotLight(Mapping* info);
    void readCameraParameters(Mapping* info, SgCamera* camera);
    SgNode* readPerspectiveCamera(Mapping* info);
    SgNode* readOrthographicCamera(Mapping* info);
    SgNode* readFog(Mapping* info);
    SgNode* readResourceAsScene(Mapping* info);
    Resource readResourceNode(Mapping* info, bool doSetUri);
    void extractNamedSceneNodes(Mapping* resourceNode, ResourceInfo* info, Resource& resource);
    ResourceInfo* getOrCreateResourceInfo(Mapping* resourceNode, const string& uri);
    stdx::filesystem::path findFileInPackage(const string& file);
    void adjustNodeCoordinate(SceneNodeInfo& info);
    void makeSceneNodeMap(ResourceInfo* info);
    void makeSceneNodeMapSub(const SceneNodeInfo& nodeInfo, SceneNodeMap& nodeMap);
};

StdSceneReader::Impl::NodeFunctionMap StdSceneReader::Impl::nodeFunctionMap;
std::mutex StdSceneReader::Impl::nodeFunctionMapMutex;
unordered_map<string, StdSceneReader::UriSchemeHandler> StdSceneReader::Impl::uriSchemeHandlerMap;
std::mutex StdSceneReader::Impl::uriSchemeHandlerMutex;

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


void StdSceneReader::registerUriSchemeHandler(const std::string& scheme, UriSchemeHandler handler)
{
    std::lock_guard<std::mutex> guard(Impl::uriSchemeHandlerMutex);
    Impl::uriSchemeHandlerMap[scheme] = handler;
}
                                                 

StdSceneReader::StdSceneReader()
{
    impl = new Impl(this);
    clear();
}


StdSceneReader::Impl::Impl(StdSceneReader* self)
    : self(self)
{
    {
        lock_guard<mutex> lock(nodeFunctionMapMutex);
        
        if(nodeFunctionMap.empty()){
            nodeFunctionMap = {
                { "Node",               &Impl::readNodeNode },
                { "Group",              &Impl::readGroup },
                { "Transform",          &Impl::readTransform },
                { "Shape",              &Impl::readShape },
                { "DirectionalLight",   &Impl::readDirectionalLight },
                { "PointLight",         &Impl::readPointLight },
                { "SpotLight",          &Impl::readSpotLight },
                { "PerspectiveCamera",  &Impl::readPerspectiveCamera },
                { "OrthographicCamera", &Impl::readOrthographicCamera },
                { "Fog",                &Impl::readFog },
                { "Resource",           &Impl::readResourceAsScene }
            };
        }
    }
    
    os_ = &nullout();
    isUriSchemeRegexReady = false;
    imageIO.setUpsideDown(true);
}


StdSceneReader::~StdSceneReader()
{
    delete impl;
}


StdSceneReader::Impl::~Impl()
{

}


void StdSceneReader::setMessageSink(std::ostream& os)
{
    impl->os_ = &os;
    impl->sceneLoader.setMessageSink(os);
}


void StdSceneReader::setDefaultDivisionNumber(int n)
{
    impl->meshGenerator.setDivisionNumber(n);
    impl->sceneLoader.setDefaultDivisionNumber(n);
}


int StdSceneReader::defaultDivisionNumber() const
{
    return impl->meshGenerator.divisionNumber();
}


void StdSceneReader::setBaseDirectory(const std::string& directory)
{
    impl->getOrCreatePathVariableProcessor()->setBaseDirectory(directory);
}


std::string StdSceneReader::baseDirectory() const
{
    if(impl->pathVariableProcessor){
        return impl->pathVariableProcessor->baseDirectory();
    }
    return string();
}


stdx::filesystem::path StdSceneReader::baseDirPath() const
{
    if(impl->pathVariableProcessor){
        return impl->pathVariableProcessor->baseDirPath();
    }
    return stdx::filesystem::path();
}


void StdSceneReader::setFilePathVariableProcessor(FilePathVariableProcessor* processor)
{
    impl->pathVariableProcessor = processor;
}


FilePathVariableProcessor* StdSceneReader::Impl::getOrCreatePathVariableProcessor()
{
    if(!pathVariableProcessor){
        pathVariableProcessor = new FilePathVariableProcessor;
    }
    return pathVariableProcessor;
}


void StdSceneReader::setYAMLReader(YAMLReader* reader)
{
    impl->mainYamlReader = reader;
}


void StdSceneReader::clear()
{
    isDegreeMode_ = true;
    impl->defaultMaterial.reset();
    impl->resourceInfoMap.clear();
    impl->imagePathToSgImageMap.clear();
}


void StdSceneReader::readHeader(Mapping* info)
{
    auto versionNode = info->find({ "format_version", "formatVersion" });
    if(!versionNode){
        info->throwException(_("The version of the Choreonoid scene file format is not specified"));
    }
    double version = versionNode->toDouble();
    if(version >= 2.1){
        info->throwException(
            format(_("Version {0} of the Choreonoid scene file format is not supported"), version));
    }
    readHeader(info, version);
}


void StdSceneReader::readHeader(Mapping* info, double formatVersion)
{
    if(formatVersion >= 2.1){
        info->throwException(
            format(_("Version {0} of the Choreonoid scene file format is not supported"), formatVersion));
    }

    if(auto angleUnitNode = info->extract({ "angle_unit", "angleUnit" })){
        string unit = angleUnitNode->toString();
        if(unit == "radian"){
            if(formatVersion >= 2.0){
                angleUnitNode->throwException(_("Radian mode is not supported in Choreonoid scene file format version 2.0"));
            }
            setAngleUnit(RADIAN);
        } else if(unit == "degree"){
            setAngleUnit(DEGREE);
        } else {
            angleUnitNode->throwException(_("The \"angleUnit\" value must be either \"radian\" or \"degree\""));
        }
    }
}


void StdSceneReader::setAngleUnit(AngleUnit unit)
{
    isDegreeMode_ = (unit == DEGREE);
}


bool StdSceneReader::readAngle(const Mapping* info, const char* key, double& angle) const
{
    if(info->read(key, angle)){
        angle = toRadian(angle);
        return true;
    }
    return false;
}


bool StdSceneReader::readAngle(const Mapping& info, const char* key, double& angle) const
{
    return readAngle(&info, key, angle);
}


bool StdSceneReader::readAngle(const Mapping* info, const char* key, float& angle) const
{
    if(info->read(key, angle)){
        angle = toRadian(angle);
        return true;
    }
    return false;
}


bool StdSceneReader::readAngle(const Mapping& info, const char* key, float& angle) const
{
    return readAngle(&info, key, angle);
}


bool StdSceneReader::readAngle(const Mapping* info, std::initializer_list<const char*> keys, double& angle) const
{
    for(auto& key : keys){
        if(readAngle(info, key, angle)){
            return true;
        }
    }
    return false;
}


bool StdSceneReader::readAngle(const Mapping* info, std::initializer_list<const char*> keys, float& angle) const
{
    for(auto& key : keys){
        if(readAngle(info, key, angle)){
            return true;
        }
    }
    return false;
}


bool StdSceneReader::Impl::readAngle(const Mapping* info, const char* key, double& angle) const
{
    return self->readAngle(info, key, angle);
}


bool StdSceneReader::Impl::readAngle(const Mapping* info, std::initializer_list<const char*> keys, double& angle) const
{
    return self->readAngle(info, keys, angle);
}


AngleAxis StdSceneReader::Impl::readAngleAxis(const Listing* elements) const
{
    Vector4 r;
    cnoid::readEx(elements, r);
    Vector3 axis(r[0], r[1], r[2]);
    double size = axis.norm();
    if(size < 1.0e-6){
        elements->throwException("Rotation axis is the zero vector");
    }
    axis /= size; // normalize
    return AngleAxis(self->toRadian(r[3]), axis);
}


bool StdSceneReader::Impl::readRotation(const ValueNode* info, Matrix3& out_R) const
{
    if(!info || !info->isValid()){
        return false;
    }
    auto elements = info->toListing();
    if(!elements->empty()){
        if(!elements->at(0)->isListing()){
            out_R = readAngleAxis(elements);
        } else {
            out_R = Matrix3::Identity();
            for(auto& node : *elements){
                out_R = out_R * readAngleAxis(node->toListing());
            }
        }
    }
    return true;
}

        
bool StdSceneReader::readRotation(const Mapping* info, Matrix3& out_R) const
{
    return impl->readRotation(info->find("rotation"), out_R);
}


bool StdSceneReader::readRotation(const Mapping* info, const char* key, Matrix3& out_R) const
{
    return impl->readRotation(info->find(key), out_R);
}


bool StdSceneReader::readRotation
(const Mapping* info, std::initializer_list<const char*> keys, Matrix3& out_R) const
{
    for(auto& key : keys){
        if(readRotation(info, key, out_R)){
            return true;
        }
    }
    return false;
}


bool StdSceneReader::extractRotation(Mapping* info, Matrix3& out_R) const
{
    ValueNodePtr value = info->extract("rotation");
    return impl->readRotation(value, out_R);
}


bool StdSceneReader::extractRotation(Mapping& info, Matrix3& out_R) const
{
    return extractRotation(&info, out_R);
}


bool StdSceneReader::Impl::readTranslation(const ValueNode* info, Vector3& out_p) const
{
    if(!info || !info->isValid()){
        return false;
    }
    auto translations = info->toListing();
    if(!translations->empty()){
        if(!translations->at(0)->isListing()){
            readEx(translations, out_p);
        } else {
            out_p.setZero();
            Vector3 v;
            for(auto& node : *translations){
                if(readTranslation(node, v)){
                    out_p += v;
                }
            }
        }
    }
    return true;
}


bool StdSceneReader::readTranslation(const Mapping* info, Vector3& out_p) const
{
    return impl->readTranslation(info->find("translation"), out_p);
}


bool StdSceneReader::readTranslation(const Mapping& info, Vector3& out_p) const
{
    return readTranslation(&info, out_p);
}


bool StdSceneReader::readTranslation(const Mapping* info, const char* key, Vector3& out_p) const
{
    return impl->readTranslation(info->find(key), out_p);
}


bool StdSceneReader::readTranslation(const Mapping& info, const char* key, Vector3& out_p) const
{
    return readTranslation(&info, key, out_p);
}


bool StdSceneReader::extractTranslation(Mapping* info, Vector3& out_p) const
{
    ValueNodePtr value = info->extract("translation");
    return impl->readTranslation(value, out_p);
}


bool StdSceneReader::extractTranslation(Mapping& info, Vector3& out_p) const
{
    return extractTranslation(&info, out_p);
}


SgNode* StdSceneReader::readNode(Mapping* info)
{
    return info->isValid() ? impl->readNode(info) : nullptr;
}


SgNode* StdSceneReader::readNode(Mapping& info)
{
    return readNode(&info);
}


SgNode* StdSceneReader::readNode(Mapping* info, const std::string& type)
{
    return info->isValid() ? impl->readNode(info, type) : nullptr;
}


SgNode* StdSceneReader::readNode(Mapping& info, const std::string& type)
{
    return readNode(&info, type);
}


static SgNodePtr removeRedundantGroup(SgGroupPtr& group)
{
    SgNodePtr node;
    if(!group->empty()){
        if(group->numChildren() == 1){
            auto& g = *group.get();
            if(typeid(g) == typeid(SgGroup)){
                node = group->child(0);
            }
        }
        if(!node){
            node = group;
        }
    }
    group.reset();
    return node;
}


SgNode* StdSceneReader::readScene(ValueNode* scene)
{
    SgGroupPtr group = new SgGroup;
    impl->readNodeList(scene, group);
    return removeRedundantGroup(group).retn();
}    


SgNode* StdSceneReader::Impl::readNode(Mapping* info)
{
    auto& type = (*info)["type"].toString();
    return readNode(info, type);
}
    

SgNode* StdSceneReader::Impl::readNode(Mapping* info, const string& type)
{
    NodeFunctionMap::iterator q = nodeFunctionMap.find(type);
    if(q == nodeFunctionMap.end()){
        if(info->get({ "is_optional", "isOptional" }, false)){
            os() << format(_("Warning: the node type \"{}\" is not defined. Reading this node has been skipped."), type) << endl;
            return nullptr;
        }
        info->throwException(format(_("The node type \"{}\" is not defined."), type));
    }

    NodeFunction funcToReadNode = q->second;
    SgNodePtr scene = (this->*funcToReadNode)(info);
    if(scene){
        if(info->read("name", symbol)){
            scene->setName(symbol);
        } else {
            // remove a nameless, redundant group node
            if(SgGroupPtr group = dynamic_pointer_cast<SgGroup>(scene)){
                scene = removeRedundantGroup(group);
            }
        }
    }
    return scene.retn();
}


SgNode* StdSceneReader::Impl::readNodeNode(Mapping* info)
{
    SgNodePtr node = new SgNode;
    return node.retn();
}


SgNode* StdSceneReader::Impl::readGroup(Mapping* info)
{
    SgGroupPtr group = new SgGroup;
    readElements(info, group);
    return group.retn();
}


void StdSceneReader::Impl::readElements(Mapping* info, SgGroup* group)
{
    auto elements = info->find("elements");
    if(elements->isValid()){
        readNodeList(elements, group);
    }
}


void StdSceneReader::Impl::readNodeList(ValueNode* elements, SgGroup* group)
{
    if(elements->isListing()){
        Listing& listing = *elements->toListing();
        for(int i=0; i < listing.size(); ++i){
            auto element = listing[i].toMapping();
            auto& type = (*element)["type"].toString();
            if(SgNodePtr scene = readNode(element, type)){
                group->addChild(scene);
            }
        }
    } else if(elements->isMapping()){
        for(auto& kv : *elements->toMapping()){
            auto& type = kv.first;
            auto element = kv.second->toMapping();
            auto typeNode = element->find("type");
            if(typeNode->isValid()){
                auto& type2 = typeNode->toString();
                if(type2 != type){
                    element->throwException(
                        format(_("The node type \"{0}\" is different from the type \"{1}\" specified in the parent node"),
                                type2, type));
                }
            }
            if(SgNodePtr scene = readNode(element, type)){
                group->addChild(scene);
            }
        }
    } else {
        elements->throwException(
            format(_("A scalar value is not accepted")));
    }
}


SgNode* StdSceneReader::Impl::readTransform(Mapping* info)
{
    SgGroupPtr group;

    SgPosTransformPtr posTransform = new SgPosTransform;
    if(self->readTranslation(info, v)){
        posTransform->setTranslation(v);
        group = posTransform;
    }        
    Matrix3 R;
    if(self->readRotation(info, R)){
        posTransform->setRotation(R);
        if(!group){
            group = posTransform;
        }
    }

    if(!read(info, "scale", v)){
        if(!group){
            group = new SgGroup;
        }
        readElements(info, group);

    } else {
        SgScaleTransformPtr scale = new SgScaleTransform;
        scale->setScale(v);
        if(group){
            group->addChild(scale);
        } else {
            group = scale;
        }
        readElements(info, scale);
    }

    // Necessary to prevent group from being deleted when exiting the function
    posTransform.reset();
    
    return group.retn();
}


SgNode* StdSceneReader::Impl::readTransformParameters(Mapping* info, SgNode* scene)
{
    SgGroupPtr group;
    
    Matrix3 R;
    bool isRotated = self->readRotation(info, R);
    Vector3 v;
    bool isTranslated = self->readTranslation(info, v);
    if(isRotated || isTranslated){
        SgPosTransformPtr transform = new SgPosTransform;
        if(isRotated){
            transform->setRotation(R);
        }
        if(isTranslated){
            transform->setTranslation(v);
        }
        group = transform;
    }
    
    if(read(info, "scale", v)){
        SgScaleTransformPtr scale = new SgScaleTransform;
        scale->setScale(v);
        if(group){
            group->addChild(scale);
            scale->addChild(scene);
            scene = nullptr;
        } else {
            group = scale;
        }
    }

    if(group){
        if(scene){
            group->addChild(scene);
        }
        return group.retn();
    }
    return scene;
}


SgNode* StdSceneReader::Impl::readShape(Mapping* info)
{
    SgNode* scene = nullptr;

    SgShapePtr shape = new SgShape;

    int meshOptions = MeshGenerator::NoOption;
    auto appearance = info->findMapping("appearance");
    if(appearance->isValid()){
        readAppearance(shape, appearance);
        if(shape->texture()){
            meshOptions |= MeshGenerator::TextureCoordinate;
        }
    }

    auto geometry = info->findMapping("geometry");
    if(geometry->isValid()){
        shape->setMesh(readGeometry(geometry, meshOptions));
    }

    scene = readTransformParameters(info, shape);

    if(scene == shape){
        return shape.retn();
    }

    return scene;
}


SgMesh* StdSceneReader::Impl::readGeometry(Mapping* info, int meshOptions)
{
    SgMesh* mesh = nullptr;
    ValueNode& typeNode = (*info)["type"];
    auto& type = typeNode.toString();
    if(type == "Box"){
        mesh = readBox(info, meshOptions);
    } else if(type == "Sphere"){
        mesh = readSphere(info, meshOptions);
    } else if(type == "Cylinder"){
        mesh = readCylinder(info, meshOptions);
    } else if(type == "Cone"){
        mesh = readCone(info, meshOptions);
    } else if(type == "Capsule"){
        mesh = readCapsule(info, meshOptions);
    } else if(type == "Resource"){
        mesh = readResourceAsGeometry(info, meshOptions);
    } else if(type == "TriangleMesh"){
        mesh = readMesh(info, true, meshOptions);
    } else if(type == "IndexedFaceSet"){
        mesh = readMesh(info, false, meshOptions);
    } else if(type == "Extrusion"){
        mesh = readExtrusion(info, meshOptions);
    } else if(type == "ElevationGrid"){
        mesh = readElevationGrid(info, meshOptions);
    } else {
        typeNode.throwException(
            format(_("Unknown geometry \"{}\""), type));
    }
    return mesh;
}


void StdSceneReader::Impl::readDivisionNumbers(Mapping* info, SgMesh* mesh)
{
    int n;
    if(info->read({ "division_number", "divisionNumber" }, n)){
        mesh->setDivisionNumber(n);
    }
    if(info->read("extra_division_number", n)){
        mesh->setExtraDivisionNumber(n);
    }
}
    

SgMesh* StdSceneReader::Impl::readBox(Mapping* info, int meshOptions)
{
    SgMeshPtr mesh = new SgMesh;

    readDivisionNumbers(info, mesh);

    SgMesh::Box box;
    read(info, "size", box.size);
    mesh->setPrimitive(box);

    int edv = info->get("extra_division_number", 1);
    string symbol;
    if(info->read("extra_division_mode", symbol)){
        int mode;
        if(symbol == "preferred"){
            mode = SgMesh::ExtraDivisionPreferred;
        } else if(symbol == "x"){
            mode = SgMesh::ExtraDivisionX;
        } else if(symbol == "y"){
            mode = SgMesh::ExtraDivisionY;
        } else if(symbol == "z"){
            mode = SgMesh::ExtraDivisionZ;
        } else {
            mode = SgMesh::ExtraDivisionPreferred;
        }
        mesh->setExtraDivisionMode(mode);
    }

    if(!meshGenerator.updateMeshWithPrimitiveInformation(mesh, meshOptions)){
        info->throwException(_("A box cannot be generated with the given parameters."));
    }

    return mesh.retn();
}


SgMesh* StdSceneReader::Impl::readSphere(Mapping* info, int meshOptions)
{
    SgMeshPtr mesh = new SgMesh;

    readDivisionNumbers(info, mesh);

    SgMesh::Sphere sphere;
    info->read("radius", sphere.radius);
    mesh->setPrimitive(sphere);

    if(!meshGenerator.updateMeshWithPrimitiveInformation(mesh, meshOptions)){
        info->throwException(_("A sphere cannot be generated with the given parameters."));
    }
    return mesh.retn();
}


SgMesh* StdSceneReader::Impl::readCylinder(Mapping* info, int meshOptions)
{
    SgMeshPtr mesh = new SgMesh;
    
    readDivisionNumbers(info, mesh);

    SgMesh::Cylinder cylinder;
    info->read("radius", cylinder.radius);
    info->read("height", cylinder.height);
    info->read("top", cylinder.top);
    info->read("bottom", cylinder.bottom);
    mesh->setPrimitive(cylinder);

    if(!meshGenerator.updateMeshWithPrimitiveInformation(mesh, meshOptions)){
        info->throwException(_("A cylinder cannot be generated with the given parameters."));
    }
    return mesh.retn();
}


SgMesh* StdSceneReader::Impl::readCone(Mapping* info, int meshOptions)
{
    SgMeshPtr mesh = new SgMesh;

    readDivisionNumbers(info, mesh);

    SgMesh::Cone cone;
    info->read("radius", cone.radius);
    info->read("height", cone.height);
    info->read("bottom", cone.bottom);
    mesh->setPrimitive(cone);

    if(!meshGenerator.updateMeshWithPrimitiveInformation(mesh, meshOptions)){
        info->throwException(_("A cone cannot be generated with the given parameters."));
    }
    return mesh.retn();
}


SgMesh* StdSceneReader::Impl::readCapsule(Mapping* info, int meshOptions)
{
    SgMeshPtr mesh = new SgMesh;

    readDivisionNumbers(info, mesh);

    SgMesh::Capsule capsule;
    info->read("radius", capsule.radius);
    info->read("height", capsule.height);
    mesh->setPrimitive(capsule);
    
    if(!meshGenerator.updateMeshWithPrimitiveInformation(mesh, meshOptions)){
        info->throwException(_("A capsule cannot be generated with the given parameters."));
    }
    return mesh.retn();
}


SgMesh* StdSceneReader::Impl::readExtrusion(Mapping* info, int meshOptions)
{
    MeshGenerator::Extrusion extrusion;

    Listing& crossSectionNode = *info->findListing({ "cross_section", "crossSection" });
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

    Listing& spineNode = *info->findListing("spine");
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

    Listing& orientationNode = *info->findListing("orientation");
    if(orientationNode.isValid()){
        const int n = orientationNode.size() / 4;
        MeshGenerator::AngleAxisArray& orientation = extrusion.orientation;
        orientation.resize(n);
        for(int i=0; i < n; ++i){
            AngleAxis& aa = orientation[i];
            Vector3& axis = aa.axis();
            for(int j=0; j < 3; ++j){
                axis[j] = orientationNode[i*4+j].toDouble();
            }
            aa.angle() = self->toRadian(orientationNode[i*4+3].toDouble());
        }
    }
    
    Listing& scaleNode = *info->findListing("scale");
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

    readAngle(info, { "crease_angle", "creaseAngle" }, extrusion.creaseAngle);
    info->read({" begin_cap", "beginCap" }, extrusion.beginCap);
    info->read({ "end_cap", "endCap"}, extrusion.endCap);

    SgMesh* mesh = meshGenerator.generateExtrusion(extrusion, meshOptions);

    mesh->setSolid(info->get("solid", mesh->isSolid()));
    
    return mesh;
}


SgMesh* StdSceneReader::Impl::readElevationGrid(Mapping* info, int meshOptions)
{
    MeshGenerator::ElevationGrid grid;

    info->read({ "x_dimension", "xDimension" }, grid.xDimension);
    info->read({ "z_dimension", "zDimension" }, grid.zDimension);
    info->read({ "x_spacing", "xSpacing" }, grid.xSpacing);
    info->read({ "z_spacing", "zSpacing" }, grid.zSpacing);
    info->read("ccw", grid.ccw);
    readAngle(info, { "crease_angle", "creaseAngle" }, grid.creaseAngle);

    Listing& heightNode = *info->findListing("height");
    if(heightNode.isValid()){
        for(int i=0; i < heightNode.size(); i++){
            grid.height.push_back(heightNode[i].toDouble());
        }
    }

    SgTexCoordArray* texCoord = nullptr;
    Listing& texCoordNode = *info->findListing({ "tex_coord", "texCoord" });
    if(texCoordNode.isValid()){
        const int size = texCoordNode.size() / 2;
        texCoord = new SgTexCoordArray();
        texCoord->resize(size);
        for(int i=0; i < size; ++i){
            Vector2f& s = texCoord->at(i);
            for(int j=0; j < 2; ++j){
                s[j] = texCoordNode[i*2+j].toDouble();
            }
        }
        meshOptions &= ~MeshGenerator::TextureCoordinate;
    }

    SgMesh* mesh = meshGenerator.generateElevationGrid(grid, meshOptions);
    if(texCoord){
        mesh->setTexCoords(texCoord);
        mesh->texCoordIndices() = mesh->triangleVertices();
    }

    mesh->setSolid(info->get("solid", mesh->isSolid()));

    return mesh;
}


SgMesh* StdSceneReader::Impl::readMesh(Mapping* info, bool isTriangleMesh, int meshOptions)
{
    SgMeshBase* meshBase;
    SgMeshPtr triangleMesh;
    SgPolygonMeshPtr polygonMesh;

    if(isTriangleMesh){
        triangleMesh = new SgMesh;
        meshBase = triangleMesh;
    } else {
        polygonMesh = new SgPolygonMesh;
        meshBase = polygonMesh;
    }
        
    Listing& srcVertices = *info->findListing({ "vertices", "coordinate" });
    if(srcVertices.isValid()){
        const int numVertices = srcVertices.size() / 3;
        SgVertexArray& vertices = *meshBase->getOrCreateVertices();
        vertices.resize(numVertices);
        for(int i=0; i < numVertices; ++i){
            Vector3f& v = vertices[i];
            for(int j=0; j < 3; ++j){
                v[j] = srcVertices[i*3 + j].toFloat();
            }
        }
    }

    Listing& srcFaces = *info->findListing({ "faces", "coordIndex" });
    if(srcFaces.isValid()){
        const int numIndices = srcFaces.size();
        SgIndexArray& face = meshBase->faceVertexIndices();
        face.resize(numIndices);
        for(int i=0; i < numIndices; ++i){
            face[i] = srcFaces[i].toInt();
        }
    }

    Listing& srcNormals = *info->findListing("normals");
    if(srcNormals.isValid()){
        const int numNormals = srcNormals.size() / 3;
        SgNormalArray& normals = *meshBase->getOrCreateNormals();
        normals.resize(numNormals);
        for(int i=0; i < numNormals; ++i){
            Vector3f& n = normals[i];
            for(int j=0; j < 3; ++j){
                n[j] = srcNormals[i*3 + j].toFloat();
            }
        }
    }

    Listing& srcNormalIndices = *info->findListing("normal_indices");
    if(srcNormalIndices.isValid()){
        const int numIndices = srcNormalIndices.size();
        SgIndexArray& normalIndices = meshBase->normalIndices();
        normalIndices.resize(numIndices);
        for(int i=0; i < numIndices; ++i){
            normalIndices[i] = srcNormalIndices[i].toInt();
        }
    }

    Listing& srcTexCoords = *info->findListing({ "tex_coords", "texCoord" });
    if(srcTexCoords.isValid()){
        const int numCoords = srcTexCoords.size() / 2;
        SgTexCoordArray& texCoord = *meshBase->getOrCreateTexCoords();
        texCoord.resize(numCoords);
        for(int i=0; i < numCoords; ++i){
            Vector2f& p = texCoord[i];
            for(int j=0; j < 2; ++j){
                p[j] = srcTexCoords[i*2 + j].toFloat();
            }
        }
    }

    Listing& srcTexCoordIndices = *info->findListing({ "tex_coord_indices", "texCoordIndex" });
    if(srcTexCoordIndices.isValid()){
        const int numIndices = srcTexCoordIndices.size();
        SgIndexArray& texCoordIndices = meshBase->texCoordIndices();
        texCoordIndices.resize(numIndices);
        for(int i=0; i < numIndices; ++i){
            texCoordIndices[i] = srcTexCoordIndices[i].toInt();
        }
    }

    //polygonMeshTriangulator.setDeepCopyEnabled(true);
    if(!isTriangleMesh){
        triangleMesh = polygonMeshTriangulator.triangulate(polygonMesh);
        const string& errorMessage = polygonMeshTriangulator.errorMessage();
        if(!errorMessage.empty()){
            info->throwException("Error of an IndexedFaceSet node: \n" + errorMessage);
        }
    }

    double creaseAngle = 0.0;
    readAngle(info, { "crease_angle", "creaseAngle" }, creaseAngle);
    if(!triangleMesh->hasNormals()){
        meshFilter.generateNormals(triangleMesh, creaseAngle);
    }

    if(meshOptions & MeshGenerator::TextureCoordinate){
        if(!triangleMesh->hasTexCoords()){
            meshGenerator.generateTextureCoordinateForIndexedFaceSet(triangleMesh);
        }
    }

    triangleMesh->setSolid(info->get("solid", triangleMesh->isSolid()));

    triangleMesh->updateBoundingBox();

    return triangleMesh.retn();
}


SgMesh* StdSceneReader::Impl::readResourceAsGeometry(Mapping* info, int meshOptions)
{
    auto resource = readResourceNode(info, false);

    SgNode* scene = nullptr;
    bool isDirectResource = false;
    if(resource.scene){
        scene = resource.scene;
        isDirectResource = true;
    } else if(resource.info){
        scene = readNode(resource.info->toMapping());
    }
    auto shape = dynamic_cast<SgShape*>(scene);

    if(!shape){
        info->throwException(_("A resouce specified as a geometry must be a single mesh"));
    }
    auto mesh = shape->mesh();
    if(!mesh){
        info->throwException(_("A resouce specified as a geometry does not have a mesh"));
    }
    if(isDirectResource){
        mesh->setUriByFilePathAndBaseDirectory(
            resource.uri, getOrCreatePathVariableProcessor()->baseDirectory());
        if(!resource.fragment.empty()){
            resource.scene->setUriFragment(resource.fragment);
        }
    }
        
    double creaseAngle;
    if(readAngle(info, { "crease_angle", "creaseAngle" }, creaseAngle)){
        mesh->setCreaseAngle(creaseAngle);
        meshFilter.setNormalOverwritingEnabled(true);
        bool removeRedundantVertices =
            info->get({ "remove_redundant_vertices", "removeRedundantVertices" }, false);
        meshFilter.generateNormals(mesh, creaseAngle, removeRedundantVertices);
        meshFilter.setNormalOverwritingEnabled(false);
    }
    if(meshOptions & MeshGenerator::TextureCoordinate){
        if(mesh && !mesh->hasTexCoords()){
            meshGenerator.generateTextureCoordinateForIndexedFaceSet(mesh);
        }
    }
    return mesh;
}


void StdSceneReader::Impl::readAppearance(SgShape* shape, Mapping* info)
{
    auto material = info->findMapping("material");
    if(material->isValid()){
        readMaterial(shape, material);
    }

    auto texture = info->findMapping("texture");
    if(texture->isValid()){
        readTexture(shape, texture);

        auto textureTransform = info->findMapping({ "texture_transform", "textureTransform" });
        if(textureTransform->isValid() && shape->texture()){
            shape->texture()->setTextureTransform(readTextureTransform(textureTransform));
        }
    }
}


void StdSceneReader::Impl::readMaterial(SgShape* shape, Mapping* info)
{
    SgMaterialPtr material = new SgMaterial;

    double value;
    if(info->read({ "ambient", "ambientIntensity" }, value)){
        material->setAmbientIntensity(value);
    }
    if(read(info, { "diffuse", "diffuseColor" }, color)){
        material->setDiffuseColor(color);
    }
    if(read(info, { "emissive", "emissiveColor" }, color)){
        material->setEmissiveColor(color);
    }
    if(read(info, { "specular", "specularColor" }, color)){
        material->setSpecularColor(color);
    }
    if(info->read("specular_exponent", value)){
        material->setSpecularExponent(value);
    } else if(info->read("shininess", value)){ // deprecated
        material->setSpecularExponent(
            127.0f * std::max(0.0f, std::min((float)value, 1.0f)) + 1.0f);
    }
    
    if(info->read("transparency", value)){
        material->setTransparency(value);
    }

    shape->setMaterial(material);
}


void StdSceneReader::Impl::readTexture(SgShape* shape, Mapping* info)
{
    string& uri = symbol;
    if(info->read({ "uri", "url" }, uri) && !uri.empty()){
        SgImagePtr image;
        ImagePathToSgImageMap::iterator p = imagePathToSgImageMap.find(uri);
        if(p != imagePathToSgImageMap.end()){
            image = p->second;
        }else{
            auto fpvp = getOrCreatePathVariableProcessor();
            auto filename = fpvp->expand(uri, true);
            if(filename.empty()){
                os() << format(_("Warning: texture uri \"{0}\" is not valid: {1}"),
                               uri, fpvp->errorMessage()) << endl;
            } else {
                image = new SgImage;
                if(imageIO.load(image->image(), filename, os())){
                    image->setUriByFilePathAndBaseDirectory(
                        uri, getOrCreatePathVariableProcessor()->baseDirectory());
                    imagePathToSgImageMap[uri] = image;
                } else {
                    image.reset();
                }
            }
        }
        if(image){
            SgTexturePtr texture = new SgTexture;
            texture->setImage(image);
            bool repeatS = true;
            bool repeatT = true;
            
            auto repeatNode = info->find("repeat");
            if(repeatNode->isValid()){
                if(repeatNode->isListing()){
                    auto repeatList = repeatNode->toListing();
                    if(repeatList->size() != 2){
                        repeatList->throwException(_("The number of the repeat elements must be two"));
                    }
                    repeatS = repeatList->at(0)->toBool();
                    repeatT = repeatList->at(1)->toBool();
                } else {
                    repeatS = repeatT = repeatNode->toBool();
                }
            } else {
                info->read({ "repeat_s", "repeatS" }, repeatS);
                info->read({ "repeat_t", "repeatT" }, repeatT);
            }
            texture->setRepeat(repeatS, repeatT);
            shape->setTexture(texture);
        }
    }
}


SgTextureTransform* StdSceneReader::Impl::readTextureTransform(Mapping* info)
{
    SgTextureTransformPtr transform = new SgTextureTransform;
    if(read(info, "center", v2)) transform->setCenter(v2);
    if(read(info, "scale", v2)) transform->setScale(v2);
    if(read(info, "translation", v2)) transform->setTranslation(v2);
    if(self->readAngle(info, "rotation", value)) transform->setRotation(value);
    return transform.retn();
}


void StdSceneReader::Impl::readLightCommon(Mapping* info, SgLight* light)
{
    if(info->read("on", on)){
        light->on(on);
    }
    if(read(info, "color", color)){
        light->setColor(color);
    }
    if(info->read("intensity", value)){
        light->setIntensity(value);
    }
    if(info->read({ "ambient", "ambientIntensity" }, value)){
        light->setAmbientIntensity(value);
    }
}


SgNode* StdSceneReader::Impl::readDirectionalLight(Mapping* info)
{
    SgDirectionalLightPtr light = new SgDirectionalLight;
    readLightCommon(info, light);
    if(read(info, "direction", v)) light->setDirection(v);
    return light.retn();
}


SgNode* StdSceneReader::Impl::readPointLight(Mapping* info)
{
    SgPointLightPtr light = new SgPointLight;
    readPointLightParameters(info, light);
    return light.retn();
}


void StdSceneReader::Impl::readPointLightParameters(Mapping* info, SgPointLight* light)
{
    readLightCommon(info, light);

    if(read(info, "attenuation", color)){
        light->setConstantAttenuation(color[0]);
        light->setLinearAttenuation(color[1]);
        light->setQuadraticAttenuation(color[2]);
    }
}


SgNode* StdSceneReader::Impl::readSpotLight(Mapping* info)
{
    SgSpotLightPtr light = new SgSpotLight;

    readPointLightParameters(info, light);

    if(read(info, "direction", v)){
        light->setDirection(v);
    }
    if(readAngle(info, { "beam_width", "beamWidth" }, value)){
        light->setBeamWidth(value);
    }
    if(readAngle(info, { "cut_off_angle", "cutOffAngle" }, value)){
        light->setCutOffAngle(value);
    }
    if(info->read({ "cut_off_exponent", "cutOffExponent" }, value)){
        light->setCutOffExponent(value);
    }
    
    return light.retn();
}


void StdSceneReader::Impl::readCameraParameters(Mapping* info, SgCamera* camera)
{
    if(info->read("near_clip_distance", value)){
        camera->setNearClipDistance(value);
    }
    if(info->read("far_clip_distance", value)){
        camera->setFarClipDistance(value);
    }
}


SgNode* StdSceneReader::Impl::readPerspectiveCamera(Mapping* info)
{
    SgPerspectiveCameraPtr camera = new SgPerspectiveCamera;
    readCameraParameters(info, camera);
    if(readAngle(info, "field_of_view", value)){
        camera->setFieldOfView(value);
    }
    return camera.retn();
}


SgNode* StdSceneReader::Impl::readOrthographicCamera(Mapping* info)
{
    SgOrthographicCameraPtr camera = new SgOrthographicCamera;
    readCameraParameters(info, camera);
    if(info->read("height", value)){
        camera->setHeight(value);
    }
    return camera.retn();
}


SgNode* StdSceneReader::Impl::readFog(Mapping* info)
{
    SgFogPtr fog = new SgFog;
    if(info->read("visibility_range", value)){
        fog->setVisibilityRange(value);
    }
    return fog.retn();
}


SgNode* StdSceneReader::Impl::readResourceAsScene(Mapping* info)
{
    auto resource = readResourceNode(info, true);
    if(resource.scene){
        return resource.scene;
    } else if(resource.info){
        return readNode(resource.info->toMapping());
    }
    return nullptr;
}


StdSceneReader::Resource StdSceneReader::readResourceNode(Mapping* info)
{
    return impl->readResourceNode(info, true);
}


StdSceneReader::Resource StdSceneReader::readResourceNode(Mapping& info)
{
    return impl->readResourceNode(&info, true);
}


StdSceneReader::Resource StdSceneReader::Impl::readResourceNode(Mapping* info, bool doSetUri)
{
    Resource resource;

    resource.uri = (*info)["uri"].toString();

    auto fragmentNode = info->find({ "fragment", "node" });
    if(fragmentNode->isValid()){
        resource.fragment = fragmentNode->toString();
    }
    
    ResourceInfo* resourceInfo = getOrCreateResourceInfo(info, resource.uri);
    if(resourceInfo){
        resource.directory = resourceInfo->directory;
        bool isYamlResouce = (resourceInfo->yamlReader != nullptr);
        if(resource.fragment.empty()){
            if(isYamlResouce){
                resource.info = resourceInfo->yamlReader->document();
            } else {
                resource.scene = resourceInfo->scene;
            }
        } else {
            if(!isYamlResouce){
                extractNamedSceneNodes(info, resourceInfo, resource);
            } else {                
                resource.info = resourceInfo->yamlReader->findAnchoredNode(resource.fragment);
                if(!resource.info){
                    info->throwException(
                        format(_("Fragment \"{0}\" is not found in \"{1}\"."),
                               resource.fragment, resource.uri));
                }
            }
        }
    }

    if(resource.scene){
        resource.scene = readTransformParameters(info, resource.scene);
        if(doSetUri){
            resource.scene->setUriByFilePathAndBaseDirectory(
                resource.uri, getOrCreatePathVariableProcessor()->baseDirectory());
            if(!resource.fragment.empty()){
                resource.scene->setUriFragment(resource.fragment);
            }
        }
    }

    return resource;
}


void StdSceneReader::Impl::extractNamedSceneNodes
(Mapping* resourceNode, ResourceInfo* info, Resource& resource)
{
    unique_ptr<SceneNodeMap>& nodeMap = info->sceneNodeMap;
    if(!nodeMap){
        makeSceneNodeMap(info);
    }
    auto iter = nodeMap->find(resource.fragment);
    if(iter == nodeMap->end()){
        resourceNode->throwException(
            format(_("Fragment \"{0}\" is not found in \"{1}\"."), resource.fragment, resource.uri));
    } else {
        SceneNodeInfo& nodeInfo = iter->second;
        if(nodeInfo.parent){
            nodeInfo.parent->removeChild(nodeInfo.node);
            nodeInfo.parent.reset();
            adjustNodeCoordinate(nodeInfo);
        }
        resource.scene = nodeInfo.node;
    }
}


StdSceneReader::Impl::ResourceInfo*
StdSceneReader::Impl::getOrCreateResourceInfo(Mapping* resourceNode, const string& uri)
{
    auto iter = resourceInfoMap.find(uri);

    if(iter != resourceInfoMap.end()){
        return iter->second;
    }

    if(!isUriSchemeRegexReady){
        uriSchemeRegex.assign("^(.+)://(.+)$");
        isUriSchemeRegexReady = true;
    }
    smatch match;
    string filename;
    bool hasScheme = false;
    bool isFileScheme = false;
        
    if(regex_match(uri, match, uriSchemeRegex)){
        hasScheme = true;
        if(match.size() == 3){
            const string scheme = match.str(1);
            if(scheme == "file"){
                filename = match.str(2);
                isFileScheme = true;
            } else {
                std::lock_guard<std::mutex> guard(uriSchemeHandlerMutex);
                auto iter = uriSchemeHandlerMap.find(scheme);
                if(iter == uriSchemeHandlerMap.end()){
                    resourceNode->throwException(
                        format(_("The \"{0}\" scheme of \"{1}\" is not available"), scheme, uri));
                } else {
                    auto& handler = iter->second;
                    filename = handler(match.str(2), os());
                    isFileScheme = true;
                }
            }
        }
    }

    if(!hasScheme){
        filename = uri;
        isFileScheme = true;
    }

    if(isFileScheme){
        auto pvp = getOrCreatePathVariableProcessor();
        filename = pvp->expand(filename, true);
        if(filename.empty()){
            resourceNode->throwException(
                format(_("The resource URI \"{0}\" is not valid: {1}"),
                       uri, pvp->errorMessage()));
        }
    } else {
        if(filename.empty()){
            resourceNode->throwException(
                format(_("The resource URI \"{}\" is not valid"), uri));
        }
    }

    ResourceInfoPtr info = new ResourceInfo;

    filesystem::path filepath(fromUTF8(filename));
    string ext = filepath.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

    if(ext == ".yaml" || ext == ".yml"){
        unique_ptr<YAMLReader> reader(new YAMLReader);
        reader->importAnchors(*mainYamlReader);
        if(!reader->load(filename)){
            resourceNode->throwException(
                format(_("YAML resource \"{0}\" cannot be loaded ({1})"),
                 uri, reader->errorMessage()));
        }
        info->yamlReader = std::move(reader);

    } else {
        SgNodePtr scene = sceneLoader.load(filename);
        if(!scene){
            resourceNode->throwException(
                format(_("The resource is not found at URI \"{}\""), uri));
        }
        info->scene = scene;
    }

    info->directory = toUTF8(filepath.parent_path().string());
    
    resourceInfoMap[uri] = info;

    return info;
}


void StdSceneReader::Impl::adjustNodeCoordinate(SceneNodeInfo& info)
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
        

void StdSceneReader::Impl::makeSceneNodeMap(ResourceInfo* info)
{
    info->sceneNodeMap.reset(new SceneNodeMap);
    SceneNodeInfo nodeInfo;
    nodeInfo.parent = 0;
    nodeInfo.node = info->scene;
    nodeInfo.R = Matrix3::Identity();
    nodeInfo.isScaled = false;
    makeSceneNodeMapSub(nodeInfo, *info->sceneNodeMap);
}


void StdSceneReader::Impl::makeSceneNodeMapSub(const SceneNodeInfo& nodeInfo, SceneNodeMap& nodeMap)
{
    const string& name = nodeInfo.node->name();
    bool wasProcessed = false;
    if(!name.empty()){
        wasProcessed = !nodeMap.insert(make_pair(name, nodeInfo)).second;
    }
    if(!wasProcessed){
        SgGroup* group = dynamic_cast<SgGroup*>(nodeInfo.node.get());
        if(group){
            SceneNodeInfo childInfo;
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
                makeSceneNodeMapSub(childInfo, nodeMap);
            }
        }
    }
}
