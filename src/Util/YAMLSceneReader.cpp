/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "YAMLSceneReader.h"
#include <cnoid/SceneDrawables>
#include <cnoid/SceneLights>
#include <cnoid/MeshGenerator>
#include <cnoid/PolygonMeshTriangulator>
#include <cnoid/MeshNormalGenerator>
#include <cnoid/SceneLoader>
#include <cnoid/EigenArchive>
#include <cnoid/FileUtil>
#include <cnoid/NullOut>
#include <cnoid/Exception>
#include <cnoid/ImageIO>
#include <cnoid/Config>
#include <boost/format.hpp>
#include <unordered_map>
#include <mutex>

#ifdef CNOID_USE_BOOST_REGEX
#include <boost/regex.hpp>
using boost::regex;
using boost::smatch;
using boost::regex_match;
#else
#include <regex>
#endif

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
    Vector2 v2;
    bool on;
    
    MeshGenerator meshGenerator;
    PolygonMeshTriangulator polygonMeshTriangulator;
    MeshNormalGenerator normalGenerator;
    SgMaterialPtr defaultMaterial;
    ImageIO imageIO;

    map<string, ResourceInfoPtr> resourceInfoMap;
    SceneLoader sceneLoader;
    filesystem::path baseDirectory;
    regex uriSchemeRegex;
    bool isUriSchemeRegexReady;
    typedef map<string, SgImagePtr> ImagePathToSgImageMap;
    ImagePathToSgImageMap imagePathToSgImageMap;
    bool generateTexCoord;

    YAMLSceneReaderImpl(YAMLSceneReader* self);
    ~YAMLSceneReaderImpl();

    bool readAngle(Mapping& node, const char* key, double& angle) { return self->readAngle(node, key, angle); }
    
    SgNode* readNode(Mapping& node, const string& type);
    SgNode* readGroup(Mapping& node);
    void readElements(Mapping& node, SgGroup* group);
    void readNodeList(ValueNode& elements, SgGroup* group);
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
    SgMesh* readIndexedFaceSet(Mapping& node);
    SgMesh* readResourceAsGeometry(Mapping& node);
    void readAppearance(SgShape* shape, Mapping& node);
    void readMaterial(SgShape* shape, Mapping& node);
    void setDefaultMaterial(SgShape* shape);
    void readTexture(SgShape* shape, Mapping& node);
    void readTextureTransform(SgTexture* texture, Mapping& node);
    void readLightCommon(Mapping& node, SgLight* light);
    SgNode* readDirectionalLight(Mapping& node);
    SgNode* readSpotLight(Mapping& node);
    SgNode* readResource(Mapping& node);
    SgNode* loadResource(const string& uri);
    SgNode* loadResource(const string& uri, const string& nodeName);
    void decoupleResourceNode(const string& uri, const string& nodeName);
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
        nodeFunctionMap["DirectionalLight"] = &YAMLSceneReaderImpl::readDirectionalLight;
        nodeFunctionMap["SpotLight"] = &YAMLSceneReaderImpl::readSpotLight;
        nodeFunctionMap["Resource"] = &YAMLSceneReaderImpl::readResource;
    }
    os_ = &nullout();
    isUriSchemeRegexReady = false;
    imageIO.setUpsideDown(true);
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


int YAMLSceneReader::defaultDivisionNumber() const
{
    return impl->meshGenerator.divisionNumber();
}


void YAMLSceneReader::setBaseDirectory(const std::string& directory)
{
    impl->baseDirectory = directory;
}


std::string YAMLSceneReader::baseDirectory()
{
    return impl->baseDirectory.string();
}


void YAMLSceneReader::clear()
{
    isDegreeMode_ = true;
    impl->defaultMaterial = 0;
    impl->resourceInfoMap.clear();
    impl->imagePathToSgImageMap.clear();
}


void YAMLSceneReader::readHeader(Mapping& node)
{
    auto angleUnitNode = node.extract("angleUnit");
    if(angleUnitNode){
        string unit = angleUnitNode->toString();
        if(unit == "radian"){
            setAngleUnit(RADIAN);
        } else if(unit == "degree"){
            setAngleUnit(DEGREE);
        } else {
            angleUnitNode->throwException(_("The \"angleUnit\" value must be either \"radian\" or \"degree\""));
        }
    }
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


static SgNodePtr removeRedundantGroup(SgGroupPtr& group)
{
    SgNodePtr node;
    if(!group->empty()){
        if(group->numChildren() == 1 && typeid(group) == typeid(SgGroup)){
            node = group->child(0);
        } else {
            node = group;
        }
    }
    group.reset();
    return node;
}


SgNode* YAMLSceneReader::readNodeList(ValueNode& node)
{
    SgGroupPtr group = new SgGroup;
    impl->readNodeList(node, group);
    return removeRedundantGroup(group).retn();
}    


SgNode* YAMLSceneReaderImpl::readNode(Mapping& node, const string& type)
{
    NodeFunctionMap::iterator q = nodeFunctionMap.find(type);
    if(q == nodeFunctionMap.end()){
        node.throwException(str(format(_("The node type \"%1%\" is not defined.")) % type));
    }

    NodeFunction funcToReadNode = q->second;
    SgNodePtr scene = (this->*funcToReadNode)(node);
    if(scene){
        if(node.read("name", symbol)){
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


SgNode* YAMLSceneReaderImpl::readGroup(Mapping& node)
{
    SgGroupPtr group = new SgGroup;
    readElements(node, group);
    return group.retn();
}


void YAMLSceneReaderImpl::readElements(Mapping& node, SgGroup* group)
{
    ValueNode& elements = *node.find("elements");
    if(elements.isValid()){
        readNodeList(elements, group);
    }
}


void YAMLSceneReaderImpl::readNodeList(ValueNode& elements, SgGroup* group)
{
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


SgNode* YAMLSceneReaderImpl::readTransform(Mapping& node)
{
    SgGroupPtr group;

    SgPosTransformPtr posTransform = new SgPosTransform;
    if(read(node, "translation", v)){
        posTransform->setTranslation(v);
        group = posTransform;
    }
    Matrix3 R;
    if(self->readRotation(node, R, false)){
        posTransform->setRotation(R);
        if(!group){
            group = posTransform;
        }
    }

    if(!read(node, "scale", v)){
        if(!group){
            group = new SgGroup;
        }
        readElements(node, group);

    } else {
        SgScaleTransformPtr scale = new SgScaleTransform;
        scale->setScale(v);
        if(group){
            group->addChild(scale);
        } else {
            group = scale;
        }
        readElements(node, scale);
    }

    // Necessary to prevent group from being deleted when exiting the function
    posTransform.reset();
    
    return group.retn();
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

        Mapping& appearance = *node.findMapping("appearance");
        if(appearance.isValid()){
            readAppearance(shape, appearance);
        } else {
            setDefaultMaterial(shape);
        }

        if(shape->texture()){
            generateTexCoord = true;
        }else{
            generateTexCoord = false;
        }
        shape->setMesh(readGeometry(geometry));

        scene = readTransformParameters(node, shape);

        if(scene == shape){
            return shape.retn();
        }
    }

    return scene;
}


SgMesh* YAMLSceneReaderImpl::readGeometry(Mapping& node)
{
    SgMesh* mesh = 0;
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
    } else if(type == "IndexedFaceSet"){
        mesh = readIndexedFaceSet(node);
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
    return meshGenerator.generateBox(size, generateTexCoord);
}


SgMesh* YAMLSceneReaderImpl::readSphere(Mapping& node)
{
    return meshGenerator.generateSphere(node.get("radius", 1.0), generateTexCoord);
}


SgMesh* YAMLSceneReaderImpl::readCylinder(Mapping& node)
{
    double radius = node.get("radius", 1.0);
    double height = node.get("height", 1.0);
    bool bottom = node.get("bottom", true);
    bool top = node.get("top", true);
    return meshGenerator.generateCylinder(radius, height, bottom, top, true, generateTexCoord);

}


SgMesh* YAMLSceneReaderImpl::readCone(Mapping& node)
{
    double radius = node.get("radius", 1.0);
    double height = node.get("height", 1.0);
    bool bottom = node.get("bottom", true);
    return meshGenerator.generateCone(radius, height, bottom, true, generateTexCoord);
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

    return meshGenerator.generateExtrusion(extrusion, generateTexCoord);
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

    SgTexCoordArray* texCoord = 0;
    Listing& texCoordNode = *node.findListing("texCoord");
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
        generateTexCoord = false;
    }

    SgMesh* mesh= meshGenerator.generateElevationGrid(grid, generateTexCoord);
    if(texCoord){
        mesh->setTexCoords(texCoord);
        mesh->texCoordIndices() = mesh->triangleVertices();
    }

    return mesh;
}


SgMesh* YAMLSceneReaderImpl::readIndexedFaceSet(Mapping& node)
{
    SgPolygonMeshPtr polygonMesh = new SgPolygonMesh;

    Listing& coordinateNode = *node.findListing("coordinate");
    if(coordinateNode.isValid()){
        const int size = coordinateNode.size() / 3;
        SgVertexArray& vertices = *polygonMesh->setVertices(new SgVertexArray());
        vertices.resize(size);
        for(int i=0; i < size; ++i){
            Vector3f& s = vertices[i];
            for(int j=0; j < 3; ++j){
                s[j] = coordinateNode[i*3+j].toDouble();
            }
        }
    }

    Listing& coordIndexNode = *node.findListing("coordIndex");
    if(coordIndexNode.isValid()){
        SgIndexArray& polygonVertices = polygonMesh->polygonVertices();
        const int size = coordIndexNode.size();
        polygonVertices.reserve(size);
        for(int i=0; i<size; i++){
            polygonVertices.push_back(coordIndexNode[i].toInt());
        }
    }

    Listing& texCoordNode = *node.findListing("texCoord");
    if(texCoordNode.isValid()){
        const int size = texCoordNode.size() / 2;
        SgTexCoordArray& texCoord = *polygonMesh->setTexCoords(new SgTexCoordArray());
        texCoord.resize(size);
        for(int i=0; i < size; ++i){
            Vector2f& s = texCoord[i];
            for(int j=0; j < 2; ++j){
                s[j] = texCoordNode[i*2+j].toDouble();
            }
        }
    }

    Listing& texCoordIndexNode = *node.findListing("texCoordIndex");
    if(texCoordIndexNode.isValid()){
        SgIndexArray& texCoordIndices = polygonMesh->texCoordIndices();
        const int size = texCoordIndexNode.size();
        texCoordIndices.reserve(size);
        for(int i=0; i<size; i++){
            texCoordIndices.push_back(texCoordIndexNode[i].toInt());
        }
    }

    //polygonMeshTriangulator.setDeepCopyEnabled(true);
    SgMesh* mesh= polygonMeshTriangulator.triangulate(polygonMesh);
    const string& errorMessage = polygonMeshTriangulator.errorMessage();
    if(!errorMessage.empty()){
        node.throwException("Error of an IndexedFaceSet node: \n" + errorMessage);
    }

    if(generateTexCoord){
        if(mesh && !mesh->hasTexCoords()){
            meshGenerator.generateTextureCoordinateForIndexedFaceSet(mesh);
        }
    }

    double creaseAngle = node.get("creaseAngle", 0.0);
    normalGenerator.generateNormals(mesh, creaseAngle);

    return mesh;
}


SgMesh* YAMLSceneReaderImpl::readResourceAsGeometry(Mapping& node)
{
    SgNode* resource = readResource(node);
    if(resource){
        SgShape* shape = dynamic_cast<SgShape*>(resource);
        if(!shape){
            node.throwException(_("A resouce specified as a geometry must be a single mesh"));
        }
        if(generateTexCoord){
            SgMesh* mesh = shape->mesh();
            if(mesh && !mesh->hasTexCoords()){
                meshGenerator.generateTextureCoordinateForIndexedFaceSet(mesh);
            }
            return mesh;
        }else{
            return shape->mesh();
        }
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

    Mapping& texture = *node.findMapping("texture");
    if(texture.isValid()){
        readTexture(shape, texture);

        Mapping& textureTransform = *node.findMapping("textureTransform");
        if(textureTransform.isValid() && shape->texture()){
            readTextureTransform(shape->texture(), textureTransform);
        }
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


void YAMLSceneReaderImpl::readTexture(SgShape* shape, Mapping& node)
{
    string& url = symbol;
    if(node.read("url", url)){
        if(!url.empty()){
            SgImagePtr image=0;
            ImagePathToSgImageMap::iterator p = imagePathToSgImageMap.find(url);
            if(p != imagePathToSgImageMap.end()){
                image = p->second;
            }else{
                try{
                    image = new SgImage;
                    filesystem::path filepath(url);
                    if(!checkAbsolute(filepath)){
                        filepath = baseDirectory / filepath;
                        filepath.normalize();
                    }
                    imageIO.load(image->image(), getAbsolutePathString(filepath));
                    imagePathToSgImageMap[url] = image;
                }catch(const exception_base& ex){
                    node.throwException(*boost::get_error_info<error_info_message>(ex));
                }
            }
            if(image){
                SgTexturePtr texture = new SgTexture;
                texture->setImage(image);
                bool repeatS = true;
                bool repeatT = true;
                node.read("repeatS", repeatS);
                node.read("repeatT", repeatT);
                texture->setRepeat(repeatS, repeatT);
                texture->setTextureTransform(new SgTextureTransform);
                shape->setTexture(texture);
            }
        }
    }
}


void YAMLSceneReaderImpl::readTextureTransform(SgTexture* texture, Mapping& node)
{
    SgTextureTransform* textureTransform = texture->textureTransform();
    if(read(node, "center", v2)) textureTransform->setCenter(v2);
    if(read(node, "scale", v2)) textureTransform->setScale(v2);
    if(read(node, "translation", v2)) textureTransform->setTranslation(v2);
    if(self->readAngle(node, "rotation", value)) textureTransform->setRotation(value);
}


void YAMLSceneReaderImpl::readLightCommon(Mapping& node, SgLight* light)
{
    if(node.read("on", on)) light->on(on);
    if(read(node, "color", color)) light->setColor(color);
    if(node.read("intensity", value)) light->setIntensity(value);
    if(node.read("ambientIntensity", value)) light->setAmbientIntensity(value);
}


SgNode* YAMLSceneReaderImpl::readDirectionalLight(Mapping& node)
{
    SgDirectionalLightPtr light = new SgDirectionalLight;
    readLightCommon(node, light);
    if(read(node, "direction", v)) light->setDirection(v);
    return light.retn();
}


SgNode* YAMLSceneReaderImpl::readSpotLight(Mapping& node)
{
    SgSpotLightPtr light = new SgSpotLight;

    readLightCommon(node, light);

    if(read(node, "direction", v)) light->setDirection(v);
    if(readAngle(node, "beamWidth", value)) light->setBeamWidth(value);
    if(readAngle(node, "cutOffAngle", value)) light->setCutOffAngle(value);
    if(node.read("cutOffExponent", value)) light->setCutOffExponent(value);
    if(read(node, "attenuation", color)){
        light->setConstantAttenuation(color[0]);
        light->setLinearAttenuation(color[1]);
        light->setQuadraticAttenuation(color[2]);
    }
    
    return light.retn();
}


SgNode* YAMLSceneReaderImpl::readResource(Mapping& node)
{
    SgNode* scene = 0;

    string& uri = symbol;
    if(node.read("uri", uri)){
        SgNodePtr resource;
        string nodeName;

        ValueNode& exclude = *node.find("exclude");
        if(exclude.isValid()){
            if(exclude.isListing()){
                Listing& excludes = *exclude.toListing();
                for(auto& nodeToExclude : excludes){
                    decoupleResourceNode(uri, nodeToExclude->toString());
                }
            } else if(exclude.isString()){
                decoupleResourceNode(uri, exclude.toString());
            }
        }
        
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


void YAMLSceneReaderImpl::decoupleResourceNode(const string& uri, const string& nodeName)
{
    ResourceInfo* resourceInfo = getOrCreateResourceInfo(uri);
    if(resourceInfo){
        unique_ptr<NodeMap>& nodeMap = resourceInfo->nodeMap;
        if(!nodeMap){
            makeNodeMap(resourceInfo);
        }
        auto iter = nodeMap->find(nodeName);
        if(iter != nodeMap->end()){
            NodeInfo& nodeInfo = iter->second;
            if(nodeInfo.parent){
                nodeInfo.parent->removeChild(nodeInfo.node);
                nodeInfo.parent = 0;
                adjustNodeCoordinate(nodeInfo);
            }
        }
    }
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
    smatch match;
    bool hasScheme = false;
        
    if(regex_match(uri, match, uriSchemeRegex)){
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
