#include "URDFBodyLoader.h"
#include "URDFKeywords.h"
#include <cnoid/Body>
#include <cnoid/BodyLoader>
#include <cnoid/Camera>
#include <cnoid/EigenUtil>
#include <cnoid/ExecutablePath>
#include <cnoid/MeshGenerator>
#include <cnoid/NullOut>
#include <cnoid/RangeCamera>
#include <cnoid/RangeSensor>
#include <cnoid/SceneLoader>
#include <cnoid/UriSchemeProcessor>
#include <cnoid/UTF8>
#include <cnoid/stdx/filesystem>
#include <fmt/format.h>
#include <pugixml.hpp>
#include <memory>
#include <ostream>
#include <string>
#include <unordered_map>
#include <vector>
#include <cstdio>
#include "gettext.h"

using namespace cnoid;

namespace filesystem = cnoid::stdx::filesystem;
using fmt::format;
using pugi::xml_attribute;
using pugi::xml_node;
using std::endl;
using std::string;
using std::vector;

namespace {

struct Registration
{
    Registration()
    {
        BodyLoader::registerLoader({"urdf", "xacro"},
                                   []() -> AbstractBodyLoaderPtr {
                                       return std::make_shared<URDFBodyLoader>();
                                   });
    }
} registration;

bool toVector4(const std::string& s, Vector4& out_v)
{
    const char* nptr = s.c_str();
    char* endptr;
    for (int i = 0; i < out_v.rows(); ++i) {
        out_v[i] = strtod(nptr, &endptr);
        if (endptr == nptr) {
            return false;
        }
        nptr = endptr;
        while (isspace(*nptr)) {
            nptr++;
        }
        if (*nptr == ',') {
            nptr++;
        }
    }
    return true;
}

struct ShapeDescription
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    Isometry3 origin;
    enum ShapeType { None, Mesh, Box, Cylinder, Sphere } shapeType;
    string meshUri;
    Vector3 meshScale;
    Vector3 boxSize;
    double radius;
    double length;
    SgMaterialPtr material;

    ShapeDescription()
        : origin(Isometry3::Identity()),
          shapeType(None),
          meshScale(Vector3::Ones()),
          boxSize(Vector3::Zero()),
          radius(0.0),
          length(0.0)
    { }

    bool operator!=(const ShapeDescription& rhs) const {
        return
            !origin.isApprox(rhs.origin) ||
            shapeType != rhs.shapeType ||
            meshUri != rhs.meshUri ||
            meshScale != rhs.meshScale ||
            boxSize != rhs.boxSize ||
            radius != rhs.radius ||
            length != rhs.length;
    }
};

} // namespace

namespace cnoid {

class URDFBodyLoader::Impl
{
public:
    int jointCounter_ = 0;
    vector<ShapeDescription, Eigen::aligned_allocator<ShapeDescription>> visualDescriptions;
    vector<ShapeDescription, Eigen::aligned_allocator<ShapeDescription>> collisionDescriptions;
    SceneLoader sceneLoader;
    UriSchemeProcessor uriSchemeProcessor;
    std::unordered_map<string, Vector4> colorMap;
    std::unordered_map<string, SgNodePtr> meshMap;
    MeshGenerator meshGenerator;
    std::ostream* os_;
    std::ostream& os() { return *os_; }

    Impl();
    bool load(Body* body, const string& filename);
    void updateColorMap(const xml_node& materialNode);
    vector<LinkPtr> findRootLinks(const std::unordered_map<string, LinkPtr>& linkMap);
    bool loadLink(LinkPtr link, const xml_node& linkNode);
    bool loadInertialTag(LinkPtr& link, const xml_node& inertialNode);
    bool readOriginTag(const xml_node& originNode, Vector3& translation, Matrix3& rotation);
    bool readOriginTag(const xml_node& originNode, Isometry3& origin);
    void printReadingInertiaTagError(const string& attribute_name);
    bool readInertiaTag(const xml_node& inertiaNode, Matrix3& inertiaMatrix);
    bool readShapeDescription(const xml_node& shapeNode, ShapeDescription& description);
    bool readGeometryTag(const xml_node& geometryNode, ShapeDescription& description);
    SgNode* createShape(ShapeDescription& description);
    SgNode* createMesh(ShapeDescription& description);
    SgMaterial* readMaterialTag(const xml_node& materialNode);
    void setMaterialToAllShapeNodes(SgNode* node, SgMaterial* material);
    bool loadJoint(std::unordered_map<string, LinkPtr>& linkMap, const xml_node& jointNode);
    bool loadSensor(
        Body* body, std::unordered_map<string, LinkPtr>& linkMap, const xml_node& sensorNode);
};
}  // namespace cnoid

URDFBodyLoader::URDFBodyLoader()
{
    impl = new Impl;
}


URDFBodyLoader::Impl::Impl()
    : os_(&nullout())
{}


URDFBodyLoader::~URDFBodyLoader()
{
    delete impl;
}


void URDFBodyLoader::setMessageSink(std::ostream& os)
{
    impl->os_ = &os;
    impl->sceneLoader.setMessageSink(os);
}


void URDFBodyLoader::setDefaultDivisionNumber(int n)
{
    impl->sceneLoader.setDefaultDivisionNumber(n);
}


void URDFBodyLoader::setDefaultCreaseAngle(double theta)
{
    impl->sceneLoader.setDefaultCreaseAngle(theta);
}


bool URDFBodyLoader::load(Body* body, const std::string& filename)
{
    return impl->load(body, filename);
}


bool URDFBodyLoader::Impl::load(Body* body, const string& filename)
{
    pugi::xml_document doc;

    // initialized members
    jointCounter_ = 0;
    colorMap.clear();
    meshMap.clear();

    pugi::xml_parse_result result;

    const string suffix = ".xacro";
    if (filename.size() >= suffix.size()
        && std::equal(suffix.begin(),
                      suffix.end(),
                      filename.end() - suffix.size())) {
#ifdef _WIN32
        os() << "Error: Xacro files are not supported in Windows." << endl;
        return false;
#else
        // parses and reads a xacro-formatted URDF
        char buffer[128];
        std::string urdf_content;
        FILE* pipe = popen(format("{0}/cnoid-xacro {1}", executableDir(), filename).c_str(), "r");
        if (!pipe) {
            os() << "Error: popen() for xacro parsing failed." << endl;
            return false;
        }
        try {
            while (fgets(buffer, sizeof(buffer), pipe) != NULL) {
                urdf_content += buffer;
            }
        } catch (...) {
            pclose(pipe);
            os() << "Error: copying xacro contents failed." << endl;
            return false;
        }
        pclose(pipe);

        result = doc.load_string(urdf_content.data());
#endif
    } else {
        // loads a URDF
        result = doc.load_file(filename.c_str());
    }

    if (!result) {
        os() << "Error: parsing XML failed: " << result.description() << endl;
        return false;
    }

    // checks if only one 'robot' tag exists in the URDF
    if (doc.child(ROBOT).empty()) {
        os() << "Error: 'robot' tag is not found.";
        return false;
    } else if (++doc.children(ROBOT).begin() != doc.children(ROBOT).end()) {
        os() << "Error: multiple 'robot' tags are found.";
        return false;
    }

    uriSchemeProcessor.setBaseDirectoryFor(filename);

    // gets the 'robot' tag
    const xml_node& robotNode = doc.child(ROBOT);
    const string robotName = robotNode.attribute(NAME).as_string();
    if (!robotName.empty()) {
        body->setName(robotName);
    }

    // creates a color dictionary before parsing the robot model
    for (xml_node& linkNode : robotNode.children(LINK)) {
        for (xml_node& visualNode : linkNode.children(VISUAL)) {
            const xml_node& materialNode = visualNode.child(MATERIAL);
            if (!materialNode.empty()) {
                updateColorMap(materialNode);
            }
        }
    }

    // creates a link dictionary by loading all links for tree construction
    auto linkNodes = robotNode.children(LINK);
    std::unordered_map<string, LinkPtr> linkMap(std::distance(linkNodes.begin(), linkNodes.end()));

    // loads all links
    for (xml_node linkNode : linkNodes) {
        LinkPtr link = new Link;
        if (!loadLink(link, linkNode)) {
            return false;
        }
        const auto result = linkMap.emplace(link->name(), link);
        if (!result.second) {
            os() << "Error: multiple links named \"" << link->name()
                 << "\" are found." << endl;
            return false;
        }
    }

    // creates a joint dictionary to check independence of joint names
    auto jointNodes = robotNode.children(JOINT);
    std::unordered_map<string, int> jointMap(std::distance(jointNodes.begin(), jointNodes.end()));

    // loads all joints with creating a link tree
    for (xml_node jointNode : jointNodes) {
        if (!loadJoint(linkMap, jointNode)) {
            return false;
        }
        const string jointName = jointNode.attribute(NAME).as_string();
        const auto result = jointMap.emplace(jointName, 0);
        if (!result.second) {
            os() << "Error: multiple joints named \"" << jointName
                 << "\" are found." << endl;
            return false;
        }
    }

    // finds the unique root link
    vector<LinkPtr> rootLinks = findRootLinks(linkMap);
    if (rootLinks.empty()) {
        os() << "Error: no root link is found." << endl;
        return false;
    } else if (rootLinks.size() > 1) {
        os() << "Error: multiple root links are found." << endl;
        return false;
    }
    body->setRootLink(rootLinks.at(0));
    body->rootLink()->setJointType(Link::FreeJoint);

    // loads sensors
    auto sensorNodes = robotNode.children(SENSOR);
    for (xml_node sensorNode : sensorNodes) {
        if (!loadSensor(body, linkMap, sensorNode)) {
            return false;
        }
    }

    return true;
}


void URDFBodyLoader::Impl::updateColorMap(const xml_node& materialNode)
{
    const string materialName = materialNode.attribute(NAME).as_string();
    const xml_node& colorNode = materialNode.child(COLOR);
    const xml_node& textureNode = materialNode.child(TEXTURE);

    if (!colorNode.empty()) {
        // case: a 'color' tag exists
        const string rgbaString = colorNode.attribute(RGBA).as_string();
        Vector4 rgba = Vector4::Zero();
        if (toVector4(rgbaString, rgba)) {
            // normalizes value
            rgba.array().min(1.0).max(0.0);

            // tries to register the material
            auto result = colorMap.emplace(materialName, rgba);
            if (!result.second) {
                os() << "Warning: failed to add material named \""
                        << materialName << "\"." << endl;
            }
        }
    } else if (!textureNode.empty()) {
        // case: a 'texture' tag exists
        os() << "Warning: 'texture' tag is currently not supported."
                << endl;
    }
}


vector<LinkPtr> URDFBodyLoader::Impl::findRootLinks(
    const std::unordered_map<string, LinkPtr>& linkMap)
{
    vector<LinkPtr> rootLinks;
    for (auto mapElement : linkMap) {
        const LinkPtr link = mapElement.second;
        if (link->parent() == nullptr) {
            rootLinks.push_back(link);
        }
    }
    return rootLinks;
}


bool URDFBodyLoader::Impl::loadLink(LinkPtr link, const xml_node& linkNode)
{
    // sets name (requrired)
    std::string name(linkNode.attribute(NAME).as_string());
    if (name.empty()) {
        os() << "Error: there exist a unnamed link." << endl;
        return false;
    }
    link->setName(name);

    // 'inertial' (optional)
    const xml_node& inertialNode = linkNode.child(INERTIAL);
    if (inertialNode.empty()) {
        os() << "Debug: link \"" << name << "\" has no inertial data." << endl;
    } else {
        if (!loadInertialTag(link, inertialNode)) {
            os() << "in link \"" << name << "\"." << endl;
            return false;
        }
    }

    // 'visual' (optional, multiple definitions are allowed)
    visualDescriptions.clear();
    if (linkNode.child(VISUAL).empty()) {
        os() << "Debug: link \"" << name << "\" has no visual data." << endl;
    } else {
        const auto visualNodes = linkNode.children(VISUAL);
        for (xml_node& visualNode : visualNodes) {
            visualDescriptions.emplace_back();
            auto& description = visualDescriptions.back();
            if (!readShapeDescription(visualNode, description)) {
                os() << " in a visual tag in link \"" << name << "\"." << endl;
                return false;
            }
            // 'material' tag (optional)
            const xml_node& materialNode = visualNode.child(MATERIAL);
            if (!materialNode.empty()) {
                description.material = readMaterialTag(materialNode);
            }
        }
        for (auto& description : visualDescriptions) {
            auto shape = createShape(description);
            if (!shape) {
                os() << " in a visual tag in link \"" << name << "\"." << endl;
                return false;
            } else {
                link->addVisualShapeNode(shape);
                if (description.material) {
                    setMaterialToAllShapeNodes(shape, description.material);
                    description.material.reset(); // For comparison with collision descriptions
                }
            }
        }
    }
                
    // 'collision' (optional, multiple definitions are allowed)
    collisionDescriptions.clear();
    if (linkNode.child(COLLISION).empty()) {
        os() << "Debug: link \"" << name << "\" has no collision data." << endl;
    } else {
        const auto collisionNodes = linkNode.children(COLLISION);
        for (xml_node& collisionNode : collisionNodes) {
            collisionDescriptions.emplace_back();
            if (!readShapeDescription(collisionNode, collisionDescriptions.back())) {
                os() << " in a collision tag in link \"" << name << "\"." << endl;
                return false;
            }
        }
        bool isDifferent = true;
        if (collisionDescriptions.size() == visualDescriptions.size()) {
            isDifferent = false;
            for(size_t i=0; i < collisionDescriptions.size(); ++i){
                if (collisionDescriptions[i] != visualDescriptions[i]) {
                    isDifferent = true;
                    break;
                }
            }
        }
        if (!isDifferent) {
            // Share the shape between visual and collision
            link->visualShape()->copyChildrenTo(link->collisionShape());
        } else {
            for (auto& description : collisionDescriptions) {
                auto shape = createShape(description);
                if (!shape) {
                    os() << " in a collision tag in link \"" << name << "\"." << endl;
                    return false;
                } else {
                    link->addCollisionShapeNode(shape);
                }
            }
        }
    }

    return true;
}


bool URDFBodyLoader::Impl::loadInertialTag(LinkPtr& link,
                                           const xml_node& inertialNode)
{
    // 'origin' tag (optional)
    const xml_node& originNode = inertialNode.child(ORIGIN);
    Vector3 translation = Vector3::Zero();
    Matrix3 rotation = Matrix3::Identity();
    if (!readOriginTag(originNode, translation, rotation)) {
        os() << " in an inertial tag";
        return false;
    }
    link->setCenterOfMass(translation);

    // 'mass' tag
    const double mass = inertialNode.child(MASS).attribute(VALUE).as_double();
    if (mass > 0.0) {
        link->setMass(mass);
    } else {
        os() << "Error: mass value is invalid";
        return false;
    }

    // 'inertia' tag
    Matrix3 inertiaMatrix = Matrix3::Identity();
    const xml_node& inertiaNode = inertialNode.child(INERTIA);
    if (!inertiaNode.empty()) {
        if (!readInertiaTag(inertiaNode, inertiaMatrix)) {
            return false;
        }
    }
    link->setInertia(rotation * inertiaMatrix * rotation.transpose());

    return true;
}


bool URDFBodyLoader::Impl::readOriginTag(const xml_node& originNode,
                                         Vector3& translation,
                                         Matrix3& rotation)
{
    const string origin_xyz_str = originNode.attribute(XYZ).as_string();
    if (origin_xyz_str.empty()) {
        translation = Vector3::Zero();
    } else {
        Vector3 origin_xyz;
        if (!toVector3(origin_xyz_str, translation)) {
            os() << "Error: origin xyz is written in invalid format";
            return false;
        }
    }

    const string origin_rpy_str = originNode.attribute(RPY).as_string();
    if (origin_rpy_str.empty()) {
        rotation = Matrix3::Identity();
    } else {
        Vector3 origin_rpy;
        if (!toVector3(origin_rpy_str, origin_rpy)) {
            os() << "Error: origin rpy is written in invalid format";
            return false;
        }
        rotation = rotFromRpy(origin_rpy);
    }
    return true;
}


bool URDFBodyLoader::Impl::readOriginTag(const xml_node& originNode, Isometry3& origin)
{
    const string origin_xyz_str = originNode.attribute(XYZ).as_string();
    if (!origin_xyz_str.empty()) {
        Vector3 origin_xyz;
        if (!toVector3(origin_xyz_str, origin_xyz)) {
            os() << "Error: origin xyz is written in invalid format";
            return false;
        }
        origin.translation() = origin_xyz;
    }

    const string origin_rpy_str = originNode.attribute(RPY).as_string();
    if (!origin_rpy_str.empty()) {
        Vector3 origin_rpy;
        if (!toVector3(origin_rpy_str, origin_rpy)) {
            os() << "Error: origin rpy is written in invalid format";
            return false;
        }
        origin.linear() = rotFromRpy(origin_rpy);
    }
    
    return true;
}


void URDFBodyLoader::Impl::printReadingInertiaTagError(
    const string& attribute_name)
{
    os() << "Error: " << attribute_name << " value is not defined";
    return;
}


bool URDFBodyLoader::Impl::readInertiaTag(const xml_node& inertiaNode,
                                          Matrix3& inertiaMatrix)
{
    if (inertiaNode.attribute(IXX).empty()) {
        printReadingInertiaTagError(IXX);
        return false;
    } else {
        inertiaMatrix(0, 0) = inertiaNode.attribute(IXX).as_double();
    }
    if (inertiaNode.attribute(IXY).empty()) {
        printReadingInertiaTagError(IXY);
        return false;
    } else {
        inertiaMatrix(0, 1) = inertiaNode.attribute(IXY).as_double();
        inertiaMatrix(1, 0) = inertiaMatrix(0, 1);
    }
    if (inertiaNode.attribute(IXZ).empty()) {
        printReadingInertiaTagError(IXZ);
        return false;
    } else {
        inertiaMatrix(0, 2) = inertiaNode.attribute(IXZ).as_double();
        inertiaMatrix(2, 0) = inertiaMatrix(0, 2);
    }
    if (inertiaNode.attribute(IYY).empty()) {
        printReadingInertiaTagError(IYY);
        return false;
    } else {
        inertiaMatrix(1, 1) = inertiaNode.attribute(IYY).as_double();
    }
    if (inertiaNode.attribute(IYZ).empty()) {
        printReadingInertiaTagError(IYZ);
        return false;
    } else {
        inertiaMatrix(1, 2) = inertiaNode.attribute(IYZ).as_double();
        inertiaMatrix(2, 1) = inertiaMatrix(1, 2);
    }
    if (inertiaNode.attribute(IZZ).empty()) {
        printReadingInertiaTagError(IZZ);
        return false;
    } else {
        inertiaMatrix(2, 2) = inertiaNode.attribute(IZZ).as_double();
    }

    return true;
}


bool URDFBodyLoader::Impl::readShapeDescription(const xml_node& shapeNode, ShapeDescription& description)
{
    // 'origin' tag (optional)
    const xml_node& originNode = shapeNode.child(ORIGIN);
    if (!readOriginTag(originNode, description.origin)) {
        return false;
    }

    // 'geometry' tag (required)
    const xml_node& geometryNode = shapeNode.child(GEOMETRY);
    if (geometryNode.empty()) {
        os() << "Error: geometry is not found";
        return false;
    }

    if (!readGeometryTag(geometryNode, description)) {
        os() << "Error: loading collision geometry failed";
        return false;
    }

    return true;
}    
    

bool URDFBodyLoader::Impl::readGeometryTag(const xml_node& geometryNode, ShapeDescription& description)
{
    int n = std::distance(geometryNode.begin(), geometryNode.end());
    if (n < 1) {
        os() << "Error: no geometry is found";
        return false;
    } else if (n > 1) {
        os() << "Error: one link can have only one geometry";
        return false;
    }

    if (!geometryNode.child(MESH).empty()) {
        description.shapeType = ShapeDescription::Mesh;

        if (geometryNode.child(MESH).attribute(FILENAME).empty()) {
            os() << "Error: mesh file is not specified";
            return false;
        }
        description.meshUri = geometryNode.child(MESH).attribute(FILENAME).as_string();

        if (!geometryNode.child(MESH).attribute(SCALE).empty()) {
            if (!toVector3(geometryNode.child(MESH).attribute(SCALE).as_string(), description.meshScale)) {
                os() << "Error: mesh scale is written in invalid format";
                return false;
            }
        }
    } else if (!geometryNode.child(BOX).empty()) {
        description.shapeType = ShapeDescription::Box;
        if (!toVector3(geometryNode.child(BOX).attribute(SIZE).as_string(), description.boxSize)) {
            os() << "Error: box size is written in invalid format";
            return false;
        }
    } else if (!geometryNode.child(CYLINDER).empty()) {
        description.shapeType = ShapeDescription::Cylinder;
        if (geometryNode.child(CYLINDER).attribute(RADIUS).empty()) {
            os() << "Error: cylinder radius is not defined";
            return false;
        }
        if (geometryNode.child(CYLINDER).attribute(LENGTH).empty()) {
            os() << "Error: cylinder length is not defined";
            return false;
        }
        description.radius = geometryNode.child(CYLINDER).attribute(RADIUS).as_double();
        description.length = geometryNode.child(CYLINDER).attribute(LENGTH).as_double();
        
    } else if (!geometryNode.child(SPHERE).empty()) {
        if (geometryNode.child(SPHERE).attribute(RADIUS).empty()) {
            os() << "Error: sphere radius is not defined";
            return false;
        }
        description.radius = geometryNode.child(SPHERE).attribute(RADIUS).as_double();

    } else {
        os() << "Error: unsupported geometry \""
             << geometryNode.first_child().name() << "\" is described";
        return false;
    }

    return true;
}


SgNode* URDFBodyLoader::Impl::createShape(ShapeDescription& description)
{
    SgNodePtr shape;

    switch (description.shapeType) {
    case ShapeDescription::Mesh:
        shape = createMesh(description);
        break;
    case ShapeDescription::Box: {
        auto box = new SgShape;
        box->setMesh(meshGenerator.generateBox(description.boxSize));
        shape = box;
        break;
    }
    case ShapeDescription::Cylinder: {
        auto cylinder = new SgShape;
        cylinder->setMesh(meshGenerator.generateCylinder(description.radius, description.length));
        auto transform = new SgPosTransform;
        transform->setRotation(AngleAxis(M_PI / 2.0, Vector3::UnitX()));
        transform->addChild(cylinder);
        shape = transform;
        break;
    }
    case ShapeDescription::Sphere: {
        auto sphere = new SgShape;
        sphere->setMesh(meshGenerator.generateSphere(description.radius));
        shape = sphere;
        break;
    }
    default:
        break;
    }

    if(!description.origin.isApprox(Isometry3::Identity())){
        auto transform = new SgPosTransform(description.origin);
        transform->addChild(shape);
        shape = transform;
    }

    return shape.retn();
}


SgNode* URDFBodyLoader::Impl::createMesh(ShapeDescription& description)
{
    SgNodePtr scene;

    string filePath = uriSchemeProcessor.getFilePath(description.meshUri);
    if (filePath.empty()) {
        os() << uriSchemeProcessor.errorMessage() << endl;
        return nullptr;
    }
    
    auto it = meshMap.find(filePath);
    if (it != meshMap.end()) {
        scene = it->second;
    } else {
        bool isSupportedFormat = false;
        scene = sceneLoader.load(filePath, isSupportedFormat);
        if (!scene) {
            if(!isSupportedFormat) {
                os() << format(_("Error: format of the specified mesh file \"{0}\" is not supported"),
                               description.meshUri);
            }
            return nullptr;
        }
        if(auto uriObject = scene->findObject([](SgObject* object){ return object->hasUri(); })){
            uriObject->setUri(description.meshUri, filePath);
        }
        meshMap[filePath] = scene;
    }

    if (description.meshScale != Vector3::Ones()) {
        auto scaler = new SgScaleTransform;
        scaler->setScale(description.meshScale);
        scaler->addChild(scene);
        scene = scaler;
    }

    return scene.retn();
}


SgMaterial* URDFBodyLoader::Impl::readMaterialTag(const xml_node& materialNode)
{
    SgMaterialPtr material;
    
    const string materialName = materialNode.attribute(NAME).as_string();
    const xml_node& colorNode = materialNode.child(COLOR);
    const xml_node& textureNode = materialNode.child(TEXTURE);
    if (!colorNode.empty()) {
        // case: a 'color' tag exists
        const string rgbaString = colorNode.attribute(RGBA).as_string();
        Vector4 rgba = Vector4::Zero();
        if (toVector4(rgbaString, rgba)) {
            // normalizes value
            rgba.array().min(1.0).max(0.0);

            material = new SgMaterial;
            material->setTransparency(1.0 - rgba[3]);
            material->setDiffuseColor(Vector3(rgba[0], rgba[1], rgba[2]));
        }
    } else if (!textureNode.empty()) {
        // case: a 'texture' tag exists
        os() << "Warning: 'texture' tag is currently not supported." << endl;

    } else if (!materialName.empty()) {
        // case: neither a 'color' tag nor 'texture' tag does not exist
        //       but a material name is specified
        auto it = colorMap.find(materialName);
        if (it == colorMap.end()) {
            os() << "Warning: a material named \"" << materialName << "\" is not found." << endl;
        } else {
            const Vector4& rgba = it->second;
            material = new SgMaterial;
            material->setTransparency(1.0 - rgba[3]);
            material->setDiffuseColor(Vector3(rgba[0], rgba[1], rgba[2]));
        }
    }

    return material.retn();
}


void URDFBodyLoader::Impl::setMaterialToAllShapeNodes(SgNode* node, SgMaterial* material)
{
    if (auto group = node->toGroupNode()) {
        for (auto& child : *group) {
            setMaterialToAllShapeNodes(child, material);
        }
    } else if (auto shape = dynamic_cast<SgShape*>(node)) {
        shape->setMaterial(material);
    }
}


bool URDFBodyLoader::Impl::loadJoint(
    std::unordered_map<string, LinkPtr>& linkMap, const xml_node& jointNode)
{
    // 'name' attribute (required)
    if (jointNode.attribute(NAME).empty()) {
        os() << "Error: an unnamed joint is found." << endl;
        return false;
    }
    const string jointName = jointNode.attribute(NAME).as_string();

    // 'parent' tag (required)
    if (jointNode.child(PARENT).attribute(LINK).empty()) {
        os() << "Error: joint \"" << jointName << "\" has no parent." << endl;
        return false;
    }
    const string parentName = jointNode.child(PARENT).attribute(LINK).as_string();
    auto parentSearchResult = linkMap.find(parentName);
    if (parentSearchResult == linkMap.end()) {
        os() << "Error: parent in joint \"" << jointName << "\" is invalid."
             << endl;
        return false;
    }
    const LinkPtr parent = parentSearchResult->second;

    // 'child' tag (required)
    if (jointNode.child(CHILD).attribute(LINK).empty()) {
        os() << "Error: joint \"" << jointName << "\" has no child." << endl;
        return false;
    }
    const string childName = jointNode.child(CHILD).attribute(LINK).as_string();
    auto childSearchResult = linkMap.find(childName);
    if (childSearchResult == linkMap.end()) {
        os() << "Error: child in joint \"" << jointName << "\" is invalid."
             << endl;
        return false;
    }
    const LinkPtr child = childSearchResult->second;

    parent->appendChild(child);
    child->setParent(parent);
    child->setJointName(jointName);

    // 'type' attribute (required)
    if (jointNode.attribute(TYPE).empty()) {
        os() << "Error: type of joint \"" << jointName << "\" is not defined."
             << endl;
        return false;
    }
    const string jointType = jointNode.attribute(TYPE).as_string();
    if (jointType == REVOLUTE || jointType == CONTINUOUS) {
        child->setJointType(Link::RevoluteJoint);
        child->setJointId(jointCounter_++);
    } else if (jointType == PRISMATIC) {
        child->setJointType(Link::PrismaticJoint);
        child->setJointId(jointCounter_++);
    } else if (jointType == FIXED) {
        child->setJointType(Link::FixedJoint);
    }

    // 'origin' tag (optional)
    if (!jointNode.child(ORIGIN).empty()) {
        Vector3 translation;
        Matrix3 rotation;
        if (!readOriginTag(jointNode.child(ORIGIN), translation, rotation)) {
            os() << " in reading joint \"" << jointName << "\"." << endl;
            return false;
        }
        child->setOffsetTranslation(translation);
        child->setOffsetRotation(rotation);
    }

    // 'axis' tag (optional)
    const xml_node& axisNode = jointNode.child(AXIS);
    if (axisNode.empty()) {
        child->setJointAxis(Vector3::UnitX());
    } else {
        const xml_attribute& xyzAttribute = axisNode.attribute(XYZ);
        if (xyzAttribute.empty()) {
            os() << "Error: axis of joint \"" << jointName
                 << "\" is not defined while 'axis' tag is written.";
            return false;
        }

        Vector3 axis = Vector3::UnitX();
        if (!toVector3(xyzAttribute.as_string(), axis)) {
            os() << "Error: axis of joint \"" << jointName
                 << "\" is written in invalid format." << endl;
            return false;
        }
        axis.normalize();
        child->setJointAxis(axis);
    }

    // 'limit' tag (partially required)
    const xml_node& limitNode = jointNode.child(LIMIT);
    if (limitNode.empty()) {
        if (jointType == REVOLUTE || jointType == PRISMATIC) {
            os() << "Error: limit of joint \"" << jointName
                 << "\" is not defined." << endl;
            return false;
        }
    } else {
        // 'lower' and 'upper' attributes (optional, default: 0.0)
        if (jointType == REVOLUTE || jointType == PRISMATIC) {
            double lower = 0.0, upper = 0.0;
            const xml_attribute& lowerAttribute = limitNode.attribute(LOWER);
            if (!lowerAttribute.empty()) {
                lower = lowerAttribute.as_double();
            }
            const xml_attribute& upperAttribute = limitNode.attribute(UPPER);
            if (!upperAttribute.empty()) {
                upper = upperAttribute.as_double();
            }
            child->setJointRange(lower, upper);
        }
        // 'velocity' and 'effort' attributes (required)
        if (jointType == REVOLUTE || jointType == PRISMATIC
            || jointType == CONTINUOUS) {
            const xml_attribute& velocityAttribute = limitNode.attribute(
                VELOCITY);
            if (velocityAttribute.empty()) {
                os() << "Error: velocity limit of joint \"" << jointName
                     << "\" is not defined." << endl;
                return false;
            }
            const double velocityLimit = velocityAttribute.as_double();
            if (velocityLimit < 0.0) {
                os() << "Error: velocity limit of joint \"" << jointName
                     << "\" has to be positive." << endl;
                return false;
            }
            child->setJointVelocityRange(-velocityLimit, velocityLimit);

            const xml_attribute& effortAttribute = limitNode.attribute(EFFORT);
            if (effortAttribute.empty()) {
                os() << "Error: effort limit of joint \"" << jointName
                     << "\" is not defined." << endl;
                return false;
            }
            const double effortLimit = effortAttribute.as_double();
            if (effortLimit < 0.0) {
                os() << "Error: effort limit of joint \"" << jointName
                     << "\" has to be positive." << endl;
                return false;
            }
            child->setJointEffortRange(-effortLimit, effortLimit);
        }
    }

    // 'dynamics' tag (not supported in choreonoid)
    if (!jointNode.child(DYNAMICS).empty()) {
        os() << "Warning: 'dynamics' tag is currently not supported." << endl;
    }

    // 'mimic' tag (not supported in choreonoid)
    if (!jointNode.child(MIMIC).empty()) {
        os() << "Warning: mimic joint is currently not supported." << endl;
    }
    return true;
}


bool URDFBodyLoader::Impl::loadSensor(
    Body* body,
    std::unordered_map<string, LinkPtr>& linkMap,
    const xml_node& sensorNode)
{
    // 'name' attribute (required)
    const string sensorName = sensorNode.attribute(NAME).as_string();
    if (sensorName.empty()) {
        os() << "Error: an unnamed sensor is found." << endl;
        return false;
    }

    // 'update_rate' attribute (optional)
    // default: 1.0
    const double update_rate = sensorNode.attribute(UPDATE_RATE).as_double(1.0);

    // 'parent' tag (required)
    LinkPtr parentLink = nullptr;
    if (sensorNode.child(PARENT).empty()) {
        os() << "Error: the sensor \"" << sensorName << "\" has no parent tag."
             << endl;
        return false;
    } else {
        // 'link' attribute (required)
        const string parentName
            = sensorNode.child(PARENT).attribute(LINK).as_string();
        if (parentName.empty()) {
            os() << "Error: the sensor \"" << sensorName
                 << "\" has no parent link name." << endl;
            return false;
        }

        auto parentSearchResult = linkMap.find(parentName);
        if (parentSearchResult == linkMap.end()) {
            os() << "Error: the parent name \"" << parentName
                 << "\" of the sensor \"" << sensorName << "\" is invalid."
                 << endl;
            return false;
        }
        parentLink = parentSearchResult->second;
    }
    // 'origin' tag (optional)
    const xml_node& originNode = sensorNode.child(ORIGIN);
    Vector3 translation = Vector3::Zero();
    Matrix3 rotation = Matrix3::Identity();
    if (!readOriginTag(originNode, translation, rotation)) {
        os() << " in the sensor \"" << sensorName << "\" definition." << endl;
        return false;
    }

    // creates a sensor
    if (!sensorNode.child(CAMERA).empty()) {
        // 'image' tag (required)
        const xml_node& imageNode = sensorNode.child(CAMERA).child(IMAGE);
        if (imageNode.empty()) {
            os() << "Error: the camera \"" << sensorName
                 << "\" has no image tag." << endl;
            return false;
        }

        // 'width' attribute (required)
        const int width = imageNode.attribute(WIDTH).as_int();
        if (width <= 0) {
            os() << "Error: image width of camera \"" << sensorName
                 << "\" is invalid." << endl;
            return false;
        }

        // 'height' attribute (required)
        const int height = imageNode.attribute(HEIGHT).as_int();
        if (height <= 0) {
            os() << "Error: image height of camera \"" << sensorName
                 << "\" is invalid." << endl;
            return false;
        }

        // 'format' attribute (required) is currently ignored
        // c.f. http://docs.ros.org/en/noetic/api/sensor_msgs/html/image__encodings_8h_source.html

        // 'hfov' attribute (required)
        const double hfov = imageNode.attribute(HFOV).as_double();
        if (hfov <= 0.0) {
            os() << "Error: the hfov of camera \"" << sensorName
                 << "\" is invalid." << endl;
            return false;
        }

        // 'near' attribute (required)
        const double near = imageNode.attribute(NEAR).as_double();
        if (near <= 0.0) {
            os() << "Error: near clip distance of camera \"" << sensorName
                 << "\" is invalid." << endl;
            return false;
        }

        // 'far' attribute (required)
        const double far = imageNode.attribute(FAR).as_double();
        if (far <= 0.0) {
            os() << "Error: far clip distance of camera \"" << sensorName
                 << "\" is invalid." << endl;
            return false;
        } else if (far < near) {
            os() << "Error: far clip distance must be larger than near one."
                 << endl;
        }

        if (!imageNode.attribute(DEPTH_FORMAT).empty()) {
            const std::string depthFormat = imageNode.attribute(DEPTH_FORMAT).as_string();
            if (depthFormat == MONO8 || depthFormat == MONO16) {
                // TODO: Support a depth-only camera

                // constructs a range camera
                RangeCameraPtr camera = new RangeCamera;
                camera->setName(sensorName);
                camera->setFrameRate(update_rate);
                camera->setLocalRotation(rotation);
                camera->setLocalTranslation(translation);
                camera->setResolution(width, height);
                camera->setHorizontalFieldOfView(hfov);
                camera->setNearClipDistance(near);
                camera->setFarClipDistance(far);
                camera->setOpticalFrame(VisionSensor::OpticalFrameType::CV);

                // set range camera parameters
                camera->setMinDistance(near);
                camera->setMaxDistance(far);
                camera->setImageType(Camera::COLOR_IMAGE);

                // registers the camera
                return body->addDevice(camera, parentLink);
            } else {
                os() << "Error: depth_format of camera \"" << sensorName
                     << "\" is invalid." << endl;
                return false;
            }
        } else {
            // constructs a camera
            CameraPtr camera = new Camera;
            camera->setName(sensorName);
            camera->setFrameRate(update_rate);
            camera->setLocalRotation(rotation);
            camera->setLocalTranslation(translation);
            camera->setResolution(width, height);
            camera->setHorizontalFieldOfView(hfov);
            camera->setNearClipDistance(near);
            camera->setFarClipDistance(far);
            camera->setOpticalFrame(VisionSensor::OpticalFrameType::CV);

            // registers the camera
            return body->addDevice(camera, parentLink);
        }
    } else if (!sensorNode.child(RAY).empty()) {
        RangeSensorPtr rangeSensor = new RangeSensor;
        rangeSensor->setName(sensorName);
        rangeSensor->setScanRate(update_rate);
        rangeSensor->setLocalRotation(rotation);
        rangeSensor->setLocalTranslation(translation);

        // 'horizontal' tag (optional)
        const xml_node& horizontalNode = sensorNode.child(RAY).child(HORIZONTAL);
        if (!horizontalNode.empty()) {
            // 'samples' attribute (optional, default: 1)
            const int horizontalSamples = horizontalNode.attribute(SAMPLES)
                                              .as_int(1);
            if (horizontalSamples < 1) {
                os() << "Error: the number of horizontal samples is invalid "
                        "(ray sensor \""
                     << sensorName << "\")." << endl;
                return false;
            }

            // 'resolution' attribute (optional, default: 1.0) is currently ignored

            // 'min_angle' attribute (optional, default: 0.0)
            const double horizontalMinAngle
                = horizontalNode.attribute(MIN_ANGLE).as_double(0.0);
            // 'max_angle' attribute (optional, default: 0.0)
            const double horizontalMaxAngle
                = horizontalNode.attribute(MAX_ANGLE).as_double(0.0);
            if (horizontalMinAngle > horizontalMaxAngle) {
                os() << "Error: horizontal max_angle < min_angle (ray sensor \""
                     << sensorName << "\")." << endl;
                return false;
            }

            rangeSensor->setYawRange(horizontalMaxAngle - horizontalMinAngle);
            rangeSensor->setYawStep(rangeSensor->yawRange()
                                    / static_cast<double>(horizontalSamples));
        }

        // 'vertical' tag (optional)
        const xml_node& verticalNode = sensorNode.child(RAY).child(VERTICAL);
        if (!verticalNode.empty()) {
            // 'samples' attribute (optional, default: 1)
            const int verticalSamples = verticalNode.attribute(SAMPLES).as_int(
                1);
            if (verticalSamples < 1) {
                os() << "Error: the number of vertical samples is invalid (ray "
                        "sensor \""
                     << sensorName << "\")." << endl;
                return false;
            }

            // 'resolution' attribute (optional, default: 1.0) is currently ignored

            // 'min_angle' attribute (optional, default: 0.0)
            const double verticalMinAngle = verticalNode.attribute(MIN_ANGLE)
                                                .as_double();
            // 'max_angle' attribute (optional, default: 0.0)
            const double verticalMaxAngle = verticalNode.attribute(MAX_ANGLE)
                                                .as_double();
            if (verticalMinAngle > verticalMaxAngle) {
                os() << "Error: vertical max_angle < min_angle (ray sensor \""
                     << sensorName << "\")." << endl;
                return false;
            }

            rangeSensor->setPitchRange(verticalMaxAngle - verticalMinAngle);
            rangeSensor->setPitchStep(rangeSensor->pitchRange()
                                      / static_cast<double>(verticalSamples));
        }

        // transforms the frame for the ROS-standard specification
        Matrix3 R;
        R << 1.0, 0.0, 0.0,
             0.0, 0.0, 1.0,
             0.0, -1.0, 0.0;
        rangeSensor->setOpticalFrameRotation(R);

        // registers the range sensor
        body->addDevice(rangeSensor, parentLink);
    }

    return true;
}
