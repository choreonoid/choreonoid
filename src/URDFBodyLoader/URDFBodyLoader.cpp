#include "URDFBodyLoader.h"
#include "URDFKeywords.h"

#include <memory>
#include <ostream>
#include <string>
#include <unordered_map>
#include <vector>

#include <stdio.h>

#include <boost/range/size.hpp>  // becomes STL from C++17

#include <cnoid/Body>
#include <cnoid/BodyLoader>
#include <cnoid/EigenUtil>
#include <cnoid/MeshGenerator>
#include <cnoid/NullOut>
#include <cnoid/SceneLoader>
#include <cnoid/UTF8>
#include <cnoid/stdx/filesystem>
#include <fmt/format.h>
#include <pugixml.hpp>

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


class ROSPackageSchemeHandler
{
    vector<string> packagePaths;

public:
    ROSPackageSchemeHandler()
    {
        const char* str = getenv("ROS_PACKAGE_PATH");
        if (str) {
            do {
                const char* begin = str;
                while (*str != ':' && *str)
                    str++;
                packagePaths.push_back(string(begin, str));
            } while (0 != *str++);
        }
    }

    string operator()(const string& path, std::ostream& os)
    {
        const string prefix = "package://";
        if (!(path.size() >= prefix.size()
              && std::equal(prefix.begin(), prefix.end(), path.begin()))) {
            return path;
        }

        filesystem::path filepath(fromUTF8(path.substr(prefix.size())));
        auto iter = filepath.begin();
        if (iter == filepath.end()) {
            return string();
        }

        filesystem::path directory = *iter++;
        filesystem::path relativePath;
        while (iter != filepath.end()) {
            relativePath /= *iter++;
        }

        bool found = false;
        filesystem::path combined;

        for (auto element : packagePaths) {
            filesystem::path packagePath(element);
            combined = packagePath / filepath;
            if (exists(combined)) {
                found = true;
                break;
            }
            combined = packagePath / relativePath;
            if (exists(combined)) {
                found = true;
                break;
            }
        }

        if (found) {
            return toUTF8(combined.string());
        } else {
            os << format("\"{}\" is not found in the ROS package directories.",
                         path)
               << endl;
            return string();
        }
    }
};

}  // namespace

namespace cnoid {

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


class URDFBodyLoader::Impl
{
public:
    std::ostream* os_;
    std::ostream& os() { return *os_; }

    Impl();
    bool load(Body* body, const string& filename);

private:
    int jointCounter_ = 0;
    SceneLoader sceneLoader_;
    ROSPackageSchemeHandler ROSPackageSchemeHandler_;
    std::unordered_map<string, Vector4> colorMap;

    void createColorMap(
        const pugi::xml_object_range<pugi::xml_named_node_iterator>&
            materialNodes);
    vector<LinkPtr> findRootLinks(
        const std::unordered_map<string, LinkPtr>& linkMap);
    bool loadLink(LinkPtr link, const xml_node& linkNode);
    bool loadInertialTag(LinkPtr& link, const xml_node& inertialNode);
    bool loadVisualTag(LinkPtr& link, const xml_node& visualNode);
    bool loadCollisionTag(LinkPtr& link, const xml_node& collisionNode);
    bool readOriginTag(const xml_node& originNode,
                       Vector3& translation,
                       Matrix3& rotation);
    void printReadingInertiaTagError(const string& attribute_name);
    bool readInertiaTag(const xml_node& inertiaNode, Matrix3& inertiaMatrix);
    bool readGeometryTag(const xml_node& geometryNode, SgNodePtr& mesh);
    void setMaterialToAllShapeNodes(SgNodePtr& node, SgMaterialPtr& material);
    bool loadJoint(std::unordered_map<string, LinkPtr>& linkMap,
                   const xml_node& jointNode);
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

    const string suffix = ".xacro";
    if (filename.size() >= suffix.size()
        && std::equal(suffix.begin(),
                      suffix.end(),
                      filename.end() - suffix.size())) {
        // parses and reads a xacro-formatted URDF
        char buffer[128];
        const string command = "xacro " + filename;
        std::string urdf_content;
        FILE* pipe = popen(command.data(), "r");
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

        const pugi::xml_parse_result result = doc.load_string(
            urdf_content.data());
        if (!result) {
            os() << "Error: parsing XML failed: " << result.description()
                 << endl;
            return false;
        }
    } else {
        // loads a URDF
        const pugi::xml_parse_result result = doc.load_file(filename.c_str());
        if (!result) {
            os() << "Error: parsing XML failed: " << result.description()
                 << endl;
        }
    }

    // checks if only one 'robot' tag exists in the URDF
    if (doc.child(ROBOT).empty()) {
        os() << "Error: 'robot' tag is not found.";
        return false;
    } else if (++doc.children(ROBOT).begin() != doc.children(ROBOT).end()) {
        os() << "Error: multiple 'robot' tags are found.";
        return false;
    }

    // gets the 'robot' tag
    const xml_node& robotNode = doc.child(ROBOT);
    const string robotName = robotNode.attribute(NAME).as_string();
    if (!robotName.empty()) {
        body->setName(robotName);
    }

    // creates a color dictionary before parsing the robot model
    auto materialNodes = robotNode.children(MATERIAL);
    createColorMap(materialNodes);

    // creates a link dictionary by loading all links for tree construction
    auto linkNodes = robotNode.children(LINK);
    std::unordered_map<string, LinkPtr> linkMap(boost::size(linkNodes));

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
    std::unordered_map<string, int> jointMap(boost::size(jointNodes));

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

    return true;
}


void URDFBodyLoader::Impl::createColorMap(
    const pugi::xml_object_range<pugi::xml_named_node_iterator>& materialNodes)
{
    colorMap.reserve(boost::size(materialNodes));

    for (xml_node& materialNode : materialNodes) {
        const string materialName = materialNode.attribute(NAME).as_string();
        const xml_node& colorNode = materialNode.child(COLOR);
        const xml_node& textureNode = materialNode.child(TEXTURE);

        if (materialName.empty()) {
            continue;
        } else if (!colorNode.empty()) {
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

    // 'visual' (optional, multiple definition are allowed)
    if (linkNode.child(VISUAL).empty()) {
        os() << "Debug: link \"" << name << "\" has no visual data." << endl;
    } else {
        const auto visualNodes = linkNode.children(VISUAL);
        for (xml_node& visualNode : visualNodes) {
            if (!loadVisualTag(link, visualNode)) {
                os() << "in link \"" << name << "\"." << endl;
                return false;
            }
        }
    }

    // 'collision' (optional, multiple definition are allowed)
    if (linkNode.child(COLLISION).empty()) {
        os() << "Debug: link \"" << name << "\" has no collision data." << endl;
    } else {
        const auto collisionNodes = linkNode.children(COLLISION);
        for (xml_node& collisionNode : collisionNodes) {
            if (!loadCollisionTag(link, collisionNode)) {
                os() << " in link \"" << name << "\"." << endl;
                return false;
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


bool URDFBodyLoader::Impl::loadVisualTag(LinkPtr& link,
                                         const xml_node& visualNode)
{
    // 'origin' tag (optional)
    const xml_node& originNode = visualNode.child(ORIGIN);
    Vector3 translation = Vector3::Zero();
    Matrix3 rotation = Matrix3::Identity();
    if (!readOriginTag(originNode, translation, rotation)) {
        os() << " in a visual tag";
        return false;
    }
    Isometry3 originalPose;
    originalPose.linear() = rotation;
    originalPose.translation() = translation;

    // 'geometry' tag (required)
    const xml_node& geometryNode = visualNode.child(GEOMETRY);
    if (geometryNode.empty()) {
        os() << "Error: visual geometry is not found";
        return false;
    }

    SgNodePtr mesh = new SgNode;
    if (!readGeometryTag(geometryNode, mesh)) {
        os() << "Error: loading visual geometry failed";
    }

    // 'material' tag (optional)
    const xml_node& materialNode = visualNode.child(MATERIAL);
    SgMaterialPtr material = new SgMaterial;
    if (!materialNode.empty()) {
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

                material->setTransparency(1.0 - rgba[3]);
                material->setDiffuseColor(Vector3(rgba[0], rgba[1], rgba[2]));
                setMaterialToAllShapeNodes(mesh, material);
            }
        } else if (!textureNode.empty()) {
            // case: a 'texture' tag exists
            os() << "Warning: 'texture' tag is currently not supported."
                 << endl;
        } else if (!materialName.empty()) {
            // case: neither a 'color' tag nor 'texture' tag does not exist
            //       but a material name is specified
            try {
                const Vector4 rgba = colorMap.at(materialName);
                material->setTransparency(1.0 - rgba[3]);
                material->setDiffuseColor(Vector3(rgba[0], rgba[1], rgba[2]));
                setMaterialToAllShapeNodes(mesh, material);
            } catch (std::out_of_range&) {
                os() << "Warning: a material named \"" << materialName
                     << "\" is not found." << endl;
            }
        }
    }

    SgPosTransformPtr transformation = new SgPosTransform(originalPose);
    transformation->addChild(mesh);
    link->addVisualShapeNode(transformation);

    return true;
}


bool URDFBodyLoader::Impl::loadCollisionTag(LinkPtr& link,
                                            const xml_node& collisionNode)
{
    // 'origin' tag (optional)
    const xml_node& originNode = collisionNode.child(ORIGIN);
    Vector3 translation = Vector3::Zero();
    Matrix3 rotation = Matrix3::Identity();
    if (!readOriginTag(originNode, translation, rotation)) {
        os() << " in a collision tag";
        return false;
    }
    Isometry3 originalPose;
    originalPose.linear() = rotation;
    originalPose.translation() = translation;

    // 'geometry' tag (required)
    const xml_node& geometryNode = collisionNode.child(GEOMETRY);
    if (geometryNode.empty()) {
        os() << "Error: collision geometry is not found";
        return false;
    }

    SgNodePtr mesh = new SgNode;
    if (!readGeometryTag(geometryNode, mesh)) {
        os() << "Error: loading collision geometry failed";
    }

    SgPosTransformPtr transformation = new SgPosTransform(originalPose);
    transformation->addChild(mesh);
    link->addCollisionShapeNode(transformation);

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


bool URDFBodyLoader::Impl::readGeometryTag(const xml_node& geometryNode,
                                           SgNodePtr& mesh)
{
    if (boost::size(geometryNode.children()) < 1) {
        os() << "Error: no geometry is found." << endl;
    } else if (boost::size(geometryNode.children()) > 1) {
        os() << "Error: one link can have only one geometry." << endl;
    }

    MeshGenerator meshGenerator;
    SgShapePtr shape = new SgShape;

    if (!geometryNode.child(BOX).empty()) {
        Vector3 size = Vector3::Zero();
        if (!toVector3(geometryNode.child(BOX).attribute(SIZE).as_string(),
                       size)) {
            os() << "Error: box size is written in invalid format." << endl;
        }

        shape->setMesh(meshGenerator.generateBox(size));
        mesh = shape;
    } else if (!geometryNode.child(CYLINDER).empty()) {
        if (geometryNode.child(CYLINDER).attribute(RADIUS).empty()) {
            os() << "Error: cylinder radius is not defined." << endl;
            return false;
        }
        if (geometryNode.child(CYLINDER).attribute(LENGTH).empty()) {
            os() << "Error: cylinder length is not defined." << endl;
            return false;
        }
        const double radius
            = geometryNode.child(CYLINDER).attribute(RADIUS).as_double();
        const double length
            = geometryNode.child(CYLINDER).attribute(LENGTH).as_double();

        shape->setMesh(meshGenerator.generateCylinder(radius, length));

        SgPosTransformPtr transformation = new SgPosTransform;
        transformation->setRotation(rotFromRpy(Vector3(M_PI / 2.0, 0.0, 0.0)));
        transformation->translation().setZero();
        transformation->addChild(shape);
        mesh = transformation;
    } else if (!geometryNode.child(SPHERE).empty()) {
        if (geometryNode.child(SPHERE).attribute(RADIUS).empty()) {
            os() << "Error: sphere radius is not defined." << endl;
            return false;
        }
        const double radius
            = geometryNode.child(SPHERE).attribute(RADIUS).as_double();

        shape->setMesh(meshGenerator.generateSphere(radius));
        mesh = shape;
    } else if (!geometryNode.child(MESH).empty()) {
        if (geometryNode.child(MESH).attribute(FILENAME).empty()) {
            os() << "Error: mesh file is not specified." << endl;
            return false;
        }

        // loads a mesh file
        const string filename
            = geometryNode.child(MESH).attribute(FILENAME).as_string();
        bool isSupportedFormat = false;
        mesh = dynamic_cast<SgNode*>(
            sceneLoader_.load(ROSPackageSchemeHandler_(filename, os()),
                              isSupportedFormat));
        if (!isSupportedFormat) {
            os() << "Error: format of the specified mesh file \"" << filename
                 << "\" is not supported." << endl;
            return false;
        }

        // scales the mesh
        if (!geometryNode.child(MESH).attribute(SCALE).empty()) {
            Vector3 scale = Vector3::Ones();
            if (!toVector3(geometryNode.child(MESH).attribute(SCALE).as_string(),
                           scale)) {
                os() << "Error: mesh scale is written in invalid format."
                     << endl;

                return false;
            }

            SgScaleTransformPtr scaler = new SgScaleTransform;
            scaler->setScale(scale);
            scaler->addChild(mesh);
            mesh = scaler;
        }
    } else {
        os() << "Error: unsupported geometry \""
             << geometryNode.first_child().name() << "\" is described." << endl;
        return false;
    }

    return true;
}


void URDFBodyLoader::Impl::setMaterialToAllShapeNodes(SgNodePtr& node,
                                                      SgMaterialPtr& material)
{
    if (auto group = node->toGroupNode()) {
        for (auto& child : *group) {
            setMaterialToAllShapeNodes(child, material);
        }
    } else if (auto shape = dynamic_cast<SgShape*>(node.get())) {
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

            // if (limitNode.attribute(EFFORT).empty()) {
            //     os() << "Error: effort limit of joint \"" << jointName
            //          << "\" is not defined." << endl;
            //     return false;
            // }
            // const double effortLimit = limitNode.attribute(EFFORT)
            //                                  .as_double();
            // if (effortLimit < 0.0) {
            //     os() << "Error: effort limit of joint \"" << jointName
            //          << "\" has to be positive." << endl;
            //     return false;
            // }
            // child->setJointEffortRange(-effortLimit, effortLimit);
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
