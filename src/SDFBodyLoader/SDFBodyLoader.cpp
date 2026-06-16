#include "SDFBodyLoader.h"
#include <cnoid/AccelerationSensor>
#include <cnoid/Body>
#include <cnoid/BodyLoader>
#include <cnoid/Camera>
#include <cnoid/EigenUtil>
#include <cnoid/ForceSensor>
#include <cnoid/Imu>
#include <cnoid/MeshGenerator>
#include <cnoid/NullOut>
#include <cnoid/RangeCamera>
#include <cnoid/RangeSensor>
#include <cnoid/RateGyroSensor>
#include <cnoid/SceneLoader>
#include <cnoid/UriSchemeProcessor>
#include <cnoid/VisionSensor>
#include <cnoid/Format>
#include <cnoid/UTF8>
#include <pugixml.hpp>
#include <filesystem>
#include <memory>
#include <ostream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <limits>
#include <vector>
#include <cstdlib>
#include "gettext.h"

using namespace cnoid;

namespace filesystem = std::filesystem;
using pugi::xml_attribute;
using pugi::xml_node;
using std::endl;
using std::string;
using std::vector;

namespace {

// Resolves the "model://name/sub/path" URI to an actual file path. The first path component
// is the model name, which is looked up as a directory under each search path. The search
// paths are taken from the invoking UriSchemeProcessor's modelSearchDirectories(), which the
// caller (here SDFBodyLoader) populates with the parent of the model currently being loaded
// and any directories from the Gazebo/Ignition environment variables.
class ModelSchemeHandler
{
public:
    string operator()(const string& path, UriSchemeProcessor& processor)
    {
        const filesystem::path uriPath(fromUTF8(path));

        for(const auto& dir : processor.modelSearchDirectories()){
            filesystem::path combined = filesystem::path(fromUTF8(dir)) / uriPath;
            if(filesystem::exists(combined)){
                // Returns an absolute path so that the base directory is not prepended again
                // by the URI scheme processor.
                return toUTF8(filesystem::absolute(combined).lexically_normal().string());
            }
        }

        processor.setErrorMessage(
            formatR(_("\"{0}\" is not found in the model search paths. Set GAZEBO_MODEL_PATH "
                      "(or GZ_SIM_RESOURCE_PATH) if the referenced model is in another "
                      "directory."), path));
        return string();
    }
};

struct ModelSchemeHandlerRegistration
{
    ModelSchemeHandlerRegistration()
    {
        UriSchemeProcessor::registerUriSchemeHandler("model", ModelSchemeHandler());
    }
} modelSchemeHandlerRegistration;


// This global object registers the SDF loader to the BodyLoader when this library is
// loaded, so that a body file with the ".sdf" extension can be read via the BodyLoader
// without referring to SDFBodyLoader directly.
//
// Note: when this library is used independently of the Choreonoid GUI (i.e. it is linked
// directly into an executable rather than being loaded as a plugin via dlopen), the linker
// must be told to keep this library even though none of its symbols are referenced by name.
// With the GNU linker the default "--as-needed" behavior drops this library, which prevents
// the registration below from running. Link with "--no-as-needed" for this library (e.g.
// "-Wl,--no-as-needed -lCnoidSDFBodyLoader -Wl,--as-needed") to ensure the loader is
// registered. This is not necessary when the library is loaded by the Choreonoid plugin
// mechanism, because dlopen loads it regardless of symbol references.
struct Registration
{
    Registration()
    {
        BodyLoader::registerLoader({"sdf", "world"},
                                   []() -> AbstractBodyLoaderPtr {
                                       return std::make_shared<SDFBodyLoader>();
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

    Isometry3 pose;
    enum ShapeType { None, Mesh, Box, Cylinder, Sphere, Polyline } shapeType;
    string meshUri;
    // SDF <mesh><submesh><name>: when non-empty, only the named subtree of the mesh file is
    // used and the rest is discarded.
    string submeshName;
    // SDF <mesh><submesh><center>: when true, the extracted submesh is recentered so that its
    // bounding box center sits at the origin.
    bool submeshCenter;
    Vector3 meshScale;
    Vector3 boxSize;
    double radius;
    double length;
    // SDF <polyline>: 2D vertices of the polyline cross section in the model's X-Y plane.
    MeshGenerator::Vector2Array polylinePoints;
    SgMaterialPtr material;

    ShapeDescription()
        : pose(Isometry3::Identity()),
          shapeType(None),
          submeshCenter(false),
          meshScale(Vector3::Ones()),
          boxSize(Vector3::Zero()),
          radius(0.0),
          length(0.0)
    { }
};

// Holds the parsed data of a link before tree construction.
struct LinkInfo
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    LinkPtr link;
    // Pose of the link frame expressed in the model frame.
    Isometry3 modelPose;
    // Name of the frame the pose is relative to ("" means the model frame).
    string poseRelativeTo;
    bool poseResolved;

    LinkInfo()
        : modelPose(Isometry3::Identity()),
          poseResolved(false)
    { }
};

// An <include>ed sub-model contributes its links and joints to the surrounding model with
// their names prefixed by "<include name>::" (where <include name> defaults to the included
// model's name attribute). These structures hold the source XML node together with the
// prefix and the included sub-model's pose so the main load loop can process them uniformly.
struct CollectedLink
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    xml_node node;
    string namePrefix;            // "" for links in the primary model
    Isometry3 includePose;        // identity for links in the primary model
    string baseDirectoryForMeshes; // base directory used to resolve "model://" inside this scope

    CollectedLink() : includePose(Isometry3::Identity()) {}
};

struct CollectedJoint
{
    xml_node node;
    string namePrefix;            // "" for joints in the primary model
};

// A joint after its names have been resolved (with sub-model prefixes applied), ready to be
// used for parent selection and tree building.
struct ResolvedJoint
{
    xml_node node;
    string jointName;
    string parentName;            // "world" for a fixed-to-world joint
    string childName;
};

} // namespace

namespace cnoid {

class SDFBodyLoader::Impl
{
public:
    int jointCounter_;
    SceneLoader sceneLoader;
    UriSchemeProcessor uriSchemeProcessor;
    MeshGenerator meshGenerator;
    std::unordered_map<string, SgNodePtr> meshMap;
    // XML documents created for <include>ed sub-models. They are kept alive for the duration
    // of load() because the collected xml_nodes reference into them.
    vector<std::shared_ptr<pugi::xml_document>> includedDocs;
    // Material features encountered but not (yet) supported by this loader. Tracked so that
    // the same feature is only reported once per load() rather than once per use site.
    std::unordered_set<string> warnedUnsupportedMaterialFeatures;
    // SDF specification version of the file being loaded, parsed from the 'version' attribute
    // of the <sdf> root element. The defaults below are used when no version is present (e.g.
    // a legacy <gazebo> root) and are deliberately conservative: 1.4 means the joint axis
    // defaults to the joint frame and <use_parent_model_frame> is recognized.
    int sdfMajor_ = 1;
    int sdfMinor_ = 4;
    std::ostream* os_;
    std::ostream& os() { return *os_; }

    Impl();
    bool load(Body* body, const string& filename);

    void setupModelSearchDirectories(const string& filename, string& out_primaryBaseDir);

    bool readPose(const xml_node& poseNode, Isometry3& out_pose);
    bool readPoseElement(const xml_node& parentNode, Isometry3& out_pose, string& out_relativeTo);
    string resolveIncludeUri(const string& uri, const string& currentBaseDir);
    bool expandIncludes(
        const xml_node& modelNode, const string& namePrefix, const Isometry3& parentIncludePose,
        const string& currentBaseDir,
        vector<CollectedLink>& outLinks, vector<CollectedJoint>& outJoints);
    bool loadLink(Body* body, LinkInfo& info, const xml_node& linkNode,
                  const string& namePrefix, const Isometry3& includePose,
                  const string& baseDirectoryForMeshes);
    void loadSensor(Body* body, LinkPtr link, const xml_node& sensorNode, const string& namePrefix);
    DevicePtr createCameraDevice(const xml_node& sensorNode, const string& sensorType,
                                 const xml_node& cameraNode);
    DevicePtr createRangeSensorDevice(const xml_node& sensorNode, const xml_node& rayNode);
    void readCommonSensorAttributes(Device* device, const xml_node& sensorNode);
    bool loadInertial(LinkPtr link, const xml_node& inertialNode);
    bool readInertiaElement(const xml_node& inertiaNode, Matrix3& inertiaMatrix);
    bool readShapeDescription(const xml_node& shapeNode, ShapeDescription& description);
    bool readGeometry(const xml_node& geometryNode, ShapeDescription& description);
    SgNode* createShape(ShapeDescription& description);
    SgNode* createMesh(ShapeDescription& description);
    // Recursively searches 'node' for a descendant whose SgNode::name() equals targetName.
    // On a match, sets out_node and out_transform (the accumulated transform from 'node' to
    // the matched descendant) and returns true. The first match in pre-order wins; the search
    // does not descend below a matched node.
    bool findNamedSubNode(SgNode* node, const string& targetName, const Affine3& parentTransform,
                         SgNode*& out_node, Affine3& out_transform);
    SgMaterial* readMaterial(const xml_node& materialNode);
    void setMaterialToAllShapeNodes(SgNode* node, SgMaterial* material);
    size_t selectParentJoint(const vector<ResolvedJoint>& candidates);
    bool buildJoint(
        std::unordered_map<string, LinkInfo>& linkInfoMap, const ResolvedJoint& joint);
    int countDescendants(Link* link);
    void attachStrayRoot(LinkInfo* strayInfo, Link* newParent, const string& jointName,
                        bool isReversed);
};

}  // namespace cnoid


SDFBodyLoader::SDFBodyLoader()
{
    impl = new Impl;
}


SDFBodyLoader::Impl::Impl()
    : jointCounter_(0),
      os_(&nullout())
{

}


SDFBodyLoader::~SDFBodyLoader()
{
    delete impl;
}


void SDFBodyLoader::setMessageSink(std::ostream& os)
{
    impl->os_ = &os;
    impl->sceneLoader.setMessageSink(os);
}


void SDFBodyLoader::setDefaultDivisionNumber(int n)
{
    impl->sceneLoader.setDefaultDivisionNumber(n);
}


void SDFBodyLoader::setDefaultCreaseAngle(double theta)
{
    impl->sceneLoader.setDefaultCreaseAngle(theta);
}


bool SDFBodyLoader::load(Body* body, const std::string& filename)
{
    return impl->load(body, filename);
}


bool SDFBodyLoader::Impl::load(Body* body, const string& filename)
{
    pugi::xml_document doc;

    jointCounter_ = 0;
    meshMap.clear();
    warnedUnsupportedMaterialFeatures.clear();

    // The SceneLoader may carry over image search directories registered during a previous
    // SDF load. Make sure each load starts with a clean slate, and ensure the slate stays
    // clean on exit so that nothing leaks to whoever uses the same SceneLoader instance next.
    sceneLoader.clearImageSearchDirectories();
    struct ImageSearchDirCleanup {
        SceneLoader& sceneLoader;
        ~ImageSearchDirCleanup() { sceneLoader.clearImageSearchDirectories(); }
    } imageSearchDirCleanup{sceneLoader};

    pugi::xml_parse_result result = doc.load_file(filename.c_str());
    if (!result) {
        os() << formatR(_("Error: parsing the SDF file failed: {0}"), result.description()) << endl;
        return false;
    }

    // The root element is 'sdf' in SDF 1.3 and later; SDF 1.2 used 'gazebo' instead. Both are
    // accepted so that legacy model-1_2.sdf files can still be loaded.
    xml_node sdfNode = doc.child("sdf");
    if (sdfNode.empty()) {
        sdfNode = doc.child("gazebo");
    }
    if (sdfNode.empty()) {
        os() << _("Error: the 'sdf' (or 'gazebo') root element is not found.") << endl;
        return false;
    }

    // Parses the SDF version from the root element. It governs the default frame in which
    // <axis><xyz> is expressed: SDF 1.6 and earlier default to the joint frame and recognize
    // <use_parent_model_frame>; SDF 1.7 and later default to the child link frame and accept
    // the 'expressed_in' attribute instead.
    sdfMajor_ = 1;
    sdfMinor_ = 4;
    const string versionStr = sdfNode.attribute("version").as_string();
    if (!versionStr.empty()) {
        int maj = 0, min = 0;
        if (sscanf(versionStr.c_str(), "%d.%d", &maj, &min) >= 1) {
            sdfMajor_ = maj;
            sdfMinor_ = min;
        }
    }

    // Finds the 'model' element. It may be directly under 'sdf' or under a 'world'.
    xml_node modelNode = sdfNode.child("model");
    if (modelNode.empty()) {
        const xml_node worldNode = sdfNode.child("world");
        if (!worldNode.empty()) {
            modelNode = worldNode.child("model");
        }
    }
    if (modelNode.empty()) {
        os() << _("Error: no 'model' element is found in the SDF file.") << endl;
        return false;
    }
    // Only the first model is loaded.
    if (++modelNode.parent().children("model").begin() != modelNode.parent().children("model").end()) {
        os() << _("Warning: multiple models are found; only the first one is loaded.") << endl;
    }

    const string modelName = modelNode.attribute("name").as_string();
    if (!modelName.empty()) {
        body->setName(modelName);
    }

    // Ensures the per-load model search paths are cleared whether load() returns normally or
    // bails out partway, so a reused SDFBodyLoader instance starts clean on the next call.
    struct UriSchemeProcessorCleanup {
        UriSchemeProcessor& processor;
        ~UriSchemeProcessorCleanup() { processor.clearModelSearchDirectories(); }
    } uspCleanup{uriSchemeProcessor};

    string primaryBaseDir;
    setupModelSearchDirectories(filename, primaryBaseDir);

    // Collects links and joints, expanding any <include> recursively so that sub-models are
    // merged into the same link/joint set with their names prefixed.
    vector<CollectedLink> collectedLinks;
    vector<CollectedJoint> collectedJoints;
    for (xml_node linkNode : modelNode.children("link")) {
        CollectedLink cl;
        cl.node = linkNode;
        cl.baseDirectoryForMeshes = primaryBaseDir;
        collectedLinks.push_back(cl);
    }
    for (xml_node jointNode : modelNode.children("joint")) {
        CollectedJoint cj;
        cj.node = jointNode;
        collectedJoints.push_back(cj);
    }
    if (!expandIncludes(modelNode, "", Isometry3::Identity(), primaryBaseDir,
                        collectedLinks, collectedJoints)) {
        return false;
    }

    // Loads all collected links into linkInfoMap, applying the name prefix and include pose
    // of the originating sub-model (identity for the primary model).
    std::unordered_map<string, LinkInfo> linkInfoMap(collectedLinks.size());
    for (const CollectedLink& cl : collectedLinks) {
        LinkInfo info;
        info.link = new Link;
        if (!loadLink(body, info, cl.node, cl.namePrefix, cl.includePose, cl.baseDirectoryForMeshes)) {
            return false;
        }
        const auto inserted = linkInfoMap.emplace(info.link->name(), info);
        if (!inserted.second) {
            os() << formatR(_("Error: multiple links named \"{0}\" are found."), info.link->name())
                 << endl;
            return false;
        }
    }

    if (linkInfoMap.empty()) {
        os() << _("Error: the model has no link.") << endl;
        return false;
    }

    // Collects all joints first so that the parent of each link can be selected before the
    // link tree is built. A link may be referenced as the child of more than one joint when the
    // model contains a closed loop, which a tree-structured Body cannot represent. In that case
    // a single parent joint is chosen (see selectParentJoint) and the others are dropped.
    std::unordered_map<string, int> jointNameMap(collectedJoints.size());
    // Candidate parent joints for each child link, kept in the order they appear in the SDF.
    std::unordered_map<string, vector<ResolvedJoint>> jointsByChild;
    vector<string> childOrder; // child link names in first-seen order, for deterministic output

    // Helper to apply a sub-model's name prefix to a link reference. A name like "create::base"
    // already includes a prefix and is passed through; an unprefixed name from inside an
    // included sub-model gets the sub-model's prefix prepended.
    auto applyPrefix = [](const string& namePrefix, const string& name) -> string {
        if (namePrefix.empty() || name.find("::") != string::npos) {
            return name;
        }
        return namePrefix + "::" + name;
    };

    for (const CollectedJoint& cj : collectedJoints) {
        const xml_node& jointNode = cj.node;
        const string rawJointName = jointNode.attribute("name").as_string();
        const string jointName = applyPrefix(cj.namePrefix, rawJointName);
        const auto inserted = jointNameMap.emplace(jointName, 0);
        if (!inserted.second) {
            os() << formatR(_("Error: multiple joints named \"{0}\" are found."), jointName) << endl;
            return false;
        }

        if (jointNode.attribute("type").empty()) {
            os() << formatR(_("Error: the type of the joint \"{0}\" is not defined."), jointName)
                 << endl;
            return false;
        }
        const xml_node parentNode = jointNode.child("parent");
        const xml_node childNode = jointNode.child("child");
        if (parentNode.empty() || childNode.empty()) {
            os() << formatR(_("Error: the joint \"{0}\" must have a parent and a child."), jointName)
                 << endl;
            return false;
        }
        const string rawParentName = parentNode.text().as_string();
        const string rawChildName = childNode.text().as_string();
        const string parentName =
            (rawParentName == "world") ? rawParentName : applyPrefix(cj.namePrefix, rawParentName);
        const string childName = applyPrefix(cj.namePrefix, rawChildName);

        // A joint whose parent is the special "world" link fixes its child to the world.
        if (parentName == "world") {
            auto childIt = linkInfoMap.find(childName);
            if (childIt == linkInfoMap.end()) {
                os() << formatR(_("Error: the child \"{0}\" of the joint \"{1}\" is invalid."),
                                childName, jointName) << endl;
                return false;
            }
            const LinkPtr child = childIt->second.link;
            child->setJointName(jointName);
            child->setJointType(Link::FixedJoint);
            continue;
        }

        if (linkInfoMap.find(parentName) == linkInfoMap.end()) {
            os() << formatR(_("Error: the parent \"{0}\" of the joint \"{1}\" is invalid."),
                            parentName, jointName) << endl;
            return false;
        }
        if (linkInfoMap.find(childName) == linkInfoMap.end()) {
            os() << formatR(_("Error: the child \"{0}\" of the joint \"{1}\" is invalid."),
                            childName, jointName) << endl;
            return false;
        }

        if (jointsByChild.find(childName) == jointsByChild.end()) {
            childOrder.push_back(childName);
        }
        ResolvedJoint rj;
        rj.node = jointNode;
        rj.jointName = jointName;
        rj.parentName = parentName;
        rj.childName = childName;
        jointsByChild[childName].push_back(rj);
    }

    // For each child link, selects one parent joint and builds the corresponding tree edge.
    // The joints that are not selected are kept in droppedJoints; they may be used later to
    // reconnect links that would otherwise be left as stray roots after the loop is broken.
    vector<ResolvedJoint> droppedJoints;
    for (const string& childName : childOrder) {
        const vector<ResolvedJoint>& candidates = jointsByChild[childName];
        size_t selected = selectParentJoint(candidates);
        if (candidates.size() > 1) {
            os() << formatR(_("Warning: the link \"{0}\" has multiple parent joints, which forms "
                              "a closed loop. Only the joint \"{1}\" is used and the others are "
                              "dropped to keep the model tree-structured:"),
                            childName,
                            candidates[selected].jointName);
            for (size_t i = 0; i < candidates.size(); ++i) {
                if (i != selected) {
                    os() << " \"" << candidates[i].jointName << "\"";
                    droppedJoints.push_back(candidates[i]);
                }
            }
            os() << "." << endl;
        }
        if (!buildJoint(linkInfoMap, candidates[selected])) {
            return false;
        }
    }

    // Collects all links with no parent. A well-formed model has exactly one such link, but
    // breaking a closed loop above can leave additional stray roots that were originally
    // reachable only through a dropped joint.
    vector<LinkInfo*> rootInfos;
    for (auto& kv : linkInfoMap) {
        if (kv.second.link->parent() == nullptr) {
            rootInfos.push_back(&kv.second);
        }
    }
    if (rootInfos.empty()) {
        os() << _("Error: no root link is found.") << endl;
        return false;
    }

    // Selects the root with the largest subtree as the main root. For typical robots this is
    // the true base (e.g. base_footprint), because most links descend from it.
    LinkInfo* rootInfo = rootInfos.front();
    if (rootInfos.size() > 1) {
        int bestCount = -1;
        for (LinkInfo* r : rootInfos) {
            int c = countDescendants(r->link);
            if (c > bestCount) {
                bestCount = c;
                rootInfo = r;
            }
        }
        std::unordered_set<Link*> mainTree;
        {
            vector<Link*> stack{rootInfo->link};
            while (!stack.empty()) {
                Link* l = stack.back(); stack.pop_back();
                mainTree.insert(l);
                for (Link* c = l->child(); c; c = c->sibling()) stack.push_back(c);
            }
        }

        // Attempts to attach each stray root to the main tree. Preferred is reversing a dropped
        // joint that has the stray as its parent and a main-tree link as its child; the stray
        // then becomes a child of that main-tree link via a fixed joint. If no such joint is
        // available, the stray is attached directly to the main root as a fallback.
        for (LinkInfo* stray : rootInfos) {
            if (stray == rootInfo) continue;
            bool attached = false;
            for (const ResolvedJoint& dj : droppedJoints) {
                if (dj.parentName != stray->link->name()) continue;
                auto cIt = linkInfoMap.find(dj.childName);
                if (cIt == linkInfoMap.end()) continue;
                if (mainTree.count(cIt->second.link) == 0) continue;
                attachStrayRoot(stray, cIt->second.link, dj.jointName, true);
                mainTree.insert(stray->link);
                attached = true;
                break;
            }
            if (!attached) {
                attachStrayRoot(stray, rootInfo->link, string(), false);
                mainTree.insert(stray->link);
            }
        }
    }

    // In SDF, each link pose is expressed in the model frame by default. Choreonoid links hold
    // the offset position relative to the parent link, so the relative offsets are computed by
    // traversing the tree from the root.
    rootInfo->link->setOffsetPosition(rootInfo->modelPose);

    // Traverses the tree to set the offset position of each non-root link.
    vector<Link*> stack;
    stack.push_back(rootInfo->link);
    while (!stack.empty()) {
        Link* parent = stack.back();
        stack.pop_back();
        const Isometry3& parentModelPose = linkInfoMap[parent->name()].modelPose;
        for (Link* child = parent->child(); child; child = child->sibling()) {
            const Isometry3& childModelPose = linkInfoMap[child->name()].modelPose;
            // T_parent_child = T_model_parent^-1 * T_model_child
            child->setOffsetPosition(parentModelPose.inverse(Eigen::Isometry) * childModelPose);
            stack.push_back(child);
        }
    }

    body->setRootLink(rootInfo->link);

    // A static model is fixed to the world.
    const bool isStatic = modelNode.child("static").text().as_bool(false);
    body->rootLink()->setJointType(isStatic ? Link::FixedJoint : Link::FreeJoint);

    body->updateLinkTree();

    return true;
}


// Configures the URI scheme processor and the SceneLoader's image search list with the
// directories that should be consulted while loading 'filename': the model's parent directory
// (so "model://<model-dir-name>/..." self-references resolve locally), the model's textures
// subdirectory, and any directories listed in the Gazebo / Ignition resource-path environment
// variables. Returns the parent directory through out_primaryBaseDir for use as the base of
// <include>d sub-models. Order of registration matters: the model being loaded is tried
// before the environment-variable directories, so self-referencing URIs resolve to the local
// copy.
void SDFBodyLoader::Impl::setupModelSearchDirectories(
    const string& filename, string& out_primaryBaseDir)
{
    uriSchemeProcessor.setBaseDirectoryFor(filename);
    uriSchemeProcessor.clearModelSearchDirectories();

    // A Gazebo model commonly refers to its own meshes as "model://<model-dir-name>/...", so
    // the parent directory of the directory containing this SDF file is added as a search
    // path. This lets a self-contained model be loaded without setting GAZEBO_MODEL_PATH.
    filesystem::path filePath(fromUTF8(filename));
    filesystem::path modelDir = filePath.parent_path();
    out_primaryBaseDir = toUTF8(modelDir.parent_path().string());
    uriSchemeProcessor.addModelSearchDirectory(out_primaryBaseDir);

    // Gazebo SDF models typically keep textures under "<model>/materials/textures/" while
    // mesh files live under "<model>/meshes/", so a relative texture path inside a mesh file
    // will not resolve against the mesh's own directory. Register the textures directory as
    // a fallback for texture image lookup; the SceneLoader propagates it to every concrete
    // mesh loader (current and future) automatically.
    sceneLoader.addImageSearchDirectory(
        toUTF8((modelDir / "materials" / "textures").string()));

    // Adds search paths from the Gazebo / Ignition environment variables, if any are set.
    for (const char* var : {"GAZEBO_MODEL_PATH", "GZ_SIM_RESOURCE_PATH",
                            "IGN_GAZEBO_RESOURCE_PATH"}) {
        const char* value = getenv(var);
        if (!value) continue;
        const char* p = value;
        do {
            const char* begin = p;
            while (*p != ':' && *p) p++;
            string element(begin, p);
            if (!element.empty()) {
                uriSchemeProcessor.addModelSearchDirectory(element);
            }
        } while (0 != *p++);
    }
}


// Reads a 6-value pose string "x y z roll pitch yaw" into an Isometry3.
bool SDFBodyLoader::Impl::readPose(const xml_node& poseNode, Isometry3& out_pose)
{
    out_pose.setIdentity();

    const string s = poseNode.text().as_string();
    if (s.empty()) {
        return true;
    }

    const char* nptr = s.c_str();
    char* endptr;
    double values[6];
    for (int i = 0; i < 6; ++i) {
        values[i] = strtod(nptr, &endptr);
        if (endptr == nptr) {
            os() << _("Error: a pose must have six values (x y z roll pitch yaw).") << endl;
            return false;
        }
        nptr = endptr;
    }

    out_pose.translation() = Vector3(values[0], values[1], values[2]);
    out_pose.linear() = rotFromRpy(values[3], values[4], values[5]);
    return true;
}


// Reads the child 'pose' element of a parent node, with its 'relative_to' attribute.
bool SDFBodyLoader::Impl::readPoseElement(
    const xml_node& parentNode, Isometry3& out_pose, string& out_relativeTo)
{
    out_pose.setIdentity();
    out_relativeTo.clear();

    const xml_node poseNode = parentNode.child("pose");
    if (poseNode.empty()) {
        return true;
    }
    out_relativeTo = poseNode.attribute("relative_to").as_string();
    return readPose(poseNode, out_pose);
}


// Resolves an <include><uri>...</uri></include> value to the path of a model.sdf file.
// Supports "model://<name>[/sub/path]" and plain file paths (absolute or relative to the
// current sub-model's directory). When the uri identifies a model directory rather than a
// concrete .sdf file, "model.sdf" is appended.
string SDFBodyLoader::Impl::resolveIncludeUri(const string& uri, const string& currentBaseDir)
{
    string resolved;
    if (uri.compare(0, 8, "model://") == 0) {
        // Make sure the current sub-model's directory is among the processor's search paths
        // so the "model://" handler can resolve self-references inside the sub-model. The
        // entry persists for the rest of this load(); it will be cleared on the next load().
        if (!currentBaseDir.empty()) {
            uriSchemeProcessor.addModelSearchDirectory(currentBaseDir);
        }
        resolved = uriSchemeProcessor.getFilePath(uri);
    } else {
        filesystem::path p(fromUTF8(uri));
        if (!p.is_absolute() && !currentBaseDir.empty()) {
            p = filesystem::path(fromUTF8(currentBaseDir)) / p;
        }
        resolved = toUTF8(p.lexically_normal().string());
    }
    if (resolved.empty()) {
        return resolved;
    }
    // If the resolved path is a directory (typical for "model://name"), append model.sdf.
    std::error_code ec;
    if (filesystem::is_directory(filesystem::path(fromUTF8(resolved)), ec)) {
        filesystem::path candidate = filesystem::path(fromUTF8(resolved)) / "model.sdf";
        resolved = toUTF8(candidate.string());
    }
    return resolved;
}


// Walks the <include> children of the given model node, parses each referenced sub-model SDF,
// and appends its links and joints to outLinks/outJoints with a name prefix and pose offset
// applied. <include> elements are processed recursively, so a sub-model that itself uses
// <include> is also expanded.
bool SDFBodyLoader::Impl::expandIncludes(
    const xml_node& modelNode, const string& namePrefix, const Isometry3& parentIncludePose,
    const string& currentBaseDir,
    vector<CollectedLink>& outLinks, vector<CollectedJoint>& outJoints)
{
    for (xml_node includeNode : modelNode.children("include")) {
        const string uri = includeNode.child("uri").text().as_string();
        if (uri.empty()) {
            os() << _("Warning: an <include> element has no <uri> and is skipped.") << endl;
            continue;
        }
        const string includedFile = resolveIncludeUri(uri, currentBaseDir);
        if (includedFile.empty() || !filesystem::exists(filesystem::path(fromUTF8(includedFile)))) {
            os() << formatR(_("Warning: the included model \"{0}\" was not found and is skipped."),
                            uri) << endl;
            continue;
        }

        auto doc = std::make_shared<pugi::xml_document>();
        pugi::xml_parse_result result = doc->load_file(includedFile.c_str());
        if (!result) {
            os() << formatR(_("Warning: failed to parse the included model \"{0}\": {1}"),
                            uri, result.description()) << endl;
            continue;
        }
        includedDocs.push_back(doc);

        xml_node sdfNode = doc->child("sdf");
        if (sdfNode.empty()) {
            sdfNode = doc->child("gazebo");
        }
        xml_node includedModel = sdfNode.child("model");
        if (includedModel.empty()) {
            xml_node worldNode = sdfNode.child("world");
            if (!worldNode.empty()) {
                includedModel = worldNode.child("model");
            }
        }
        if (includedModel.empty()) {
            os() << formatR(_("Warning: the included file \"{0}\" has no <model> element and is "
                              "skipped."), uri) << endl;
            continue;
        }

        // The <include> may override the sub-model's name and pose.
        string subName = includeNode.child("name").text().as_string();
        if (subName.empty()) {
            subName = includedModel.attribute("name").as_string();
        }
        const string subPrefix = namePrefix.empty() ? subName : (namePrefix + "::" + subName);

        Isometry3 includePose = Isometry3::Identity();
        const xml_node poseNode = includeNode.child("pose");
        if (!poseNode.empty()) {
            if (!readPose(poseNode, includePose)) {
                os() << formatR(_(" in the <include> of \"{0}\"."), uri) << endl;
                continue;
            }
        }
        const Isometry3 accumulatedPose = parentIncludePose * includePose;

        // The base directory used to resolve "model://" inside the included sub-model is the
        // directory that contains the included model directory (e.g. ".../gazebo_models" when
        // the included file is ".../gazebo_models/create/model.sdf"). This matches how the
        // primary model's base directory is computed.
        filesystem::path includedPath(fromUTF8(includedFile));
        filesystem::path includedModelDir = includedPath.parent_path();
        string includedBaseDir = toUTF8(includedModelDir.parent_path().string());

        // Make the sub-model's container directory available to the "model://" handler so
        // that meshes referenced from inside this sub-model can be resolved against it.
        uriSchemeProcessor.addModelSearchDirectory(includedBaseDir);

        // Register the sub-model's textures directory as an image search fallback, just as
        // the primary model does.
        sceneLoader.addImageSearchDirectory(
            toUTF8((includedModelDir / "materials" / "textures").string()));

        for (xml_node linkNode : includedModel.children("link")) {
            CollectedLink cl;
            cl.node = linkNode;
            cl.namePrefix = subPrefix;
            cl.includePose = accumulatedPose;
            cl.baseDirectoryForMeshes = includedBaseDir;
            outLinks.push_back(cl);
        }
        for (xml_node jointNode : includedModel.children("joint")) {
            CollectedJoint cj;
            cj.node = jointNode;
            cj.namePrefix = subPrefix;
            outJoints.push_back(cj);
        }

        // Recurse into nested <include> elements.
        if (!expandIncludes(includedModel, subPrefix, accumulatedPose, includedBaseDir,
                            outLinks, outJoints)) {
            return false;
        }
    }
    return true;
}


bool SDFBodyLoader::Impl::loadLink(
    Body* body, LinkInfo& info, const xml_node& linkNode,
    const string& namePrefix, const Isometry3& includePose,
    const string& baseDirectoryForMeshes)
{
    LinkPtr link = info.link;

    const string rawName = linkNode.attribute("name").as_string();
    if (rawName.empty()) {
        os() << _("Error: there is an unnamed link.") << endl;
        return false;
    }
    const string name = namePrefix.empty() ? rawName : (namePrefix + "::" + rawName);
    link->setName(name);

    // Ensure the sub-model's directory is available to the "model://" handler for any meshes
    // referenced by this link. Adding it here (instead of relying on expandIncludes to have
    // already done so) keeps things robust even when nesting becomes deep.
    if (!baseDirectoryForMeshes.empty()) {
        uriSchemeProcessor.addModelSearchDirectory(baseDirectoryForMeshes);
    }

    // 'pose' element (optional). The 'relative_to' attribute is not supported in this
    // minimal implementation; only model-frame-relative poses are handled.
    if (!readPoseElement(linkNode, info.modelPose, info.poseRelativeTo)) {
        os() << formatR(_(" in the link \"{0}\"."), name) << endl;
        return false;
    }
    // Apply the enclosing sub-model's include pose so that the link's pose ends up expressed
    // in the primary model frame, matching the convention used by the rest of load().
    info.modelPose = includePose * info.modelPose;
    if (!info.poseRelativeTo.empty()) {
        os() << formatR(_("Warning: the 'relative_to' attribute of the link \"{0}\" pose is not "
                          "supported and is ignored."), name) << endl;
    }

    // 'inertial' element (optional).
    const xml_node inertialNode = linkNode.child("inertial");
    if (!inertialNode.empty()) {
        if (!loadInertial(link, inertialNode)) {
            os() << formatR(_(" in the link \"{0}\"."), name) << endl;
            return false;
        }
    }

    // 'visual' elements (optional, multiple allowed). A shape that cannot be read (e.g. an
    // unsupported geometry or a mesh that fails to load) is skipped with a warning so that the
    // rest of the link is still loaded.
    for (xml_node visualNode : linkNode.children("visual")) {
        ShapeDescription description;
        if (!readShapeDescription(visualNode, description)) {
            os() << formatR(_(" The visual of the link \"{0}\" is skipped."), name) << endl;
            continue;
        }
        const xml_node materialNode = visualNode.child("material");
        if (!materialNode.empty()) {
            description.material = readMaterial(materialNode);
        }
        SgNode* shape = createShape(description);
        if (!shape) {
            os() << formatR(_(" The visual of the link \"{0}\" is skipped."), name) << endl;
            continue;
        }
        if (description.material) {
            setMaterialToAllShapeNodes(shape, description.material);
        }
        link->addVisualShapeNode(shape);
    }

    // 'collision' elements (optional, multiple allowed). Skipped with a warning on failure.
    for (xml_node collisionNode : linkNode.children("collision")) {
        ShapeDescription description;
        if (!readShapeDescription(collisionNode, description)) {
            os() << formatR(_(" The collision of the link \"{0}\" is skipped."), name) << endl;
            continue;
        }
        SgNode* shape = createShape(description);
        if (!shape) {
            os() << formatR(_(" The collision of the link \"{0}\" is skipped."), name) << endl;
            continue;
        }
        link->addCollisionShapeNode(shape);
    }

    // 'sensor' elements (optional, multiple allowed). Each one is dispatched by its 'type'
    // attribute; unrecognized or unsupported types are reported with a warning and skipped, so
    // that the rest of the link is still loaded.
    for (xml_node sensorNode : linkNode.children("sensor")) {
        loadSensor(body, link, sensorNode, namePrefix);
    }

    return true;
}


bool SDFBodyLoader::Impl::loadInertial(LinkPtr link, const xml_node& inertialNode)
{
    // 'pose' element (optional): the pose of the inertial frame relative to the link frame.
    Isometry3 inertialPose = Isometry3::Identity();
    string relativeTo;
    if (!readPoseElement(inertialNode, inertialPose, relativeTo)) {
        os() << _(" in an inertial element");
        return false;
    }
    link->setCenterOfMass(inertialPose.translation());

    // 'mass' element (optional). In SDF the mass may be omitted, which is common for
    // visual-only links. In that case the link keeps its default mass.
    const xml_node massNode = inertialNode.child("mass");
    if (!massNode.empty()) {
        const double mass = massNode.text().as_double();
        if (mass > 0.0) {
            link->setMass(mass);
        } else {
            os() << formatR(_("Warning: the mass of the link \"{0}\" is not positive and is "
                              "ignored."), link->name()) << endl;
        }
    }

    // 'inertia' element.
    Matrix3 inertiaMatrix = Matrix3::Identity();
    const xml_node inertiaNode = inertialNode.child("inertia");
    if (!inertiaNode.empty()) {
        if (!readInertiaElement(inertiaNode, inertiaMatrix)) {
            return false;
        }
    }
    // The inertia is described in the inertial frame; rotate it into the link frame.
    const Matrix3& R = inertialPose.linear();
    link->setInertia(R * inertiaMatrix * R.transpose());

    return true;
}


bool SDFBodyLoader::Impl::readInertiaElement(const xml_node& inertiaNode, Matrix3& inertiaMatrix)
{
    // Each component defaults to 0 except the diagonal moments, which must be given.
    inertiaMatrix.setZero();

    const xml_node ixx = inertiaNode.child("ixx");
    const xml_node iyy = inertiaNode.child("iyy");
    const xml_node izz = inertiaNode.child("izz");
    if (ixx.empty() || iyy.empty() || izz.empty()) {
        os() << _("Error: the diagonal moments of inertia (ixx, iyy, izz) must be defined.");
        return false;
    }
    inertiaMatrix(0, 0) = ixx.text().as_double();
    inertiaMatrix(1, 1) = iyy.text().as_double();
    inertiaMatrix(2, 2) = izz.text().as_double();

    const double ixy = inertiaNode.child("ixy").text().as_double(0.0);
    const double ixz = inertiaNode.child("ixz").text().as_double(0.0);
    const double iyz = inertiaNode.child("iyz").text().as_double(0.0);
    inertiaMatrix(0, 1) = inertiaMatrix(1, 0) = ixy;
    inertiaMatrix(0, 2) = inertiaMatrix(2, 0) = ixz;
    inertiaMatrix(1, 2) = inertiaMatrix(2, 1) = iyz;

    return true;
}


bool SDFBodyLoader::Impl::readShapeDescription(
    const xml_node& shapeNode, ShapeDescription& description)
{
    // 'pose' element (optional): the pose of the shape relative to the link frame.
    string relativeTo;
    if (!readPoseElement(shapeNode, description.pose, relativeTo)) {
        return false;
    }

    // 'geometry' element (required).
    const xml_node geometryNode = shapeNode.child("geometry");
    if (geometryNode.empty()) {
        os() << _("Warning: a geometry is not found.");
        return false;
    }
    return readGeometry(geometryNode, description);
}


bool SDFBodyLoader::Impl::readGeometry(const xml_node& geometryNode, ShapeDescription& description)
{
    if (!geometryNode.child("box").empty()) {
        description.shapeType = ShapeDescription::Box;
        const xml_node sizeNode = geometryNode.child("box").child("size");
        if (!toVector3(sizeNode.text().as_string(), description.boxSize)) {
            os() << _("Warning: the box size is written in an invalid format.");
            return false;
        }
    } else if (!geometryNode.child("cylinder").empty()) {
        description.shapeType = ShapeDescription::Cylinder;
        const xml_node cylinderNode = geometryNode.child("cylinder");
        if (cylinderNode.child("radius").empty() || cylinderNode.child("length").empty()) {
            os() << _("Warning: a cylinder must have a radius and a length.");
            return false;
        }
        description.radius = cylinderNode.child("radius").text().as_double();
        description.length = cylinderNode.child("length").text().as_double();
    } else if (!geometryNode.child("sphere").empty()) {
        description.shapeType = ShapeDescription::Sphere;
        const xml_node sphereNode = geometryNode.child("sphere");
        if (sphereNode.child("radius").empty()) {
            os() << _("Warning: a sphere must have a radius.");
            return false;
        }
        description.radius = sphereNode.child("radius").text().as_double();
    } else if (!geometryNode.child("mesh").empty()) {
        description.shapeType = ShapeDescription::Mesh;
        const xml_node meshNode = geometryNode.child("mesh");
        if (meshNode.child("uri").empty()) {
            os() << _("Warning: a mesh must have a uri.");
            return false;
        }
        description.meshUri = meshNode.child("uri").text().as_string();
        const xml_node scaleNode = meshNode.child("scale");
        if (!scaleNode.empty()) {
            if (!toVector3(scaleNode.text().as_string(), description.meshScale)) {
                os() << _("Warning: the mesh scale is written in an invalid format.");
                return false;
            }
        }
        // SDF <submesh> selects a named subtree of the mesh file (typical use: a Collada file
        // that packs the whole vehicle, with each link picking only its piece).
        const xml_node submeshNode = meshNode.child("submesh");
        if (!submeshNode.empty()) {
            description.submeshName = submeshNode.child("name").text().as_string();
            description.submeshCenter = submeshNode.child("center").text().as_bool(false);
        }
    } else if (!geometryNode.child("polyline").empty()) {
        // SDF <polyline>: a closed 2D path on the X-Y plane, extruded along +Z by <height>.
        // SDF technically allows multiple <polyline> blocks under <geometry> to define a hole-
        // bearing shape; we only handle the first one and warn if more are present.
        const xml_node polylineNode = geometryNode.child("polyline");
        if (++geometryNode.children("polyline").begin() != geometryNode.children("polyline").end()) {
            os() << _("Warning: multiple <polyline> blocks under one <geometry> are not "
                      "supported; only the first one is used.") << endl;
        }
        const double height = polylineNode.child("height").text().as_double(1.0);
        if (height <= 0.0) {
            os() << _("Warning: a <polyline> requires a positive <height>.") << endl;
            return false;
        }
        for (xml_node pointNode : polylineNode.children("point")) {
            Vector2 p(0.0, 0.0);
            const string s = pointNode.text().as_string();
            const char* nptr = s.c_str();
            char* endptr;
            p.x() = strtod(nptr, &endptr);
            if (endptr == nptr) {
                os() << _("Warning: a <point> of a <polyline> is written in an invalid format.")
                     << endl;
                return false;
            }
            nptr = endptr;
            p.y() = strtod(nptr, &endptr);
            if (endptr == nptr) {
                os() << _("Warning: a <point> of a <polyline> must have two values.") << endl;
                return false;
            }
            description.polylinePoints.push_back(p);
        }
        if (description.polylinePoints.size() < 3) {
            os() << _("Warning: a <polyline> needs at least three points.") << endl;
            return false;
        }
        description.shapeType = ShapeDescription::Polyline;
        description.length = height;
    } else if (!geometryNode.child("plane").empty()) {
        // SDF planes have a 2D <size> (X, Y) and a <normal>. Since Choreonoid's MeshGenerator
        // does not have a primitive for an unbounded plane, we approximate the plane as a thin
        // box. A non-vertical-up normal is treated as Z-up with a warning; rotating to align
        // an arbitrary normal could be added later if needed.
        const xml_node planeNode = geometryNode.child("plane");
        Vector3 normal(0.0, 0.0, 1.0);
        const xml_node normalNode = planeNode.child("normal");
        if (!normalNode.empty()) {
            if (!toVector3(normalNode.text().as_string(), normal)) {
                os() << _("Warning: the plane normal is written in an invalid format.");
                return false;
            }
            if (std::abs(normal.x()) > 1e-6 || std::abs(normal.y()) > 1e-6) {
                os() << _("Warning: a non-Z plane normal is treated as Z-up.");
            }
        }
        double sizeX = 0.0, sizeY = 0.0;
        const xml_node sizeNode = planeNode.child("size");
        if (!sizeNode.empty()) {
            const string s = sizeNode.text().as_string();
            const char* p = s.c_str();
            char* endp;
            sizeX = strtod(p, &endp);
            if (endp == p) sizeX = 0.0;
            p = endp;
            sizeY = strtod(p, &endp);
            if (endp == p) sizeY = sizeX;
        }
        if (sizeX <= 0.0) sizeX = 100.0; // unbounded plane fallback
        if (sizeY <= 0.0) sizeY = 100.0;
        constexpr double PlaneThickness = 0.002;
        description.shapeType = ShapeDescription::Box;
        description.boxSize = Vector3(sizeX, sizeY, PlaneThickness);
        // Offset the box so its top surface sits at the plane's local z=0.
        description.pose.translation() += Vector3(0.0, 0.0, -PlaneThickness * 0.5);
    } else {
        const char* geomName = geometryNode.first_child().name();
        os() << formatR(_("Warning: the geometry \"{0}\" is not supported."),
                        (geomName && *geomName) ? geomName : "(unknown)");
        return false;
    }

    return true;
}


SgNode* SDFBodyLoader::Impl::createShape(ShapeDescription& description)
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
        // In SDF a cylinder's axis is along the Z axis; the generated mesh is along the Y axis.
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
    case ShapeDescription::Polyline: {
        // The MeshGenerator's extrusion follows the VRML convention: the cross section is
        // defined on the X-Z plane and the spine runs along Y. So we put the SDF polyline's
        // 2D (x, y) points into the cross section as (x, z) and run the spine along Y for
        // <height> meters, then rotate the resulting mesh -90 degrees around X so the spine
        // ends up aligned with the SDF's +Z axis and the SDF's +Y direction is preserved
        // (a +90 degree rotation would flip the cross section's Y axis).
        MeshGenerator::Extrusion extrusion;
        extrusion.crossSection = description.polylinePoints;
        // generateExtrusion requires the cross section to be a closed loop (it detects
        // closure by comparing the first and last points, and uses that flag when wrapping
        // the side faces back to the start). SDF polylines are usually already closed; if
        // not, append the first point as the closing one.
        if (!extrusion.crossSection.empty()
            && extrusion.crossSection.front() != extrusion.crossSection.back()) {
            extrusion.crossSection.push_back(extrusion.crossSection.front());
        }
        extrusion.spine.emplace_back(0.0, 0.0, 0.0);
        extrusion.spine.emplace_back(0.0, description.length, 0.0);
        auto polyline = new SgShape;
        polyline->setMesh(meshGenerator.generateExtrusion(extrusion));
        auto transform = new SgPosTransform;
        transform->setRotation(AngleAxis(-M_PI / 2.0, Vector3::UnitX()));
        transform->addChild(polyline);
        shape = transform;
        break;
    }
    default:
        break;
    }

    if (!shape) {
        return nullptr;
    }

    if (!description.pose.isApprox(Isometry3::Identity())) {
        auto transform = new SgPosTransform(description.pose);
        transform->addChild(shape);
        shape = transform;
    }

    return shape.retn();
}


SgNode* SDFBodyLoader::Impl::createMesh(ShapeDescription& description)
{
    SgNodePtr scene;

    const string filePath = uriSchemeProcessor.getFilePath(description.meshUri);
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
            if (!isSupportedFormat) {
                os() << formatR(_("Warning: the format of the mesh file \"{0}\" is not supported."),
                                description.meshUri);
            } else {
                // The format is recognized but the loader rejected the file (e.g. Assimp may
                // refuse a Collada file with malformed texture references). The underlying
                // loader has already emitted its own diagnostic, so we just supply the mesh
                // file context here.
                os() << formatR(_("Warning: failed to load the mesh file \"{0}\"."),
                                description.meshUri);
            }
            return nullptr;
        }
        if (auto uriObject = scene->findObject([](SgObject* object){ return object->hasUri(); })) {
            uriObject->setUri(description.meshUri, filePath);
        }
        meshMap[filePath] = scene;
    }

    // SDF <submesh>: extract the named subtree. The shared, cached scene is kept intact (the
    // matched node is referenced rather than detached) so that other links referencing the
    // same mesh URI can still use their own submeshes.
    //
    // The matched node and everything below it is kept as-is: vertices and all transforms
    // (including the node's own placement) define the submesh in the mesh file's coordinate
    // system. With <center>false the SDF link is meant to inherit that placement; with
    // <center>true the bounding box center is shifted to the origin instead. Transforms
    // above the matched node (e.g. a unit-system scale on the visual_scene root) are also
    // kept so that lengths come out in meters.
    if (!description.submeshName.empty()) {
        SgNode* sub = nullptr;
        Affine3 accumT;
        if (findNamedSubNode(scene, description.submeshName, Affine3::Identity(), sub, accumT)) {
            SgNodePtr extracted = sub;
            if (description.submeshCenter) {
                // Recenter the extracted submesh's bounding box at the origin. The bbox is in
                // the extracted node's own frame; transform through accumT so the world-space
                // center sits at the origin.
                BoundingBox bbox = extracted->boundingBox();
                if (!bbox.empty()) {
                    accumT.translation() -= accumT.linear() * bbox.center();
                }
            }
            const bool hasLinear = !accumT.linear().isApprox(Matrix3::Identity());
            const bool hasTranslation = !accumT.translation().isZero();
            if (hasLinear || hasTranslation) {
                // SgAffineTransform handles rotation, scaling, or any mix, which is what a
                // generic accumulated transform may contain.
                auto transform = new SgAffineTransform;
                transform->setLinear(accumT.linear());
                transform->translation() = accumT.translation();
                transform->addChild(extracted);
                scene = transform;
            } else {
                scene = extracted;
            }
        } else {
            os() << formatR(_("Warning: the submesh \"{0}\" is not found in the mesh file "
                              "\"{1}\"; the whole mesh is used instead."),
                            description.submeshName, description.meshUri) << endl;
        }
    }

    if (description.meshScale != Vector3::Ones()) {
        auto scaler = new SgScaleTransform;
        scaler->setScale(description.meshScale);
        scaler->addChild(scene);
        scene = scaler;
    }

    return scene.retn();
}


bool SDFBodyLoader::Impl::findNamedSubNode(
    SgNode* node, const string& targetName, const Affine3& parentTransform,
    SgNode*& out_node, Affine3& out_transform)
{
    if (!node) {
        return false;
    }
    // The matched node itself is returned as the extracted subtree; its own transform stays
    // part of the subtree and is NOT folded into out_transform. out_transform accumulates
    // only ancestor transforms (so it captures things like a unit-system scale on the
    // visual_scene root) that the matched node does not already carry inside it.
    if (node->name() == targetName) {
        out_node = node;
        out_transform = parentTransform;
        return true;
    }
    Affine3 here = parentTransform;
    if (auto transform = dynamic_cast<SgTransform*>(node)) {
        Affine3 local;
        transform->getTransform(local);
        here = parentTransform * local;
    }
    if (auto group = dynamic_cast<SgGroup*>(node)) {
        for (SgNode* child : *group) {
            if (findNamedSubNode(child, targetName, here, out_node, out_transform)) {
                return true;
            }
        }
    }
    return false;
}


SgMaterial* SDFBodyLoader::Impl::readMaterial(const xml_node& materialNode)
{
    SgMaterialPtr material;

    // SDF lets a material be specified indirectly by referencing an Ogre material script
    // (e.g. <script><uri>file://media/materials/scripts/gazebo.material</uri>
    // <name>Gazebo/Wood</name></script>). Parsing those scripts is out of scope for this
    // loader, so we warn the user once per script name rather than silently substituting a
    // default color.
    const xml_node scriptNode = materialNode.child("script");
    if (!scriptNode.empty()) {
        string scriptName = scriptNode.child("name").text().as_string();
        if (scriptName.empty()) {
            scriptName = "(unnamed)";
        }
        const string key = "script:" + scriptName;
        if (warnedUnsupportedMaterialFeatures.insert(key).second) {
            os() << formatR(_("Warning: the material script \"{0}\" is not supported; "
                              "shapes that reference it will use the default color."),
                            scriptName) << endl;
        }
    }

    // SDF 1.5+ also allows physically-based rendering parameters via <material><pbr>...
    // </pbr></material>. They are not yet read either.
    const xml_node pbrNode = materialNode.child("pbr");
    if (!pbrNode.empty()) {
        if (warnedUnsupportedMaterialFeatures.insert("pbr").second) {
            os() << _("Warning: the material <pbr> element is not supported and is ignored.")
                 << endl;
        }
    }

    const xml_node diffuseNode = materialNode.child("diffuse");
    if (!diffuseNode.empty()) {
        Vector4 rgba = Vector4::Zero();
        if (toVector4(diffuseNode.text().as_string(), rgba)) {
            rgba = rgba.array().min(1.0).max(0.0);
            material = new SgMaterial;
            material->setDiffuseColor(Vector3(rgba[0], rgba[1], rgba[2]));
            material->setTransparency(1.0f - static_cast<float>(rgba[3]));
        }
    }

    const xml_node ambientNode = materialNode.child("ambient");
    if (!ambientNode.empty()) {
        Vector4 rgba = Vector4::Zero();
        if (toVector4(ambientNode.text().as_string(), rgba)) {
            rgba = rgba.array().min(1.0).max(0.0);
            if (!material) {
                material = new SgMaterial;
            }
            // Stores the ambient intensity as the average of the RGB components.
            material->setAmbientIntensity((rgba[0] + rgba[1] + rgba[2]) / 3.0f);
        }
    }

    const xml_node specularNode = materialNode.child("specular");
    if (!specularNode.empty()) {
        Vector4 rgba = Vector4::Zero();
        if (toVector4(specularNode.text().as_string(), rgba)) {
            rgba = rgba.array().min(1.0).max(0.0);
            if (!material) {
                material = new SgMaterial;
            }
            material->setSpecularColor(Vector3(rgba[0], rgba[1], rgba[2]));
        }
    }

    const xml_node emissiveNode = materialNode.child("emissive");
    if (!emissiveNode.empty()) {
        Vector4 rgba = Vector4::Zero();
        if (toVector4(emissiveNode.text().as_string(), rgba)) {
            rgba = rgba.array().min(1.0).max(0.0);
            if (!material) {
                material = new SgMaterial;
            }
            material->setEmissiveColor(Vector3(rgba[0], rgba[1], rgba[2]));
        }
    }

    return material.retn();
}


void SDFBodyLoader::Impl::setMaterialToAllShapeNodes(SgNode* node, SgMaterial* material)
{
    if (auto group = node->toGroupNode()) {
        for (auto& child : *group) {
            setMaterialToAllShapeNodes(child, material);
        }
    } else if (auto shape = dynamic_cast<SgShape*>(node)) {
        if (!shape->material()) {
            shape->setMaterial(material);
        }
    }
}


// Reads the common 'pose', 'always_on' and 'update_rate' children of a <sensor> element and
// applies them to the device. The sensor pose is interpreted as a transform relative to the
// link frame (the parent link), matching the convention used elsewhere by this loader; the
// 'relative_to' attribute is not supported and is reported once per occurrence.
void SDFBodyLoader::Impl::readCommonSensorAttributes(Device* device, const xml_node& sensorNode)
{
    Isometry3 pose = Isometry3::Identity();
    string relativeTo;
    if (readPoseElement(sensorNode, pose, relativeTo)) {
        device->setLocalTranslation(pose.translation());
        device->setLocalRotation(pose.linear());
        if (!relativeTo.empty()) {
            os() << formatR(_("Warning: the 'relative_to' attribute of the sensor \"{0}\" pose is "
                              "not supported and is ignored."), device->name()) << endl;
        }
    }

    const xml_node alwaysOnNode = sensorNode.child("always_on");
    if (!alwaysOnNode.empty()) {
        device->on(alwaysOnNode.text().as_bool(true));
    }

    if (auto visionSensor = dynamic_cast<VisionSensor*>(device)) {
        const xml_node updateRateNode = sensorNode.child("update_rate");
        if (!updateRateNode.empty()) {
            const double rate = updateRateNode.text().as_double(0.0);
            if (rate > 0.0) {
                visionSensor->setFrameRate(rate);
            }
        }
    }
}


// Creates a Camera or RangeCamera from an SDF <camera> child element. Returns nullptr when
// the element does not define a usable camera.
DevicePtr SDFBodyLoader::Impl::createCameraDevice(
    const xml_node& sensorNode, const string& sensorType, const xml_node& cameraNode)
{
    const bool isDepth = (sensorType == "depth" || sensorType == "depth_camera"
                         || sensorType == "rgbd" || sensorType == "rgbd_camera");

    CameraPtr camera;
    if (isDepth) {
        auto rangeCamera = new RangeCamera;
        // A depth/RGBD camera produces a depth field and, when the SDF specifies a color
        // format, also a color image. We default to depth-only and switch to depth+color when
        // an <image><format> indicates a non-depth pixel format.
        rangeCamera->setOrganized(true);
        camera = rangeCamera;
    } else {
        camera = new Camera;
    }

    // Match SDF's "+X is the optical axis" convention.
    camera->setOpticalFrame(VisionSensor::Robotics);

    // <horizontal_fov> is in radians.
    const xml_node hfovNode = cameraNode.child("horizontal_fov");
    if (!hfovNode.empty()) {
        const double hfov = hfovNode.text().as_double(0.0);
        if (hfov > 0.0) {
            camera->setHorizontalFieldOfView(hfov);
        }
    }

    // <image><width>, <height>, <format>.
    const xml_node imageNode = cameraNode.child("image");
    int resX = 320;
    int resY = 240;
    if (!imageNode.empty()) {
        const int w = imageNode.child("width").text().as_int(0);
        const int h = imageNode.child("height").text().as_int(0);
        if (w > 0) resX = w;
        if (h > 0) resY = h;

        const string format = imageNode.child("format").text().as_string();
        if (!format.empty()) {
            if (format.find("L8") != string::npos || format.find("L16") != string::npos
                || format == "MONO8" || format == "MONO16") {
                camera->setImageType(Camera::GRAYSCALE_IMAGE);
            } else {
                camera->setImageType(Camera::COLOR_IMAGE);
            }
        } else {
            camera->setImageType(isDepth ? Camera::NO_IMAGE : Camera::COLOR_IMAGE);
        }
    } else {
        camera->setImageType(isDepth ? Camera::NO_IMAGE : Camera::COLOR_IMAGE);
    }
    camera->setResolution(resX, resY);

    // <clip><near>, <far>.
    const xml_node clipNode = cameraNode.child("clip");
    if (!clipNode.empty()) {
        const double nearClip = clipNode.child("near").text().as_double(0.0);
        const double farClip = clipNode.child("far").text().as_double(0.0);
        if (nearClip > 0.0) camera->setNearClipDistance(nearClip);
        if (farClip > 0.0) camera->setFarClipDistance(farClip);
    }

    // <lens><type>: any non-"gnomonical" type maps to FISHEYE_LENS. SDF allows several
    // projection models (equidistant, stereographic, etc.); Choreonoid's Camera only
    // distinguishes normal from fisheye, so the detail is lost intentionally.
    const xml_node lensNode = cameraNode.child("lens");
    if (!lensNode.empty()) {
        const string lensType = lensNode.child("type").text().as_string();
        if (!lensType.empty() && lensType != "gnomonical") {
            camera->setLensType(Camera::FISHEYE_LENS);
        }
    }

    return camera;
}


// Creates a RangeSensor from an SDF <ray>/<lidar> child element. SDF describes the scan as
// horizontal and vertical sweeps in yaw/pitch; this maps directly to RangeSensor's yaw and
// pitch ranges.
DevicePtr SDFBodyLoader::Impl::createRangeSensorDevice(
    const xml_node& /*sensorNode*/, const xml_node& rayNode)
{
    auto sensor = new RangeSensor;

    const xml_node scanNode = rayNode.child("scan");
    if (!scanNode.empty()) {
        const xml_node hNode = scanNode.child("horizontal");
        if (!hNode.empty()) {
            const int samples = hNode.child("samples").text().as_int(1);
            const double minAngle = hNode.child("min_angle").text().as_double(0.0);
            const double maxAngle = hNode.child("max_angle").text().as_double(0.0);
            if (maxAngle > minAngle) {
                sensor->setYawRange(minAngle, maxAngle);
            }
            if (samples > 0) {
                sensor->setNumYawSamples(samples);
            }
        }
        const xml_node vNode = scanNode.child("vertical");
        if (!vNode.empty()) {
            const int samples = vNode.child("samples").text().as_int(1);
            const double minAngle = vNode.child("min_angle").text().as_double(0.0);
            const double maxAngle = vNode.child("max_angle").text().as_double(0.0);
            if (samples > 1) {
                sensor->setNumPitchSamples(samples);
                if (maxAngle > minAngle) {
                    sensor->setPitchRange(minAngle, maxAngle);
                }
            }
        }
    }

    const xml_node rangeNode = rayNode.child("range");
    if (!rangeNode.empty()) {
        const double minD = rangeNode.child("min").text().as_double(0.0);
        const double maxD = rangeNode.child("max").text().as_double(0.0);
        if (minD > 0.0) sensor->setMinDistance(minD);
        if (maxD > 0.0) sensor->setMaxDistance(maxD);
    }

    return sensor;
}


// Reads a single <sensor> child of a <link> and, for supported types, creates the matching
// Choreonoid device and attaches it to the body. Unsupported or malformed sensors emit a
// warning and are skipped without aborting the link.
void SDFBodyLoader::Impl::loadSensor(
    Body* body, LinkPtr link, const xml_node& sensorNode, const string& namePrefix)
{
    const string rawName = sensorNode.attribute("name").as_string();
    const string sensorType = sensorNode.attribute("type").as_string();
    const string displayName = rawName.empty() ? string("(unnamed)") : rawName;
    const string fullName = namePrefix.empty() ? rawName : (namePrefix + "::" + rawName);

    auto attach = [&](Device* device) {
        device->setName(fullName);
        readCommonSensorAttributes(device, sensorNode);
        body->addDevice(device, link);
    };

    if (sensorType == "imu") {
        attach(new Imu);
    } else if (sensorType == "force_torque") {
        const xml_node ftNode = sensorNode.child("force_torque");
        if (!ftNode.empty()) {
            const string frame = ftNode.child("frame").text().as_string();
            if (!frame.empty() && frame != "child") {
                os() << formatR(_("Warning: the <frame>{0}</frame> of the force_torque sensor "
                                  "\"{1}\" is not supported; the child link frame is assumed."),
                                frame, displayName) << endl;
            }
            const string measure = ftNode.child("measure_direction").text().as_string();
            if (!measure.empty() && measure != "child_to_parent") {
                os() << formatR(_("Warning: the <measure_direction>{0}</measure_direction> of the "
                                  "force_torque sensor \"{1}\" is not supported; "
                                  "\"child_to_parent\" is assumed."),
                                measure, displayName) << endl;
            }
        }
        attach(new ForceSensor);
    } else if (sensorType == "camera" || sensorType == "depth" || sensorType == "depth_camera"
               || sensorType == "rgbd" || sensorType == "rgbd_camera") {
        const xml_node cameraNode = sensorNode.child("camera");
        if (cameraNode.empty()) {
            os() << formatR(_("Warning: the camera sensor \"{0}\" on the link \"{1}\" has no "
                              "<camera> element and is skipped."), displayName, link->name())
                 << endl;
            return;
        }
        DevicePtr device = createCameraDevice(sensorNode, sensorType, cameraNode);
        if (device) {
            attach(device.get());
        }
    } else if (sensorType == "multicamera") {
        // SDF's multicamera packs several <camera> children into one <sensor>. We unpack them
        // into individual Camera devices so that each can be addressed separately downstream.
        int index = 0;
        for (xml_node cameraNode : sensorNode.children("camera")) {
            DevicePtr device = createCameraDevice(sensorNode, "camera", cameraNode);
            if (!device) {
                continue;
            }
            const string subName = cameraNode.attribute("name").as_string();
            // Reads the per-<camera> pose, which is layered on top of the sensor pose.
            Isometry3 cameraPose = Isometry3::Identity();
            string cameraRelativeTo;
            readPoseElement(cameraNode, cameraPose, cameraRelativeTo);

            const string baseName = subName.empty()
                ? formatC("{0}_{1}", fullName, index)
                : (fullName + "::" + subName);
            device->setName(baseName);
            readCommonSensorAttributes(device.get(), sensorNode);
            const Isometry3 combined = device->T_local() * cameraPose;
            device->setLocalTranslation(combined.translation());
            device->setLocalRotation(combined.linear());
            body->addDevice(device.get(), link);
            ++index;
        }
        if (index == 0) {
            os() << formatR(_("Warning: the multicamera sensor \"{0}\" on the link \"{1}\" has no "
                              "<camera> child and is skipped."), displayName, link->name())
                 << endl;
        }
    } else if (sensorType == "ray" || sensorType == "gpu_ray"
               || sensorType == "lidar" || sensorType == "gpu_lidar") {
        xml_node rayNode = sensorNode.child("ray");
        if (rayNode.empty()) {
            rayNode = sensorNode.child("lidar");
        }
        if (rayNode.empty()) {
            os() << formatR(_("Warning: the ray/lidar sensor \"{0}\" on the link \"{1}\" has no "
                              "<ray> (or <lidar>) element and is skipped."),
                            displayName, link->name()) << endl;
            return;
        }
        DevicePtr device = createRangeSensorDevice(sensorNode, rayNode);
        if (device) {
            attach(device.get());
        }
    } else {
        os() << formatR(_("Warning: the sensor \"{0}\" (type=\"{1}\") on the link \"{2}\" is not "
                          "supported and is skipped."),
                        displayName,
                        sensorType.empty() ? string("(unspecified)") : sensorType,
                        link->name()) << endl;
    }
}


// Returns true if the joint type can be represented as a Choreonoid joint that defines a tree
// edge with a degree of freedom or a fixed connection. Types such as "screw" or "gearbox",
// which are typically used to close a kinematic loop, are not in this set.
static bool isSupportedJointType(const string& jointType)
{
    return jointType == "revolute" || jointType == "continuous" || jointType == "prismatic"
        || jointType == "fixed" || jointType == "ball";
}


// Selects which of the candidate parent joints of a single child link to use for the tree.
// The first joint in SDF order is preferred, but if its type is not supported (e.g. a "screw"
// joint that closes a loop), the first subsequent joint with a supported type is chosen. If no
// candidate has a supported type, the first one is used.
int SDFBodyLoader::Impl::countDescendants(Link* link)
{
    int n = 0;
    vector<Link*> stack{link};
    while (!stack.empty()) {
        Link* l = stack.back(); stack.pop_back();
        ++n;
        for (Link* c = l->child(); c; c = c->sibling()) stack.push_back(c);
    }
    return n;
}


// Attaches a stray root link to the main tree as a fixed-joint child of newParent. When
// isReversed is true the connection is the reverse of a dropped joint (the original joint's
// parent is now treated as the child); the joint name is preserved for traceability and a
// warning explains the parent/child reversal. When isReversed is false the stray is attached
// as a last-resort fallback (no original joint exists between these two links).
void SDFBodyLoader::Impl::attachStrayRoot(
    LinkInfo* strayInfo, Link* newParent, const string& jointName, bool isReversed)
{
    Link* stray = strayInfo->link;
    newParent->appendChild(stray);
    stray->setJointType(Link::FixedJoint);
    if (!jointName.empty()) {
        stray->setJointName(jointName);
    }
    if (isReversed) {
        os() << formatR(_("Warning: the stray link \"{0}\" left by breaking the closed loop is "
                          "reattached to the link \"{1}\" as a fixed child by reversing the "
                          "dropped joint \"{2}\"."),
                        stray->name(), newParent->name(), jointName) << endl;
    } else {
        os() << formatR(_("Warning: the stray link \"{0}\" left by breaking the closed loop is "
                          "attached to the root link \"{1}\" as a fixed child because no dropped "
                          "joint connects it to the main tree."),
                        stray->name(), newParent->name()) << endl;
    }
}


size_t SDFBodyLoader::Impl::selectParentJoint(const vector<ResolvedJoint>& candidates)
{
    if (isSupportedJointType(candidates[0].node.attribute("type").as_string())) {
        return 0;
    }
    for (size_t i = 1; i < candidates.size(); ++i) {
        if (isSupportedJointType(candidates[i].node.attribute("type").as_string())) {
            return i;
        }
    }
    return 0;
}


bool SDFBodyLoader::Impl::buildJoint(
    std::unordered_map<string, LinkInfo>& linkInfoMap, const ResolvedJoint& joint)
{
    const xml_node& jointNode = joint.node;
    const string& jointName = joint.jointName;
    const string& parentName = joint.parentName;
    const string& childName = joint.childName;
    const string jointType = jointNode.attribute("type").as_string();

    // The existence of the parent and child links has already been verified by the caller.
    const LinkPtr parent = linkInfoMap.find(parentName)->second.link;
    const LinkPtr child = linkInfoMap.find(childName)->second.link;

    parent->appendChild(child);
    child->setJointName(jointName);

    if (jointType == "revolute" || jointType == "continuous") {
        child->setJointType(Link::RevoluteJoint);
        child->setJointId(jointCounter_++);
    } else if (jointType == "prismatic") {
        child->setJointType(Link::PrismaticJoint);
        child->setJointId(jointCounter_++);
    } else if (jointType == "fixed") {
        child->setJointType(Link::FixedJoint);
    } else if (jointType == "ball") {
        child->setJointType(Link::FreeJoint);
        os() << formatR(_("Warning: the ball joint \"{0}\" is treated as a free joint."), jointName)
             << endl;
    } else {
        // An unsupported joint type (e.g. universal, screw) is treated as a fixed joint so that
        // the rest of the model can still be loaded.
        child->setJointType(Link::FixedJoint);
        os() << formatR(_("Warning: the joint type \"{0}\" is not supported and the joint \"{1}\" "
                          "is treated as a fixed joint."), jointType, jointName) << endl;
    }

    // 'axis' element. Choreonoid stores the joint axis in the child link frame, so any axis
    // written in another frame must be rotated into the child link frame here. The frame in
    // which <axis><xyz> is expressed depends on the SDF version and on a few attributes:
    //   - SDF 1.6 and earlier: defaults to the joint frame. <use_parent_model_frame>true</...>
    //     selects the model frame instead.
    //   - SDF 1.7 and later: defaults to the child link frame. The 'expressed_in' attribute
    //     on <xyz> can select the model frame ("__model__") or another link frame.
    const xml_node axisNode = jointNode.child("axis");
    if (axisNode.empty()) {
        child->setJointAxis(Vector3::UnitZ());
    } else {
        const xml_node xyzNode = axisNode.child("xyz");
        Vector3 axis = Vector3::UnitZ();
        if (!xyzNode.empty()) {
            if (!toVector3(xyzNode.text().as_string(), axis)) {
                os() << formatR(_("Error: the axis of the joint \"{0}\" is written in an invalid "
                                  "format."), jointName) << endl;
                return false;
            }
            axis.normalize();
        }

        // Determines the frame in which the axis vector is expressed.
        enum AxisFrame { ChildLinkFrame, JointFrame, ModelFrame };
        AxisFrame axisFrame;
        const bool sdf17OrLater = (sdfMajor_ > 1) || (sdfMajor_ == 1 && sdfMinor_ >= 7);
        if (sdf17OrLater) {
            axisFrame = ChildLinkFrame;
            const string expressedIn = xyzNode.attribute("expressed_in").as_string();
            if (!expressedIn.empty()) {
                if (expressedIn == "__model__") {
                    axisFrame = ModelFrame;
                } else if (expressedIn != childName) {
                    os() << formatR(_("Warning: the axis of the joint \"{0}\" is expressed in "
                                      "the frame \"{1}\", which is not supported; the child "
                                      "link frame is assumed instead."),
                                    jointName, expressedIn) << endl;
                }
            }
        } else {
            axisFrame = JointFrame;
            const xml_node upmfNode = axisNode.child("use_parent_model_frame");
            if (!upmfNode.empty() && upmfNode.text().as_bool(false)) {
                axisFrame = ModelFrame;
            }
        }

        // Rotates the axis vector into the child link frame.
        if (axisFrame == ModelFrame) {
            const Isometry3& childModelPose = linkInfoMap.find(childName)->second.modelPose;
            axis = childModelPose.linear().transpose() * axis;
            axis.normalize();
        } else if (axisFrame == JointFrame) {
            // The joint <pose> is the joint frame expressed in the child link frame.
            Isometry3 jointPose;
            string jointPoseRelativeTo;
            if (!readPoseElement(jointNode, jointPose, jointPoseRelativeTo)) {
                return false;
            }
            if (!jointPoseRelativeTo.empty() && jointPoseRelativeTo != childName) {
                os() << formatR(_("Warning: the pose of the joint \"{0}\" is relative to the "
                                  "frame \"{1}\", which is not supported; the child link frame "
                                  "is assumed instead."),
                                jointName, jointPoseRelativeTo) << endl;
            }
            axis = jointPose.linear() * axis;
            axis.normalize();
        }

        child->setJointAxis(axis);

        // 'limit' element.
        const xml_node limitNode = axisNode.child("limit");
        if (!limitNode.empty()) {
            if (jointType == "revolute" || jointType == "prismatic") {
                // SDF often expresses an effectively unlimited joint (the equivalent of a
                // URDF "continuous" joint) as a "revolute" joint with very large limits such
                // as +/-1e16. Choreonoid represents an unlimited range with the canonical
                // value set by setUnlimitedJointDisplacementRange() (i.e. +/-DBL_MAX), which
                // Link::isUnlimitedRangeValue() recognizes downstream. Any limit whose
                // magnitude exceeds a value that no real joint would reach is normalized to
                // that canonical form.
                constexpr double UnlimitedThreshold = 1.0e6;
                constexpr double Inf = std::numeric_limits<double>::max();
                double lower = limitNode.child("lower").text().as_double(0.0);
                double upper = limitNode.child("upper").text().as_double(0.0);
                const bool lowerUnlimited = (lower <= -UnlimitedThreshold);
                const bool upperUnlimited = (upper >= UnlimitedThreshold);
                if (lowerUnlimited && upperUnlimited) {
                    child->setUnlimitedJointDisplacementRange();
                } else {
                    if (lowerUnlimited) lower = -Inf;
                    if (upperUnlimited) upper = Inf;
                    child->setJointRange(lower, upper);
                }
            }
            const xml_node velocityNode = limitNode.child("velocity");
            if (!velocityNode.empty()) {
                const double velocityLimit = velocityNode.text().as_double();
                if (velocityLimit > 0.0) {
                    child->setJointVelocityRange(-velocityLimit, velocityLimit);
                }
            }
            const xml_node effortNode = limitNode.child("effort");
            if (!effortNode.empty()) {
                const double effortLimit = effortNode.text().as_double();
                if (effortLimit > 0.0) {
                    child->setJointEffortRange(-effortLimit, effortLimit);
                }
            }
        }
    }

    return true;
}
