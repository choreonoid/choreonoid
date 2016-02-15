/**
   \file
   \author
*/

#include "SDFBodyLoader.h"
#include "SDFLoaderPseudoGazeboColor.h"
#include "SDFSensorConverter.h"
#include <cnoid/Sensor>
#include <cnoid/Camera>
#include <cnoid/RangeSensor>
#include <cnoid/Light>
#include <cnoid/STLSceneLoader>
#include <map>
#include <list>
#include <cnoid/SceneGraph>
#include <cnoid/SceneShape>
#include <cnoid/SceneDrawables>
#include <cnoid/DaeParser>
#include <cnoid/MeshGenerator>
#include <cnoid/NullOut>
#include <cnoid/MessageView>
#include <sdf/sdf.hh>
#include <sdf/parser_urdf.hh>
#include <boost/function.hpp>
#include <boost/dynamic_bitset.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

using namespace std;
using namespace boost;
using namespace cnoid;

namespace {

MeshGenerator meshGenerator;
    
double getLimitValue(double val, double defaultValue)
{
    if(val == 0){
        return defaultValue;
    }
    return val;
}

void setShape(Link* link, SgGroup* shape, bool isVisual)
{
    SgNodePtr node;
    if(shape->empty()){
        node = new SgNode;
    } else {
        // TODO: have to apply transform here when joint pose != link pose
        SgInvariantGroup* invariant = new SgInvariantGroup;
        if(link->Rs().isApprox(Matrix3::Identity())){
            shape->copyChildrenTo(invariant);
        } else {
            SgPosTransform* transformRs = new SgPosTransform;
            transformRs->setRotation(link->Rs());
            shape->copyChildrenTo(transformRs);
            invariant->addChild(transformRs);
        }
        node = invariant;
    }
    if(node){
        if(isVisual){
            link->setVisualShape(node);
        } else {
            link->setCollisionShape(node);
        }
    }
}

}


namespace cnoid {

class LinkInfo
{
public:
    std::string linkName;
    double m;
    sdf::Pose c;
    cnoid::Matrix3 I;
    cnoid::SgGroupPtr visualShape;
    cnoid::SgGroupPtr collisionShape;
    cnoid::Affine3 pose;
    LinkInfo() {
        visualShape = new SgGroup;
        collisionShape = new SgGroup;
    }
};
typedef boost::shared_ptr<LinkInfo> LinkInfoPtr;

class JointInfo
{
public:
    std::string jointName;
    std::string parentName;
    LinkInfoPtr parent;
    std::string childName;
    LinkInfoPtr child;
    std::string jointType;
    cnoid::Vector3 axis;
    double upper;
    double lower;
    double velocity;
    cnoid::Affine3 pose;
    JointInfo() {
    }
};
typedef boost::shared_ptr<JointInfo> JointInfoPtr;

class SDFBodyLoaderImpl
{
public:   
    Body* body;
    dynamic_bitset<> validJointIdSet;
    int numValidJointIds;
    ostream* os_;
    bool isVerbose;
    std::vector<JointInfoPtr> joints;
    std::map<std::string, LinkInfoPtr> linkdataMap;
    std::map<std::string, Link::JointType> jointTypeMap;

    ostream& os() { return *os_; }

    SDFBodyLoaderImpl();
    ~SDFBodyLoaderImpl();
    cnoid::Affine3 pose2affine(const sdf::Pose& pose);
    bool load(Body* body, const std::string& filename);
    BodyPtr load(const std::string& filename);        
    SgNodePtr readGeometry(sdf::ElementPtr geometry, SgMaterial* material, const sdf::Pose &pose);
    std::vector<JointInfoPtr> findRootJoints();
    std::vector<JointInfoPtr> findChildJoints(const std::string& linkName);
    void convertChildren(Link* plink, JointInfoPtr parent);

private:
    SDFLoaderPseudoGazeboColor* gazeboColor;
    SDFSensorConverter* sensorConverter;

    SgMaterial* createSgMaterial(sdf::ElementPtr material, float transparency);
    void loadDae(std::string url, SgPosTransformPtr transform, SgMaterialPtr material);
    void decideJointType(Link *link, const std::string name, const std::string type);
    void addModelSearchPath(const char *envname);
    void processShapes(LinkInfoPtr linkdata, sdf::ElementPtr link, bool isVisual);
    SgPosTransformPtr createSgPosTransform(const sdf::Pose &pose, bool isRotation = false);
};

}


std::vector<JointInfoPtr> SDFBodyLoaderImpl::findRootJoints()
{
    std::vector<JointInfoPtr> ret;
    std::map<std::string, JointInfoPtr> usedlinks;
    std::vector<JointInfoPtr>::iterator it;
    it = joints.begin();
    while(it != joints.end()){
        if(isVerbose){
            os() << "search for root joint (joint:" << (*it)->jointName
                 << ", parent:" << (*it)->parentName
                 << ", child:" << (*it)->childName << ")" << std::endl;
        }
        usedlinks[(*it)->parentName] = *it;
        usedlinks[(*it)->childName] = *it;
        it++;
    }
    it = joints.begin();
    while(it != joints.end()){
        usedlinks.erase((*it)->childName);
        it++;
    }
    std::map<std::string, JointInfoPtr>::iterator it2 = usedlinks.begin();
    while(it2 != usedlinks.end()){
        if(isVerbose){
            os() << "found root joint (joint:" << it2->second->jointName << ")" << std::endl;
        }
        ret.push_back(it2->second);
        it2++;
    }
    return ret;
}

std::vector<JointInfoPtr> SDFBodyLoaderImpl::findChildJoints(const std::string& linkName)
{
    std::vector<JointInfoPtr> ret;
    std::vector<JointInfoPtr>::iterator it = joints.begin();
    while(it != joints.end()){
        if(isVerbose){
            os() << "search for child joint of " << linkName
                 << " (joint:" << (*it)->jointName
                 << ", parent:" << (*it)->parentName
                 << ", child:" << (*it)->childName << ")" << std::endl;
        }
        if((*it)->parentName == linkName){
            if(isVerbose){
                os() << "found child joint (joint:" << (*it)->jointName << ")" << std::endl;
            }
            ret.push_back(*it);
        }
        it++;
    }
    return ret;
}

SDFBodyLoader::SDFBodyLoader()
{
    impl = new SDFBodyLoaderImpl();
}


SDFBodyLoaderImpl::SDFBodyLoaderImpl()
{
    isVerbose = false;
    body = 0;
    os_ = &nullout();

    jointTypeMap["revolute"] = Link::ROTATIONAL_JOINT;
    jointTypeMap["prismatic"] = Link::SLIDE_JOINT;
    jointTypeMap["fixed"] = Link::FIXED_JOINT;
#if 0    /* Unused */
    jointTypeMap[] = Link::FREE_JOINT;
    jointTypeMap[] = Link::CRAWLER_JOINT;
#endif   /* Unused */

    gazeboColor = new SDFLoaderPseudoGazeboColor;
    sensorConverter = new SDFSensorConverter;
}


SDFBodyLoader::~SDFBodyLoader()
{
    delete impl;
}


SDFBodyLoaderImpl::~SDFBodyLoaderImpl()
{

}


const char* SDFBodyLoader::format() const
{
    return "SDF";
}


void SDFBodyLoader::setMessageSink(std::ostream& os)
{
    impl->os_ = &os;
}


void SDFBodyLoader::setVerbose(bool on)
{
    impl->isVerbose = on;
}


bool SDFBodyLoader::load(BodyPtr body, const std::string& filename)
{
    return impl->load(body.get(), filename);
}

cnoid::Affine3 SDFBodyLoaderImpl::pose2affine(const sdf::Pose& pose)
{
    cnoid::Affine3 ret = cnoid::Affine3::Identity();
    cnoid::Vector3 trans;
    trans(0) = pose.pos.x;
    trans(1) = pose.pos.y;
    trans(2) = pose.pos.z;
    cnoid::Quaternion R;
    R.x() = pose.rot.x;
    R.y() = pose.rot.y;
    R.z() = pose.rot.z;
    R.w() = pose.rot.w;
    ret.translation() = trans;
    ret.linear() = R.matrix();

    return ret;
}

bool SDFBodyLoaderImpl::load(Body* body, const std::string& filename)
{
    bool result = false;

    this->body = body;
    joints.clear();
    linkdataMap.clear();
    validJointIdSet.clear();
    numValidJointIds = 0;
    os_ = &std::cout;
    
    try {
        sdf::SDFPtr robot(new sdf::SDF());
        sdf::init(robot);

        addModelSearchPath("HOME");
        addModelSearchPath("ROS_PACKAGE_PATH");
        addModelSearchPath("GAZEBO_MODEL_PATH");

        if (sdf::readFile(filename, robot) == false) {  // this can read both SDF and URDF
            throw std::invalid_argument("load failed");
        } else if (robot->root == NULL) {
            throw std::out_of_range("root element is NULL");
        }
        sdf::ElementPtr model = robot->root->GetElement("model");
        if (model == NULL) {
            throw std::out_of_range("model element is NULL");
        }
        std::string modelName = model->Get<std::string>("name");
        body->setModelName(modelName);

        if (isVerbose) {
            os() << "model name " << modelName << std::endl;
        }

        /*
        if(link->HasElement("static")){
            body->setStaticModel(link->Get<bool>("static"));
        }
        */
        
        LinkInfoPtr worldlink(new LinkInfo());
        worldlink->linkName = "world";
        worldlink->pose = Affine3::Identity();
        linkdataMap[worldlink->linkName] = worldlink;
        
        if(model->HasElement("link")){
            sdf::ElementPtr link = model->GetElement("link");
            while(link){
                LinkInfoPtr linkdata(new LinkInfo());
                linkdata->linkName = link->Get<std::string>("name");

                if (isVerbose) {
                    os() << " link name " << linkdata->linkName << std::endl;
                }

                if(link->HasElement("pose")){
                    linkdata->pose = pose2affine(link->Get<sdf::Pose>("pose"));
                } else {
                    linkdata->pose = Affine3::Identity();
                }
                if(link->HasElement("inertial")){
                    sdf::ElementPtr inertial = link->GetElement("inertial");
                    if(inertial->HasElement("inertia")) {
                        sdf::ElementPtr inertia = inertial->GetElement("inertia");
                        linkdata->I(0,0) = inertia->Get<double>("ixx");
                        linkdata->I(1,1) = inertia->Get<double>("iyy");
                        linkdata->I(2,2) = inertia->Get<double>("izz");
                        linkdata->I(0,1) = linkdata->I(1,0) = inertia->Get<double>("ixy");
                        linkdata->I(0,2) = linkdata->I(2,0) = inertia->Get<double>("ixz");
                        linkdata->I(1,2) = linkdata->I(2,1) = inertia->Get<double>("iyz");
                    }
                    if(inertial->HasElement("mass")){
                        linkdata->m = inertial->Get<double>("mass");
                    }
                    if(inertial->HasElement("pose")){
                        linkdata->c = inertial->Get<sdf::Pose>("pose");
                    }
                }
                if(link->HasElement("visual")) {
                    processShapes(linkdata, link, true);
                }
                if(link->HasElement("collision")){
                    processShapes(linkdata, link, false);
                }

                sensorConverter->convert(linkdata->linkName, link);

                linkdataMap[linkdata->linkName] = linkdata;
                link = link->GetNextElement("link");
            }
        }
        if(model->HasElement("joint")){
            sdf::ElementPtr joint = model->GetElement("joint");
            while(joint){
                JointInfoPtr jointdata(new JointInfo());
                jointdata->jointName = joint->Get<std::string>("name");
                jointdata->jointType = joint->Get<std::string>("type");
                jointdata->parentName = joint->Get<std::string>("parent");
                jointdata->childName = joint->Get<std::string>("child");
                jointdata->parent = linkdataMap[jointdata->parentName];
                jointdata->child = linkdataMap[jointdata->childName];

                if (isVerbose) {
                    os() << " joint name " << jointdata->jointName << std::endl;
                }

                if(joint->HasElement("pose")){
                    jointdata->pose = pose2affine(joint->Get<sdf::Pose>("pose"));
                    jointdata->pose = jointdata->child->pose * jointdata->pose;
                } else {
                    jointdata->pose = jointdata->child->pose;
                }
                if(isVerbose){
                    os() << jointdata->pose.matrix() << std::endl;
                }
                if(joint->HasElement("axis")){
                    sdf::ElementPtr axis = joint->GetElement("axis");
                    sdf::Vector3 xyz = axis->Get<sdf::Vector3>("xyz");
                    jointdata->axis[0] = xyz.x;
                    jointdata->axis[1] = xyz.y;
                    jointdata->axis[2] = xyz.z;
                    
                    // convert axis relative to child or parent frame
                    // https://github.com/yosuke/simtrans/blob/master/simtrans/sdf.py#L198
                    bool useparent = false;
                    if(axis->HasElement("use_parent_model_frame")){
                        useparent = axis->Get<bool>("use_parent_model_frame");
                    }
                    cnoid::Affine3 work = cnoid::Affine3::Identity();
                    cnoid::Affine3 work2 = cnoid::Affine3::Identity();
                    work.translation() = jointdata->axis;
                    if(useparent){
                        work2.linear() = jointdata->parent->pose.linear();
                    } else {
                        work2.linear() = jointdata->child->pose.linear();
                    }
                    work = work2.inverse() * work;
                    jointdata->axis = work.translation();
                    jointdata->axis.normalize();
                    
                    if(axis->HasElement("limit")){
                        sdf::ElementPtr limit = axis->GetElement("limit");
                        jointdata->lower = limit->Get<double>("lower");
                        jointdata->upper = limit->Get<double>("upper");
                        if(limit->HasElement("velocity")){
                            jointdata->velocity = limit->Get<double>("velocity");
                        }
                    } else {
                        /*
                         * corrupted? (axis element must be have limit element)
                         * set values use cnoid::Link constructor's value.
                         */
                        jointdata->lower = -std::numeric_limits<double>::max();
                        jointdata->upper = std::numeric_limits<double>::max();
                        os() << "Warning: '" << jointdata->jointName << "' limit not found" << std::endl;
                    }
                }

                sensorConverter->convert(jointdata->jointName, joint);

                joints.push_back(jointdata);
                joint = joint->GetNextElement("joint");
            }
        }

        // create at least one joint if no joint is specified in the file
        if(joints.size() == 0 && linkdataMap.size() > 0){
            JointInfoPtr j(new JointInfo());
            j->parent = linkdataMap.begin()->second;
            j->jointName = j->parentName = j->parent->linkName;
            joints.push_back(j);
        }

        // construct tree structure of joints and links
        vector<JointInfoPtr> roots = findRootJoints();
        std::vector<JointInfoPtr>::iterator it = roots.begin();
        while(it != roots.end()){
            if (isVerbose) {
                os() << "found root link " << (*it)->parentName << std::endl;
            }

            Link* link = body->createLink();
            link->setName((*it)->parentName);
            sensorConverter->setRootLinkName(link->name());
            JointInfoPtr root;
            if((*it)->parentName == "world"){
                decideJointType(link, (*it)->jointName, (*it)->jointType);

                if (link->isFixedJoint() == false) {
                    link->setJointType(Link::FREE_JOINT);
                }

                root = *it;
                link->Tb() = root->pose;
            }else{
                link->setJointType(Link::FREE_JOINT);
                root.reset(new JointInfo());
                root->jointName = (*it)->jointName;
                root->pose = Affine3::Identity();
                root->childName = (*it)->parentName;
                root->child = (*it)->parent;
            }
            if (root->child != NULL) {
                link->setMass(root->child->m);
                link->setCenterOfMass(Vector3(root->child->c.pos.x, root->child->c.pos.y, root->child->c.pos.z));
                link->setInertia(root->child->I);
                setShape(link, root->child->visualShape, true);
                setShape(link, root->child->collisionShape, false);
            }
            link->setJointAxis(Vector3::Zero());
            convertChildren(link, root);
            body->setRootLink(link);
            it++;
        }

        // attach sensors
#if (DEBUG_SDF_SENSOR_CONVERTER > 0) /* (DEBUG_SDF_SENSOR_CONVERTER > 0) */
        sensorConverter->dumpDeviceMap();
#endif                               /* (DEBUG_SDF_SENSOR_CONVERTER > 0) */
        sensorConverter->attachDevices(body);

        result = true;
        os().flush();
    } catch(const std::exception& ex){
        os() << "Error: " << ex.what() << endl;
    }
    
    return result;
}

void SDFBodyLoaderImpl::convertChildren(Link* plink, JointInfoPtr parent)
{
    vector<JointInfoPtr> children = findChildJoints(parent->childName);
    vector<JointInfoPtr>::iterator it = children.begin();

    while(it != children.end()){
        if((*it)->child == 0){
            it++;
            continue;
        }
        if(isVerbose){
            os() << "convert children from: " << parent->childName << std::endl;
            os() << "                   to: " << (*it)->childName << std::endl;
            os() << parent->pose.matrix() << std::endl;
            os() << (*it)->child->pose.matrix() << std::endl;
        }

        Link* link = body->createLink();
        link->setName((*it)->jointName);
        sensorConverter->mergeInfo((*it)->childName, (*it)->jointName);

        // convert to relative position
        link->setOffsetTranslation((*it)->child->pose.translation()
                                   - parent->pose.translation());
        link->setAccumulatedSegmentRotation((*it)->child->pose.linear());

        link->setMass((*it)->child->m);
        link->setCenterOfMass(link->Rs()*Vector3((*it)->child->c.pos.x, (*it)->child->c.pos.y, (*it)->child->c.pos.z));
        link->setInertia(link->Rs()*(*it)->child->I*link->Rs().transpose());

        setShape(link, (*it)->child->visualShape, true);
        setShape(link, (*it)->child->collisionShape, false);

        // joint setting
        decideJointType(link, (*it)->jointName, (*it)->jointType);

        if (link->isFreeJoint() == true || link->isFixedJoint() == true) {
            link->setJointAxis(Vector3::Zero());
        } else {
            link->setJointAxis(link->Rs() * (*it)->axis);
            double maxlimit = numeric_limits<double>::max();
            link->setJointRange((*it)->lower, (*it)->upper);
            link->setJointVelocityRange(getLimitValue(-(*it)->velocity, -maxlimit),
                                        getLimitValue((*it)->velocity, +maxlimit));
        }

        // jointid setting
        if (link->jointId() < 0) {
            link->setJointId(numValidJointIds);

            if (link->jointId() >= validJointIdSet.size()) {
                validJointIdSet.resize(link->jointId() + 1);
            }
            if (! validJointIdSet[ link->jointId() ]) {
                ++numValidJointIds;
                validJointIdSet.set(link->jointId());
            } else {
                os() << str(format("Warning: Joint ID %1% is duplicated.") % link->jointId()) << std::endl;
            }
        }

        plink->appendChild(link);
        convertChildren(link, *it);
        it++;
    }
}

SgNodePtr SDFBodyLoaderImpl::readGeometry(sdf::ElementPtr geometry, SgMaterial* material, const sdf::Pose &pose)
{
    SgNodePtr converted = 0;

    for(sdf::ElementPtr el = geometry->GetFirstElement(); el; el = el->GetNextElement()) {
        if(el->GetName() == "mesh") {
            std::string url = sdf::findFile(el->Get<std::string>("uri"));
            SgPosTransformPtr transform = createSgPosTransform(pose);

            if (! url.empty()) {
                if (isVerbose) {
                    os() << "     read mesh " << url << std::endl;
                }

                if (boost::algorithm::iends_with(url, "dae")) {
                    // TODO: might be better to use assimp here instead of cnoid::DaeParser
                    loadDae(url, transform, material);
                } else if (boost::algorithm::iends_with(url, "stl")) {
                    STLSceneLoader loader;
                    SgShapePtr shape = dynamic_cast<SgShape*>(loader.load(url));

                    if (shape != NULL) {
                        if (material != NULL) {
                            shape->setMaterial(material);
                        }

                        transform->addChild(shape);
                    }
                }
            } else {
                MessageView::instance()->putln(
                    MessageView::WARNING,
                    boost::format("%1% not found") % el->Get<std::string>("uri")
                    );
            }

            converted = transform;
        } else if(el->GetName() == "box"){
            SgShapePtr shape = new SgShape;
            sdf::Vector3 size = el->Get<sdf::Vector3>("size");
            SgPosTransformPtr transform = createSgPosTransform(pose, true);

            if (isVerbose) {
                os() << "     generate " << el->GetName() << " shape (x=" << size.x << " y=" << size.y
                     << " z=" << size.z << ")" << std::endl;
            }

            shape->setMesh(meshGenerator.generateBox(Vector3(size.x, size.y, size.z)));

            if (material != NULL) {
                shape->setMaterial(material);
            }

            transform->addChild(shape);
            converted = transform;
        } else if(el->GetName() == "sphere"){
            SgShapePtr shape = new SgShape;
            double radius = el->Get<double>("radius");
            SgPosTransformPtr transform = createSgPosTransform(pose, true);

            if (isVerbose) {
                os() << "     generate " << el->GetName() << " shape (radius=" << radius << ")" << std::endl;
            }

            shape->setMesh(meshGenerator.generateSphere(radius));

            if (material != NULL) {
                shape->setMaterial(material);
            }

            transform->addChild(shape);
            converted = transform;
        } else if(el->GetName() == "cylinder"){
            SgShapePtr shape = new SgShape;
            double radius = el->Get<double>("radius");
            double length = el->Get<double>("length");
            SgPosTransformPtr transform = createSgPosTransform(pose, true);

            if (isVerbose) {
                os() << "     generate " << el->GetName() << " shape (radius=" << radius << " length="
                     << length << ")" << std::endl;
            }

            shape->setMesh(meshGenerator.generateCylinder(radius, length));

            if (material != NULL) {
                shape->setMaterial(material);
            }

            transform->addChild(shape);
            converted = transform;
        } else {
            os() << "Warning: unsupported SDF node type " << el->GetName() << std::endl;
        }
    }

    return converted;
}

/**
 */
SgMaterial* SDFBodyLoaderImpl::createSgMaterial(sdf::ElementPtr material, float transparency)
{
    SgMaterial* ret;
    SDFLoaderPseudoGazeboColorInfo* cinfo;
    sdf::ElementPtr p;
    sdf::Color color;
    Vector3f v;

    ret = NULL;
    cinfo = NULL;

    if (material != NULL) {
        if (material->HasElement("script")) {
            p = material->GetElement("script");

            /* script element must be have name element */
            if (p->HasElement("name")) {
                cinfo = gazeboColor->get(p->Get<std::string>("name"));

                if (cinfo != NULL) {
                    ret = cinfo->material;
                }
            }
        }

        if (cinfo == NULL) {
            cinfo = new SDFLoaderPseudoGazeboColorInfo;
            ret = cinfo->material;
        }

        if (material->HasElement("ambient") && cinfo->isSettingAmbient == false) {
            color = material->Get<sdf::Color>("ambient");
            ret->setAmbientIntensity((color.r + color.g + color.b) / 3.0f);
        }

        if (material->HasElement("diffuse") && cinfo->isSettingDiffuse == false) {
            color = material->Get<sdf::Color>("diffuse");
            v << color.r, color.g, color.b;
            ret->setDiffuseColor(v);
        }

        if (material->HasElement("specular") && cinfo->isSettingSpecular == false) {
            color = material->Get<sdf::Color>("specular");
            v << color.r, color.g, color.b;
            ret->setSpecularColor(v);
        }

        if (material->HasElement("emissive") && cinfo->isSettingEmissive == false) {
            color = material->Get<sdf::Color>("emissive");
            v << color.r, color.g, color.b;
            ret->setEmissiveColor(v);
        }

        if (transparency >= 0.0) {
            ret->setTransparency(transparency);
        }
    }

    return ret;
}

/**
 */
void SDFBodyLoaderImpl::loadDae(std::string url, SgPosTransformPtr transform, SgMaterialPtr material)
{
    SgGroupPtr p;
    SgGroup::iterator it;
    SgShapePtr shape;

    if (transform == NULL) {
        return;
    }

    DaeParser parser(&os());

    if ((p = parser.createScene(url)) != NULL) {
        if (material != NULL) {
            for (it = p->begin(); it != p->end(); it++) {
                shape = dynamic_cast<SgShape*>((*it).get());

                if (shape != NULL) {
                    shape->setMaterial(material);
                }
            }
        }

        transform->addChild(p);
    }

    return;
}

/**
   @brief Decide joint type.
   @param[in/out] link Instance of Link.
   @param[in] name Joint name.
   @param[in] type Joint type of SDF.
 */
void SDFBodyLoaderImpl::decideJointType(Link *link, const std::string name, const std::string type)
{
    std::map<std::string, Link::JointType>::iterator it;

    if (link != NULL) {
        it = jointTypeMap.find(type);

        if (it != jointTypeMap.end()) {
            link->setJointType(it->second);
        } else {
            os() << "Warning: unable to handle joint type " << type << " of joint "
                 << name << " assume as fixed joint." << std::endl;
            link->setJointType(Link::FIXED_JOINT);
        }
    }

    return;
}

/**
   @brief Create instance of SgPosTransform.
   @param[in] pose Shape's pose.
   @param[in] isRotation Set true, 90 degree rotation of X axes. (default false)
              Choreonoid MeshGenerator/VRML     URDF/SDF
                         Y                          Z
                         |                          |
                         +- X                       +- Y
                        /                          /
                       Z                          X
   @return Instance of SgPosTransform.
 */
SgPosTransformPtr SDFBodyLoaderImpl::createSgPosTransform(const sdf::Pose &pose, bool isRotation)
{
    cnoid::SgPosTransformPtr ret = new SgPosTransform(pose2affine(pose));

    if (isRotation) {
        cnoid::AngleAxis aaX(0.5 * M_PI, cnoid::Vector3::UnitX());
        ret->setRotation(aaX);
    }

    return ret;
}

/**
   @brief Add model search path from environment variable.
   @param[in] envname Environment variable name.
   special process by set 'HOME'. (add ${HOME}/.gazebo/models)
 */
void SDFBodyLoaderImpl::addModelSearchPath(const char *envname)
{
    std::list<std::string> paths;
    std::string path;
    char *p;

    if (envname != NULL && (p = getenv(envname)) != NULL) {
        if (envname != "HOME") {
            boost::split(paths, p, boost::is_any_of(":"));
            BOOST_FOREACH(path, paths) {
                if (path != "") {
                    sdf::addURIPath("model://", path);
                }
            }
        } else {
            path = p;
            sdf::addURIPath("model://", path + "/.gazebo/models");
        }
    }

    return;
}

/**
   @brief Process shapes.
   @param[in] linkdata Created links.
   @param[in] link Target link.
   @param[in] isVisual Set true is visual, false is collision.
 */
void SDFBodyLoaderImpl::processShapes(LinkInfoPtr linkdata, sdf::ElementPtr link, bool isVisual)
{
    cnoid::SgGroupPtr shape;
    std::string       type;
    sdf::Pose         pose;
    sdf::ElementPtr   geometry;
    sdf::ElementPtr   material;
    float             transparency;
    SgMaterial*       sgmaterial;
    sdf::ElementPtr   p;

    if (linkdata == NULL || link == NULL) {
        return;
    }

    if (isVisual) {
        shape = linkdata->visualShape;
        type = "visual";
    } else {
        shape = linkdata->collisionShape;
        type = "collision";
    }

    if (link->HasElement(type) == false) {
        return;
    }

    p = link->GetElement(type);

    while (p) {
        if (isVerbose) {
            os() << "  " << type << " name: " << p->Get<std::string>("name") << std::endl;
        }

        if (p->HasElement("pose")) {
            pose = p->Get<sdf::Pose>("pose");
        } else {
            pose.pos.x = pose.pos.y = pose.pos.z = pose.rot.w = pose.rot.x = pose.rot.y = pose.rot.z = 0.0;
        }

        transparency = (isVisual && p->HasElement("transparency")) ? p->Get<float>("transparency") : -1.0;
        material = (isVisual && p->HasElement("material")) ? p->GetElement("material") : sdf::ElementPtr();

        if (p->HasElement("geometry")) {
            geometry = p->GetElement("geometry");
            sgmaterial = createSgMaterial(material, transparency);
            shape->addChild(readGeometry(geometry, sgmaterial, pose));
        } else {
            // corrupted? (geometry must be set)
            os() << "Warning: '" << p->Get<std::string>("name") << "' geometry not found" << std::endl;
        }

        p = p->GetNextElement(type);
    }

    return;
}
