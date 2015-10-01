/**
   \file
   \author
*/

#include "SDFBodyLoader.h"
#include <cnoid/Sensor>
#include <cnoid/Camera>
#include <cnoid/RangeSensor>
#include <cnoid/Light>
#include <cnoid/STLSceneLoader>
#include <map>
#include <list>
#include <cnoid/DaeParser>
#include <cnoid/MeshGenerator>
#include <cnoid/NullOut>
#include <sdf/sdf.hh>
#include <sdf/parser_urdf.hh>
#include <boost/function.hpp>
#include <boost/format.hpp>
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
    sdf::Pose pose;
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
    Vector3 axis;
    double upper;
    double lower;
    double velocity;
    sdf::Pose pose;
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
    bool load(Body* body, const std::string& filename);
    BodyPtr load(const std::string& filename);        
    SgNode* readGeometry(sdf::ElementPtr geometry, const sdf::Pose &pose);
    std::vector<JointInfoPtr> findRootJoints();
    std::vector<JointInfoPtr> findChildJoints(const std::string& linkName);
    void convertChildren(Link* plink, JointInfoPtr parent);
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
    //jointTypeMap[] = Link::FREE_JOINT;
    jointTypeMap["fixed"] = Link::FIXED_JOINT;
    //jointTypeMap[] = Link::CRAWLER_JOINT;
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
    body->clearDevices();
    body->clearExtraJoints();
    return impl->load(body.get(), filename);
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
    isVerbose = true;
    
    try {
        sdf::SDFPtr robot(new sdf::SDF());
        sdf::init(robot);
        char* homePath = getenv("HOME");
        std::string home = homePath;
        sdf::addURIPath("model://", home + "/.gazebo/models");
        char* rosPackagePath = getenv("ROS_PACKAGE_PATH");
        if(rosPackagePath != NULL){
            std::list<std::string> rospkgs;
            boost::split(rospkgs, rosPackagePath, boost::is_any_of(":"));
            BOOST_FOREACH(std::string p, rospkgs){
                sdf::addURIPath("model://", p);
            }
        }
        sdf::readFile(filename, robot);  // this can read both SDF and URDF
        sdf::ElementPtr model = robot->root->GetElement("model");
        std::string modelName = model->Get<std::string>("name");
        os() << "model name " << modelName << std::endl;
        body->setModelName(modelName);
        if(model->HasElement("link")){
            sdf::ElementPtr link = model->GetElement("link");
            while(link){
                LinkInfoPtr linkdata(new LinkInfo());
                linkdata->linkName = link->Get<std::string>("name");
                os() << " link name " << linkdata->linkName << std::endl;
                if(link->HasElement("pose")){
                    linkdata->pose = link->Get<sdf::Pose>("pose");
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
                    sdf::ElementPtr visual = link->GetElement("visual");
                    sdf::Pose pose;
                    if(visual->HasElement("pose")){
                        pose = visual->Get<sdf::Pose>("pose");
                    }
                    if(visual->HasElement("geometry")){
                        sdf::ElementPtr geometry = visual->GetElement("geometry");
                        linkdata->visualShape->addChild(readGeometry(geometry, pose));
                    }
                }
                if(link->HasElement("collision")){
                    sdf::ElementPtr collision = link->GetElement("collision");
                    sdf::Pose pose;
                    if(collision->HasElement("pose")){
                        pose = collision->Get<sdf::Pose>("pose");
                    }
                    if(collision->HasElement("geometry")){
                        sdf::ElementPtr geometry = collision->GetElement("geometry");
                        linkdata->collisionShape->addChild(readGeometry(geometry, pose));
                    }
                }
                linkdataMap[linkdata->linkName] = linkdata;
                link = link->GetNextElement("link");
            }
        }
        if(model->HasElement("joint")){
            sdf::ElementPtr joint = model->GetElement("joint");
            while(joint){
                JointInfoPtr jointdata(new JointInfo());
                jointdata->jointName = joint->Get<std::string>("name");
                os() << " joint name " << jointdata->jointName << std::endl;
                jointdata->jointType = joint->Get<std::string>("type");
                jointdata->parentName = joint->Get<std::string>("parent");
                jointdata->childName = joint->Get<std::string>("child");
                jointdata->parent = linkdataMap[jointdata->parentName];
                jointdata->child = linkdataMap[jointdata->childName];
                jointdata->pose = joint->Get<sdf::Pose>("pose");
                if(joint->HasElement("axis")){
                    sdf::ElementPtr axis = joint->GetElement("axis");
                    sdf::Vector3 xyz = axis->Get<sdf::Vector3>("xyz");
                    jointdata->axis[0] = xyz.x;
                    jointdata->axis[1] = xyz.y;
                    jointdata->axis[2] = xyz.z;
                    if(axis->HasElement("limit")){
                        sdf::ElementPtr limit = axis->GetElement("limit");
                        jointdata->lower = limit->Get<double>("lower");
                        jointdata->upper = limit->Get<double>("upper");
                        if(limit->HasElement("velocity")){
                            jointdata->velocity = limit->Get<double>("velocity");
                        }
                    }
                }
                joints.push_back(jointdata);
                joint = joint->GetNextElement("joint");
            }
        }
        if(joints.size() == 0 && linkdataMap.size() > 0){
            JointInfoPtr j(new JointInfo());
            j->parent = linkdataMap.begin()->second;
            j->jointName = j->parentName = j->parent->linkName;
            joints.push_back(j);
        }

        vector<JointInfoPtr> roots = findRootJoints();
        std::vector<JointInfoPtr>::iterator it = roots.begin();
        while(it != roots.end()){
            os() << "found root link " << (*it)->parentName << std::endl;
            Link* link = body->createLink();
            link->setName(modelName);
            JointInfoPtr root;
            if((*it)->parentName == "world"){
                link->setJointType(Link::FIXED_JOINT);
                root = *it;
                Vector3 pos;
                pos(0) = root->pose.pos.x;
                pos(1) = root->pose.pos.y;
                pos(2) = root->pose.pos.z;
                Quat R;
                R.x() = root->pose.rot.x;
                R.y() = root->pose.rot.y;
                R.z() = root->pose.rot.z;
                R.w() = root->pose.rot.w;
                link->setOffsetTranslation(pos);
                link->setOffsetRotation(R.matrix());
            }else{
                link->setJointType(Link::FREE_JOINT);
                root.reset(new JointInfo());
                root->jointName = modelName;
                root->childName = (*it)->parentName;
                root->child = (*it)->parent;
            }
            setShape(link, root->child->visualShape, true);
            setShape(link, root->child->collisionShape, false);
            link->setJointAxis(Vector3::Zero());
            convertChildren(link, root);
            body->setRootLink(link);
            it++;
        }
        result = true;
        os().flush();
    } catch(const std::exception& ex){
        os() << ex.what() << endl;
    }
    
    return result;
}

void SDFBodyLoaderImpl::convertChildren(Link* plink, JointInfoPtr parent)
{
    vector<JointInfoPtr> children = findChildJoints(parent->childName);
    std::vector<JointInfoPtr>::iterator it = children.begin();
    while(it != children.end()){
        if((*it)->child == 0){
            it++;
            continue;
        }
        Link* link = body->createLink();
        link->setName((*it)->jointName);
        link->setMass((*it)->child->m);
        link->setCenterOfMass(Vector3((*it)->child->c.pos.x, (*it)->child->c.pos.y, (*it)->child->c.pos.z));
        link->setInertia((*it)->child->I);

        // convert to relative position (ref to gazebo/math/Pose.hh)
        sdf::Quaternion tmp;
        tmp.x = (*it)->child->pose.pos.x - (*it)->parent->pose.pos.x;
        tmp.y = (*it)->child->pose.pos.y - (*it)->parent->pose.pos.y;
        tmp.z = (*it)->child->pose.pos.z - (*it)->parent->pose.pos.z;
        tmp.w = 0;
        tmp = (*it)->parent->pose.rot.GetInverse() * (tmp * (*it)->child->pose.rot);
        sdf::Quaternion tmp2;
        //tmp2 = (*it)->parent->pose.rot.GetInverse() * (*it)->child->pose.rot;
        tmp2 = (*it)->child->pose.rot;
    
        Vector3 trans;
        trans(0) = tmp.x;
        trans(1) = tmp.y;
        trans(2) = tmp.z;
        Quat R;
        R.x() = tmp2.x;
        R.y() = tmp2.y;
        R.z() = tmp2.z;
        R.w() = tmp2.w;
        link->setOffsetTranslation(trans);
        link->setOffsetRotation(R.matrix());
        setShape(link, (*it)->child->visualShape, true);
        setShape(link, (*it)->child->collisionShape, false);
        
        std::map<std::string, Link::JointType>::iterator it2;
        it2 = jointTypeMap.find((*it)->jointType);
        if(it2 != jointTypeMap.end()){
            link->setJointType(it2->second);
        } else {
            os() << " unable to handle joint type " << (*it)->jointType << " of joint " << (*it)->jointName << " assume as fixed joint." << std::endl;
            link->setJointType(Link::FIXED_JOINT);
        }
        if(link->jointType() == Link::FREE_JOINT || link->jointType() == Link::FIXED_JOINT){
            link->setJointAxis(Vector3::Zero());
        } else {
            link->setJointAxis(link->Rs() * (*it)->axis);
            double maxlimit = numeric_limits<double>::max();
            link->setJointRange(getLimitValue((*it)->lower, -maxlimit),
                                getLimitValue((*it)->upper, +maxlimit));
            link->setJointVelocityRange(getLimitValue(-(*it)->velocity, -maxlimit),
                                        getLimitValue((*it)->velocity, +maxlimit));
        }
        plink->appendChild(link);
        convertChildren(link, *it);
        it++;
    }
}

SgNode* SDFBodyLoaderImpl::readGeometry(sdf::ElementPtr geometry, const sdf::Pose &pose)
{
    SgNode* converted = 0;

    for(sdf::ElementPtr el = geometry->GetFirstElement(); el;
        el = el->GetNextElement()) {
        if(el->GetName() == "mesh") {
            std::string url = sdf::findFile(el->Get<std::string>("uri"));
            SgPosTransform* transform = new SgPosTransform;
            Vector3 trans;
            trans(0) = pose.pos.x;
            trans(1) = pose.pos.y;
            trans(2) = pose.pos.z;
            Quat R;
            R.x() = pose.rot.x;
            R.y() = pose.rot.y;
            R.z() = pose.rot.z;
            R.w() = pose.rot.w;
            transform->setTranslation(trans);
            transform->setRotation(R.matrix());
            os() << "  read mesh " << url << std::endl;
            if (boost::algorithm::iends_with(url, "dae")) {
                DaeParser parser(&os());
                transform->addChild(parser.createScene(url));
            } else if (boost::algorithm::iends_with(url, "stl")) {
                STLSceneLoader loader;
                transform->addChild(loader.load(url));
            }
            converted = transform;
        } else if(el->GetName() == "box"){
            SgShape* shape = new SgShape;
            sdf::Vector3 size = el->Get<sdf::Vector3>("size");
            shape->setMesh(meshGenerator.generateBox(Vector3(size.x, size.y, size.z)));
            converted = shape;
        } else if(el->GetName() == "sphere"){
            SgShape* shape = new SgShape;
            double radius = el->Get<double>("radius");
            shape->setMesh(meshGenerator.generateSphere(radius));
            converted = shape;
        } else if(el->GetName() == "cylinder"){
            SgShape* shape = new SgShape;
            double radius = el->Get<double>("radius");
            double length = el->Get<double>("length");
            shape->setMesh(meshGenerator.generateCylinder(radius, length));
            converted = shape;
        } else {
            os() << "unsupported SDF node type " << el->GetName() << std::endl;
        }
    }

    return converted;
}


