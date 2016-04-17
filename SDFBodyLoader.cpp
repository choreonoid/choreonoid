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
#include <cassert>
#include <cnoid/SceneGraph>
#include <cnoid/SceneShape>
#include <cnoid/SceneDrawables>
#include <cnoid/DaeParser>
#include <cnoid/MeshGenerator>
#include <cnoid/NullOut>
#include <cnoid/ImageIO>
#include <cnoid/Exception>
#include <cnoid/MessageView>
#include <sdf/sdf.hh>
#include <sdf/parser_urdf.hh>
#include <boost/function.hpp>
#include <boost/dynamic_bitset.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

#if defined(ASSIMP_UNIFIED_HEADER_NAMES)
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <assimp/IOStream.hpp>
#include <assimp/IOSystem.hpp>
#else
#include <assimp/assimp.hpp>
#include <assimp/aiScene.h>
#include <assimp/aiPostProcess.h>
#include <assimp/IOStream.h>
#include <assimp/IOSystem.h>
#endif

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
        /*
          initialize value by Choreonoid's constructor value.
          see Choreonoid src/Body/Link.cpp Link::Link().
         */
        m = 0.0;
        c = sdf::Pose();
        I.setZero();
        visualShape = new SgGroup;
        collisionShape = new SgGroup;
        pose = cnoid::Affine3::Identity();
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
        /*
          initialize value by Choreonoid's constructor value.
          see Choreonoid src/Body/Link.cpp Link::Link().
         */
        parent = LinkInfoPtr();
        child = LinkInfoPtr();
        axis = Vector3::UnitZ();
        upper = std::numeric_limits<double>::max();
        lower = -std::numeric_limits<double>::max();
        velocity = 0.0;
        pose = cnoid::Affine3::Identity();
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
    typedef std::map<std::string, SgImagePtr> ImagePathToSgImageMap;
    ImagePathToSgImageMap imagePathToSgImageMap;

    ostream& os() { return *os_; }

    SDFBodyLoaderImpl();
    ~SDFBodyLoaderImpl();
    cnoid::Affine3 pose2affine(const sdf::Pose& pose);
    bool load(Body* body, const std::string& filename);
    BodyPtr load(const std::string& filename);        
    SgNodePtr readGeometry(sdf::ElementPtr geometry, SgMaterial* material, const cnoid::Affine3 &pose);
    std::vector<std::string> findRootLinks();
    bool hasOpenLink(const std::string& linkName);
    std::vector<JointInfoPtr> findChildJoints(const std::string& linkName);
    std::vector<JointInfoPtr> findParentJoints(const std::string& linkName);
    void convertChildren(Link* plink, JointInfoPtr parent);

private:
    SDFLoaderPseudoGazeboColor* gazeboColor;
    SDFSensorConverter* sensorConverter;

    void buildMesh(const aiScene* scene, const aiNode* node, SgTransform* sgnode,
                   std::vector<SgMaterial*>& material_table,
                   std::vector<SgTexture*>& texture_table);
    void loadMaterials(const std::string& resource_path, const aiScene* scene,
                       std::vector<SgMaterial*>& material_table,
                       std::vector<SgTexture*>& texture_table);
    void getDaeScaleAndAxis(const std::string& url, float& scale, std::string& axis);
    SgMaterial* createSgMaterial(sdf::ElementPtr material, float transparency);
    void convertJointType(Link *link, JointInfoPtr info);
    void addModelSearchPath(const char *envname);
    void readShapes(cnoid::SgGroup* shapes, sdf::ElementPtr link, const std::string tagname);
};

}


std::vector<std::string> SDFBodyLoaderImpl::findRootLinks()
{
    std::vector<std::string> ret;
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
        if(hasOpenLink(it2->first)){
            if(isVerbose){
                os() << "found root joint (joint:" << it2->first << ")" << std::endl;
            }
            ret.push_back(it2->first);
        }
        it2++;
    }
    return ret;
}

bool SDFBodyLoaderImpl::hasOpenLink(const std::string& linkName)
{
    vector<JointInfoPtr> children = findChildJoints(linkName);
    std::vector<JointInfoPtr>::iterator it = children.begin();
    while(it != children.end()){
        std::map<std::string, bool> parentnames;
        vector<JointInfoPtr> parents = findParentJoints((*it)->childName);
        std::vector<JointInfoPtr>::iterator it2 = parents.begin();
        while(it2 != parents.end()){
            parentnames[(*it2)->parentName] = true;
            it2++;
        }
        if(parentnames.size() == 1) return true;
        it++;
    }
    return false;
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

std::vector<JointInfoPtr> SDFBodyLoaderImpl::findParentJoints(const std::string& linkName)
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
        if((*it)->childName == linkName){
            if(isVerbose){
                os() << "found parent joint (joint:" << (*it)->jointName << ")" << std::endl;
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


bool SDFBodyLoader::load(Body* body, const std::string& filename)
{
    return impl->load(body, filename);
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
                    readShapes(linkdata->visualShape, link, "visual");
                }
                if(link->HasElement("collision")){
                    readShapes(linkdata->collisionShape, link, "collision");
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
                        // corrupted? (axis element must be have limit element)
                        os() << "Warning: '" << jointdata->jointName << "' limit not found" << std::endl;
                    }
                }

                sensorConverter->convert(jointdata->jointName, joint);

                joints.push_back(jointdata);
                joint = joint->GetNextElement("joint");
            }
        }

        // create fixed joint if link is not referred by joint at all
        if(linkdataMap.size() > 0){
            std::map<std::string, bool> usedlinks;
            std::vector<JointInfoPtr>::iterator it = joints.begin();
            while(it != joints.end()){
                usedlinks[(*it)->parentName] = true;
                usedlinks[(*it)->childName] = true;
                it++;
            }
            std::map<std::string, LinkInfoPtr>::const_iterator it2 = linkdataMap.begin();
            while(it2 != linkdataMap.end()){
                if (it2->second->linkName != "world" && usedlinks.find(it2->second->linkName) == usedlinks.end()) {
                    JointInfoPtr j(new JointInfo());
                    j->parent = worldlink;
                    j->child = it2->second;
                    j->parentName = j->parent->linkName;
                    j->jointName = j->childName = j->child->linkName;
                    j->jointType = "revolute";
                    j->upper = j->lower = 0.0;
                    joints.push_back(j);
                }
                it2++;
            }
        }

        // construct tree structure of joints and links
        std::vector<std::string> roots = findRootLinks();

        if (roots.size() != 1) {
            os() << "Error: multiple models defined in one model file. please consider reading each separate files." << endl;
            return NULL;
        }
        
        std::vector<std::string>::iterator it = roots.begin();
        if (isVerbose) {
            os() << "found root link " << *it << std::endl;
        }
        
        Link* link = body->createLink();
        JointInfoPtr root;
        if (*it == "world") {
            vector<JointInfoPtr> children = findChildJoints(*it);
            if (children.size() == 1) {
                vector<JointInfoPtr>::iterator c = children.begin();
                link->setName((*c)->childName);
                link->setJointType(Link::FIXED_JOINT);
                link->setJointAxis(Vector3::Zero());
                link->Tb() = (*c)->pose;
                root.reset(new JointInfo());
                root->jointName = (*c)->jointName;
                root->childName = (*c)->parentName;
                root->child = (*c)->parent;
            } else {
                link->setName(*it);
                link->setJointType(Link::FIXED_JOINT);
                link->setJointAxis(Vector3::Zero());
                root.reset(new JointInfo());
                root->jointName = *it;
                root->childName = *it;
                root->child = linkdataMap[*it];
            }
        } else {
            link->setName(*it);
            link->setJointType(Link::FREE_JOINT);
            link->setJointAxis(Vector3::Zero());
            root.reset(new JointInfo());
            root->jointName = *it;
            root->childName = *it;
            root->child = linkdataMap[*it];
        }
        if (root->child != NULL) {
            link->setMass(root->child->m);
            link->setCenterOfMass(Vector3(root->child->c.pos.x, root->child->c.pos.y, root->child->c.pos.z));
            link->setInertia(root->child->I);
            setShape(link, root->child->visualShape, true);
            setShape(link, root->child->collisionShape, false);
            convertChildren(link, root);
        }
        body->setRootLink(link);

        // attach sensors
        sensorConverter->setRootLinkName(link->name());
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

        // set joint type
        convertJointType(link, *it);

        if (link->isFreeJoint() == true || link->isFixedJoint() == true) {
            link->setJointAxis(Vector3::Zero());
        } else {
            link->setJointAxis(link->Rs() * (*it)->axis);
            double maxlimit = numeric_limits<double>::max();
            link->setJointRange((*it)->lower, (*it)->upper);
            link->setJointVelocityRange(getLimitValue(-(*it)->velocity, -maxlimit),
                                        getLimitValue((*it)->velocity, +maxlimit));
        }

        // assign joint ID
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

void SDFBodyLoaderImpl::buildMesh(const aiScene* scene, const aiNode* node, SgTransform* sgnode,
                                  std::vector<SgMaterial*>& material_table,
                                  std::vector<SgTexture*>& texture_table)
{
    if(!node){
        return;
    }

    // get absolute transformation to this node
    aiMatrix4x4 transform = node->mTransformation;
    aiNode *pnode = node->mParent;
    while(pnode){
        transform = pnode->mTransformation * transform;
        pnode = pnode->mParent;
    }
    
    aiMatrix3x3 rotation(transform);
	  aiMatrix3x3 inverse_transpose_rotation(rotation);
	  inverse_transpose_rotation.Inverse();
	  inverse_transpose_rotation.Transpose();
    
    for(uint32_t i = 0; i < node->mNumMeshes; i++){
        aiMesh* input_mesh = scene->mMeshes[node->mMeshes[i]];

        SgShapePtr shape = new SgShape();
        SgMeshPtr mesh = shape->setMesh(new SgMesh());
        SgVertexArrayPtr vertices = mesh->setVertices(new SgVertexArray());
        
        // set vertices
        for(uint32_t j = 0; j < input_mesh->mNumVertices; j++){
            aiVector3D p = input_mesh->mVertices[j];
            p *= transform;
            Vector3f v(p.x, p.y, p.z);
            vertices->push_back(v);
        }

        // set normals
        if(input_mesh->HasNormals()){
            SgNormalArrayPtr normals = mesh->setNormals(new SgNormalArray());
            for(uint32_t j = 0; j < input_mesh->mNumVertices; j++){
                aiVector3D n = inverse_transpose_rotation * input_mesh->mNormals[j];
                n.Normalize();
                Vector3f nv(n.x, n.y, n.z);
                normals->push_back(nv);
            }
        }

        // set uvmaps
        if(input_mesh->HasTextureCoords(0)){
            SgTexCoordArrayPtr texcoord = mesh->setTexCoords(new SgTexCoordArray());
            for(uint32_t j = 0; j < input_mesh->mNumVertices; j++){
                Vector2f t(input_mesh->mTextureCoords[0][j].x, input_mesh->mTextureCoords[0][j].y);
                texcoord->push_back(t);
            }
        }
        
        // set vertex index
        SgIndexArray& triangleVertices = mesh->triangleVertices();
        triangleVertices.reserve(input_mesh->mNumFaces);
        for(uint32_t j = 0; j < input_mesh->mNumFaces; j++){
            aiFace& face = input_mesh->mFaces[j];
            if (face.mNumIndices == 3) { // most of the element are the triangulated faces
                triangleVertices.push_back(face.mIndices[0]);
                triangleVertices.push_back(face.mIndices[1]);
                triangleVertices.push_back(face.mIndices[2]);
            } else if (face.mNumIndices == 2) {
                SgLineSetPtr lineset = new SgLineSet();
                lineset->setVertices(vertices);
                lineset->addLine(face.mIndices[0], face.mIndices[1]);
                lineset->setLineWidth(0.1);  // not sure about the width
                sgnode->addChild(lineset);
            }
        }
        
        shape->setMaterial(material_table[input_mesh->mMaterialIndex]);
        SgTexture* texture = texture_table[input_mesh->mMaterialIndex];
        if (texture) {
            shape->setTexture(texture);
        }
        sgnode->addChild(shape);
    }
    
    for(uint32_t i=0; i < node->mNumChildren; ++i){
        // recursive call to construct scenegraph
        buildMesh(scene, node->mChildren[i], sgnode, material_table, texture_table);
    }
}

void SDFBodyLoaderImpl::loadMaterials(const std::string& resource_path,
                                      const aiScene* scene,
                                      std::vector<SgMaterial*>& material_table,
                                      std::vector<SgTexture*>& texture_table)
{
    ImageIO imageIO;
    for (uint32_t i = 0; i < scene->mNumMaterials; i++) {
        aiMaterial* amat = scene->mMaterials[i];
        SgMaterial* material = new SgMaterial;
        SgTexture* texture = NULL;
        
        for (uint32_t j=0; j < amat->mNumProperties; j++) {
            aiMaterialProperty* prop = amat->mProperties[j];
            std::string propKey = prop->mKey.data;
            
            if (propKey == "$tex.file") {
                aiString texName;
                aiTextureMapping mapping;
                uint32_t uvIndex;
                amat->GetTexture(aiTextureType_DIFFUSE, 0, &texName, &mapping, &uvIndex);
                
                // Assume textures are in paths relative to the mesh
                std::string texture_path = boost::filesystem::path(resource_path).parent_path().string()
                    + "/" + texName.data;

                SgImagePtr image;
                SgImagePtr imageForLoading;

                ImagePathToSgImageMap::iterator p = imagePathToSgImageMap.find(texture_path);
                if(p != imagePathToSgImageMap.end()){
                    image = p->second;
                } else {
                    try {
                        if(!imageForLoading){
                            imageForLoading = new SgImage;
                        }
                        imageIO.load(imageForLoading->image(), texture_path);
                        image = imageForLoading;
                        imagePathToSgImageMap[texture_path] = image;
                    } catch(const exception_base& ex){
                        os() << *boost::get_error_info<error_info_message>(ex) << endl;
                    }
                }
                if(image){
                    texture = new SgTexture;
                    texture->setImage(image);
                }
            } else if (propKey == "$clr.diffuse") {
                aiColor3D clr;
                amat->Get(AI_MATKEY_COLOR_DIFFUSE, clr);
                material->setDiffuseColor(Vector3f(clr.r, clr.g, clr.b));
            } else if (propKey == "$clr.ambient") {
                aiColor3D clr;
                amat->Get(AI_MATKEY_COLOR_AMBIENT, clr);
                material->setAmbientIntensity((clr.r + clr.g + clr.b) / 3.0f);
            } else if (propKey == "$clr.specular") {
                aiColor3D clr;
                amat->Get(AI_MATKEY_COLOR_SPECULAR, clr);
                material->setSpecularColor(Vector3f(clr.r, clr.g, clr.b));
            } else if (propKey == "$clr.emissive") {
                aiColor3D clr;
                amat->Get(AI_MATKEY_COLOR_EMISSIVE, clr);
                material->setEmissiveColor(Vector3f(clr.r, clr.g, clr.b));
            } else if (propKey == "$clr.transparent") {
                aiColor3D clr;
                amat->Get(AI_MATKEY_COLOR_TRANSPARENT, clr);
                material->setTransparency((clr.r + clr.g + clr.b) / 3.0f);
            /*
            } else if (propKey == "$mat.opacity") {
                float o;
                amat->Get(AI_MATKEY_OPACITY, o);
                material->setTransparency(1.0f - o);
            */
            } else if (propKey == "$mat.shininess") {
                float s;
                amat->Get(AI_MATKEY_SHININESS, s);
                material->setShininess(s);
            }
        }
        material_table.push_back(material);
        texture_table.push_back(texture);
    }
}

void SDFBodyLoaderImpl::getDaeScaleAndAxis(const std::string& url, float& scale, std::string& axis)
{
    TiXmlDocument xmlDoc;
    xmlDoc.LoadFile(url);

    scale = 1.0;
    axis = "Y_UP";
    if(!xmlDoc.Error()) {
        TiXmlElement * colladaXml = xmlDoc.FirstChildElement("COLLADA");
        if(colladaXml) {
            TiXmlElement *assetXml = colladaXml->FirstChildElement("asset");
            if(assetXml) {
                TiXmlElement *unitXml = assetXml->FirstChildElement("unit");
                if (unitXml && unitXml->Attribute("meter")) {
                    unitXml->QueryFloatAttribute("meter", &scale);
                }
                TiXmlElement *up_axisXml = assetXml->FirstChildElement("up_axis");
                if (up_axisXml) {
                    axis = up_axisXml->GetText();
                }
            }
        }
    }
}

SgNodePtr SDFBodyLoaderImpl::readGeometry(sdf::ElementPtr geometry, SgMaterial* material, const cnoid::Affine3 &pose)
{
    SgNodePtr converted = 0;

    assert(geometry);

    for(sdf::ElementPtr el = geometry->GetFirstElement(); el; el = el->GetNextElement()) {
        if(el->GetName() == "mesh") {
            std::string url = sdf::findFile(el->Get<std::string>("uri"));
            std::string urllower = url;
            boost::algorithm::to_lower(urllower);
            SgPosTransformPtr transform = new SgPosTransform(pose);

            if (!url.empty()) {
                if (isVerbose) {
                    os() << "     read mesh " << url << std::endl;
                }

                if (boost::algorithm::iends_with(urllower, "dae")) {
                    SgScaleTransformPtr scaletrans = new SgScaleTransform;
                    Assimp::Importer importer;
                    const aiScene* scene = importer.ReadFile(url, aiProcess_SortByPType|aiProcess_GenNormals|aiProcess_Triangulate|aiProcess_GenUVCoords|aiProcess_FlipUVs);
                    std::vector<SgMaterial*> material_table;
                    std::vector<SgTexture*> texture_table;
                    float scale;
                    std::string axis;
                    getDaeScaleAndAxis(url, scale, axis);
                    scaletrans->setScale(scale);
                    if (axis == "Z_UP") {
                        cnoid::AngleAxis xaxis(M_PI/2.0, cnoid::Vector3::UnitX());
                        transform->rotation() = transform->rotation() * xaxis.toRotationMatrix();
                    }
                    loadMaterials(url, scene, material_table, texture_table);
                    buildMesh(scene, scene->mRootNode, scaletrans, material_table, texture_table);
                    transform->addChild(scaletrans);
                } else if (boost::algorithm::iends_with(urllower, "stl")) {
                    STLSceneLoader loader;
                    SgShapePtr shape = dynamic_cast<SgShape*>(loader.load(url));

                    if (shape != NULL) {
                        if (material != NULL) {
                            shape->setMaterial(material);
                        }

                        if(el->HasElement("scale")){
                            sdf::Vector3 scale = el->Get<sdf::Vector3>("scale");
                            SgScaleTransformPtr scaletrans = new SgScaleTransform;
                            scaletrans->setScale(Vector3(scale.x, scale.y, scale.z));
                            scaletrans->addChild(shape);
                            transform->addChild(scaletrans);
                        } else {
                            transform->addChild(shape);
                        }
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
            SgPosTransformPtr transform = new SgPosTransform(pose);

            if (isVerbose) {
                os() << "     generate box shape (x=" << size.x << " y=" << size.y
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
            SgPosTransformPtr transform = new SgPosTransform(pose);

            if (isVerbose) {
                os() << "     generate sphere shape (radius=" << radius << ")" << std::endl;
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
            SgPosTransformPtr transform = new SgPosTransform(pose);

            // direction of the cylinder in SDF is different from the VRML, so we rotate here
            cnoid::AngleAxis xaxis(M_PI/2.0, cnoid::Vector3::UnitX());
            transform->rotation() = transform->rotation() * xaxis.toRotationMatrix();

            if (isVerbose) {
                os() << "     generate cylinder shape (radius=" << radius << " length="
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

    assert(material);
    
    ret = NULL;
    cinfo = NULL;

    if (material != NULL) {
        if (material->HasElement("script")) {
            p = material->GetElement("script");

            /* script element must have name element */
            if (p->HasElement("name")) {
                cinfo = gazeboColor->get(p->Get<std::string>("name"));
                if (cinfo != NULL) {
                    ret = cinfo->material;
                }
            }
        }

        if (cinfo == NULL) {
            // [note from YM] this implementation may consume extra memory
            // (might be better to avoid dynamic allocation or use smart pointer)
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
   @brief Convert joint type.
   @param[in/out] link Instance of Link.
   @param[in] info Joint info
 */
void SDFBodyLoaderImpl::convertJointType(Link *link, JointInfoPtr info)
{
    std::map<std::string, Link::JointType>::iterator it;

    assert(link);
    assert(info);

    if (info->jointType == "revolute" && info->upper == info->lower) {
        // Gazebo's way to express fixed joint
        link->setJointType(Link::FIXED_JOINT);
    } else {
        it = jointTypeMap.find(info->jointType);
        if (it != jointTypeMap.end()) {
            link->setJointType(it->second);
        } else {
            os() << "Warning: unable to handle joint type " << info->jointType << " of joint "
                 << info->jointName << " assume as fixed joint." << std::endl;
            link->setJointType(Link::FIXED_JOINT);
        }
    }

    return;
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
   @brief Read shapes.
   @param[in/out] shapes Pointer to SgGroup to store shape data.
   @param[in] link SDF element to read from.
   @param[in] tagname Tag name of SDF element to read from.
 */
void SDFBodyLoaderImpl::readShapes(cnoid::SgGroup* shapes, sdf::ElementPtr link, const std::string tagname)
{
    cnoid::Affine3  pose;
    SgMaterial*     sgmaterial;
    sdf::ElementPtr p;

    assert(shapes);
    assert(link);
    
    p = link->GetElement(tagname);

    while (p) {
        if (isVerbose) {
            os() << "  " << tagname << " name: " << p->Get<std::string>("name") << std::endl;
        }

        if (p->HasElement("pose")) {
            pose = pose2affine(p->Get<sdf::Pose>("pose"));
        } else {
            pose = cnoid::Affine3::Identity();
        }

        float transparency = -1.0;
        if (p->HasElement("transparency")){
            transparency = p->Get<float>("transparency");
        }
        
        sgmaterial = NULL;
        if (p->HasElement("material")){
            sgmaterial = createSgMaterial(p->GetElement("material"), transparency);
        }

        if (p->HasElement("geometry")) {
            shapes->addChild(readGeometry(p->GetElement("geometry"), sgmaterial, pose));
        } else {
            // corrupted? (geometry must be set)
            os() << "Warning: '" << p->Get<std::string>("name") << "' geometry not found" << std::endl;
        }

        p = p->GetNextElement(tagname);
    }

    return;
}
