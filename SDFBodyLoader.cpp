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
#include <cnoid/SceneGraph>
#include <cnoid/SceneShape>
#include <cnoid/DaeParser>
#include <cnoid/MeshGenerator>
#include <cnoid/NullOut>
#include <cnoid/ImageIO>
#include <cnoid/Exception>
#include <sdf/sdf.hh>
#include <sdf/parser_urdf.hh>
#include <boost/function.hpp>
#include <boost/format.hpp>
#include <boost/dynamic_bitset.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

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
    typedef std::map<std::string, SgImagePtr> ImagePathToSgImageMap;
    ImagePathToSgImageMap imagePathToSgImageMap;

    ostream& os() { return *os_; }

    SDFBodyLoaderImpl();
    ~SDFBodyLoaderImpl();
    cnoid::Affine3 pose2affine(const sdf::Pose& pose);
    bool load(Body* body, const std::string& filename);
    BodyPtr load(const std::string& filename);        
    SgNodePtr readGeometry(sdf::ElementPtr geometry, const sdf::Pose &pose);
    std::vector<JointInfoPtr> findRootJoints();
    std::vector<JointInfoPtr> findChildJoints(const std::string& linkName);
    void convertChildren(Link* plink, JointInfoPtr parent);
    void buildMesh(const aiScene* scene, const aiNode* node, SgMesh* mesh,
                   std::vector<SgMaterial*>& material_table);
    void loadMaterials(const std::string& resource_path,
                       const aiScene* scene, std::vector<SgMaterial*>& material_table);
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

cnoid::Affine3 SDFBodyLoaderImpl::pose2affine(const sdf::Pose& pose)
{
    cnoid::Affine3 ret = cnoid::Affine3::Identity();
    cnoid::Vector3 trans;
    trans(0) = pose.pos.x;
    trans(1) = pose.pos.y;
    trans(2) = pose.pos.z;
    cnoid::Quat R;
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
                os() << " link name " << linkdata->linkName << std::endl;
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
                    }
                }
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
            os() << "found root link " << (*it)->parentName << std::endl;
            Link* link = body->createLink();
            link->setName(modelName);
            JointInfoPtr root;
            if((*it)->parentName == "world"){
                link->setJointType(Link::FIXED_JOINT);
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
            link->setMass(root->child->m);
            link->setCenterOfMass(Vector3(root->child->c.pos.x, root->child->c.pos.y, root->child->c.pos.z));
            link->setInertia(root->child->I);
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
    vector<JointInfoPtr>::iterator it = children.begin();
    while(it != children.end()){
        if((*it)->child == 0){
            it++;
            continue;
        }
        if(isVerbose){
            os() << "convert children from: " << parent->childName << std::endl;
            os() << "                 to: " << (*it)->childName << std::endl;
            os() << parent->pose.matrix() << std::endl;
            os() << (*it)->child->pose.matrix() << std::endl;
        }
        Link* link = body->createLink();
        link->setName((*it)->jointName);
        link->setMass((*it)->child->m);
        link->setCenterOfMass(Vector3((*it)->child->c.pos.x, (*it)->child->c.pos.y, (*it)->child->c.pos.z));
        link->setInertia((*it)->child->I);

        // convert to relative position
        cnoid::Affine3 m = parent->pose.inverse() * (*it)->child->pose;
        link->Tb() = m;
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

void SDFBodyLoaderImpl::buildMesh(const aiScene* scene, const aiNode* node,
                                  SgMesh* mesh, std::vector<SgMaterial*>& material_table)
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
                //aiVector3D n = inverse_transpose_rotation * input_mesh->mNormals[j];
                aiVector3D n = input_mesh->mNormals[j];
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
            if (face.mNumIndices == 3) { // only support triangulated face
                triangleVertices.push_back(face.mIndices[0]);
                triangleVertices.push_back(face.mIndices[1]);
                triangleVertices.push_back(face.mIndices[2]);
            }
        }
        
        shape->setMaterial(material_table[input_mesh->mMaterialIndex]);
    }
    
    for(uint32_t i=0; i < node->mNumChildren; ++i){
        // recursive call to construct scenegraph
        buildMesh(scene, node->mChildren[i], mesh, material_table);
    }
}

void SDFBodyLoaderImpl::loadMaterials(const std::string& resource_path,
                                      const aiScene* scene,
                                      std::vector<SgMaterial*>& material_table)
{
    ImageIO imageIO;
    for (uint32_t i = 0; i < scene->mNumMaterials; i++) {
        aiMaterial* amat = scene->mMaterials[i];
        SgMaterial* material = new SgMaterial;
        SgTexture* texture = new SgTexture;
        
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
                    break;
                } else {
                    try {
                        if(!imageForLoading){
                            imageForLoading = new SgImage;
                        }
                        imageIO.load(imageForLoading->image(), texture_path);
                        image = imageForLoading;
                        imagePathToSgImageMap[texture_path] = image;
                        break;
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
                //material->setAmbientIntensity(Vector3f(clr.r, clr.g, clr.b));
            } else if (propKey == "$clr.specular") {
                aiColor3D clr;
                amat->Get(AI_MATKEY_COLOR_SPECULAR, clr);
                material->setSpecularColor(Vector3f(clr.r, clr.g, clr.b));
            } else if (propKey == "$clr.emissive") {
                aiColor3D clr;
                amat->Get(AI_MATKEY_COLOR_EMISSIVE, clr);
                material->setEmissiveColor(Vector3f(clr.r, clr.g, clr.b));
            } else if (propKey == "$clr.opacity") {
                float o;
                amat->Get(AI_MATKEY_OPACITY, o);
                material->setTransparency(o);
            } else if (propKey == "$mat.shininess") {
                float s;
                amat->Get(AI_MATKEY_SHININESS, s);
                material->setShininess(s);
            }
        }
        material_table.push_back(material);
    }
}

SgNodePtr SDFBodyLoaderImpl::readGeometry(sdf::ElementPtr geometry, const sdf::Pose &pose)
{
    SgNodePtr converted = 0;

    for(sdf::ElementPtr el = geometry->GetFirstElement(); el;
        el = el->GetNextElement()) {
        if(el->GetName() == "mesh") {
            std::string url = sdf::findFile(el->Get<std::string>("uri"));
            SgPosTransformPtr transform = new SgPosTransform;
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
                Assimp::Importer importer;
                const aiScene* scene = importer.ReadFile(url, aiProcess_SortByPType|aiProcess_GenNormals|aiProcess_Triangulate|aiProcess_GenUVCoords|aiProcess_FlipUVs);
                std::vector<SgMaterial*> material_table;
                SgMesh* mesh = new SgMesh;
                buildMesh(scene, scene->mRootNode, mesh, material_table);
                SgShape* shape = new SgShape;
                shape->setMesh(mesh);
                //shape->setTexture();
            } else if (boost::algorithm::iends_with(url, "stl")) {
                STLSceneLoader loader;
                transform->addChild(loader.load(url));
            }
            converted = transform;
        } else if(el->GetName() == "box"){
            SgShapePtr shape = new SgShape;
            sdf::Vector3 size = el->Get<sdf::Vector3>("size");
            shape->setMesh(meshGenerator.generateBox(Vector3(size.x, size.y, size.z)));
            converted = shape;
        } else if(el->GetName() == "sphere"){
            SgShapePtr shape = new SgShape;
            double radius = el->Get<double>("radius");
            shape->setMesh(meshGenerator.generateSphere(radius));
            converted = shape;
        } else if(el->GetName() == "cylinder"){
            SgShapePtr shape = new SgShape;
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


