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
#include <cnoid/WorldItem>
#include <cnoid/RootItem>
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

std::map<std::string, Link::JointType> jointTypeMap = {
        { "revolute", Link::REVOLUTE_JOINT },
        { "prismatic", Link::SLIDE_JOINT },
        { "fixed", Link::FIXED_JOINT },
        { "free", Link::FREE_JOINT }
};

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

class GeometryInfo : public Referenced
{
public:
    Vector3 scale;
    SgNodePtr sgNode;
};
typedef ref_ptr<GeometryInfo> GeometryInfoPtr;

class MaterialInfo : public Referenced
{
public:
    string scriptName;
    string uri;
    sdf::Color ambient;
    sdf::Color diffuse;
    sdf::Color specular;
    sdf::Color emissive;
};
typedef ref_ptr<MaterialInfo> MaterialInfoPtr;

class VisualInfo : public Referenced
{
public:
    Affine3 pose;
    float transparency;
    MaterialInfoPtr material;
    GeometryInfoPtr geometry;

};
typedef ref_ptr<VisualInfo> VisualInfoPtr;

class CollisionInfo : public Referenced
{
public:
    Affine3 pose;
    GeometryInfoPtr geometry;
};
typedef ref_ptr<CollisionInfo> CollisionInfoPtr;

class JointInfo;

class LinkInfo : public Referenced
{
public:
    std::string linkName;
    double m;
    cnoid::Affine3 c;
    cnoid::Matrix3 I;
    cnoid::SgGroupPtr visualShape;
    cnoid::SgGroupPtr collisionShape;
    cnoid::Affine3 pose;
    bool isRoot;
    vector<VisualInfoPtr> visuals;
    vector<CollisionInfoPtr> collisions;

    LinkInfo* parent;
    LinkInfo* child;
    LinkInfo* sibling;
    JointInfo* jointInfo;
    Affine3 origin;

    LinkInfo() {
        /*
          initialize value by Choreonoid's constructor value.
          see Choreonoid src/Body/Link.cpp Link::Link().
         */
        m = 0.0;
        c = Affine3::Identity();
        I.setZero();
        visualShape = new SgGroup;
        collisionShape = new SgGroup;
        pose = cnoid::Affine3::Identity();
        parent = child = sibling  = 0;
        jointInfo = 0;
        isRoot = false;
    }

    void addChild(LinkInfo* link) {
        if(!child){
            child = link;
            link->sibling = 0;
        } else {
            LinkInfo* lastChild = child;
            while(lastChild->sibling){
                lastChild = lastChild->sibling;
            }
            lastChild->sibling = link;
            link->sibling = 0;
        }
        link->parent = this;
    }

    Link* createLink();

};
typedef ref_ptr<LinkInfo> LinkInfoPtr;

class JointInfo: public Referenced
{
public:
    std::string jointName;
    std::string parentName;
    LinkInfoPtr parent;
    std::string childName;
    LinkInfoPtr child;
    std::string jointType;
    cnoid::Vector3 axis;
    bool useparent;
    double upper;
    double lower;
    double velocity;
    cnoid::Affine3 pose;

    JointInfo() {
        /*
          initialize value by Choreonoid's constructor value.
          see Choreonoid src/Body/Link.cpp Link::Link().
         */
        axis = Vector3::UnitZ();
        upper = std::numeric_limits<double>::max();
        lower = -std::numeric_limits<double>::max();
        velocity = 0.0;
        pose = cnoid::Affine3::Identity();
        useparent = false;
        parent = child = 0;
    }

    Link::JointType convertJointType();
    void convertAxis(Vector3& axis);

};
typedef ref_ptr<JointInfo> JointInfoPtr;

class ModelInfo;
typedef ref_ptr<ModelInfo> ModelInfoPtr;

class ModelInfo : public Referenced
{
public:
    std::string name;
    bool isStatic;
    bool selfCollide;
    cnoid::Affine3 pose;
    vector<ModelInfoPtr> nestedModels;
    std::map<std::string, LinkInfoPtr> linkInfos;
    vector<JointInfoPtr> jointInfos;
    LinkInfo* root;

    ModelInfo(){
        isStatic = false;
        selfCollide = false;
        pose = cnoid::Affine3::Identity();
        nestedModels.clear();
        root = 0;
    }

    LinkInfo* findRootLinks();
    void nestModel(string parentName, ModelInfo* orgModel);
};

class SDFBodyLoaderImpl
{
public:   
    dynamic_bitset<> validJointIdSet;
    int numValidJointIds;
    ostream* os_;
    bool isVerbose;
    std::vector<JointInfoPtr> joints;
    std::map<std::string, LinkInfoPtr> linkdataMap;
    typedef std::map<std::string, SgImagePtr> ImagePathToSgImageMap;
    ImagePathToSgImageMap imagePathToSgImageMap;
    sdf::ElementPtr root;

    ostream& os() { return *os_; }

    SDFBodyLoaderImpl();
    ~SDFBodyLoaderImpl();
    cnoid::Affine3 pose2affine(const sdf::Pose& pose);

    void readSDF(const std::string& filename, vector<ModelInfoPtr>& modelInfos);

    bool load(Body* body, const std::string& filename);
    bool load(BodyItem* item, const std::string& filename);
    BodyPtr load(const std::string& filename);

    bool createBody(Body* body, ModelInfo* model);
    SgNodePtr readGeometry(sdf::ElementPtr geometry, SgMaterial* material, const cnoid::Affine3 &pose);
    std::vector<std::string> findRootLinks();
    bool hasOpenLink(const std::string& linkName);
    std::vector<JointInfoPtr> findChildJoints(const std::string& linkName);
    std::vector<JointInfoPtr> findParentJoints(const std::string& linkName);
    void convertChildren(Link* plink, JointInfoPtr parent);

    void readWorld(sdf::ElementPtr world, vector<ModelInfoPtr>& modelInfos);
    void readInclude(sdf::ElementPtr include, vector<ModelInfoPtr>& modelInfos);
    void readModel(sdf::ElementPtr model, vector<ModelInfoPtr>& modelInfos);
    void readLink(sdf::ElementPtr link, std::map<std::string, LinkInfoPtr>& linksInfos);
    void readJoint(sdf::ElementPtr joint, vector<JointInfoPtr>& jointInfos);
    void readVisual(sdf::ElementPtr visual, vector<VisualInfoPtr>& visuals);
    void readCollision(sdf::ElementPtr collision, vector<CollisionInfoPtr>& collisions);
    GeometryInfo* readGeometry(sdf::ElementPtr geometry);
    MaterialInfo* readMaterial(sdf::ElementPtr material);

private:
    SDFLoaderPseudoGazeboColor* gazeboColor;
    SDFSensorConverter* sensorConverter;

    void buildMesh(const aiScene* scene, const aiNode* node, SgTransform* sgnode,
                   std::vector<SgMaterial*>& material_table,
                   std::vector<SgTexture*>& texture_table);
    void loadMaterials(const std::string& resource_path, const aiScene* scene,
                       std::vector<SgMaterial*>& material_table,
                       std::vector<SgTexture*>& texture_table);
    void getDaeScaleAndAxis(const std::string& url, float& scale, std::string& axis, bool& hasMeter);
    SgMaterial* createSgMaterial(sdf::ElementPtr material, float transparency);
    void convertJointType(Link *link, JointInfoPtr info);
    void readShapes(cnoid::SgGroup* shapes, sdf::ElementPtr link, const std::string tagname);
};

}


/**
   @brief Convert joint type.
 */
Link::JointType JointInfo::convertJointType()
{
    std::map<std::string, Link::JointType>::iterator it;

    if (jointType == "revolute" && upper == lower) {
        // Gazebo's way to express fixed joint
        return Link::FIXED_JOINT;
    } else {
        it = jointTypeMap.find(jointType);
        if (it != jointTypeMap.end()) {
            return it->second;
        } else {
            std::cout  << "Warning: unable to handle joint type " << jointType << " of joint "
                 << jointName << " assume as fixed joint." << std::endl;
            return Link::FIXED_JOINT;
        }
    }

    return Link::FIXED_JOINT;
}


void JointInfo::convertAxis(Vector3& axis_)
{

    if(useparent){
        axis_ = child->origin.linear().transpose() * parent->pose.linear() * axis;
        axis_.normalize();
    }else{
        axis_ = axis;
    }

}


Link* LinkInfo::createLink()
{
    Link* link = new Link();

    link->setName(linkName);

    if(parent){
        origin = pose * jointInfo->pose;
        link->setOffsetTranslation(origin.translation() -
                parent->origin.translation());
    }else{
        origin = pose;
        link->setOffsetTranslation(origin.translation());
    }
    link->setAccumulatedSegmentRotation(origin.linear());

    link->setMass(m);
    Affine3 c_ = jointInfo->pose.inverse() * c;
    link->setCenterOfMass(link->Rs() * c_.translation());
    link->setInertia(link->Rs() * c_.linear() * I * c_.linear().transpose() * link->Rs().transpose());

    setShape(link, visualShape, true);
    setShape(link, collisionShape, false);

    link->setJointType( jointInfo->convertJointType() );

    Vector3 axis;
    jointInfo->convertAxis(axis);
    link->setJointAxis(link->Rs() * axis);

    double maxlimit = numeric_limits<double>::max();
    link->setJointRange(jointInfo->lower, jointInfo->upper);
    link->setJointVelocityRange(getLimitValue(-jointInfo->velocity, -maxlimit),
            getLimitValue(jointInfo->velocity, +maxlimit));


    for(LinkInfo* linkInfo = child; child; child = child->sibling){
        link->appendChild(linkInfo->createLink());
    }

    return link;
}


void ModelInfo::nestModel(string parentName, ModelInfo* orgModel)
{
    string modelName;
    if(parentName=="")
        modelName = name;
    else
        modelName = parentName + "::" + name;

    for(vector<ModelInfoPtr>::iterator it = nestedModels.begin();
            it != nestedModels.end(); it++){
        (*it)->nestModel(modelName, orgModel);
    }

    if(orgModel==this)
        return;

    // modelのposeが反映されない？
    for(std::map<std::string, LinkInfoPtr>::iterator it = linkInfos.begin();
            it != linkInfos.end(); it++){
        orgModel->linkInfos[modelName + "::" + it->second->linkName] = it->second;
    }

    for(vector<JointInfoPtr>::iterator it = jointInfos.begin();
            it != jointInfos.end(); it++){
        orgModel->jointInfos.push_back(*it);
    }
}


LinkInfo* ModelInfo::findRootLinks()
{
    if(root)
        return root;

    // parentが二人以上いてはいけない　closed Linkは?　　TODO
    for(vector<JointInfoPtr>::iterator it = jointInfos.begin();
                it != jointInfos.end(); it++){
        LinkInfo* parent = linkInfos[(*it)->parentName].get();
        LinkInfo* child = linkInfos[(*it)->childName].get();
        if(parent && child){
            (*it)->parent = parent;
            (*it)->child = child;
            child->jointInfo = (*it).get();
            parent->addChild(child);
        }else{
               // TODO  エラー表示
        }
    }

    // base指定されているLinkがあればそれがroot
    for(map<string, LinkInfoPtr>::iterator it = linkInfos.begin();
            it != linkInfos.end(); it++){
        LinkInfo* link = it->second;
        if(link->isRoot){
            root = link;
        }
    }

    //rootがない場合、parentがいないLinkがroot、それ以外は別モデルにしなきゃいけない？
    for(map<string, LinkInfoPtr>::iterator it = linkInfos.begin();
                it != linkInfos.end(); it++){
        LinkInfo* link = it->second;
        if(!link->parent){
            if(root){
                ;
            }else{
                root = link;
            }
        }
    }

    if(root->linkName=="world"){
         root = root->child;
         root->parent = 0;
     }

    return root;

/*
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
    */
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
            os() << "search for parent joint of " << linkName
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
    os_ = &nullout();


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


bool SDFBodyLoader::load(BodyItem* item, const std::string& filename)
{
    return impl->load(item, filename);
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


bool SDFBodyLoaderImpl::load(BodyItem* item, const std::string& filename)
{
    vector<ModelInfoPtr> models;

    try{
        readSDF(filename, models);
    } catch(const std::exception& ex){
        os() << "Error: " << ex.what() << endl;
        return false;
    }

    std::cout << "num of models= " << models.size() << std::endl;

    if(!models.size()){
        os() <<  "The model is not declared." << std::endl;
        return false;
    }

    WorldItem* worldItem = 0;
    if( models.size()>1 ){
        worldItem = new WorldItem;
        Item* parentItem = item->parentItem();
        if(!parentItem){
            parentItem = RootItem::instance();
        }
        parentItem->addChildItem(worldItem);
    }

    for(size_t i=0; i<models.size(); i++){
        //  アイテムを追加して、それに対してモデルを読み込む

        Body* body = new Body();
        createBody(body, models[i].get());

        BodyItem* bodyItem;
        if(worldItem){
            bodyItem = new BodyItem;
            worldItem->addChildItem(bodyItem);
        }else{
            bodyItem = item;
        }

        bodyItem->setBody(body);

        if(bodyItem->name().empty()){
            bodyItem->setName(body->modelName());
        }
        bodyItem->setEditable(!body->isStaticModel());
    }

    if(worldItem)
        return false;          // メッセージが失敗になる。
    else
        return true;
}


void  SDFBodyLoaderImpl::readSDF(const std::string& filename, vector<ModelInfoPtr>& models)
{
    try {
        sdf::ElementPtr root = NULL;

        sdf::SDFPtr sdf(new sdf::SDF());
        sdf::init(sdf);

        if (sdf::readFile(filename, sdf) == false) {  // this can read both SDF and URDF
            throw std::invalid_argument("load failed");
        }
        root = sdf->Root();
        if (root == NULL) {
            throw std::out_of_range("root element is NULL");
        }

        if(root->HasElement("world")){
            for(sdf::ElementPtr world = root->GetElement("world"); world != NULL;
                    world = world->GetNextElement("world")){
                readWorld(world, models);
            }
        }

        if(root->HasElement("model")){
            for(sdf::ElementPtr model = root->GetElement("model"); model != NULL;
                    model = model->GetNextElement("model")){
                readModel(model, models);
            }
        }
        /*
        for(sdf::ElementPtr element = root->GetFirstElement(); element != NULL; element = element->GetNextElement() ){
            if(element->GetName()=="world"){
                readWorld(element, models);
            }else if(element->GetName()=="model"){
                readModel(element, models);
                //string s = element->GetName();
                //models.push_back(element);
            }
        }
    */

    } catch(const std::exception& ex){
        throw ex;
    }
}


void SDFBodyLoaderImpl::readWorld(sdf::ElementPtr world, vector<ModelInfoPtr>& modelInfos)
{
    if(world->HasElement("model")){
        for(sdf::ElementPtr model = world->GetElement("model"); model != NULL;
                model = model->GetNextElement("model")){
            readModel(model, modelInfos);
        }
    }
    /*
    for(sdf::ElementPtr element = world->GetFirstElement(); element != NULL; element = element->GetNextElement() ){
        cout << element->GetName() << std::endl;
//        if(element->GetName()=="include"){
//            readInclude(element, modelInfos);
//        }else
        if(element->GetName()=="model"){
            readModel(element, modelInfos);
        }
    }
    */
}


void SDFBodyLoaderImpl::readInclude(sdf::ElementPtr include, vector<ModelInfoPtr>& modelInfos)
{ /*
    for(sdf::ElementPtr element = include->GetFirstElement(); element != NULL; element = element->GetNextElement() ){
        if(element->GetName()=="uri"){
            string uri = element->Get<std::string>("uri");
            // TODO
            try{
                readSDF(uri, modelInfos);
            }catch(const std::exception& ex){
                throw ex;
            }
        }else if(element->GetName()=="name"){
            modelInfos.back()->name = element->Get<string>("name");
        }else if(element->GetName()=="static"){
            modelInfos.back()->isStatic = element->Get<bool>("static");
        }else if(element->GetName()=="pose"){
            modelInfos.back()->pose = pose2affine(element->Get<sdf::Pose>("pose"));
        }
    }
*/
}


void SDFBodyLoaderImpl::readModel(sdf::ElementPtr model, vector<ModelInfoPtr>& modelInfos)
{
    ModelInfoPtr modelInfo(new ModelInfo());

    modelInfo->name = model->Get<std::string>("name");

    if(model->HasElement("static")){
        sdf::ElementPtr staticE = model->GetElement("static");
        modelInfo->isStatic = staticE->Get<bool>("static");
    }

    if(model->HasElement("self_collide")){
        sdf::ElementPtr selfCollide = model->GetElement("self_collide");
        modelInfo->selfCollide = selfCollide->Get<bool>("self_collide");
    }

    /*
    if(model->HasElement("include")){
        for(sdf::ElementPtr include = model->GetElement("include"); include != NULL;
                include =include->GetNextElement("include")){
                    readInclude(include, modelInfo->nestedModels);
                }
    }*/

    if(model->HasElement("model")){
        for(sdf::ElementPtr nestedModel = model->GetElement("model"); nestedModel != NULL;
                nestedModel = nestedModel->GetNextElement("model")){
                    readModel(nestedModel, modelInfo->nestedModels);
                }
    }

    if(model->HasElement("link")){
        for(sdf::ElementPtr link = model->GetElement("link"); link != NULL;
                link = link->GetNextElement("link")){
                    readLink(link, modelInfo->linkInfos);
                }
    }

    if(model->HasElement("joint")){
        for(sdf::ElementPtr joint = model->GetElement("joint"); joint != NULL;
                joint = joint->GetNextElement("joint")){
                    readJoint(joint, modelInfo->jointInfos);
                }
    }

    modelInfos.push_back(modelInfo);

}


void SDFBodyLoaderImpl::readLink(sdf::ElementPtr link, std::map<std::string, LinkInfoPtr>& linkInfos)
{
    LinkInfoPtr linkdata(new LinkInfo());

    linkdata->linkName = link->Get<std::string>("name");

    if (isVerbose) {
        os() << " link name " << linkdata->linkName << std::endl;
    }

    if(link->HasElement("must_be_base_link")){
        linkdata->isRoot = link->Get<bool>("must_be_base_link");
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
            linkdata->c = pose2affine(inertial->Get<sdf::Pose>("pose"));
        }
    }
    if(link->HasElement("visual")) {
        for(sdf::ElementPtr visual = link->GetElement("visual"); visual != NULL;
                visual = visual->GetNextElement("visual")){
            readVisual(visual, linkdata->visuals);
        }
    //    readShapes(linkdata->visualShape, link, "visual");
    }

    if(link->HasElement("collision")){
        for(sdf::ElementPtr collision = link->GetElement("collision"); collision != NULL;
                collision = collision->GetNextElement("collision")){
            readCollision(collision, linkdata->collisions);
        }
        //readShapes(linkdata->collisionShape, link, "collision");
    }

    if(link->HasElement("sensor")){
        sensorConverter->convert(linkdata->linkName, link);
    }

    linkInfos[linkdata->linkName] = linkdata;
}


void SDFBodyLoaderImpl::readJoint(sdf::ElementPtr joint, vector<JointInfoPtr>& jointInfos)
{
    JointInfoPtr jointdata(new JointInfo());

    jointdata->jointName = joint->Get<std::string>("name");
    jointdata->jointType = joint->Get<std::string>("type");
    jointdata->parentName = joint->Get<std::string>("parent");
    jointdata->childName = joint->Get<std::string>("child");
    //jointdata->parent = linkdataMap[jointdata->parentName];
    //jointdata->child = linkdataMap[jointdata->childName];

    if (isVerbose) {
        os() << " joint name " << jointdata->jointName << std::endl;
    }


    if(joint->HasElement("pose")){
        jointdata->pose = pose2affine(joint->Get<sdf::Pose>("pose"));
        //jointdata->pose = jointdata->child->pose * jointdata->pose;
    } else {
        //jointdata->pose = jointdata->child->pose;
        jointdata->pose = cnoid::Affine3::Identity();
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
        jointdata->useparent = false;
        if(axis->HasElement("use_parent_model_frame")){
            jointdata->useparent = axis->Get<bool>("use_parent_model_frame");
        }
        /*
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
*/
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

    if(joint->HasElement("sensor")){
        sensorConverter->convert(jointdata->jointName, joint);
    }

    jointInfos.push_back(jointdata);

}


void SDFBodyLoaderImpl::readVisual(sdf::ElementPtr visual, vector<VisualInfoPtr>& visuals)
{
    VisualInfoPtr visualInfo(new VisualInfo());

    if (visual->HasElement("pose")) {
        visualInfo->pose = pose2affine(visual->Get<sdf::Pose>("pose"));
    } else {
        visualInfo->pose = cnoid::Affine3::Identity();
    }

    visualInfo->transparency = -1.0;
    if (visual->HasElement("transparency")){
        visualInfo->transparency = visual->Get<float>("transparency");
    }

    if (visual->HasElement("material")){
        visualInfo->material = readMaterial(visual->GetElement("material"));
    }else{
        visualInfo->material = 0;
    }

    if (visual->HasElement("geometry")) {
        visualInfo->geometry = readGeometry(visual->GetElement("geometry"));
    } else {
        // corrupted? (geometry must be set)
        os() << "Warning: '" << visual->Get<std::string>("name") << "' geometry not found" << std::endl;
    }

    visuals.push_back(visualInfo);
}


void SDFBodyLoaderImpl::readCollision(sdf::ElementPtr collision, vector<CollisionInfoPtr>& collisions)
{
    CollisionInfoPtr collisionInfo(new CollisionInfo());

    if (collision->HasElement("pose")) {
        collisionInfo->pose = pose2affine(collision->Get<sdf::Pose>("pose"));
    } else {
        collisionInfo->pose = cnoid::Affine3::Identity();
    }

    if (collision->HasElement("geometry")) {
        collisionInfo->geometry = readGeometry(collision->GetElement("geometry"));
    } else {
        // corrupted? (geometry must be set)
        os() << "Warning: '" << collision->Get<std::string>("name") << "' geometry not found" << std::endl;
    }

    collisions.push_back(collisionInfo);
}


MaterialInfo* SDFBodyLoaderImpl::readMaterial(sdf::ElementPtr material)
{
    MaterialInfo* materialInfo = new MaterialInfo;

    if (material->HasElement("script")) {
        sdf::ElementPtr script = material->GetElement("script");
        /* script element must have name element */
        if (script->HasElement("name")) {
            materialInfo->scriptName = script->Get<string>("name");
        }
        if (script->HasElement("uri")) {
            materialInfo->uri = script->Get<string>("uri");
        }
    }

    if (material->HasElement("ambient")) {
        materialInfo->ambient = material->Get<sdf::Color>("ambient");
    }

    if (material->HasElement("diffuse")) {
        materialInfo->diffuse = material->Get<sdf::Color>("diffuse");
    }

    if (material->HasElement("specular")) {
        materialInfo->specular = material->Get<sdf::Color>("specular");
    }

    if (material->HasElement("emissive")) {
        materialInfo->emissive = material->Get<sdf::Color>("emissive");
    }

    return materialInfo;
}


GeometryInfo* SDFBodyLoaderImpl::readGeometry(sdf::ElementPtr geometry)
{
    GeometryInfo* geometryInfo = new GeometryInfo;

    for(sdf::ElementPtr el = geometry->GetFirstElement(); el; el = el->GetNextElement()) {
        if(el->GetName() == "mesh") {
            std::string url = sdf::findFile(el->Get<std::string>("uri"));
            std::string urllower = url;
            boost::algorithm::to_lower(urllower);

            if (!url.empty()) {
                if (isVerbose) {
                    os() << "     read mesh " << url << std::endl;
                }

                if (boost::algorithm::iends_with(urllower, "dae")) {
                    Assimp::Importer importer;
                    const aiScene* scene = importer.ReadFile(url, aiProcess_SortByPType|aiProcess_GenNormals|aiProcess_Triangulate|aiProcess_GenUVCoords|aiProcess_FlipUVs);
                    if(!scene)
                    {
                        std::cout << importer.GetErrorString() <<std::endl;
                    }
                    //std::vector<SgMaterial*> material_table;
                    //std::vector<SgTexture*> texture_table;
                    float meter;
                    std::string axis;
                    //Vector3 scale;
                    bool hasMeter;

                    getDaeScaleAndAxis(url, meter, axis, hasMeter);

                    if (el->HasElement("scale")) {
                        sdf::Vector3 v = el->Get<sdf::Vector3>("scale");
                        geometryInfo->scale = Vector3(v.x, v.y, v.z);
                    } else {
                        geometryInfo->scale = Vector3(1.0, 1.0, 1.0);    // SDF default values.
                    }

                    if (hasMeter) {
                        geometryInfo->scale *= meter;
                    }

                    //scaletrans->setScale(scale);
                    //if (axis == "Z_UP") {
                    //    cnoid::AngleAxis xaxis(M_PI/2.0, cnoid::Vector3::UnitX());
                   //     transform->rotation() = transform->rotation() * xaxis.toRotationMatrix();
                   // }
             //       loadMaterials(url, scene, material_table, texture_table);
             //       buildMesh(scene, scene->mRootNode, scaletrans, material_table, texture_table);

                    // TODO  assImpのローダを入れる
                    //geometryInfo->sgNode = node;

                } else if (boost::algorithm::iends_with(urllower, "stl")) {
                    STLSceneLoader loader;
                    geometryInfo->sgNode = loader.load(url);

                    if(el->HasElement("scale")){
                        sdf::Vector3 scale = el->Get<sdf::Vector3>("scale");
                        geometryInfo->scale = Vector3(scale.x, scale.y, scale.z);
                    } else {
                        geometryInfo->scale = Vector3(1,1,1);
                    }
                }
            } else {
                MessageView::instance()->putln(
                    MessageView::WARNING,
                    boost::format("%1% not found") % el->Get<std::string>("uri")
                    );
            }

        } else if(el->GetName() == "box"){
            SgShapePtr shape = new SgShape;
            sdf::Vector3 size = el->Get<sdf::Vector3>("size");
            shape->setMesh(meshGenerator.generateBox(Vector3(size.x, size.y, size.z)));
            geometryInfo->sgNode = shape;
        } else if(el->GetName() == "sphere"){
            SgShapePtr shape = new SgShape;
            double radius = el->Get<double>("radius");
            shape->setMesh(meshGenerator.generateSphere(radius));
            geometryInfo->sgNode = shape;
        } else if(el->GetName() == "cylinder"){
            SgShapePtr shape = new SgShape;
            double radius = el->Get<double>("radius");
            double length = el->Get<double>("length");
            shape->setMesh(meshGenerator.generateCylinder(radius, length));
            geometryInfo->sgNode = shape;
        } else if(el->GetName() == "empty"){
            geometryInfo->sgNode = 0;
        }
    }

    return geometryInfo;
}


bool SDFBodyLoaderImpl::load(Body* body, const std::string& filename)
{
    vector<ModelInfoPtr> models;

    try{
        readSDF(filename, models);
    } catch(const std::exception& ex){
        os() << "Error: " << ex.what() << endl;
        return false;
    }

    if(models.size() != 1){
        os() << "Error: multiple models defined in one model file. please consider reading each separate files." << endl;
        return false;
    }

    createBody(body, models[0].get());

}


bool SDFBodyLoaderImpl::createBody(Body* body, ModelInfo* model)
{
    //validJointIdSet.clear();
   // numValidJointIds = 0;

    body->setModelName(model->name);

    model->nestModel("", model);
    LinkInfo* root = model->findRootLinks();

    if(!root->jointInfo){
        JointInfo* jointInfo = new JointInfo();
        jointInfo->jointType = "free";
        model->jointInfos.push_back(jointInfo);
        root->jointInfo = jointInfo;
    }
    Link* rootLink = root->createLink();

    body->setRootLink(rootLink);

    return true;
}

#if 0
    /*
        if(link->HasElement("static")){
            body->setStaticModel(link->Get<bool>("static"));
            }
     */
        
    LinkInfoPtr worldlink(new LinkInfo());
    worldlink->linkName = "world";
    linkdataMap[worldlink->linkName] = worldlink;
/*
    cnoid::Affine3 rootpose;
    if(model->HasElement("pose")){
        if (isVerbose) {
            os() << " this model has root pose node" << std::endl;
        }
        rootpose = pose2affine(model->Get<sdf::Pose>("pose"));
    } else {
        rootpose = cnoid::Affine3::Identity();
    }
*/
/*
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
*/

    // create fixed joint if link is not referred by joint at all
    if(model->linkInfos.size() > 0){
        std::map<std::string, bool> usedlinks;
        for(std::vector<JointInfoPtr>::iterator it = model->jointInfos.begin();
                it != model->jointInfos.end(); it++){
            usedlinks[(*it)->parentName] = true;
            usedlinks[(*it)->childName] = true;
        }

        for(std::map<std::string, LinkInfoPtr>::const_iterator it2 = model->linkInfos.begin();
                it2 != model->linkInfos.end(); it2++){
            if (it2->second->linkName != "world" && usedlinks.find(it2->second->linkName) == usedlinks.end()) {
                JointInfoPtr j(new JointInfo());
                j->parent = worldlink;
                j->child = it2->second;
                j->parentName = j->parent->linkName;
                j->jointName = j->childName = j->child->linkName;
                j->jointType = "fixed";
                j->upper = j->lower = 0.0;
                joints.push_back(j);
            }
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
                //link->Tb() = rootpose * (*c)->pose;
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
#endif

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
            /*
            } else if (propKey == "$clr.transparent") {
                aiColor3D clr;
                amat->Get(AI_MATKEY_COLOR_TRANSPARENT, clr);
                material->setTransparency((clr.r + clr.g + clr.b) / 3.0f);
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

void SDFBodyLoaderImpl::getDaeScaleAndAxis(const std::string& url, float& scale, std::string& axis, bool& hasMeter)
{
    TiXmlDocument xmlDoc;
    xmlDoc.LoadFile(url);

    scale = 1.0;
    axis = "Y_UP";
    hasMeter = false;
    if(!xmlDoc.Error()) {
        TiXmlElement * colladaXml = xmlDoc.FirstChildElement("COLLADA");
        if(colladaXml) {
            TiXmlElement *assetXml = colladaXml->FirstChildElement("asset");
            if(assetXml) {
                TiXmlElement *unitXml = assetXml->FirstChildElement("unit");
                if (unitXml && unitXml->Attribute("meter") &&
                    unitXml->QueryFloatAttribute("meter", &scale) == TIXML_SUCCESS) {
                    hasMeter = true;
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
                    if(!scene)
                    {
                        std::cout << importer.GetErrorString() <<std::endl;
                    }
                    std::vector<SgMaterial*> material_table;
                    std::vector<SgTexture*> texture_table;
                    float meter;
                    std::string axis;
                    Vector3 scale;
                    bool hasMeter;

                    getDaeScaleAndAxis(url, meter, axis, hasMeter);

                    if (el->HasElement("scale")) {
                        sdf::Vector3 v = el->Get<sdf::Vector3>("scale");
                        scale = Vector3(v.x, v.y, v.z);
                    } else {
                        scale = Vector3(1.0, 1.0, 1.0);    // SDF default values.
                    }

                    if (hasMeter) {
                        scale *= meter;
                    }

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
