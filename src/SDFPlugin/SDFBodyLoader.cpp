/**
   \file
   \author Yosuke Matsusaka, Shizuko Hattori, Shin'ichiro Nakaoka
*/

#include "SDFBodyLoader.h"
#include "SDFLoaderPseudoGazeboColor.h"
#include <cnoid/Sensor>
#include <cnoid/Camera>
#include <cnoid/RangeSensor>
#include <cnoid/RangeCamera>
#include <cnoid/SceneDrawables>
#include <cnoid/SceneLoader>
#include <cnoid/MeshGenerator>
#include <cnoid/ImageIO>
#include <cnoid/Exception>
#include <cnoid/MessageView>
#include <cnoid/WorldItem>
#include <cnoid/RootItem>
#include <cnoid/EigenUtil>
#include <cnoid/PolymorphicFunctionSet>
#include <cnoid/std/filesystem>
#include <sdf/sdf.hh>
#include <sdf/parser_urdf.hh>
#include <ignition/math.hh>
#include <OGRE/OgreRoot.h>
#include <OGRE/OgreMaterialManager.h>
#include <boost/algorithm/string.hpp>
#include <fmt/format.h>
#include <map>
#include "gettext.h"

using namespace std;
using namespace cnoid;
namespace filesystem = cnoid::stdx::filesystem;

using ignition::math::Pose3d;

namespace {

std::map<std::string, Link::JointType> jointTypeMap = {
        { "revolute", Link::REVOLUTE_JOINT },
        { "prismatic", Link::SLIDE_JOINT },
        { "fixed", Link::FIXED_JOINT },
        { "free", Link::FREE_JOINT }
};

double getLimitValue(double val, double defaultValue)
{
    if(val == 0){
        return defaultValue;
    }
    return val;
}

}

namespace cnoid {

class MaterialTextureInstructor
{
public:
    PolymorphicFunctionSet<SgNode> functions;

    MaterialTextureInstructor( SgMaterial* material, SgTexture* texture ) :
        material(material), texture(texture) {
        functions.setFunction<SgGroup>(
                [&](SgNode* node){ visitGroup(static_cast<SgGroup*>(node)); });
        functions.setFunction<SgShape>(
                [&](SgNode* node){ visitShape(static_cast<SgShape*>(node)); });
        functions.updateDispatchTable();
    };

    bool set(SgNode* node){
        functions.dispatch(node);
        return true;
    }

    void visitGroup(SgGroup* group)
    {
        for(SgGroup::const_iterator p = group->begin(); p != group->end(); ++p){
            functions.dispatch(*p);
        }
    }

    void visitShape(SgShape* shape){
        if(material)
            shape->setMaterial(material);
        if(texture)
            shape->setTexture(texture);
    };

private:
    SgMaterial* material;
    SgTexture* texture;
};

class GeometryInfo : public Referenced
{
public:
    bool haveScale;
    Vector3 scale;
    SgNodePtr sgNode;
};
typedef ref_ptr<GeometryInfo> GeometryInfoPtr;

class MaterialInfo : public Referenced
{
public:
    string scriptName;
    vector<string> uri;
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

class SensorInfo : public Referenced
{
public:
    enum SensorType { UNDEFINED=0,  ALTIMETER, CAMERA, CONTACT, DEPTH,
        FORCE_TORQUE, GPS, GPU_RAY, IMU, LOGICAL_CAMERA,
        MAGNETOMETER, MULTICAMERA, RAY, RFID, RFIDTAG,
        SONAR, WIRELESS_RECEIVER, WIRELESS_TRANSMITTER };

    typedef boost::variant<bool, int, double, string> ParamVariant;

    string name;
    Affine3 pose;
    SensorType type;
    double update_rate;
    bool always_on;
    typedef map<string, ParamVariant> SensorParam;
    SensorParam param;

    DevicePtr device;

    SensorInfo(){
        pose = cnoid::Affine3::Identity();
        type = UNDEFINED;
        update_rate = 0;
        always_on = true;
        device = 0;
        param.clear();
    }
    SensorInfo(SensorInfo* org){
        name = org->name;
        pose = org->pose;
        type = org->type;
        update_rate = org->update_rate;
        always_on = org->always_on;
        device = org->device;
        param = org->param;
    }

};
typedef ref_ptr<SensorInfo> SensorInfoPtr;


class JointInfo;

class LinkInfo : public Referenced
{
public:
    std::string linkName;
    double m;
    cnoid::Affine3 c;
    cnoid::Matrix3 I;
    cnoid::Affine3 pose;
    bool isRoot;
    vector<VisualInfoPtr> visuals;
    vector<CollisionInfoPtr> collisions;
    vector<SensorInfoPtr> sensors;

    LinkInfo* parent;
    LinkInfo* child;
    LinkInfo* sibling;
    JointInfo* jointInfo;
    Affine3 origin;

    SDFBodyLoaderImpl* impl;
    Body* body;

    LinkInfo(SDFBodyLoaderImpl* impl) : impl(impl){
        m = 0.0;
        c = Affine3::Identity();
        I.setZero();
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

    Link* createLink(Body* body);
    SgNode* convertVisualShape(Matrix3& Rs);
    SgNode* convertCollisionShape(Matrix3& Rs);
    void ConvertForceSensorFrame(SensorInfo* sensorInfo, Affine3& pose);

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
    vector<SensorInfoPtr> sensors;

    JointInfo() {
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
    ostream* os_;
    bool isVerbose;
    std::vector<JointInfoPtr> joints;
    std::map<std::string, LinkInfoPtr> linkdataMap;
    typedef std::map<std::string, SgImagePtr> ImagePathToSgImageMap;
    ImagePathToSgImageMap imagePathToSgImageMap;
    SceneLoader sceneLoader;
    MeshGenerator meshGenerator;

    ostream& os() { return *os_; }

    SDFBodyLoaderImpl();
    ~SDFBodyLoaderImpl();
    void pose2affine(const ignition::math::Pose3d& pose, cnoid::Affine3& out );

    void readSDF(const std::string& filename, vector<ModelInfoPtr>& modelInfos);

    bool load(Body* body, const std::string& filename);
    bool load(BodyItem* item, const std::string& filename);
    BodyPtr load(const std::string& filename);


    void readWorld(sdf::ElementPtr world, vector<ModelInfoPtr>& modelInfos);
    void readInclude(sdf::ElementPtr include, vector<ModelInfoPtr>& modelInfos);
    void readModel(sdf::ElementPtr model, vector<ModelInfoPtr>& modelInfos);
    void readLink(sdf::ElementPtr link, std::map<std::string, LinkInfoPtr>& linksInfos);
    void readJoint(sdf::ElementPtr joint, vector<JointInfoPtr>& jointInfos);
    void readVisual(sdf::ElementPtr visual, vector<VisualInfoPtr>& visuals);
    void readCollision(sdf::ElementPtr collision, vector<CollisionInfoPtr>& collisions);
    void readSensor(sdf::ElementPtr sensor, vector<SensorInfoPtr>& sensors);
    GeometryInfo* readGeometry(sdf::ElementPtr geometry);
    MaterialInfo* readMaterial(sdf::ElementPtr material);
    SgMaterial* convertMaterial(MaterialInfo* material, float transparency);
    SgTexture* convertTexture(MaterialInfo* material, const string& modelName);
    void readCamera(sdf::ElementPtr camera, SensorInfo* cameraInfo);
    void readForceSensor(sdf::ElementPtr el, SensorInfo* sensorInfo);
    void readRay(sdf::ElementPtr el, SensorInfo* sensorInfo);
    void readIMU(sdf::ElementPtr el, SensorInfo* sensorInfo, int flg);
    //void readMultiCamera(sdf::ElementPtr el, SensorInfo* sensorInfo);
    bool createBody(Body* body, ModelInfo* model);
    bool convertAngle(double* angle, double min, double max, const double defaultValue,
                                          const std::string name);

private:
    SDFLoaderPseudoGazeboColor* gazeboColor;

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


Link* LinkInfo::createLink(Body* body_)
{
    body = body_;
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

    link->setVisualShape(convertVisualShape(link->Rs()));
    link->setCollisionShape(convertCollisionShape(link->Rs()));

    link->setJointType( jointInfo->convertJointType() );

    Vector3 axis;
    jointInfo->convertAxis(axis);
    link->setJointAxis(link->Rs() * axis);

    double maxlimit = numeric_limits<double>::max();
    link->setJointRange(jointInfo->lower, jointInfo->upper);
    link->setJointVelocityRange(getLimitValue(-jointInfo->velocity, -maxlimit),
            getLimitValue(jointInfo->velocity, +maxlimit));

    for(vector<SensorInfoPtr>::iterator it = sensors.begin(); it != sensors.end();
            it++){
        Device* device = (*it)->device;
        if(!device)
            continue;

        device->setLink(link);
        device->setName((*it)->name);

        if((*it)->type == SensorInfo::SensorType::FORCE_TORQUE){
            ConvertForceSensorFrame(*it, (*it)->pose);
        }

        const Matrix3& RsT = link->Rs();
        Affine3 pose0 = jointInfo->pose.inverse() * (*it)->pose;
        device->setLocalTranslation(RsT * pose0.translation());
        device->setLocalRotation(RsT * pose0.linear());
        body->addDevice(device);
    }

    for(LinkInfo* linkInfo = child; linkInfo; linkInfo = linkInfo->sibling){
        link->appendChild(linkInfo->createLink(body));
    }

    return link;
}


void LinkInfo::ConvertForceSensorFrame(SensorInfo* sensorInfo, Affine3& pose)
{
    SensorInfo::SensorParam& param = sensorInfo->param;

    SensorInfo::SensorParam::iterator it = param.find("frame");
    if(it!=param.end()){
        string& frame = boost::get<string>(it->second);
        if(frame=="parent"){
            pose.linear() = parent->pose.linear(); //! \todo do debug
        }else if(frame=="child"){
            pose.linear() = pose.linear();
        }else if(frame=="sensor"){
            pose.linear() = origin.linear();
        }
    }

    it = param.find("measure_direction");
    if(it!=param.end()){
        string& direction = boost::get<string>(it->second);
        if(direction=="parent_to_child"){
            // TODO
        }else if(direction=="child_to_parent"){
            //
        }
    }
}


SgNode* LinkInfo::convertVisualShape(Matrix3& Rs)
{
    if(!visuals.size())
        return 0;

    SgPosTransform* transformRs = new SgPosTransform;
    transformRs->setRotation(Rs);

    for(vector<VisualInfoPtr>::iterator it = visuals.begin();
            it != visuals.end(); it++){
        SgMaterial* material = impl->convertMaterial((*it)->material.get(), (*it)->transparency);
        SgTexture* texture = impl->convertTexture((*it)->material.get(), body->modelName());

        GeometryInfo* geometry = (*it)->geometry.get();
        if(geometry){
            SgNode* node = geometry->sgNode.get();
            if(!node)
                continue;

            MaterialTextureInstructor materialTextureInstructor(material, texture);
            materialTextureInstructor.set(node);

            SgShape* shape = dynamic_cast<SgShape*>(node);
            Affine3& pose = (*it)->pose;
            if(shape && shape->getOrCreateMesh()->primitiveType() == SgMesh::CYLINDER){
                pose *= AngleAxis(radian(90), Vector3::UnitX());
            }
            Affine3 pose_ = jointInfo->pose.inverse() * pose;

            SgPosTransformPtr transform = new SgPosTransform(pose_);
            if(geometry->haveScale){
                const Vector3& scale = geometry->scale;
                SgScaleTransformPtr scaletrans = new SgScaleTransform;
                scaletrans->setScale(scale);
                scaletrans->addChild(node);
                transform->addChild(scaletrans);
            }else{
                transform->addChild(node);
            }
            transformRs->addChild(transform);
        }
    }

    return transformRs;
}


SgNode* LinkInfo::convertCollisionShape(Matrix3& Rs)
{
    if(!collisions.size())
        return 0;

    SgPosTransform* transformRs = new SgPosTransform;
    transformRs->setRotation(Rs);

    for(vector<CollisionInfoPtr>::iterator it = collisions.begin();
            it != collisions.end(); it++){
        GeometryInfo* geometry = (*it)->geometry.get();
        if(geometry){
            SgNode* node = geometry->sgNode.get();
            if(node)
                continue;

            SgShape* shape = dynamic_cast<SgShape*>(node);
            Affine3& pose = (*it)->pose;
            if(shape && shape->getOrCreateMesh()->primitiveType() == SgMesh::CYLINDER){
                pose *= AngleAxis(radian(90), Vector3::UnitX());
            }
            Affine3 pose_ = jointInfo->pose.inverse() * pose;

            SgPosTransformPtr transform = new SgPosTransform(pose_);
            if(geometry->haveScale){
                const Vector3& scale = geometry->scale;
                SgScaleTransformPtr scaletrans = new SgScaleTransform;
                scaletrans->setScale(scale);
                scaletrans->addChild(node);
                transform->addChild(scaletrans);
            }else{
                transform->addChild(node);
            }
            transformRs->addChild(transform);
        }
    }

    return transformRs;
}


void ModelInfo::nestModel(string parentName, ModelInfo* orgModel)
{
    //! \todo do debug
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

    // There must not be more than two parents
    for(vector<JointInfoPtr>::iterator it = jointInfos.begin();
                it != jointInfos.end(); it++){
        LinkInfo* parent = linkInfos[(*it)->parentName].get();
        LinkInfo* child = linkInfos[(*it)->childName].get();
        if(parent && child && !child->parent){
            (*it)->parent = parent;
            (*it)->child = child;
            child->jointInfo = (*it).get();
            parent->addChild(child);
        }else{
               //! \todo Error indication
        }
    }

    // If there is a Link specified as base, it becomes root.
    for(map<string, LinkInfoPtr>::iterator it = linkInfos.begin();
            it != linkInfos.end(); it++){
        LinkInfo* link = it->second;
        if(link->isRoot){
            root = link;
        }
    }

    // If there is no root, Link without parent is root
    for(map<string, LinkInfoPtr>::iterator it = linkInfos.begin();
                it != linkInfos.end(); it++){
        LinkInfo* link = it->second;
        if(!link->parent){
            if(root){
                ; // There are other links with no parent.
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

}


SDFBodyLoader::~SDFBodyLoader()
{
    delete impl;
}


SDFBodyLoaderImpl::~SDFBodyLoaderImpl()
{
    delete gazeboColor;
}


void SDFBodyLoader::setMessageSink(std::ostream& os)
{
    impl->os_ = &os;
    impl->sceneLoader.setMessageSink(os);
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


void SDFBodyLoaderImpl::pose2affine(const Pose3d& pose, Affine3& out)
{
    Vector3 trans(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());
    out.translation() = trans;
    Quaternion R(pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W());
    out.linear() = R.matrix();

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
        BodyPtr body = new Body();
        createBody(body, models[i].get());

        BodyItemPtr bodyItem;
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
        return false; // The message fails.
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
        if (!root) {
            throw std::out_of_range("root element is NULL");
        }

        if(root->HasElement("world")){
            for(sdf::ElementPtr world = root->GetElement("world"); world;
                    world = world->GetNextElement("world")){
                readWorld(world, models);
            }
        }

        if(root->HasElement("model")){
            for(sdf::ElementPtr model = root->GetElement("model"); model;
                    model = model->GetNextElement("model")){
                readModel(model, models);
            }
        }

    } catch(const std::exception& ex){
        throw ex;
    }
}


void SDFBodyLoaderImpl::readWorld(sdf::ElementPtr world, vector<ModelInfoPtr>& modelInfos)
{
    if(world->HasElement("model")){
        for(sdf::ElementPtr model = world->GetElement("model"); model;
                model = model->GetNextElement("model")){
            readModel(model, modelInfos);
        }
    }

    if(world->HasElement("include")){
        for(sdf::ElementPtr include = world->GetElement("include"); include;
                include = include->GetNextElement("include")){
            readInclude(include, modelInfos);
        }
    }
}


void SDFBodyLoaderImpl::readInclude(sdf::ElementPtr include, vector<ModelInfoPtr>& modelInfos)
{
    for(sdf::ElementPtr element = include->GetFirstElement(); element; element = element->GetNextElement() ){
        if(element->GetName()=="uri"){
            string uri = element->Get<std::string>("uri");
            string url = sdf::findFile(uri);
            if(url.empty())
                continue;
            try{
                readSDF(url, modelInfos);
            }catch(const std::exception& ex){
                throw ex;
            }
        }else if(element->GetName()=="name"){
            modelInfos.back()->name = element->Get<string>("name");
        }else if(element->GetName()=="static"){
            modelInfos.back()->isStatic = element->Get<bool>("static");
        }else if(element->GetName()=="pose"){
            pose2affine(element->Get<Pose3d>("pose"), modelInfos.back()->pose);
        }
    }
}


void SDFBodyLoaderImpl::readModel(sdf::ElementPtr model, vector<ModelInfoPtr>& modelInfos)
{
    ModelInfoPtr modelInfo(new ModelInfo());

    modelInfo->name = model->Get<std::string>("name");

    if(model->HasElement("static")){
        modelInfo->isStatic = model->Get<bool>("static");
    }

    if(model->HasElement("self_collide")){
        modelInfo->selfCollide = model->Get<bool>("self_collide");
    }

    if(model->HasElement("include")){
        for(sdf::ElementPtr include = model->GetElement("include"); include;
                include =include->GetNextElement("include")){
                    readInclude(include, modelInfo->nestedModels);
                }
    }

    if(model->HasElement("model")){
        for(sdf::ElementPtr nestedModel = model->GetElement("model"); nestedModel;
                nestedModel = nestedModel->GetNextElement("model")){
                    readModel(nestedModel, modelInfo->nestedModels);
                }
    }

    if(model->HasElement("link")){
        for(sdf::ElementPtr link = model->GetElement("link"); link;
                link = link->GetNextElement("link")){
                    readLink(link, modelInfo->linkInfos);
                }
    }

    if(model->HasElement("joint")){
        for(sdf::ElementPtr joint = model->GetElement("joint"); joint;
                joint = joint->GetNextElement("joint")){
                    readJoint(joint, modelInfo->jointInfos);
                }
    }

    modelInfos.push_back(modelInfo);

}


void SDFBodyLoaderImpl::readLink(sdf::ElementPtr link, std::map<std::string, LinkInfoPtr>& linkInfos)
{
    LinkInfoPtr linkdata(new LinkInfo(this));

    linkdata->linkName = link->Get<std::string>("name");

    if (isVerbose) {
        os() << " link name " << linkdata->linkName << std::endl;
    }

    if(link->HasElement("must_be_base_link")){
        linkdata->isRoot = link->Get<bool>("must_be_base_link");
    }

    if(link->HasElement("pose")){
        pose2affine(link->Get<Pose3d>("pose"), linkdata->pose);
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
            pose2affine(inertial->Get<Pose3d>("pose"), linkdata->c);
        }
    }

    if(link->HasElement("visual")) {
        for(sdf::ElementPtr visual = link->GetElement("visual"); visual;
                visual = visual->GetNextElement("visual")){
            readVisual(visual, linkdata->visuals);
        }
    }

    if(link->HasElement("collision")){
        for(sdf::ElementPtr collision = link->GetElement("collision"); collision;
                collision = collision->GetNextElement("collision")){
            readCollision(collision, linkdata->collisions);
        }
    }

    if(link->HasElement("sensor")){
        for(sdf::ElementPtr sensor = link->GetElement("sensor"); sensor;
                sensor = sensor->GetNextElement("sensor")){
            readSensor(sensor, linkdata->sensors);
        }
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

    if (isVerbose) {
        os() << " joint name " << jointdata->jointName << std::endl;
    }

    if(joint->HasElement("pose")){
        pose2affine(joint->Get<Pose3d>("pose"), jointdata->pose);
    } else {
        jointdata->pose = cnoid::Affine3::Identity();
    }
    if(isVerbose){
        os() << jointdata->pose.matrix() << std::endl;
    }
    if(joint->HasElement("axis")){
        sdf::ElementPtr axis = joint->GetElement("axis");
        ignition::math::Vector3d xyz = axis->Get<ignition::math::Vector3d>("xyz");
        jointdata->axis[0] = xyz.X();
        jointdata->axis[1] = xyz.Y();
        jointdata->axis[2] = xyz.Z();

        jointdata->useparent = false;
        if(axis->HasElement("use_parent_model_frame")){
            jointdata->useparent = axis->Get<bool>("use_parent_model_frame");
        }

        if(axis->HasElement("limit")){
            sdf::ElementPtr limit = axis->GetElement("limit");
            jointdata->lower = limit->Get<double>("lower");
            jointdata->upper = limit->Get<double>("upper");
            if(limit->HasElement("velocity")){
                jointdata->velocity = limit->Get<double>("velocity");
            }
        } else {
            os() << "Warning: '" << jointdata->jointName << "' limit not found" << std::endl;
        }
    }

    if(joint->HasElement("sensor")){
        for(sdf::ElementPtr sensor = joint->GetElement("sensor"); sensor;
                sensor = sensor->GetNextElement("sensor")){
            readSensor(sensor, jointdata->sensors);
        }
    }

    jointInfos.push_back(jointdata);

}


void SDFBodyLoaderImpl::readVisual(sdf::ElementPtr visual, vector<VisualInfoPtr>& visuals)
{
    VisualInfoPtr visualInfo(new VisualInfo());

    if (visual->HasElement("pose")) {
        pose2affine(visual->Get<Pose3d>("pose"), visualInfo->pose);
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
        os() << "Warning: '" << visual->Get<std::string>("name") << "' geometry not found" << std::endl;
    }

    visuals.push_back(visualInfo);
}


void SDFBodyLoaderImpl::readCollision(sdf::ElementPtr collision, vector<CollisionInfoPtr>& collisions)
{
    CollisionInfoPtr collisionInfo(new CollisionInfo());

    if (collision->HasElement("pose")) {
        pose2affine(collision->Get<Pose3d>("pose"), collisionInfo->pose);
    } else {
        collisionInfo->pose = cnoid::Affine3::Identity();
    }

    if (collision->HasElement("geometry")) {
        collisionInfo->geometry = readGeometry(collision->GetElement("geometry"));
    } else {
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
            for(sdf::ElementPtr uri = script->GetElement("uri"); uri; uri = uri->GetNextElement("uri")){
                materialInfo->uri.push_back(uri->Get<string>());
            }
        }
    }

    materialInfo->ambient.r = -1;
    if (material->HasElement("ambient")) {
        materialInfo->ambient = material->Get<sdf::Color>("ambient");
    }

    materialInfo->diffuse.r = -1;
    if (material->HasElement("diffuse")) {
        materialInfo->diffuse = material->Get<sdf::Color>("diffuse");
    }

    materialInfo->specular.r = -1;
    if (material->HasElement("specular")) {
        materialInfo->specular = material->Get<sdf::Color>("specular");
    }

    materialInfo->emissive.r = -1;
    if (material->HasElement("emissive")) {
        materialInfo->emissive = material->Get<sdf::Color>("emissive");
    }

    return materialInfo;
}


GeometryInfo* SDFBodyLoaderImpl::readGeometry(sdf::ElementPtr geometry)
{
    GeometryInfo* geometryInfo = new GeometryInfo;

    geometryInfo->haveScale = false;

    for(sdf::ElementPtr el = geometry->GetFirstElement(); el; el = el->GetNextElement()) {
        if(el->GetName() == "mesh") {
            std::string url = sdf::findFile(el->Get<std::string>("uri"));
            std::string urllower = url;
            boost::algorithm::to_lower(urllower);

            if (!url.empty()) {
                if (isVerbose) {
                    os() << "     read mesh " << url << std::endl;
                }

                float scale0 = 1;

                if (boost::algorithm::iends_with(urllower, "dae")){
                    TiXmlDocument xmlDoc;
                    xmlDoc.LoadFile(url);
                    string dae_axis = "Y_UP";
                    if(!xmlDoc.Error()) {
                        TiXmlElement * colladaXml = xmlDoc.FirstChildElement("COLLADA");
                        if(colladaXml) {
                            TiXmlElement *assetXml = colladaXml->FirstChildElement("asset");
                            if(assetXml) {
                                TiXmlElement *unitXml = assetXml->FirstChildElement("unit");
                                if (unitXml && unitXml->Attribute("meter") &&
                                    unitXml->QueryFloatAttribute("meter", &scale0) == TIXML_SUCCESS) {
                                }
                                TiXmlElement *up_axisXml = assetXml->FirstChildElement("up_axis");
                                if (up_axisXml) {
                                    dae_axis = up_axisXml->GetText();
                                }
                            }
                        }
                    }else{
                        cout << xmlDoc.ErrorDesc() << endl;
                    }

                    SgNode* node = sceneLoader.load(url);

                    if (dae_axis == "Z_UP") {
                        SgPosTransformPtr transform = new SgPosTransform;
                        transform->setRotation(AngleAxis(radian(90), Vector3::UnitX()));
                        transform->addChild(node);
                        geometryInfo->sgNode = transform;
                    }else{
                        geometryInfo->sgNode = node;
                    }
                } else {
                    geometryInfo->sgNode = sceneLoader.load(url);
                }

                if(el->HasElement("scale")){
                    ignition::math::Vector3d scale = el->Get<ignition::math::Vector3d>("scale");
                    geometryInfo->scale = (double)scale0 * Vector3(scale.X(), scale.Y(), scale.Z());
                    geometryInfo->haveScale = true;
                } else {
                    geometryInfo->scale = Vector3(1,1,1);
                }

            } else {
                MessageView::instance()->putln(
                    fmt::format(_("{} not found"), el->Get<std::string>("uri")),
                    MessageView::WARNING
                    );
            }

        } else if(el->GetName() == "box"){
            SgShapePtr shape = new SgShape;
            ignition::math::Vector3d size = el->Get<ignition::math::Vector3d>("size");
            shape->setMesh(meshGenerator.generateBox(Vector3(size.X(), size.Y(), size.Z())));
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
            geometryInfo->sgNode = shape    ;
        } else if(el->GetName() == "empty"){
            geometryInfo->sgNode = 0;
        }
    }

    return geometryInfo;
}


void SDFBodyLoaderImpl::readSensor(sdf::ElementPtr sensor, vector<SensorInfoPtr>& sensors)
{
    if (!sensor->HasAttribute("name") || !sensor->HasAttribute("type")) {
        return;
    }

    SensorInfoPtr sensorInfo(new SensorInfo);

    sensorInfo->name = sensor->Get<std::string>("name");
    string type = sensor->Get<std::string>("type");

    if(sensor->HasElement("pose"))
        pose2affine(sensor->Get<Pose3d>("pose"), sensorInfo->pose);

    if(sensor->HasElement("update_rate"))
        sensorInfo->update_rate = sensor->Get<double>("update_rate");

    if(sensor->HasElement("always_on"))
        sensorInfo->always_on = sensor->Get<bool>("always_on");

    if (type == "camera" || type == "depth") {

        if(!sensor->HasElement("camera"))
            return;
        if(type=="camera")
            sensorInfo->type = SensorInfo::CAMERA;
        else if(type=="depth")
            sensorInfo->type = SensorInfo::DEPTH;

        readCamera(sensor->GetElement("camera"), sensorInfo.get());

    } else if (type == "force_torque") {

        sensorInfo->type = SensorInfo::FORCE_TORQUE;
        readForceSensor(sensor, sensorInfo.get());

    } else if (type == "gpu_ray") {

        sensorInfo->type = SensorInfo::GPU_RAY;
        readRay(sensor, sensorInfo.get());

    } else if (type == "imu") {

        sensorInfo->type = SensorInfo::IMU;
        readIMU(sensor, sensorInfo.get(), 0);

        SensorInfoPtr sensorInfo0(new SensorInfo(sensorInfo));
        readIMU(sensor, sensorInfo0.get(), 1);
        sensors.push_back(sensorInfo0);

        sensorInfo->name += "_accel";
        sensorInfo0->name += "_rate";

    } else if (type == "ray") {

        sensorInfo->type = SensorInfo::RAY;
        readRay(sensor, sensorInfo.get());

    } else {
        // Error message
    }

    sensors.push_back(sensorInfo);
}


void SDFBodyLoaderImpl::readCamera(sdf::ElementPtr el, SensorInfo* cameraInfo)
{
    Camera* camera;
    if(cameraInfo->type==SensorInfo::CAMERA)
        camera = new Camera;
    else if(cameraInfo->type==SensorInfo::DEPTH){
        camera = new RangeCamera;
        dynamic_cast<RangeCamera*>(camera)->setOrganized(true);
    }else
        return;

    if(el->HasElement("horizontal_fov")){
        camera->setFieldOfView( el->Get<double>("horizontal_fov") );
    }

    if(el->HasElement("clip")){
        sdf::ElementPtr clip = el->GetElement("clip");
        if(clip->HasElement("near"))
            camera->setNearClipDistance( clip->Get<double>("near") );
        if(clip->HasElement("far"))
            camera->setFarClipDistance( clip->Get<double>("far") );
    }

    if(el->HasElement("image")){
        sdf::ElementPtr image = el->GetElement("image");
        if(image->HasElement("width") && image->HasElement("height"))
            camera->setResolution( image->Get<int>("width"),
                    image->Get<int>("height"));
        if(image->HasElement("format")){
            string format = image->Get<string>("format");
            if(format=="R8G8B8")
                camera->setImageType(Camera::COLOR_IMAGE);
            else if(format=="L8")
                camera->setImageType(Camera::GRAYSCALE_IMAGE);
            else   //  unsupported image format
                ;
        }
    }

    if(cameraInfo->type==SensorInfo::DEPTH){
        camera->setImageType(Camera::COLOR_IMAGE);
    }

    if(cameraInfo->update_rate)
        camera->setFrameRate(cameraInfo->update_rate);

    camera->on(cameraInfo->always_on);

    cameraInfo->device = camera;
}


void SDFBodyLoaderImpl::readForceSensor(sdf::ElementPtr el, SensorInfo* sensorInfo)
{
    ForceSensorPtr fsensor = new ForceSensor;

    if(el->HasElement("force_trque")){
        sdf::ElementPtr el0 = el->GetElement("force_torque");
        if(el0->HasElement("frame")){
            sensorInfo->param["frame"] = el0->Get<string>("frame");
        }
        if(el0->HasElement("measure_direction")){
            sensorInfo->param["measure_direction"] = el0->Get<string>("measure_direction");
        }
    }

    sensorInfo->device = fsensor;
}


bool SDFBodyLoaderImpl::convertAngle(double* angle, double min, double max, const double defaultValue,
                                      const std::string name)
{
    double result;
    double dmin;
    double dmax;

    result = defaultValue;

    if (! angle) {
        return false;
    } else if (min > max) {
        MessageView::instance()->putln(
            fmt::format(_("max_angle must be greater of equal to min_angle [ min={0} max={1} ] ({2})"),
                        min, max, name),
            MessageView::ERROR
            );
        return false;
    } else if (min != max) {
        dmin = (min < 0.0) ? (min * -1.0d) : min;
        dmax = (max < 0.0) ? (max * -1.0d) : max;

        if (dmin == dmax) {
            /*
              range sensor possible range threshold.
              see VisionRenderer::initializeCamera(). (in src/BodyPlugin/GLVisionSimulatorItem.cpp)
             */
            const double thresh = (170.0 * M_PI / 180.0); // 170[deg]

            result = dmin + dmax;

            if (result >= thresh) {
                MessageView::instance()->putln(
                    fmt::format(_("sensor range too big [ setting={0} threshold={1} ] ({2})"),
                                result, thresh, name),
                    MessageView::ERROR
                );
                return false;
            }
        } else {
           MessageView::instance()->putln(
               fmt::format(_("unable to convert angle, use default value "
                             "[ min={0} max={1} default={2} ] ({3})"),
                           dmin, dmax, defaultValue, name),
               MessageView::WARNING
               );
        }
    } else {
        result = 0.017;

        if (min != 0.0) {
           MessageView::instance()->putln(
               fmt::format(_("unable to convert angle, use alternate value "
                             "[ min={0} max={1} alternate={2} ] ({3})"),
                           min, max, result, name),
               MessageView::WARNING
               );
        }
    }

    *angle = result;

    return true;
}


void SDFBodyLoaderImpl::readRay(sdf::ElementPtr el, SensorInfo* sensorInfo)
{
    RangeSensorPtr  rsensor = new RangeSensor;

    if(el->HasElement("ray")){
        sdf::ElementPtr ray = el->GetElement("ray");

        if(ray->HasElement("scan")){
            sdf::ElementPtr scan = ray->GetElement("scan");

            if(scan->HasElement("horizontal")){
               sdf::ElementPtr horizontal = scan->GetElement("horizontal");
               unsigned int samples = 640;
               double resolution = 1;

               if( horizontal->HasElement("samples")){
                   samples = horizontal->Get<unsigned int>("samples");
               }
               if(horizontal->HasElement("resolution")){
                   resolution = horizontal->Get<double>("resolution");
               }

               double min_angle, max_angle, angle;
               min_angle = max_angle = angle = 0;
               if(horizontal->HasElement("min_angle")){
                   min_angle = horizontal->Get<double>("min_angle");
               }
               if(horizontal->HasElement("max_angle")){
                   max_angle = horizontal->Get<double>("max_angle");
               }
               if (convertAngle(&angle, min_angle, max_angle,
                       rsensor->yawRange(), sensorInfo->name)) {
                   rsensor->setYawRange(angle);
               }

               int num_points = static_cast<int>(samples * resolution);
               double yawStep = (num_points > 1) ? angle / (num_points - 1) : 0;
               rsensor->setYawStep(yawStep);
            }

            if(scan->HasElement("vertical")){
                sdf::ElementPtr vertical = scan->GetElement("vertical");
                unsigned int samples = 1;
                double resolution = 1;

                if( vertical->HasElement("samples")){
                    samples = vertical->Get<unsigned int>("samples");
                }
                if(vertical->HasElement("resolution")){
                    resolution = vertical->Get<double>("resolution");
                }

                double min_angle, max_angle, angle;
                min_angle = max_angle = angle = 0;
                if(vertical->HasElement("min_angle")){
                    min_angle = vertical->Get<double>("min_angle");
                }
                if(vertical->HasElement("max_angle")){
                    max_angle = vertical->Get<double>("max_angle");
                }
                if (convertAngle(&angle, min_angle, max_angle,
                        rsensor->yawRange(), sensorInfo->name)) {
                    rsensor->setPitchRange(angle);
                }

                int num_points = static_cast<int>(samples * resolution);
                double pitchStep = (num_points > 1) ? angle / (num_points - 1) : 0;
                rsensor->setPitchStep(pitchStep);
            }

        }

        if(ray->HasElement("range")){
            sdf::ElementPtr range = ray->GetElement("range");

            if( range->HasElement("min")){
                rsensor->setMinDistance( range->Get<double>("min") );
            }
            if( range->HasElement("max")){
                rsensor->setMaxDistance( range->Get<double>("max") );
            }
            if( range->HasElement("resolution")){
                double resolution = range->Get<double>("resolution");
            }
        }
    }

    sensorInfo->device = rsensor;
}


void SDFBodyLoaderImpl::readIMU(sdf::ElementPtr el, SensorInfo* sensorInfo, int flg)
{
    if(!flg){
        AccelerationSensorPtr asensor = new AccelerationSensor;
        sensorInfo->device = asensor;
    }else{
        RateGyroSensorPtr gsensor = new RateGyroSensor;
        sensorInfo->device = gsensor;
    }
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

    return createBody(body, models[0].get());

}


bool SDFBodyLoaderImpl::createBody(Body* body, ModelInfo* model)
{
    body->setModelName(model->name);

    model->nestModel("", model);
    LinkInfo* root = model->findRootLinks();

    if(!root->jointInfo){
        JointInfoPtr jointInfo = new JointInfo();
        jointInfo->jointType = "free";
        model->jointInfos.push_back(jointInfo);
        root->jointInfo = jointInfo;
    }
    Link* rootLink = root->createLink(body);

    body->setRootLink(rootLink);

    // Since there is no Joint ID in sdf, attach joint ID in order.
    for(int i=0, j=0; i<body->numLinks(); i++){
        Link::JointType type = body->link(i)->jointType();
        if( type != Link::FIXED_JOINT && type != Link::FREE_JOINT )
            body->link(i)->setJointId(j++);
    }

    // DeviceID
    map<std::string, uint32_t> ids;
    for(int i=0; i<body->numDevices(); i++){
        const string& type = body->device(i)->typeName();
        if (ids.find(type) != ids.end()) {
            body->device(i)->setId(++ids[type]);
        } else {
            ids[type] = 0;
            body->device(i)->setId(0);
        }
    }

    body->setRootLink(rootLink); // Call this again to make jointID recognized

    return true;
}


SgTexture* SDFBodyLoaderImpl::convertTexture(MaterialInfo* material, const string& modelName)
{
    if(!material)
        return 0;

    string textureName;
    string texturePath;

    vector<string>& uri = material->uri;
    for(int i=0; i<uri.size(); i++){
        if( boost::algorithm::iends_with( uri[i], "/scripts") ) {
            const string url = sdf::findFile(uri[i]);
            if(url.empty())
                continue;

            Ogre::ResourceGroupManager& gm = Ogre::ResourceGroupManager::getSingleton();
            if (!gm.resourceLocationExists(url, "General")) {
                gm.addResourceLocation(url, "FileSystem", "General", true);
                gm.initialiseResourceGroup("General");

                filesystem::path scriptDir(url);
                if (filesystem::exists(scriptDir) && filesystem::is_directory(scriptDir)) {
                    vector<filesystem::path> files;
                    copy(filesystem::directory_iterator(scriptDir), filesystem::directory_iterator(),
                            back_inserter(files));
                    sort(files.begin(), files.end());

                    for (vector<filesystem::path>::iterator it=files.begin(); it!=files.end(); it++){
                        if (it->filename().extension() == ".material") {
                            filesystem::path fullPath = url / it->filename();
                            try {

                                Ogre::DataStreamPtr stream = gm.openResource( fullPath.string(), "General" );
                                Ogre::MaterialManager::getSingleton().parseScript( stream, "General" );
                                stream->close();
                            } catch (Ogre::Exception& e){
                                cout << e.getFullDescription() << endl;
                            }
                        }
                    }
                }
            }

            Ogre::MaterialPtr ogreMat = Ogre::MaterialManager::getSingleton().getByName( material->scriptName );
            if(ogreMat.get()){
                Ogre::Technique* technique = ogreMat->getTechnique(0);
                if(technique){
                    Ogre::Pass *pass = technique->getPass(0);
                    if(pass){
                        int n = pass->getNumTextureUnitStates();
                        // Support only one
                        if(n)
                            textureName = pass->getTextureUnitState(0)->getTextureName();
                    }
                }
            }

        }else if( boost::algorithm::iends_with( uri[i], "/textures") ){
            texturePath = sdf::findFile(uri[i]);
        }
    }

    if(textureName.empty() || texturePath.empty())
        return 0;

    string textureFile = texturePath + "/" + textureName;
    ImagePathToSgImageMap::iterator p = imagePathToSgImageMap.find(textureFile);

    SgImage* image;
    if(p != imagePathToSgImageMap.end()){
        image = p->second;
    } else {
        try {
            image = new SgImage;
            ImageIO imageIO;
            imageIO.setUpsideDown(true);
            imageIO.load(image->image(), textureFile);
            imagePathToSgImageMap[textureFile] = image;
        } catch(const exception_base& ex){
            cout << boost::get_error_info<error_info_message>(ex) << endl;
            image = 0;
        }
    }

    if(image){
        SgTexture* texture = new SgTexture;
        texture->setImage(image);

        return texture;
    }

    return 0;

}


SgMaterial* SDFBodyLoaderImpl::convertMaterial(MaterialInfo* material, float transparency)
{
    if(!material)
        return 0;

    SgMaterial* sgMaterial;
    SDFLoaderPseudoGazeboColorInfo* cinfo = gazeboColor->get(material->scriptName);
    sgMaterial = cinfo->material;

    sdf::Color& ambient = material->ambient;
    if(ambient.r != -1){
        sgMaterial->setAmbientIntensity((ambient.r + ambient.g + ambient.b) / 3.0f);
    }

    sdf::Color& diffuse = material->diffuse;
    if(diffuse.r != -1){
        sgMaterial->setDiffuseColor(Vector3f(diffuse.r, diffuse.g, diffuse.b));
    }

    sdf::Color& specular = material->specular;
    if(specular.r != -1){
        sgMaterial->setSpecularColor(Vector3f(specular.r, specular.g, specular.b));
    }

    sdf::Color& emissive = material->emissive;
    if(emissive.r != -1){
        sgMaterial->setEmissiveColor(Vector3f(emissive.r, emissive.g, emissive.b));
    }

    if (transparency >= 0.0) {
        sgMaterial->setTransparency(transparency);
    }

    return sgMaterial;
}
