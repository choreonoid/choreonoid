/*!
 * @brief  Defines the minimum processing for performing pasing file for DAE. 
 * @author Hisashi Ikari 
 * @file
 */
#include <map>
#include <vector>
#include <string>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>

#include <cnoid/Body>
#include <cnoid/Link>
#include <cnoid/BasicSensors>
#include <cnoid/Camera>
#include <cnoid/Exception>
#include <cnoid/SceneGraph>
#include <cnoid/ValueTree>

#ifdef _OLD_VERSION
#include <cnoid/ColdetModel>
#endif
#include <cnoid/DaeParser>
#include <cnoid/DaeNode>
#include <cnoid/FileUtil>
#include "ColladaBodyLoader.h"

#include "gettext.h"

using namespace std;
using namespace boost::algorithm;
using boost::format;
using boost::lexical_cast;

namespace cnoid {

typedef map<string, Link*> LoaderLinks;
typedef vector<string>     LoaderJoints;

/*!
 * @brief Creating a model of Joint and Link by using the DaeParser.
 */
class ColladaBodyLoaderImpl
{
public:
    ColladaBodyLoaderImpl();
    ~ColladaBodyLoaderImpl();

    void loadBody(const string& filename, Body& body);

    void convertToBody(Body& body);

    void buildLinks   (DaeNode* extNode, DaeNode* parentNode, Link* parentLink, Body& body, int& jointId);
    void setLink      (DaeNode* extNode, Link* parentLink, Body& body);
    void setJoint     (DaeNode* extNode, DaeNode* parentNode, Link* link, int& jointId);
    void setPosition  (DaeNode* extNode, Link* link);
    void setMass      (DaeNode* extNode, Link* link);
    void setSensor    (DaeNode* extNode, Link* link, Body& body);

#ifdef _OLD_VERSION
    void setColdetModel   (Link* link, SgGroup* shape);
    void createColdetModel(SgGroup* group, SgPosTransform* transParent, ColdetModel* model);
#endif

    Link*     createLink  (Body& body, string name, bool* duplicate);
    DevicePtr createSensor(DaeSensor* sensor);    

protected:
    void throwException(const string& message);
public:
    ostream* os;

protected:
    DaeParser* parser;
    string fileName;

    LoaderLinks  links;
    LoaderJoints joints;
};


ColladaBodyLoader::ColladaBodyLoader()
{
    impl = new ColladaBodyLoaderImpl;        
}


ColladaBodyLoader::~ColladaBodyLoader()
{
    if (impl) delete impl;
}


const char* ColladaBodyLoader::format() const
{
    return "Collada-1.5";
}


void ColladaBodyLoader::setMessageSink(std::ostream& os)
{
    impl->os = &os;
}


void ColladaBodyLoader::setVerbose(bool on)
{
}


void ColladaBodyLoader::enableShapeLoading(bool on)
{
}
    

void ColladaBodyLoader::setDefaultDivisionNumber(int n)
{
}

   
ColladaBodyLoaderImpl::ColladaBodyLoaderImpl()
{
}


ColladaBodyLoaderImpl::~ColladaBodyLoaderImpl()
{
}


bool ColladaBodyLoader::load(Body* body, const std::string& filename)
{
    impl->loadBody(filename, *body); 
    return true;
}


void ColladaBodyLoaderImpl::loadBody(const string& fileName, Body& body)
{
    parser = new DaeParser(os);

    this->fileName = fileName;
    parser->parse(fileName);
    convertToBody(body);
    
    body.installCustomizer();

    if (parser) delete parser;
}


void ColladaBodyLoaderImpl::throwException(const string& message)
{
    *os << message << endl;
    ValueNode::SyntaxException error;
    error.setMessage(message);
    throw error;
}


void ColladaBodyLoaderImpl::convertToBody(Body& body)
{
    DaeNode* extNode = parser->findRootLink();

    if (!extNode) {
        SgGroup* scene = parser->createScene(fileName);
        if(scene){
            Link* link = body.createLink();
            link->setName("Root");
            link->setShape(scene);
            link->setMass(1.0);
            link->setInertia(Matrix3::Identity());
            body.setRootLink(link);
            body.setModelName(getBasename(fileName));
        }
        return;
    }

    int jointId = 0;
    links.clear();
    joints.clear();

    bool duplicate;
    Link* link = createLink(body, static_cast<DaeLink*>(extNode)->name, &duplicate);
    link->setName(static_cast<DaeLink*>(extNode)->name);

    body.setRootLink(link);
    // !!! important !!!
    // There is no category in the joint of collada.
    // But joint of the root node must be "FREE-TYPE" or "FIXED-TYPE".
    // Set to "FIXED-TYPE" if the link of the route had been "grounded". (Ex. PA10)
    // I have to set "FREE-TYPE" otherwise. (Ex. HRP4C, SR1, GR001, etc, etc)
    link->setJointType(extNode->transform.translate[2] == 0 ? Link::FIXED_JOINT : Link::FREE_JOINT); 
    body.setModelName(parser->findRootName());
    setPosition(extNode, link);

    buildLinks(extNode, extNode, link, body, jointId);

    body.updateLinkTree();
}


void ColladaBodyLoaderImpl::buildLinks(DaeNode* extNode, DaeNode* parentNode, Link* parentLink, Body& body, int& jointId)
{
    bool duplicate; Link* link = NULL;


    if (DaeLink* extLink = dynamic_cast<DaeLink*>(extNode)) {
        // It will record the link.
        // The link and joint is one in two. 
        // In addition, link will be child relationship (except root) in the parent joint always.
        setLink(extNode, parentLink, body);
        setMass(extLink, parentLink);
       
        for (DaeLinkChildren::iterator iterl = extLink->children.begin(); iterl != extLink->children.end(); iterl++) {
            DaeNode* extJoint = parser->findJointByLink(*iterl);
            buildLinks(extJoint, extLink, parentLink, body, jointId);
        }

    } else
        if (DaeJoint* extJoint = dynamic_cast<DaeJoint*>(extNode)) {
            if (!parentLink) {
                throwException((format(_("joint node can not be root link"))).str());
            }
            // It will record the link.
            link = createLink(body, static_cast<DaeJoint*>(extNode)->name, &duplicate);
            // The joint and link is one in the link.
            setJoint(extNode, parentNode, link, jointId);
            parentLink->appendChild(link);

            for (DaeJointChildren::iterator iterj = extJoint->children.begin(); iterj != extJoint->children.end(); iterj++) {
                // Joint is same as link. (Comprehension)
                DaeNode* extLink = parser->findLinkByJoint(*iterj);
                buildLinks(extLink, extJoint, link, body, jointId);
            }

        } else {
            return;
        }
}


void ColladaBodyLoaderImpl::setMass(DaeNode* extNode, Link* targetLink)
{
    if (DaeLink*  extLink = dynamic_cast<DaeLink*>(extNode)) {
        DaeRigid* rigid   = static_cast <DaeRigid*>(parser->findRigidByLink(extLink->name));
        if (!rigid->massFrame) {
            *os << format(_("[WARNING]there is no mass:%1%")) % rigid->name << endl;
            return;
        }
        DaeNode* mass = static_cast<DaeNode*>(rigid->massFrame.get()); 

        // We must use only the last tranlate of mass_frame on collada.
        // All translate is a translation from the origin.
        if (mass->transform.translates.size() <= 0) targetLink->setCenterOfMass(Vector3(0.0, 0.0, 0.0));
        else targetLink->setCenterOfMass(mass->transform.translates[mass->transform.translates.size() - 1]);
        targetLink->setMass(rigid->mass);

        // It defines only the diagonal elements in collada.
        // And it is x,z,y-values rather than x,y,z-values.
        Matrix3d mat3 = Matrix3d::Zero();
        mat3.diagonal() << rigid->inertia[0], rigid->inertia[2], rigid->inertia[1];
        targetLink->setInertia(mat3);

    }
}


Link* ColladaBodyLoaderImpl::createLink(Body& body, string name, bool* duplicate)
{
    // It will again use the previous link of the same name.
    LoaderLinks::iterator iter = links.find(name);
    Link* result =  (iter == links.end() ? body.createLink() : iter->second);
    *duplicate   = !(iter == links.end()); 
    if (iter == links.end()) {
        links.insert(pair<string, Link*>(name, result));
    }
    return result;
}


void ColladaBodyLoaderImpl::setLink(DaeNode* extNode, Link* parentLink, Body& body)
{
    // joint and link and rigid are associated with attributes of name.
    DaeNode* value = parser->findNode(static_cast<DaeLink*>(extNode)->name);

    // Using the joint name only.
    SgInvariantGroup* group = (!(parentLink->shape()) ? new SgInvariantGroup : static_cast<SgInvariantGroup*>(parentLink->shape()));
    parser->createNode(value, static_cast<SgGroup*>(group));
    if (!(parentLink->shape())) parentLink->setShape(group);                

#ifdef _OLD_VERSION
    setColdetModel(parentLink, group);
#endif
    setSensor(extNode, parentLink, body);
}


void ColladaBodyLoaderImpl::setJoint(DaeNode* extNode, DaeNode* parentNode, Link* link, int& jointId)
{
    // joint and link and rigid are associated with attributes of name.
    string name = static_cast<DaeJoint*>(extNode)->name;
    link->setName(name);

    vector<string> split;
    boost::algorithm::split(split, extNode->id, boost::is_any_of("/"));

    if (istarts_with(split[split.size() - 1], "jointsid")) {
        // If the data was converted to dae from wrl in export-collada of OpenHRP.
        // There is no jointid to dae, but I substitute it with a prefix that jointsid to sid.
        // If it is a prefix, I will use "THIS NUMBER". 
        // Ex. jointsid29(in collada) -> jointid=29(in wrl).
        link->setJointId(lexical_cast<int>(split[split.size() - 1].substr(8)));  
    } else {
        // If the prefix is different, I automatically grants.
        link->setJointId(jointId);  
    }

    DaeJoint* joint = static_cast<DaeJoint*>(extNode);
    if (1 < joint->revolutes.size()) {
        throwException((format(_("we are sorry, It does not support axis more than one."))).str());
    }
    for (DaeRevoluteChildren::iterator iterr  = joint->revolutes.begin();
         iterr != joint->revolutes.end();
         iterr++) {
        // Collada does not impose a joint-type.
        link->setJointType (Link::ROTATIONAL_JOINT);
        link->setJointAxis ((*iterr)->axis);
        link->setJointRange((*iterr)->limitMin, (*iterr)->limitMax);
    }
    setPosition(extNode, link);

    try {
        DaeActuator* actuator = static_cast<DaeActuator*>(parser->findActuator(joint->id));
        link->setEquivalentRotorInertia(actuator->rotorInertia);        
        link->setJointVelocityRange(actuator->noLoadSpeed, actuator->maxSpeed); 

    } catch (...) { /* actuator is not required. */ }

    // joint-id must be unique each link(link same as joint).
    jointId++;
}


void ColladaBodyLoaderImpl::setPosition(DaeNode* extNode, Link* link)
{
    SgGroup* sgTransParent = NULL; SgGroup* sgTransChild = NULL;
    parser->createTransform(extNode, &sgTransParent, &sgTransChild);

    // it does not need to rotate and translation of the parent.
    link->setPosition         (extNode->transform.rotate.matrix(), extNode->transform.translate.matrix());
    link->setOffsetTranslation(extNode->transform.translate.matrix());
    link->setOffsetRotation   (extNode->transform.rotate.matrix());
}


void ColladaBodyLoaderImpl::setSensor(DaeNode* extNode, Link* link, Body& body)
{
    DaeLink* nodeLink = static_cast<DaeLink*>(extNode);

    DaeResultSensors* sensors = parser->findSensor(nodeLink->id);
    if (sensors->size() <= 0) {
        return;
    }

    // ForceSensor--------->force6d
    // GyroSensor---------->no type(empty string)
    // AccelerrationSensor->imu(unit sensor)
    // VisionSensor-------->pin-hole-camera
       
    for (DaeResultSensors::iterator iters = sensors->begin(); iters != sensors->end(); iters++) {
        DaeSensor* sensor = *iters;
        DevicePtr device = createSensor(sensor);
        device->setLink(link);
        device->setLocalTranslation(extNode->transform.translate + sensor->transform.translate);
        device->setLocalRotation   (extNode->transform.rotate.matrix() * sensor->transform.rotate.matrix());
        body.addDevice(device); 
    } 

}

   
DevicePtr ColladaBodyLoaderImpl::createSensor(DaeSensor* sensor)
{
    DevicePtr device;
    if (sensor->type.size() <= 0) {
        device = new RateGyroSensor;
    } else 
        if (iequals(sensor->type, "base_force6d")) {
            device = new ForceSensor;
        } else
            if (iequals(sensor->type, "base_imu")) {
                device = new AccelerationSensor;
            } else 
                if (iequals(sensor->type, "base_pinhole_camera")) {
                    device = new Camera;
                    Camera* camera = static_cast<Camera*>(device.get());
                    camera->setResolution(sensor->imageDimensions[0], sensor->imageDimensions[1]);
                    camera->setNearClipDistance(sensor->focalLength);
                    camera->setFieldOfView (sensor->focalLength);
                } else {
                    throwException((format(_("invalid sensor-type:%1%")) % sensor->type).str());
                }      

    return device;
}
     

#ifdef _OLD_VERSION
void ColladaBodyLoaderImpl::setColdetModel(Link* link, SgGroup* group)
{
    ColdetModelPtr model(std::make_shared<ColdetModel>());
    createColdetModel(group, NULL, model.get());
    model->setName(link->name());
    model->build();
    link->setColdetModel(model);
}
#endif


#ifdef _OLD_VERSION
void ColladaBodyLoaderImpl::createColdetModel(SgGroup* parentGroup, SgPosTransform* transParent, ColdetModel* model)
{
    if (!parentGroup || parentGroup->empty()) {
        return;
    }
    // Get the coordinates and transforms to create a ColdetModel.
    if (SgPosTransform* transform = dynamic_cast<SgPosTransform*>(parentGroup)) {
        for (SgGroup::iterator iter = transform->begin(); iter != transform->end(); iter++) {
            if (SgShape* child = dynamic_cast<SgShape*>((*iter).get())) {

                SgMesh* mesh = child->mesh();
                const int vertexIndex = model->getNumVertices();
                const SgVertexArray* vertices = mesh->vertices();
                const Affine3& t = static_cast<Affine3>(transform->T());

                for (unsigned int i = 0; i < vertices->size(); i++) {
                    const Vector3 v = t * vertices->at(i).cast<Position::Scalar>();
                    model->addVertex(v.x(), v.y(), v.z());
                }

                for (int i = 0; i < mesh->numTriangles(); i++) {
                    int v1 = vertexIndex + mesh->triangle(i)[0];
                    int v2 = vertexIndex + mesh->triangle(i)[1];
                    int v3 = vertexIndex + mesh->triangle(i)[2];
                    model->addTriangle(v1, v2, v3);
                }

            } else 
                if (SgGroup* child = dynamic_cast<SgGroup*>((*iter).get())) {
                    // It will follow the SceneGraph recursive.
                    createColdetModel(child, transform, model);
                }
        }

    } else
        if (SgGroup* group = dynamic_cast<SgGroup*>(parentGroup)){
            // It will follow the SceneGraph recursive.
            for (SgGroup::iterator iter = group->begin(); iter != group->end(); iter++) {
                SgGroup* child = static_cast<SgGroup*>((*iter).get());
                createColdetModel(child, transParent, model);
            }

        }
}
#endif

}; // end of namespace



