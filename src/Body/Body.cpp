/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "Body.h"
#include "BodyHandler.h"
#include "BodyCustomizerInterface.h"
#include <cnoid/CloneMap>
#include <cnoid/SceneGraph>
#include <cnoid/EigenUtil>
#include <cnoid/ValueTree>
#include <iostream>

using namespace std;
using namespace cnoid;

namespace {

const bool PUT_DEBUG_MESSAGE = true;

#ifndef uint
typedef unsigned int uint;
#endif

struct BodyHandleEntity {
    Body* body;
};

typedef std::map<std::string, LinkPtr> NameToLinkMap;
typedef std::map<std::string, Device*> DeviceNameMap;
typedef std::map<std::string, ReferencedPtr> CacheMap;

double getCurrentTime()
{
    return 0.0;
}

}

namespace cnoid {

class BodyImpl
{
public:
    NameToLinkMap nameToLinkMap;
    DeviceNameMap deviceNameMap;
    CacheMap cacheMap;
    MappingPtr info;
    Vector3 centerOfMass;
    double mass;
    std::string name;
    std::string modelName;

    std::vector<BodyHandlerPtr> handlers;

    // Members for the customizer
    BodyCustomizerHandle customizerHandle;
    BodyCustomizerInterface* customizerInterface;
    BodyHandleEntity bodyHandleEntity;
    BodyHandle bodyHandle;

    bool installCustomizer(BodyCustomizerInterface* customizerInterface);
    void expandLinkOffsetRotations(Body* body, Link* link, const Matrix3& parentRs, vector<bool>& validRsFlags);
    void setRsToShape(const Matrix3& Rs, SgNode* shape, std::function<void(SgNode* node)> setShape);
    void applyLinkOffsetRotationsToDevices(Body* body, vector<bool>& validRsFlags);    
};

}


Body::Body()
    : Body(new Link)
{

}


Body::Body(Link* rootLink)
{
    initialize();
    currentTimeFunction = getCurrentTime;
    impl->centerOfMass.setZero();
    impl->info = new Mapping();

    rootLink_ = nullptr;
    setRootLink(rootLink);
}


void Body::initialize()
{
    impl = new BodyImpl;
    
    impl->customizerHandle = 0;
    impl->customizerInterface = 0;
    impl->bodyHandleEntity.body = this;
    impl->bodyHandle = &impl->bodyHandleEntity;
}


void Body::copyFrom(const Body* org, CloneMap* cloneMap)
{
    initialize();

    currentTimeFunction = org->currentTimeFunction;

    impl->centerOfMass = org->impl->centerOfMass;
    impl->name = org->impl->name;
    impl->modelName = org->impl->modelName;
    impl->info = org->impl->info;

    setRootLink(cloneLinkTree(org->rootLink(), cloneMap));

    // deep copy of the devices
    for(auto& device : org->devices()){
        Device* clone;
        if(cloneMap){
            clone = cloneMap->getClone<Device>(device);
        } else {
            clone = device->clone();
        }
        addDevice(clone, link(device->link()->index()));
    }

    // deep copy of the extraJoints
    for(auto& orgExtraJoint : org->extraJoints_){
        ExtraJoint extraJoint(orgExtraJoint);
        for(int j=0; j < 2; ++j){
            extraJoint.link[j] = link(orgExtraJoint.link[j]->index());
        }
        extraJoint.body[0] = extraJoint.body[1] = this;
        extraJoints_.push_back(extraJoint);
    }

    if(!org->impl->handlers.empty()){
        impl->handlers.reserve(org->impl->handlers.size());
        for(auto& handler : org->impl->handlers){
            impl->handlers.push_back(handler->clone());
        }
    }

    if(org->impl->customizerInterface){
        installCustomizer(org->impl->customizerInterface);
    }
}


Link* Body::cloneLinkTree(const Link* orgLink, CloneMap* cloneMap)
{
    Link* link = createLink(orgLink);
    if(cloneMap){
        cloneMap->setClone(orgLink, link);
    }
    for(Link* child = orgLink->child(); child; child = child->sibling()){
        link->appendChild(cloneLinkTree(child, cloneMap));
    }
    return link;
}


Referenced* Body::doClone(CloneMap* cloneMap) const
{
    auto body = new Body;
    body->copyFrom(this, cloneMap);
    return body;
}


Link* Body::createLink(const Link* org) const
{
    return org ? new Link(*org) : new Link();
}


void Body::cloneShapes(CloneMap& cloneMap)
{
    const int n = linkTraverse_.numLinks();
    for(int i=0; i < n; ++i){
        Link* link = linkTraverse_[i];
        SgNode* visualShape = link->visualShape();
        if(visualShape){
            link->setVisualShape(visualShape->cloneNode(cloneMap));
        }
        SgNode* collisionShape = link->collisionShape();
        if(collisionShape){
            if(collisionShape == visualShape){
                link->setCollisionShape(link->visualShape());
            } else {
                link->setCollisionShape(collisionShape->cloneNode(cloneMap));
            }
        }
    }
}


Body::~Body()
{
    setRootLink(0);
    
    if(impl->customizerHandle){
        impl->customizerInterface->destroy(impl->customizerHandle);
    }
    delete impl;
}


void Body::setRootLink(Link* link)
{
    if(rootLink_){
        rootLink_->setBody(0);
    }
    rootLink_ = link;
    if(rootLink_){
        rootLink_->setBody(this);
        updateLinkTree();
    }
}


Link* Body::createEmptyJoint(int jointId)
{
    Link* empty = createLink();
    empty->setJointId(jointId);
    return empty;
}


void Body::updateLinkTree()
{
    isStaticModel_ = true;
    
    impl->nameToLinkMap.clear();
    linkTraverse_.find(rootLink());

    const int numLinks = linkTraverse_.numLinks();
    jointIdToLinkArray.clear();
    jointIdToLinkArray.reserve(numLinks - 1);
    numActualJoints = 0;
    Link** virtualJoints = (Link**)alloca(numLinks * sizeof(Link*));
    int numVirtualJoints = 0;
    double m = 0.0;
    
    for(int i=0; i < numLinks; ++i){
        Link* link = linkTraverse_[i];
        link->setIndex(i);
        impl->nameToLinkMap[link->name()] = link;

        const int id = link->jointId();
        if(id >= 0){
            if(id >= static_cast<int>(jointIdToLinkArray.size())){
                jointIdToLinkArray.resize(id + 1, 0);
            }
            if(!jointIdToLinkArray[id]){
                jointIdToLinkArray[id] = link;
                ++numActualJoints;
            }
        }
        if(link->jointType() != Link::FIXED_JOINT){
            isStaticModel_ = false;
            if(i > 0 && id < 0){
                virtualJoints[numVirtualJoints++] = link;
            }
        }
        m += link->mass();
    }

    for(size_t i=0; i < jointIdToLinkArray.size(); ++i){
        if(!jointIdToLinkArray[i]){
            jointIdToLinkArray[i] = createEmptyJoint(i);
            ++numActualJoints;
        }
    }

    if(numVirtualJoints > 0){
        const int n = jointIdToLinkArray.size();
        jointIdToLinkArray.resize(n + numVirtualJoints);
        for(int i=0; i < numVirtualJoints; ++i){
            jointIdToLinkArray[n + i] = virtualJoints[i];
        }
    }

    impl->mass = m;
}


void Body::resetDefaultPosition(const Position& T)
{
    rootLink_->setOffsetPosition(T);
}


const std::string& Body::name() const
{
    return impl->name;
}


void Body::setName(const std::string& name)
{
    impl->name = name;
}


const std::string& Body::modelName() const
{
    return impl->modelName;
}


void Body::setModelName(const std::string& name)
{
    impl->modelName = name;
}


void Body::setParent(Link* parentBodyLink)
{
    rootLink_->parent_ = parentBodyLink;
}


void Body::resetParent()
{
    rootLink_->parent_ = nullptr;
}


Link* Body::findUniqueEndLink() const
{
    Link* endLink = nullptr;
    Link* link = rootLink_;
    while(true){
        if(!link->child_){
            endLink = link;
            break;
        }
        if(link->child_->sibling_){
            break;
        }
        link = link->child_;
    }
    return endLink;
}


const Mapping* Body::info() const
{
    return impl->info;
}


Mapping* Body::info()
{
    return impl->info;
}


void Body::resetInfo(Mapping* info)
{
    impl->info = info;
}


void Body::addDevice(Device* device, Link* link)
{
    device->setLink(link);
    device->setIndex(devices_.size());
    devices_.push_back(device);
    if(!device->name().empty()){
        impl->deviceNameMap[device->name()] = device;
    }
}


void Body::addDevice(Device* device)
{
    addDevice(device, device->link());
}


void Body::clearDevices()
{
    devices_.clear();
    impl->deviceNameMap.clear();
}


Device* Body::findDeviceSub(const std::string& name) const
{
    DeviceNameMap::const_iterator p = impl->deviceNameMap.find(name);
    if(p != impl->deviceNameMap.end()){
        return p->second;
    }
    return nullptr;
}


Referenced* Body::findCacheSub(const std::string& name)
{
    CacheMap::iterator p = impl->cacheMap.find(name);
    if(p != impl->cacheMap.end()){
        return p->second;
    }
    return nullptr;
}


const Referenced* Body::findCacheSub(const std::string& name) const
{
    CacheMap::iterator p = impl->cacheMap.find(name);
    if(p != impl->cacheMap.end()){
        return p->second;
    }
    return nullptr;
}


void Body::insertCache(const std::string& name, Referenced* cache)
{
    impl->cacheMap[name] = cache;
}


bool Body::getCaches(PolymorphicReferencedArrayBase<>& out_caches, std::vector<std::string>& out_names) const
{
    out_caches.clear_elements();
    out_names.clear();
    for(CacheMap::const_iterator p = impl->cacheMap.begin(); p != impl->cacheMap.end(); ++p){
        if(out_caches.try_push_back(p->second)){
            out_names.push_back(p->first);
        }
    }
    return !out_names.empty();
}


void Body::removeCache(const std::string& name)
{
    impl->cacheMap.erase(name);
}


Link* Body::link(const std::string& name) const
{
    NameToLinkMap::const_iterator p = impl->nameToLinkMap.find(name);
    return (p != impl->nameToLinkMap.end()) ? p->second : 0;
}


double Body::mass() const
{
    return impl->mass;
}


const Vector3& Body::centerOfMass() const
{
    return impl->centerOfMass;
}


void Body::initializePosition()
{
    rootLink_->T() = rootLink_->Tb();

    for(auto& link : linkTraverse_){
        link->q() = link->q_initial();
        link->initializeState();
    }

    calcForwardKinematics(true, true);
    initializeDeviceStates();
}


void Body::initializeState()
{
    for(auto& link : linkTraverse_){
        link->initializeState();
    }

    calcForwardKinematics(true, true);
    initializeDeviceStates();
}


void Body::clearExternalForces()
{
    int n = linkTraverse_.numLinks();
    for(int i=0; i < n; ++i){
        Link* link = linkTraverse_[i];
        link->F_ext().setZero();
    }
}


void Body::initializeDeviceStates()
{
    for(size_t i=0; i < devices_.size(); ++i){
        devices_[i]->clearState();
    }
}


const Vector3& Body::calcCenterOfMass()
{
    double m = 0.0;
    Vector3 mc = Vector3::Zero();
    int n = linkTraverse_.numLinks();
    
    for(int i=0; i < n; i++){
        Link* link = linkTraverse_[i];
        link->wc().noalias() = link->R() * link->c() + link->p();
        mc.noalias() += link->m() * link->wc();
        m += link->m();
    }

    impl->centerOfMass = mc / m;
    impl->mass = m;

    return impl->centerOfMass;
}


/**
   assuming Link::v,w is already computed by calcForwardKinematics(true);
   assuming Link::wc is already computed by calcCenterOfMass();
*/
void Body::calcTotalMomentum(Vector3& out_P, Vector3& out_L)
{
    out_P.setZero();
    out_L.setZero();

    Vector3 dwc;    // Center of mass speed in world frame
    Vector3 P;	    // Linear momentum of the link
    Vector3 L;	    // Angular momentum with respect to the world frame origin 
    Vector3 Llocal; // Angular momentum with respect to the center of mass of the link

    int n = linkTraverse_.numLinks();
    
    for(int i=0; i < n; i++){
        Link* link = linkTraverse_[i];
        dwc = link->v() + link->w().cross(link->R() * link->c());
        P   = link->m() * dwc;

        //L   = cross(link->wc, P) + link->R * link->I * trans(link->R) * link->w; 
        Llocal.noalias() = link->I() * link->R().transpose() * link->w();
        L     .noalias() = link->wc().cross(P) + link->R() * Llocal; 

        out_P += P;
        out_L += L;
    }
}


void Body::expandLinkOffsetRotations()
{
    Matrix3 Rs = Matrix3::Identity();
    vector<bool> validRsFlags;

    for(Link* child = rootLink()->child(); child; child = child->sibling()){
        impl->expandLinkOffsetRotations(this, child, Rs, validRsFlags);
    }

    if(!validRsFlags.empty()){
        impl->applyLinkOffsetRotationsToDevices(this, validRsFlags);
    }
}


void BodyImpl::expandLinkOffsetRotations(Body* body, Link* link, const Matrix3& parentRs, vector<bool>& validRsFlags)
{
    link->setOffsetTranslation(parentRs * link->offsetTranslation());

    Matrix3 Rs = parentRs * link->offsetRotation();

    if(!Rs.isApprox(Matrix3::Identity())){

        if(validRsFlags.empty()){
            validRsFlags.resize(body->numLinks());
        }
        validRsFlags[link->index()] = true;
        
        link->setAccumulatedSegmentRotation(Rs);

        link->setCenterOfMass(Rs * link->centerOfMass());
        link->setInertia(Rs * link->I() * Rs.transpose());
        link->setJointAxis(Rs * link->jointAxis());

        SgNode* visualShape = link->visualShape();
        SgNode* collisionShape = link->collisionShape();

        if(visualShape && visualShape == collisionShape){
            setRsToShape(Rs, visualShape, [&](SgNode* node) { link->setShape(node); });
        } else {
            if(visualShape){
                setRsToShape(Rs, visualShape, [&](SgNode* node) { link->setVisualShape(node); });
            }
            if(collisionShape){
                setRsToShape(Rs, collisionShape, [&](SgNode* node) { link->setCollisionShape(node); });
            }
        }
    }
    
    for(Link* child = link->child(); child; child = child->sibling()){
        expandLinkOffsetRotations(body, child, Rs, validRsFlags);
    }
}


void BodyImpl::setRsToShape(const Matrix3& Rs, SgNode* shape, std::function<void(SgNode* node)> setShape)
{
    SgPosTransform* transformRs = new SgPosTransform;
    transformRs->setRotation(Rs);
    transformRs->addChild(shape);
    setShape(transformRs);
}


void BodyImpl::applyLinkOffsetRotationsToDevices(Body* body, vector<bool>& validRsFlags)
{
    for(int i=0; i < body->numDevices(); ++i){
        Device* device = body->device(i);
        Link* link = device->link();
        if(validRsFlags[link->index()]){
            device->setLocalTranslation(link->Rs() * device->localTranslation());
            device->setLocalRotation(link->Rs() * device->localRotation());
        }
    }
}


void Body::setCurrentTimeFunction(std::function<double()> func)
{
    currentTimeFunction = func;
}


bool Body::addHandler(BodyHandler* handler, bool isTopPriority)
{
    if(isTopPriority){
        impl->handlers.insert(impl->handlers.begin(), handler);
    } else {
        impl->handlers.push_back(handler);
    }
    return true;
}


BodyHandler* Body::findHandler(std::function<bool(BodyHandler*)> isTargetHandlerType)
{
    for(auto& handler : impl->handlers){
        if(isTargetHandlerType(handler)){
            return handler;
        }
    }
    return nullptr;
}

    
BodyCustomizerHandle Body::customizerHandle() const
{
    return impl->customizerHandle;
}


BodyCustomizerInterface* Body::customizerInterface() const
{
    return impl->customizerInterface;
}


bool Body::hasVirtualJointForces() const
{
    if(impl->customizerInterface){
        if(impl->customizerInterface->setVirtualJointForces){
            return true;
        }
        if(impl->customizerInterface->version >= 2 &&
           impl->customizerInterface->setVirtualJointForces2){
            return true;
        }
    }
    return false;
}


void Body::setVirtualJointForces(double timeStep)
{
    auto customizer = impl->customizerInterface;
    if(customizer){
        if(customizer->version >= 2 && customizer->setVirtualJointForces2){
            customizer->setVirtualJointForces2(impl->customizerHandle, timeStep);
        } else if(customizer->setVirtualJointForces){
            customizer->setVirtualJointForces(impl->customizerHandle);
        }
    }
}


/**
   The function installs the pre-loaded customizer corresponding to the model name.
*/
bool Body::installCustomizer()
{
    loadDefaultBodyCustomizers(std::cerr);
    BodyCustomizerInterface* interface = findBodyCustomizer(impl->modelName);
    if(interface){
        return installCustomizer(interface);
    }
    return false;
}


bool Body::installCustomizer(BodyCustomizerInterface* customizerInterface)
{
    return impl->installCustomizer(customizerInterface);
}


bool BodyImpl::installCustomizer(BodyCustomizerInterface* customizerInterface)
{
    if(this->customizerInterface){
        if(customizerHandle){
            this->customizerInterface->destroy(customizerHandle);
            customizerHandle = 0;
        }
        this->customizerInterface = 0;
    }
	
    if(customizerInterface){
        customizerHandle = customizerInterface->create(bodyHandle, modelName.c_str());
        if(customizerHandle){
            this->customizerInterface = customizerInterface;
        }
    }

    return (customizerHandle != 0);
}


static inline Link* extractLink(BodyHandle bodyHandle, int linkIndex)
{
    return static_cast<BodyHandleEntity*>(bodyHandle)->body->link(linkIndex);
}


static int getLinkIndexFromName(BodyHandle bodyHandle, const char* linkName)
{
    Body* body = static_cast<BodyHandleEntity*>(bodyHandle)->body;
    Link* link = body->link(linkName);
    return (link ? link->index() : -1);
}


static const char* getLinkName(BodyHandle bodyHandle, int linkIndex)
{
    return extractLink(bodyHandle, linkIndex)->name().c_str();
}


static double* getJointValuePtr(BodyHandle bodyHandle, int linkIndex)
{
    return &(extractLink(bodyHandle,linkIndex)->q());
}


static double* getJointVelocityPtr(BodyHandle bodyHandle, int linkIndex)
{
    return &(extractLink(bodyHandle, linkIndex)->dq());
}


static double* getJointTorqueForcePtr(BodyHandle bodyHandle, int linkIndex)
{
    return &(extractLink(bodyHandle, linkIndex)->u());
}


BodyInterface* Body::bodyInterface()
{
    static BodyInterface interface = {
        BODY_INTERFACE_VERSION,
        getLinkIndexFromName,
        getLinkName,
        getJointValuePtr,
        getJointVelocityPtr,
        getJointTorqueForcePtr,
    };

    return &interface;
}
