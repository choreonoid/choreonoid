#include "Body.h"
#include "BodyHandler.h"
#include "BodyCustomizerInterface.h"
#include <cnoid/CloneMap>
#include <cnoid/SceneGraph>
#include <cnoid/EigenUtil>
#include <cnoid/ValueTree>
#include <utility>
#include <iostream>

using namespace std;
using namespace cnoid;

namespace {

const bool PUT_DEBUG_MESSAGE = true;

constexpr int MainEndLinkGuessBufSize = 10;

#ifndef uint
typedef unsigned int uint;
#endif

struct BodyHandleEntity {
    Body* body;
};

typedef std::map<std::string, LinkPtr> NameToLinkMap;
typedef std::map<std::string, Device*> DeviceNameMap;
typedef std::map<std::string, ReferencedPtr> CacheMap;

struct MultiplexInfo : public Referenced
{
    Body* mainBody;
    Body* lastBody;
    int numBodies;
    vector<BodyPtr> bodyCache;
    Signal<void(Body* body, bool isAdded)> sigMultiplexBodyAddedOrRemoved;
};
typedef ref_ptr<MultiplexInfo> MultiplexInfoPtr;

double getCurrentTime()
{
    return 0.0;
}

}

namespace cnoid {

class Body::Impl
{
public:
    NameToLinkMap nameToLinkMap;
    NameToLinkMap jointSpecificNameToLinkMap;
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

    MultiplexInfoPtr multiplexInfo;
    Signal<void(bool on)> sigExistenceChanged;
        
    Impl(Body* self);
    void initialize(Body* self, Link* rootLink);
    void guessMainEndLinkSub(Link* link, Link* rootLink, int dof, Link** linkOfDof);
    void removeDeviceFromDeviceNameMap(Device* device);
    MultiplexInfo* getOrCreateMultiplexInfo(Body* self);
    bool installCustomizer(BodyCustomizerInterface* customizerInterface);
};

}


Body::Body()
{
    impl = new Impl(this);

    auto rootLink = new Link;
    rootLink->setJointType(Link::FreeJoint);
    impl->initialize(this, rootLink);
}


Body::Body(const std::string& name)
    : Body()
{
    impl->name = name;
}


Body::Body(Link* rootLink)
{
    impl = new Impl(this);
    impl->initialize(this, rootLink);
}


Body::Impl::Impl(Body* self)
{
    customizerHandle = 0;
    customizerInterface = nullptr;
    bodyHandleEntity.body = self;
    bodyHandle = &bodyHandleEntity;
}


void Body::Impl::initialize(Body* self, Link* rootLink)
{
    self->currentTimeFunction = getCurrentTime;
    centerOfMass.setZero();
    info = new Mapping;

    self->rootLink_ = nullptr;
    self->setRootLink(rootLink);
    self->existence_ = true;
}


void Body::copyFrom(const Body* org, CloneMap* cloneMap)
{
    currentTimeFunction = org->currentTimeFunction;

    impl->centerOfMass = org->impl->centerOfMass;
    impl->name = org->impl->name;
    impl->modelName = org->impl->modelName;
    impl->info = org->impl->info;

    setRootLink(cloneLinkTree(org->rootLink(), cloneMap));

    if(cloneMap){
        // reference to the parent body
        if(auto orgParentBodyLink = org->parentBodyLink()){
            parentBodyLink_ = cloneMap->getClone<Link>(orgParentBodyLink);
        }
    }

    for(auto& device : org->devices()){
        addDevice(CloneMap::getClone(device, cloneMap), link(device->link()->index()));
    }

    for(auto& orgExtraJoint : org->extraJoints_){
        ExtraJointPtr extraJoint = new ExtraJoint(orgExtraJoint->type());
        for(int j=0; j < 2; ++j){
            if(auto orgLink = orgExtraJoint->link(j)){
                extraJoint->setLink(j, link(orgLink->index()));
            }
            extraJoint->setLocalPosition(j, orgExtraJoint->localPosition(j));
        }
        if(auto info = orgExtraJoint->info()){
            extraJoint->resetInfo(info->clone());
        }
        extraJoints_.push_back(extraJoint);
    }

    if(!org->impl->handlers.empty()){
        impl->handlers.reserve(org->impl->handlers.size());
        for(auto& handler : org->impl->handlers){
            if(auto handlerClone = handler->clone()){
                handlerClone->body_ = this;
                handlerClone->filename_ = handler->filename_;
                if(handlerClone->initialize(this)){
                    impl->handlers.push_back(handlerClone);
                } else {
                    delete handlerClone;
                }
            } else if(auto handlerClone = handler->clone(this)){ // For backward compatibility
                handlerClone->body_ = this;
                handlerClone->filename_ = handler->filename_;
                impl->handlers.push_back(handlerClone);
            }
        }
    }

    if(org->impl->customizerInterface){
        installCustomizer(org->impl->customizerInterface);
    }
}


Link* Body::cloneLinkTree(const Link* orgLink, CloneMap* cloneMap)
{
    Link* link = createLink(orgLink, cloneMap);
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


Link* Body::createLink(const Link* orgLink, CloneMap* cloneMap) const
{
    auto link = doCreateLink(orgLink, cloneMap);
    if(orgLink && cloneMap){
        cloneMap->setClone(orgLink, link);
    }
    return link;
}


Link* Body::doCreateLink(const Link* org, CloneMap* cloneMap) const
{
    return org ? new Link(*org, cloneMap) : new Link();
}


void Body::cloneShapes(CloneMap& cloneMap)
{
    const int n = linkTraverse_.numLinks();
    for(int i=0; i < n; ++i){
        Link* link = linkTraverse_[i];
        for(auto& node : *link->visualShape()){
            link->addVisualShapeNode(node->cloneNode(cloneMap));
        }
        for(auto& node : *link->collisionShape()){
            link->addCollisionShapeNode(node->cloneNode(cloneMap));
        }
    }
}


Body::~Body()
{
    clearDevices();
    setRootLink(nullptr);
    
    if(impl->customizerHandle){
        impl->customizerInterface->destroy(impl->customizerHandle);
    }
    delete impl;
}


void Body::setRootLink(Link* link)
{
    if(rootLink_){
        rootLink_->setBodyToSubTree(nullptr);
    }
    rootLink_ = link;
    if(rootLink_){
        rootLink_->setBodyToSubTree(this);
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
    impl->jointSpecificNameToLinkMap.clear();
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
        if(!link->name().empty()){
            impl->nameToLinkMap[link->name()] = link;
        }
        if(!link->jointSpecificName().empty()){
            impl->jointSpecificNameToLinkMap[link->jointSpecificName()] = link;
        }
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
        if(link->jointType() != Link::FixedJoint){
            if(i > 0 && id < 0){
                virtualJoints[numVirtualJoints++] = link;
            }
            if(!link->isFixedJoint()){
                isStaticModel_ = false;
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


void Body::setRootLinkFixed(bool on)
{
    if(on){
        if(!rootLink_->isFixedJoint()){
            rootLink_->setJointType(Link::FixedJoint);
            isStaticModel_ = true;
            int n = numLinks();
            for(int i=1; i < n; ++i){
                if(!link(i)->isFixedJoint()){
                    isStaticModel_ = false;
                    break;
                }
            }
        }
    } else {
        if(rootLink_->isFixedJoint()){
            rootLink_->setJointType(Link::FreeJoint);
            isStaticModel_ = false;
        }
    }
}


bool Body::hasMovableJoints() const
{
    int n = numLinks();
    for(int i=1; i < n; ++i){
        if(!link(i)->isFixedJoint()){
            return true;
        }
    }
    return false;
}


void Body::resetDefaultPosition(const Isometry3& T)
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
    parentBodyLink_ = parentBodyLink;
}


void Body::resetParent()
{
    parentBodyLink_.reset();
}


void Body::syncPositionWithParentBody(bool doForwardKinematics)
{
    if(auto parentLink = parentBodyLink()){
        Isometry3 Ts = parentLink->T() * rootLink_->Tb();
        if(doForwardKinematics){
            rootLink_->setPosition(Ts);
            calcForwardKinematics();
        } else {
            Isometry3 Tr = Ts * rootLink_->T().inverse();
            for(auto& link : links()){
                link->T() = Tr * link->T();
            }
        }
    }
}


Link* Body::findUniqueEndLink() const
{
    Link* endLink = nullptr;
    Link* link = rootLink_;
    while(true){
        if(!link->child()){
            if(link != rootLink_){
                endLink = link;
            }
            break;
        }
        if(link->child()->sibling()){
            break;
        }
        link = link->child();
    }
    return endLink;
}


Link* Body::lastSerialLink() const
{
    Link* link = rootLink_;
    while(true){
        if(!link->child() || link->child()->sibling()){
            break;
        }
        link = link->child();
    }
    return link;
}


//  Find a maximum DOF link whose DOF is different from any other links.
Link* Body::guessMainEndLink() const
{
    Link* linkOfDof[MainEndLinkGuessBufSize];
    for(int i=0; i < MainEndLinkGuessBufSize; ++i){
        linkOfDof[i] = nullptr;
    }
    
    impl->guessMainEndLinkSub(rootLink_, rootLink_, 0, linkOfDof);

    for(int i = MainEndLinkGuessBufSize - 1; i > 0; --i){
        auto link = linkOfDof[i];
        if(link && link != rootLink_){
            return link;
        }
    }
    
    return nullptr;
}


void Body::Impl::guessMainEndLinkSub(Link* link, Link* rootLink, int dof, Link** linkOfDof)
{
    if(!linkOfDof[dof]){
        linkOfDof[dof] = link;
    } else {
        linkOfDof[dof] = rootLink;
    }
    ++dof;
    if(dof < MainEndLinkGuessBufSize){
        for(Link* child = link->child(); child; child = child->sibling()){
            guessMainEndLinkSub(child, rootLink, dof, linkOfDof);
        }
    }
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


std::vector<LinkPtr> Body::getIdentifiedJoints() const
{
    std::vector<LinkPtr> identified;
    identified.reserve(numActualJoints);
    std::copy(jointIdToLinkArray.begin(), jointIdToLinkArray.end(), back_inserter(identified));
    return identified;
}


bool Body::addDevice(Device* device, Link* link)
{
    if(auto linkBody = link->body()){
        if(linkBody != this){
            return false;
        }
    }
    if(auto currentLink = device->link()){
        if(currentLink->body() == this){
            auto it = std::find(devices_.begin(), devices_.end(), device);
            if(it != devices_.end()){
                return true;
            }
        }
    }

    device->setLink(link);
    device->setIndex(devices_.size());
    devices_.push_back(device);
    if(!device->name().empty()){
        impl->deviceNameMap[device->name()] = device;
    }
    return true;
}


void Body::Impl::removeDeviceFromDeviceNameMap(Device* device)
{
    auto it = deviceNameMap.find(device->name());
    if(it != deviceNameMap.end()){
        if(it->second == device){
            deviceNameMap.erase(it);
        }
    }
}


bool Body::removeDevice(Device* device)
{
    bool removed = false;
    int index = 0;
    auto it = devices_.begin();
    while(it != devices_.end()){
        auto& dev = *it;
        if(dev == device){
            impl->removeDeviceFromDeviceNameMap(device);
            device->setLink(nullptr);
            devices_.erase(it);
            removed = true;
            break;
        } else {
            dev->setIndex(index++);
            ++it;
        }
    }
    return removed;
}


void Body::removeDevicesOfLink(Link* link)
{
    int index = 0;
    auto it = devices_.begin();
    while(it != devices_.end()){
        auto& device = *it;
        if(device->link() == link){
            impl->removeDeviceFromDeviceNameMap(device);
            device->setLink(nullptr);
            it = devices_.erase(it);
        } else {
            device->setIndex(index++);
            ++it;
        }
    }
}


void Body::clearDevices()
{
    for(auto& device : devices_){
        device->setLink(nullptr);
    }
    devices_.clear();
    impl->deviceNameMap.clear();
}


void Body::sortDevicesByLinkOrder()
{
    std::stable_sort(
        devices_.begin(), devices_.end(),
        [](const Device* a, const Device* b){
            return a->link()->index() < b->link()->index();
        });

    int index = 0;
    for(auto& device : devices_){
        device->setIndex(index++);
    }
}


Device* Body::findDevice_(const std::string& name) const
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
    auto p = impl->nameToLinkMap.find(name);
    return (p != impl->nameToLinkMap.end()) ? p->second : 0;
}


Link* Body::joint(const std::string& name) const
{
    auto p = impl->jointSpecificNameToLinkMap.find(name);
    if(p != impl->jointSpecificNameToLinkMap.end()){
        return p->second;
    }
    return link(name);
}


void Body::resetLinkName(Link* link, const std::string& name)
{
    auto p = impl->nameToLinkMap.find(link->name());
    if(p != impl->nameToLinkMap.end()){
        if(p->second == link){
            impl->nameToLinkMap.erase(p);
        }
    }
    impl->nameToLinkMap[name] = link;
}


void Body::resetJointSpecificName(Link* link)
{
    const auto& currentName = link->jointSpecificName();
    if(!currentName.empty()){
        auto p = impl->jointSpecificNameToLinkMap.find(currentName);
        if(p != impl->jointSpecificNameToLinkMap.end()){
            if(p->second == link){
                impl->jointSpecificNameToLinkMap.erase(p);
            }
        }
    }
}
    

void Body::resetJointSpecificName(Link* link, const std::string& name)
{
    resetJointSpecificName(link);
    if(!name.empty()){
        impl->jointSpecificNameToLinkMap[name] = link;
    }
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


void Body::setCurrentTimeFunction(std::function<double()> func)
{
    currentTimeFunction = func;
}


void Body::setExistence(bool on)
{
    if(on != existence_){
        existence_ = on;
        impl->sigExistenceChanged(on);
    }
}


SignalProxy<void(bool on)> Body::sigExistenceChanged()
{
    return impl->sigExistenceChanged;
}


bool Body::isMultiplexBody() const
{
    if(impl->multiplexInfo){
        if(this != impl->multiplexInfo->mainBody){
            return true;
        }
    }
    return false;
}


bool Body::isMultiplexMainBody() const
{
    return impl->multiplexInfo ? (impl->multiplexInfo->mainBody == this) : true;
}


Body* Body::multiplexMainBody()
{
    return impl->multiplexInfo ? impl->multiplexInfo->mainBody : this;
}


int Body::getNumMultiplexBodies() const
{
    return impl->multiplexInfo->numBodies;
}


MultiplexInfo* Body::Impl::getOrCreateMultiplexInfo(Body* self)
{
    if(!multiplexInfo){
        multiplexInfo = new MultiplexInfo;
        multiplexInfo->mainBody = self;
        multiplexInfo->lastBody = self;
        multiplexInfo->numBodies = 1;
    }
    return multiplexInfo;
}


Body* Body::addMultiplexBody()
{
    auto multiplexInfo = impl->getOrCreateMultiplexInfo(this);
    
    BodyPtr newBody;
    if(multiplexInfo->bodyCache.empty()){
        newBody = clone();
    } else {
        newBody = multiplexInfo->bodyCache.back();
        // \todo copy all the link positions
        newBody->rootLink()->setPosition(rootLink()->position());
        multiplexInfo->bodyCache.pop_back();
    }
        
    newBody->impl->multiplexInfo = multiplexInfo;

    multiplexInfo->lastBody->nextMultiplexBody_ = newBody;
    multiplexInfo->lastBody = newBody;
    multiplexInfo->numBodies++;

    multiplexInfo->sigMultiplexBodyAddedOrRemoved(newBody, true);
    
    return newBody;
}


bool Body::doClearMultiplexBodies(bool doClearCache)
{
    bool removed = false;

    BodyPtr nextBody = nextMultiplexBody_;
    while(nextBody){
        if(!doClearCache){
            impl->multiplexInfo->bodyCache.push_back(nextBody);
        }
        nextBody->impl->multiplexInfo.reset();
        BodyPtr nextNextBody = nextBody->nextMultiplexBody_;
        nextBody->nextMultiplexBody_.reset();
        nextMultiplexBody_ = nextNextBody;
        if(!nextNextBody){
            impl->multiplexInfo->lastBody = this;
        }
        --impl->multiplexInfo->numBodies;
        impl->multiplexInfo->sigMultiplexBodyAddedOrRemoved(nextBody, false);
        nextBody = nextNextBody;
        removed = true;
    }

    return removed;
}


void Body::exchangePositionWithMultiplexBody(Body* multiplexBody)
{
    int n = numLinks();
    for(int i=0; i < n; ++i){
        Link* link_ = link(i);
        Link* multiplexLink = multiplexBody->link(i);
        Isometry3 T = link_->T();
        link_->setPosition(multiplexLink->T());
        multiplexLink->setPosition(T);
        std::swap(link_->q(), multiplexLink->q());
    }
}


SignalProxy<void(Body* body, bool isAdded)> Body::sigMultiplexBodyAddedOrRemoved()
{
    return impl->getOrCreateMultiplexInfo(this)->sigMultiplexBodyAddedOrRemoved;
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


int Body::numHandlers() const
{
    return impl->handlers.size();
}


BodyHandler* Body::handler(int index)
{
    return impl->handlers[index];
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


bool Body::Impl::installCustomizer(BodyCustomizerInterface* customizerInterface)
{
    if(this->customizerInterface){
        if(customizerHandle){
            this->customizerInterface->destroy(customizerHandle);
            customizerHandle = 0;
        }
        this->customizerInterface = nullptr;
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


template<> double Body::info(const std::string& key) const
{
    return impl->info->get(key).toDouble();
}


template<> double Body::info(const std::string& key, const double& defaultValue) const
{
    double value;
    if(impl->info->read(key, value)){
        return value;
    }
    return defaultValue;
}


template<> bool Body::info(const std::string& key, const bool& defaultValue) const
{
    bool value;
    if(impl->info->read(key, value)){
        return value;
    }
    return defaultValue;
}


template<> void Body::setInfo(const std::string& key, const double& value)
{
    impl->info->write(key, value);
}


template<> void Body::setInfo(const std::string& key, const bool& value)
{
    impl->info->write(key, value);
}
