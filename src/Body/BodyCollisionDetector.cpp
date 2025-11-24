#include "BodyCollisionDetector.h"
#include "BodyCollisionLinkFilter.h"
#include "Body.h"
#include "Link.h"
#include <cnoid/SceneGraph>
#include <cnoid/ConnectionSet>
#include <unordered_map>
#include <vector>

using namespace std;
using namespace cnoid;

namespace {

struct BodyInfo : public Referenced
{
    ScopedConnectionSet connections;
};
typedef ref_ptr<BodyInfo> BodyInfoPtr;

typedef unordered_map<BodyPtr, BodyInfoPtr> BodyInfoMap;

}

namespace cnoid {

class BodyCollisionDetector::Impl
{
public:
    BodyCollisionDetector* self;
    CollisionDetectorPtr collisionDetector;
    BodyInfoMap bodyInfoMap;
    unordered_map<LinkPtr, GeometryHandle> linkToGeometryHandleMap;
    LinkAssociatedObjectFunc linkAssociatedObjectFunc;
    vector<GeometryHandle> linkIndexToGeometryHandleMap;
    BodyCollisionLinkFilter bodyCollisionLinkFilter;
    bool needToMakeCollisionDetectorReady;
    bool isGeometryHandleMapEnabled;
    bool isGeometryRemovalSupported;
    bool isMultiplexBodySupportEnabled;

    Impl(BodyCollisionDetector* self);
    bool addBody(Body* body, bool isSelfCollisionDetectionEnabled, int groupId, bool isMultiplexBody);
    bool addLinkRecursively(Link* link, bool isParentStatic, int groupId);
    stdx::optional<CollisionDetector::GeometryHandle> addLink(Link* link, bool isStatic, int groupId);
    bool removeBody(Body* body);
    void onMultiplexBodyAddedOrRemoved(Body* body, bool isAdded, bool isSelfCollisionDetectionEnabled, int groupId);
    void setLinksInAttachmentIgnored(
        GeometryHandle attachedLinkGeometry, Link* linkInParentBody, Link* traverseParent, Link* traverseChild,
        bool ignored);
    double findClosestPoints(Link* link1, Link* link2, Vector3& out_point1, Vector3& out_point2);
    void onBodyExistenceChanged(Body* body, bool on);    
};

}


BodyCollisionDetector::BodyCollisionDetector()
{
    impl = new Impl(this);
}


BodyCollisionDetector::Impl::Impl(BodyCollisionDetector* self)
    : self(self)
{
    bodyCollisionLinkFilter.setFuncToDisableLinkPair(
        [this](int linkIndex1, int linkIndex2){
            GeometryHandle geometry1 = linkIndexToGeometryHandleMap[linkIndex1];
            GeometryHandle geometry2 = linkIndexToGeometryHandleMap[linkIndex2];
            if(geometry1 && geometry2){
                collisionDetector->ignoreGeometryPair(geometry1, geometry2);
            }
        });
    
    needToMakeCollisionDetectorReady = true;
    isGeometryHandleMapEnabled = false;
    isGeometryRemovalSupported = false;
    isMultiplexBodySupportEnabled = false;
}


BodyCollisionDetector::BodyCollisionDetector(CollisionDetector* collisionDetector)
{
    impl = new Impl(this);
    setCollisionDetector(collisionDetector);
}


BodyCollisionDetector::~BodyCollisionDetector()
{
    delete impl;
}


void BodyCollisionDetector::setCollisionDetector(CollisionDetector* collisionDetector)
{
    impl->collisionDetector = collisionDetector;
    if(collisionDetector){
        impl->isGeometryRemovalSupported = collisionDetector->isGeometryRemovalSupported();
    } else {
        impl->isGeometryRemovalSupported = false;
    }
    clearBodies();
}


CollisionDetector* BodyCollisionDetector::collisionDetector()
{
    return impl->collisionDetector;
}


bool BodyCollisionDetector::isGeometryHandleMapEnabled() const
{
    return impl->isGeometryHandleMapEnabled;
}


void BodyCollisionDetector::setGeometryHandleMapEnabled(bool on)
{
    if(!on){
        impl->linkToGeometryHandleMap.clear();
    }
    impl->isGeometryHandleMapEnabled = on;
}


void BodyCollisionDetector::enableGeometryHandleMap(bool on)
{
    setGeometryHandleMapEnabled(on);
}


void BodyCollisionDetector::clearBodies()
{
    if(impl->collisionDetector){
        impl->collisionDetector->clearGeometries();
    }
    impl->bodyInfoMap.clear();
    impl->linkToGeometryHandleMap.clear();
    impl->needToMakeCollisionDetectorReady = true;
}


void BodyCollisionDetector::setLinkAssociatedObjectFunction(LinkAssociatedObjectFunc func)
{
    impl->linkAssociatedObjectFunc = func;
}


bool BodyCollisionDetector::isMultiplexBodySupported() const
{
    return impl->isGeometryRemovalSupported;
}


bool BodyCollisionDetector::isMultiplexBodySupportEnabled() const
{
    return impl->isMultiplexBodySupportEnabled;
}


void BodyCollisionDetector::setMultiplexBodySupportEnabled(bool on)
{
    impl->isMultiplexBodySupportEnabled = on && isMultiplexBodySupported();
}


void BodyCollisionDetector::addBody(Body* body, bool isSelfCollisionDetectionEnabled, int groupId)
{
    impl->addBody(body, isSelfCollisionDetectionEnabled, groupId, false);
}


bool BodyCollisionDetector::Impl::addBody(Body* body, bool isSelfCollisionDetectionEnabled, int groupId, bool isMultiplexBody)
{
    BodyInfoPtr info = new BodyInfo;
    auto inserted = bodyInfoMap.insert(BodyInfoMap::value_type(body, info));
    if(!inserted.second){
        return false; // already added
    }
    
    const int numLinks = body->numLinks();
    linkIndexToGeometryHandleMap.clear();
    linkIndexToGeometryHandleMap.resize(numLinks, 0);

    bodyCollisionLinkFilter.setTargetBody(body, isSelfCollisionDetectionEnabled);
    
    bool added = addLinkRecursively(body->rootLink(), true, groupId);

    if(!added){
        bodyInfoMap.erase(inserted.first);

    } else {
        bodyCollisionLinkFilter.apply();

        if(!isMultiplexBody){
            info->connections.add(
                body->sigExistenceChanged().connect(
                    [this, body](bool on){ onBodyExistenceChanged(body, on); }));

            if(isMultiplexBodySupportEnabled){
                info->connections.add(
                    body->sigMultiplexBodyAddedOrRemoved().connect(
                        [this, isSelfCollisionDetectionEnabled, groupId](Body* body, bool isAdded){
                            onMultiplexBodyAddedOrRemoved(body, isAdded, isSelfCollisionDetectionEnabled, groupId);
                        }));
            }
        }

        if(isMultiplexBodySupportEnabled){
            if(auto multiplexBody = body->nextMultiplexBody()){
                addBody(multiplexBody, isSelfCollisionDetectionEnabled, groupId, true);
            }
        }

        needToMakeCollisionDetectorReady = true;
    }

    return added;
}


bool BodyCollisionDetector::Impl::addLinkRecursively(Link* link, bool isParentStatic, int groupId)
{
    int linkIndex = link->index();
    bool added = false;
    bool isStatic = isParentStatic && link->isFixedJoint();

    if(bodyCollisionLinkFilter.checkIfEnabledLinkIndex(linkIndex)){
        if(auto handle = addLink(link, isStatic, groupId)){
            linkIndexToGeometryHandleMap[linkIndex] = *handle;
            added = true;
        }
    }
    for(Link* child = link->child(); child; child = child->sibling()){
        added |= addLinkRecursively(child, isStatic, groupId);
    }

    return added;
}


stdx::optional<CollisionDetector::GeometryHandle> BodyCollisionDetector::Impl::addLink(Link* link, bool isStatic, int groupId)
{
    bool added = false;

    stdx::optional<GeometryHandle> handle = collisionDetector->addGeometry(link->collisionShape());
    
    if(handle){
        Referenced* object;
        if(linkAssociatedObjectFunc){
            object = linkAssociatedObjectFunc(link, *handle);
        } else {
            object = link;
        }
        collisionDetector->setCustomObject(*handle, object);
        if(isStatic){
            collisionDetector->setGeometryStatic(*handle, object);
        }
        if(groupId != 0){
            collisionDetector->setGroup(*handle, groupId);
        }
        if(isGeometryHandleMapEnabled){
            linkToGeometryHandleMap[link] = *handle;
        }
    }

    return handle;
}


stdx::optional<CollisionDetector::GeometryHandle> BodyCollisionDetector::addLink(Link* link, int groupId)
{
    impl->needToMakeCollisionDetectorReady = true;
    return impl->addLink(link, link->isStatic(), groupId);
}


void BodyCollisionDetector::setGroup(Body* body, int groupId)
{
    for(auto& link : body->links()){
        setGroup(link, groupId);
    }
}


void BodyCollisionDetector::setGroup(Link* link, int groupId)
{
    auto it = impl->linkToGeometryHandleMap.find(link);
    if(it != impl->linkToGeometryHandleMap.end()){
        auto& handle = it->second;
        impl->collisionDetector->setGroup(handle, groupId);
    }
}


bool BodyCollisionDetector::removeBody(Body* body)
{
    return impl->removeBody(body);
}


bool BodyCollisionDetector::Impl::removeBody(Body* body)
{
    bool removed = false;
    if(isGeometryHandleMapEnabled && isGeometryRemovalSupported){
        for(auto& link : body->links()){
            auto it = linkToGeometryHandleMap.find(link);
            if(it != linkToGeometryHandleMap.end()){
                auto& handle = it->second;
                if(collisionDetector->removeGeometry(handle)){
                    linkToGeometryHandleMap.erase(it);
                    removed = true;
                }
            }
        }
        bodyInfoMap.erase(body);
    }
    if(removed){
        needToMakeCollisionDetectorReady = true;
    }
    return removed;
}


void BodyCollisionDetector::Impl::onMultiplexBodyAddedOrRemoved
(Body* body, bool isAdded, bool isSelfCollisionDetectionEnabled, int groupId)
{
    if(isAdded){
        addBody(body, isSelfCollisionDetectionEnabled, groupId, true);
    } else {
        removeBody(body);
    }
}


bool BodyCollisionDetector::hasBodies() const
{
    return impl->collisionDetector->numGeometries() > 0;
}


void BodyCollisionDetector::setLinksInAttachmentIgnored(Link* attachedLink, Link* parentLink, bool ignored)
{
    if(parentLink){
        if(auto attachedLinkGeometry = findGeometryHandle(attachedLink)){
            impl->setLinksInAttachmentIgnored(*attachedLinkGeometry, parentLink, nullptr, nullptr, ignored);
        }
        for(auto child = attachedLink->child(); child; child = child->sibling()){
            if(child->isFixedJoint()){
                setLinksInAttachmentIgnored(child, parentLink, ignored);
            }
        }
    }
}


void BodyCollisionDetector::Impl::setLinksInAttachmentIgnored
(GeometryHandle attachedLinkGeometry, Link* linkInParentBody, Link* traverseParent, Link* traverseChild, bool ignored)
{
    auto linkInParentBodyGeometry = self->findGeometryHandle(linkInParentBody);
    if(linkInParentBodyGeometry){
        collisionDetector->ignoreGeometryPair(
            attachedLinkGeometry, *linkInParentBodyGeometry, ignored);
    }
    if(!traverseParent && linkInParentBody->isFixedJoint()){
        if(auto parentLink = linkInParentBody->parent()){
            setLinksInAttachmentIgnored(
                attachedLinkGeometry, parentLink, nullptr, linkInParentBody, ignored);
        }
    }
    for(auto child = linkInParentBody->child(); child; child = child->sibling()){
        if(child != traverseChild && child->isFixedJoint()){
            setLinksInAttachmentIgnored(
                attachedLinkGeometry, child, linkInParentBody, nullptr, ignored);
        }
    }
}


bool BodyCollisionDetector::makeReady(bool doForce)
{
    bool ready = !impl->needToMakeCollisionDetectorReady;
    if(doForce || !ready){
        ready = impl->collisionDetector->makeReady();
        impl->needToMakeCollisionDetectorReady = false;
    }
    return ready;
}


stdx::optional<CollisionDetector::GeometryHandle> BodyCollisionDetector::findGeometryHandle(Link* link)
{
    auto iter = impl->linkToGeometryHandleMap.find(link);
    if(iter != impl->linkToGeometryHandleMap.end()){
        return iter->second;
    }
    return stdx::nullopt;
}


void BodyCollisionDetector::updatePositions()
{
    impl->collisionDetector->updatePositions(
        [](Referenced* object, Isometry3*& out_position){
            out_position = &(static_cast<Link*>(object)->position()); });
}


void BodyCollisionDetector::updatePositions
(std::function<void(Referenced* object, Isometry3*& out_position)> positionQuery)
{
    impl->collisionDetector->updatePositions(positionQuery);
}


bool BodyCollisionDetector::detectCollisions(std::function<bool(const CollisionPair& collisionPair)> callback)
{
    return impl->collisionDetector->detectCollisions(callback);
}


bool BodyCollisionDetector::detectCollisions
(Link* link, std::function<bool(const CollisionPair& collisionPair)> callback)
{
    if(auto handle = findGeometryHandle(link)){
        return impl->collisionDetector->detectCollisions(*handle, callback);
    }
    return false; // No geometry handle found
}


void BodyCollisionDetector::Impl::onBodyExistenceChanged(Body* body, bool on)
{
    for(auto& link : body->links()){
        auto it = linkToGeometryHandleMap.find(link);
        if(it != linkToGeometryHandleMap.end()){
            auto& handle = it->second;
            collisionDetector->setGeometryEnabled(handle, on);
        }
    }
}
