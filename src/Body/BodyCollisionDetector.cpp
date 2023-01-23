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

namespace cnoid {

class BodyCollisionDetector::Impl
{
public:
    CollisionDetectorPtr collisionDetector;
    unordered_map<LinkPtr, GeometryHandle> linkToGeometryHandleMap;
    LinkAssociatedObjectFunc linkAssociatedObjectFunc;
    vector<GeometryHandle> linkIndexToGeometryHandleMap;
    BodyCollisionLinkFilter bodyCollisionLinkFilter;
    ScopedConnectionSet bodyExistenceConnections;
    bool hasCustomObjectsAssociatedWithLinks;
    bool isGeometryHandleMapEnabled;

    Impl();
    bool addBody(Body* body, bool isSelfCollisionDetectionEnabled, int groupId);
    bool addLinkRecursively(Link* link, bool isParentStatic, int groupId);
    double findClosestPoints(Link* link1, Link* link2, Vector3& out_point1, Vector3& out_point2);
    void onBodyExistenceChanged(Body* body, bool on);    
};

}


BodyCollisionDetector::BodyCollisionDetector()
{
    impl = new Impl;
}


BodyCollisionDetector::Impl::Impl()
{
    bodyCollisionLinkFilter.setFuncToDisableLinkPair(
        [this](int linkIndex1, int linkIndex2){
            GeometryHandle geometry1 = linkIndexToGeometryHandleMap[linkIndex1];
            GeometryHandle geometry2 = linkIndexToGeometryHandleMap[linkIndex2];
            if(geometry1 && geometry2){
                collisionDetector->ignoreGeometryPair(geometry1, geometry2);
            }
        });
    
    hasCustomObjectsAssociatedWithLinks = false;
    isGeometryHandleMapEnabled = false;
}


BodyCollisionDetector::BodyCollisionDetector(CollisionDetector* collisionDetector)
{
    impl = new Impl;
    impl->collisionDetector = collisionDetector;
}


BodyCollisionDetector::~BodyCollisionDetector()
{
    delete impl;
}


void BodyCollisionDetector::setCollisionDetector(CollisionDetector* collisionDetector)
{
    impl->collisionDetector = collisionDetector;
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
    impl->linkToGeometryHandleMap.clear();
    impl->bodyExistenceConnections.disconnect();
    impl->hasCustomObjectsAssociatedWithLinks = false;
}


void BodyCollisionDetector::setLinkAssociatedObjectFunction(LinkAssociatedObjectFunc func)
{
    impl->linkAssociatedObjectFunc = func;
}


void BodyCollisionDetector::addBody(Body* body, bool isSelfCollisionDetectionEnabled, int groupId)
{
    impl->addBody(body, isSelfCollisionDetectionEnabled, groupId);
}


/*
void BodyCollisionDetector::addBody(Body* body, bool isSelfCollisionDetectionEnabled, int groupId)
{
    //impl->linkAssociatedObjectFunc = getObjectAssociatedWithLink;
    if(impl->addBody(body, isSelfCollisionDetectionEnabled)){
        impl->hasCustomObjectsAssociatedWithLinks = true;
    }
}
*/


bool BodyCollisionDetector::Impl::addBody(Body* body, bool isSelfCollisionDetectionEnabled, int groupId)
{
    const int numLinks = body->numLinks();
    linkIndexToGeometryHandleMap.clear();
    linkIndexToGeometryHandleMap.resize(numLinks, 0);

    bodyCollisionLinkFilter.setTargetBody(body, isSelfCollisionDetectionEnabled);
    
    bool added = addLinkRecursively(body->rootLink(), true, groupId);

    if(added){
        bodyCollisionLinkFilter.apply();

        bodyExistenceConnections.add(
            body->sigExistenceChanged().connect(
                [this, body](bool on){ onBodyExistenceChanged(body, on); }));
    }

    return added;
}


bool BodyCollisionDetector::Impl::addLinkRecursively(Link* link, bool isParentStatic, int groupId)
{
    int linkIndex = link->index();
    bool added = false;
    bool isStatic = isParentStatic && link->isFixedJoint();

    if(bodyCollisionLinkFilter.checkIfEnabledLinkIndex(linkIndex)){
        if(auto handle = collisionDetector->addGeometry(link->collisionShape())){
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
            linkIndexToGeometryHandleMap[linkIndex] = *handle;
            if(isGeometryHandleMapEnabled){
                linkToGeometryHandleMap[link] = *handle;
            }
            added = true;
        }
    }    
    for(Link* child = link->child(); child; child = child->sibling()){
        added |= addLinkRecursively(child, isStatic, groupId);
    }

    return added;
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


bool BodyCollisionDetector::hasBodies() const
{
    return impl->collisionDetector->numGeometries() > 0;
}


bool BodyCollisionDetector::makeReady()
{
    return impl->collisionDetector->makeReady();
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
    if(!impl->hasCustomObjectsAssociatedWithLinks){
        impl->collisionDetector->updatePositions(
            [](Referenced* object, Isometry3*& out_position){
                out_position = &(static_cast<Link*>(object)->position()); });
    }
}


void BodyCollisionDetector::updatePositions
(std::function<void(Referenced* object, Isometry3*& out_position)> positionQuery)
{
    impl->collisionDetector->updatePositions(positionQuery);
}


void BodyCollisionDetector::detectCollisions(std::function<void(const CollisionPair& collisionPair)> callback)
{
    impl->collisionDetector->detectCollisions(callback);
}


void BodyCollisionDetector::detectCollisions
(Link* link, std::function<void(const CollisionPair& collisionPair)> callback)
{
    if(auto handle = findGeometryHandle(link)){
        impl->collisionDetector->detectCollisions(*handle, callback);
    }
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
