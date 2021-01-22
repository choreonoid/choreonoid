/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "BodyCollisionDetector.h"
#include "Body.h"
#include "Link.h"
#include <cnoid/SceneGraph>
#include <cnoid/IdPair>
#include <cnoid/ValueTree>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using namespace std;
using namespace cnoid;

namespace {

typedef CollisionDetector::GeometryHandle GeometryHandle;

}

namespace cnoid {

class BodyCollisionDetector::Impl
{
public:
    CollisionDetectorPtr collisionDetector;
    unordered_map<LinkPtr, GeometryHandle> linkToGeometryHandleMap;
    std::function<Referenced*(Link* link, CollisionDetector::GeometryHandle geometry)>
    funcToGetObjectAssociatedWithLink;
    vector<GeometryHandle> linkIndexToGeometryHandleMap;
    vector<bool> linkExclusionFlags;
    unordered_set<IdPair<int>> ignoredLinkPairs;
    bool hasCustomObjectsAssociatedWithLinks;
    bool isGeometryHandleMapEnabled;

    Impl();
    bool addBody(Body* body, bool isSelfCollisionEnabled);
    void ignoreLinkPair(int linkIndex1, int linkIndex2);
    void checkCollisionDetectionTargets(
        Body* body, Listing* rules, bool isSelfCollisionDetectionEnabled);
    void checkCollisionDetectionTargets_OldFormat(
        Body* body, Mapping* info, bool isSelfCollisionDetectionEnabled);
    void setIgnoredLinkPairsWithinLinkChainLevel(Body* body, int distance);
    void setIgnoredLinkPairsWithinLinkChainLevelIter(Link* link, Link* currentLink, Link* prevLink, int distance);
    bool addLinkRecursively(Link* link, bool isParentStatic);
    double findClosestPoints(Link* link1, Link* link2, Vector3& out_point1, Vector3& out_point2);    
};

}


BodyCollisionDetector::BodyCollisionDetector()
{
    impl = new Impl;
}


BodyCollisionDetector::Impl::Impl()
{
    hasCustomObjectsAssociatedWithLinks = false;
    isGeometryHandleMapEnabled = false;
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


void BodyCollisionDetector::enableGeometryHandleMap(bool on)
{
    if(!on){
        impl->linkToGeometryHandleMap.clear();
    }
    impl->isGeometryHandleMapEnabled = on;
}


void BodyCollisionDetector::clearBodies()
{
    if(impl->collisionDetector){
        impl->collisionDetector->clearGeometries();
    }
    impl->linkToGeometryHandleMap.clear();
    impl->hasCustomObjectsAssociatedWithLinks = false;
}


void BodyCollisionDetector::addBody(Body* body, bool isSelfCollisionDetectionEnabled)
{
    impl->funcToGetObjectAssociatedWithLink = nullptr;
    impl->addBody(body, isSelfCollisionDetectionEnabled);
}


void BodyCollisionDetector::addBody
(Body* body, bool isSelfCollisionDetectionEnabled,
 std::function<Referenced*(Link* link, CollisionDetector::GeometryHandle geometry)> getObjectAssociatedWithLink)
{
    impl->funcToGetObjectAssociatedWithLink = getObjectAssociatedWithLink;
    if(impl->addBody(body, isSelfCollisionDetectionEnabled)){
        impl->hasCustomObjectsAssociatedWithLinks = true;
    }
}


bool BodyCollisionDetector::Impl::addBody(Body* body, bool isSelfCollisionDetectionEnabled)
{
    const int numLinks = body->numLinks();
    linkIndexToGeometryHandleMap.clear();
    linkIndexToGeometryHandleMap.resize(numLinks, 0);
    linkExclusionFlags.clear();
    linkExclusionFlags.resize(numLinks, false);
    ignoredLinkPairs.clear();

    ListingPtr rules = body->info()->findListing("collision_detection_rules");
    if(rules->isValid()){
        checkCollisionDetectionTargets(body, rules, isSelfCollisionDetectionEnabled);
    } else {
        MappingPtr info = body->info()->findMapping("collisionDetection");
        if(info->isValid()){
            checkCollisionDetectionTargets_OldFormat(body, info, isSelfCollisionDetectionEnabled);
        } else {
            if(isSelfCollisionDetectionEnabled){
                // Self-collision detection does not apply to pairs of adjacent links by default
                setIgnoredLinkPairsWithinLinkChainLevel(body, 1);
            }
        }
    }

    bool added = addLinkRecursively(body->rootLink(), true);

    if(isSelfCollisionDetectionEnabled){
        for(auto& pair : ignoredLinkPairs){
            if(!linkExclusionFlags[pair[0]] && !linkExclusionFlags[pair[1]]){
                ignoreLinkPair(pair[0], pair[1]);
            }
        }
    } else {
        // exclude all the self link pairs
        for(int i = 0; i < numLinks; ++i){
            if(!linkExclusionFlags[i]){
                for(int j = i + 1; j < numLinks; ++j){
                    if(!linkExclusionFlags[j]){
                        ignoreLinkPair(i, j);
                    }
                }
            }
        }
    }

    return added;
}


void BodyCollisionDetector::Impl::ignoreLinkPair(int linkIndex1, int linkIndex2)
{
    GeometryHandle geometry1 = linkIndexToGeometryHandleMap[linkIndex1];
    GeometryHandle geometry2 = linkIndexToGeometryHandleMap[linkIndex2];
    if(geometry1 && geometry2){
        collisionDetector->ignoreGeometryPair(geometry1, geometry2);
    }
}


//! \todo Implement all the rules
void BodyCollisionDetector::Impl::checkCollisionDetectionTargets
(Body* body, Listing* rules, bool isSelfCollisionDetectionEnabled)
{
    const int numLinks = body->numLinks();
    
    for(auto& node : *rules){
        for(auto& kv : *node->toMapping()){
            auto& rule = kv.first;
            auto& value = kv.second;
            if(rule == "disabled_link_chain_level"){
                if(isSelfCollisionDetectionEnabled){
                    setIgnoredLinkPairsWithinLinkChainLevel(body, value->toInt());
                }
            } else if(rule == "enabled_links"){
                for(auto& node : *value->toListing()){
                    if(auto link = body->link(node->toString())){
                        int index = link->index();
                        linkExclusionFlags[index] = false;
                        auto p = ignoredLinkPairs.begin();
                        while(p != ignoredLinkPairs.end()){
                            auto& indexPair = *p;
                            if(indexPair[0] == index || indexPair[1] == index){
                                p = ignoredLinkPairs.erase(p);
                            } else {
                                ++p;
                            }
                        }
                    }
                }
            } else if(rule == "enabled_link_group"){
                auto& enabledLinks = *value->toListing();
                if(isSelfCollisionDetectionEnabled){
                    for(int i=0; i < enabledLinks.size(); ++i){
                        for(int j = i + 1; j < enabledLinks.size(); ++j){
                            auto link1 = body->link(enabledLinks[i].toString());
                            auto link2 = body->link(enabledLinks[j].toString());
                            ignoredLinkPairs.erase(IdPair<int>(link1->index(), link2->index()));
                        }
                    }
                }
            } else if(rule == "disabled_links"){
                if(value->isString() && value->toString() == "all"){
                    for(int i = 0; i < numLinks; ++i){
                        linkExclusionFlags[i] = true;
                        for(int j = i + 1; j < numLinks; ++j){
                            ignoredLinkPairs.emplace(i, j);
                        }
                    }
                } else if(value->isListing()){
                    for(auto& node : *value->toListing()){
                        if(auto link = body->link(node->toString())){
                            int index = link->index();
                            linkExclusionFlags[index] = true;
                            for(int i = 0; i < numLinks; ++i){
                                if(i != index){
                                    ignoredLinkPairs.emplace(index, i);
                                }
                            }
                        }
                    }
                }
            } else if(rule == "disabled_link_group"){
                auto& disabledLinks = *value->toListing();
                if(isSelfCollisionDetectionEnabled){
                    for(int i=0; i < disabledLinks.size(); ++i){
                        for(int j = i + 1; j < disabledLinks.size(); ++j){
                            auto link1 = body->link(disabledLinks[i].toString());
                            auto link2 = body->link(disabledLinks[j].toString());
                            if(link1 && link2){
                                ignoredLinkPairs.emplace(link1->index(), link2->index());
                            }
                        }
                    }
                }

            } else {
                // put warning
            }
        }
    }
}


void BodyCollisionDetector::Impl::checkCollisionDetectionTargets_OldFormat
(Body* body, Mapping* info, bool isSelfCollisionDetectionEnabled)
{
    if(isSelfCollisionDetectionEnabled){
        int excludeTreeDepth = 1;
        info->read("excludeTreeDepth", excludeTreeDepth);
        setIgnoredLinkPairsWithinLinkChainLevel(body, excludeTreeDepth);
    }

    const Listing& excludeLinks = *info->findListing("excludeLinks");
    for(int i=0; i < excludeLinks.size(); ++i){
        if(auto link = body->link(excludeLinks[i].toString())){
            linkExclusionFlags[link->index()] = true;
        }
    }

    if(isSelfCollisionDetectionEnabled){
        auto& excludeLinkGroupList = *info->findListing("excludeLinkGroups");
        for(int i=0; i < excludeLinkGroupList.size(); ++i){
            auto groupInfo = excludeLinkGroupList[i].toMapping();
            if(groupInfo->isValid()){
                auto& excludeLinks = *groupInfo->findListing("links");
                for(int j=0; j < excludeLinks.size(); ++j){
                    for(int k = j + 1; k < excludeLinks.size(); ++k){
                        auto link0 = body->link(excludeLinks[j].toString());
                        auto link1 = body->link(excludeLinks[k].toString());
                        if(link0 && link1){
                            ignoredLinkPairs.emplace(link0->index(), link1->index());
                        }
                    }
                }
            }
        }
    }
}


void BodyCollisionDetector::Impl::setIgnoredLinkPairsWithinLinkChainLevel
(Body* body, int distance)
{
    if(distance > 0){
        for(auto& link : body->links()){
            setIgnoredLinkPairsWithinLinkChainLevelIter(
                link, link, nullptr, distance);
        }
    }
}


void BodyCollisionDetector::Impl::setIgnoredLinkPairsWithinLinkChainLevelIter
(Link* link, Link* currentLink, Link* prevLink, int distance)
{
    auto parent = currentLink->parent();
    if(parent && parent != prevLink){
        ignoredLinkPairs.emplace(link->index(), parent->index());
        if(distance >= 2){
            setIgnoredLinkPairsWithinLinkChainLevelIter(
                link, parent, currentLink, distance - 1);
        }
    }
    for(auto child = currentLink->child(); child; child = child->sibling()){
        if(child != prevLink){
            ignoredLinkPairs.emplace(link->index(), child->index());
            if(distance >= 2){
                setIgnoredLinkPairsWithinLinkChainLevelIter(
                    link, child, currentLink, distance - 1);
            }
        }
    }
}


bool BodyCollisionDetector::Impl::addLinkRecursively(Link* link, bool isParentStatic)
{
    int linkIndex = link->index();
    bool added = false;
    bool isStatic = isParentStatic && link->isFixedJoint();

    if(!linkExclusionFlags[linkIndex]){
        if(auto handle = collisionDetector->addGeometry(link->collisionShape())){
            Referenced* object;
            if(funcToGetObjectAssociatedWithLink){
                object = funcToGetObjectAssociatedWithLink(link, *handle);
            } else {
                object = link;
            }
            collisionDetector->setCustomObject(*handle, object);
            if(isStatic){
                collisionDetector->setGeometryStatic(*handle, object);
            }
            linkIndexToGeometryHandleMap[linkIndex] = *handle;
            if(isGeometryHandleMapEnabled){
                linkToGeometryHandleMap[link] = *handle;
            }
            added = true;
        }
    }    
    for(Link* child = link->child(); child; child = child->sibling()){
        added |= addLinkRecursively(child, isStatic);
    }

    return added;
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
