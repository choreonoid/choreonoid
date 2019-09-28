/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "BodyCollisionDetector.h"
#include "Body.h"
#include "Link.h"
#include <cnoid/ValueTree>
#include <unordered_map>
#include <vector>

using namespace std;
using namespace cnoid;

namespace {

typedef CollisionDetector::GeometryHandle GeometryHandle;

}

namespace cnoid {

class BodyCollisionDetectorImpl
{
public:
    CollisionDetectorPtr collisionDetector;
    unordered_map<LinkPtr, GeometryHandle> linkToGeometryHandleMap;
    std::function<Referenced*(Link* link, CollisionDetector::GeometryHandle geometry)> funcToGetObjectAssociatedWithLink;
    bool hasCustomObjectsAssociatedWithLinks;
    bool isGeometryHandleMapEnabled;

    BodyCollisionDetectorImpl();
    bool addBody(Body* body, bool isSelfCollisionEnabled);
    bool addLinkRecursively(
        Link* link, vector<GeometryHandle>& linkIndexToGeometryIdSetMap, vector<bool>& exclusions, bool isParentStatic);
    void setNonInterfarenceLinkPair(
        int link1Index, int link2Index, vector<GeometryHandle>& linkIndexToGeometryIdSetMap);
    void setNonInterfarenceLinkPairs(
        Body* body, vector<GeometryHandle>& linkIndexToGeometryIdSetMap,
        int excludeTreeDepth, vector<bool>& exclusions, vector<vector<int>>& excludeLinkGroups);
    double findClosestPoints(Link* link1, Link* link2, Vector3& out_point1, Vector3& out_point2);    
};

}


BodyCollisionDetector::BodyCollisionDetector()
{
    impl = new BodyCollisionDetectorImpl;
}


BodyCollisionDetectorImpl::BodyCollisionDetectorImpl()
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


bool BodyCollisionDetectorImpl::addBody(Body* body, bool isSelfCollisionDetectionEnabled)
{
    const int numLinks = body->numLinks();

    // Self-collision detection does not apply to pairs of adjacent links by default
    int excludeTreeDepth = 1;
    
    vector<bool> exclusions(numLinks);
    vector<vector<int>> excludeLinkGroups;
    
    auto cdInfo = body->info()->findMapping("collisionDetection");
    if(cdInfo->isValid()){
        excludeTreeDepth = cdInfo->get("excludeTreeDepth", excludeTreeDepth);
        const Listing& excludeLinks = *cdInfo->findListing("excludeLinks");
        for(size_t i=0; i < excludeLinks.size(); ++i){
            Link* link = body->link(excludeLinks[i].toString());
            if(link){
                exclusions[link->index()] = true;
            }
        }
        auto& excludeLinkGroupList = *cdInfo->findListing("excludeLinkGroups");
        excludeLinkGroups.reserve(excludeLinkGroupList.size());
        for(int i=0; i < excludeLinkGroupList.size(); ++i){
            auto groupInfo = excludeLinkGroupList[i].toMapping();
            if(groupInfo->isValid()){
                vector<int> excludeLinkGroup;
                //string groupName;
                //groupInfo->read("name",groupName);
                auto& excludeLinks = *groupInfo->findListing("links");
                for(size_t j=0; j < excludeLinks.size(); ++j){
                    Link* link = body->link(excludeLinks[j].toString());
                    if(link){
                        excludeLinkGroup.push_back(link->index());
                    }
                }
                if(!excludeLinkGroup.empty()){
                    excludeLinkGroups.push_back(excludeLinkGroup);
                }
            }
        }
    }

    vector<GeometryHandle> linkIndexToGeometryHandleMap(numLinks);

    bool added = addLinkRecursively(body->rootLink(), linkIndexToGeometryHandleMap, exclusions, true);

    if(isSelfCollisionDetectionEnabled){
        setNonInterfarenceLinkPairs(
            body, linkIndexToGeometryHandleMap, excludeTreeDepth, exclusions, excludeLinkGroups);
    } else {
        // exclude all the self link pairs
        for(int i=0; i < numLinks; ++i){
            if(!exclusions[i]){
                for(int j=i+1; j < numLinks; ++j){
                    if(!exclusions[j]){
                        setNonInterfarenceLinkPair(i, j, linkIndexToGeometryHandleMap);
                    }
                }
            }
        }
    }

    return added;
}


bool BodyCollisionDetectorImpl::addLinkRecursively
(Link* link, vector<GeometryHandle>& linkIndexToGeometryHandleMap, vector<bool>& exclusions, bool isParentStatic)
{
    bool added = false;
    bool isStatic = isParentStatic && link->isFixedJoint();

    if(!exclusions[link->index()]){
        auto handle = collisionDetector->addGeometry(link->collisionShape());
        if(handle){
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
            linkIndexToGeometryHandleMap[link->index()] = *handle;
            if(isGeometryHandleMapEnabled){
                linkToGeometryHandleMap[link] = *handle;
            }
            added = true;
        }
    }    
    for(Link* child = link->child(); child; child = child->sibling()){
        added |= addLinkRecursively(child, linkIndexToGeometryHandleMap, exclusions, isStatic);
    }

    return added;
}


void BodyCollisionDetectorImpl::setNonInterfarenceLinkPair
(int link1Index, int link2Index, vector<GeometryHandle>& linkIndexToGeometryHandleMap)
{
    GeometryHandle geometry1 = linkIndexToGeometryHandleMap[link1Index];
    GeometryHandle geometry2 = linkIndexToGeometryHandleMap[link2Index];
    collisionDetector->setNonInterfarenceGeometyrPair(geometry1, geometry2);
}


void BodyCollisionDetectorImpl::setNonInterfarenceLinkPairs
(Body* body, vector<GeometryHandle>& linkIndexToGeometryHandleMap,
 int excludeTreeDepth, vector<bool>& exclusions, vector<vector<int>>& excludeLinkGroups)
{
    const int numLinks = body->numLinks();
    
    // exclude the link pairs whose distance in the tree is less than excludeTreeDepth
    if(excludeTreeDepth > 0){
        for(int i=0; i < numLinks; ++i){
            if(!exclusions[i]){
                Link* link1 = body->link(i);
                for(int j=i+1; j < numLinks; ++j){
                    if(!exclusions[j]){
                        Link* link2 = body->link(j);
                        Link* parent1 = link1;
                        Link* parent2 = link2;
                        for(int k=0; k < excludeTreeDepth; ++k){
                            if(parent1){
                                parent1 = parent1->parent();
                            }
                            if(parent2){
                                parent2 = parent2->parent();
                            }
                            if(!parent1 && !parent2){
                                break;
                            }
                            if(parent1 == link2 || parent2 == link1){
                                setNonInterfarenceLinkPair(i, j, linkIndexToGeometryHandleMap);
                            }
                        }
                    }
                }
            }
        }
    }

    for(int i=0; i < excludeLinkGroups.size(); ++i){
        const vector<int>& excludeLinkGroup = excludeLinkGroups[i];
        for(int j=0; j < excludeLinkGroup.size(); ++j){
            int index1 = excludeLinkGroup[j];
            for(int k = j + 1; k < excludeLinkGroup.size(); ++k){
                int index2 = excludeLinkGroup[k];
                if(!exclusions[index1] && !exclusions[index2]){
                    setNonInterfarenceLinkPair(index1, index2, linkIndexToGeometryHandleMap);
                }
            }
        }
    }
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
            [](Referenced* object, Position*& out_position){
                out_position = &(static_cast<Link*>(object)->position()); });
    }
}


void BodyCollisionDetector::updatePositions
(std::function<void(Referenced* object, Position*& out_position)> positionQuery)
{
    impl->collisionDetector->updatePositions(positionQuery);
}


void BodyCollisionDetector::detectCollisions(std::function<void(const CollisionPair& collisionPair)> callback)
{
    impl->collisionDetector->detectCollisions(callback);
}
