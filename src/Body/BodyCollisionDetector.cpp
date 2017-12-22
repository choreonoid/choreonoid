/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "BodyCollisionDetector.h"
#include "Body.h"
#include "Link.h"
#include <cnoid/ValueTree>
#include <boost/variant.hpp>
#include <unordered_map>
#include <vector>

using namespace std;
using namespace cnoid;
using boost::variant;

namespace {

typedef variant<int, vector<int>> GeometryIdSet;

class GeometryIdSetAccessor
{
    GeometryIdSet& ids;
    int* begin_;
    int* end_;

public:
    GeometryIdSetAccessor(GeometryIdSet& ids) : ids(ids)
    {
        if(ids.which() == 0){
            begin_ = &get<int>(ids);
            end_ = begin_ + 1;
        } else {
            auto& v = get<vector<int>>(ids);
            begin_ = &v.front();
            end_ = begin_ + v.size();
        }
    }
    int* begin() { return begin_; }
    int* end() { return end_; }
};
    
}


namespace cnoid {

class BodyCollisionDetectorImpl
{
public:
    CollisionDetectorPtr collisionDetector;
    unordered_map<LinkPtr, GeometryIdSet> linkToGeometryIdSetMap;

    BodyCollisionDetectorImpl();
    void addBody(Body* body, bool isSelfCollisionEnabled);
    void addLinkRecursively(
        Link* link, vector<GeometryIdSet>& linkIndexToGeometryIdSetMap, vector<bool>& exclusions, bool isParentStatic);
    void setNonInterfarenceLinkPair(
        int link1Index, int link2Index, vector<GeometryIdSet>& linkIndexToGeometryIdSetMap);
    void setNonInterfarenceLinkPairs(
        Body* body, vector<GeometryIdSet>& linkIndexToGeometryIdSetMap,
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

}


BodyCollisionDetector::~BodyCollisionDetector()
{
    delete impl;
}


void BodyCollisionDetector::setCollisionDetector(CollisionDetector* collisionDetector)
{
    impl->collisionDetector = collisionDetector;
    impl->linkToGeometryIdSetMap.clear();
}


CollisionDetector* BodyCollisionDetector::collisionDetector()
{
    return impl->collisionDetector;
}


void BodyCollisionDetector::clearBodies()
{
    impl->linkToGeometryIdSetMap.clear();
    impl->collisionDetector->clearGeometries();
}


void BodyCollisionDetector::addBody(Body* body, bool isSelfCollisionDetectionEnabled)
{
    impl->addBody(body, isSelfCollisionDetectionEnabled);
}


void BodyCollisionDetectorImpl::addBody(Body* body, bool isSelfCollisionDetectionEnabled)
{
    const int numLinks = body->numLinks();
    int excludeTreeDepth = 0;
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
                string groupName = groupInfo->get<string>("name");
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

    vector<GeometryIdSet> linkIndexToGeometryIdSetMap(numLinks);

    addLinkRecursively(body->rootLink(), linkIndexToGeometryIdSetMap, exclusions, true);

    if(isSelfCollisionDetectionEnabled){
        setNonInterfarenceLinkPairs(
            body, linkIndexToGeometryIdSetMap, excludeTreeDepth, exclusions, excludeLinkGroups);
    } else {
        // exclude all the self link pairs
        for(int i=0; i < numLinks; ++i){
            if(!exclusions[i]){
                for(int j=i+1; j < numLinks; ++j){
                    if(!exclusions[j]){
                        setNonInterfarenceLinkPair(i, j, linkIndexToGeometryIdSetMap);
                    }
                }
            }
        }
    }
}


void BodyCollisionDetectorImpl::addLinkRecursively
(Link* link, vector<GeometryIdSet>& linkIndexToGeometryIdSetMap, vector<bool>& exclusions, bool isParentStatic)
{
    bool isStatic = isParentStatic && link->isFixedJoint();

    if(!exclusions[link->index()]){
        GeometryIdSet idSet = collisionDetector->addGeometry(link->collisionShape(), link);

        linkToGeometryIdSetMap[link] = idSet;
        linkIndexToGeometryIdSetMap[link->index()] = idSet;
        
        GeometryIdSetAccessor accessor(idSet);
        for(auto p = accessor.begin(); p != accessor.end(); ++p){
            if(isStatic){
                collisionDetector->setGeometryStatic(*p);
            }
            // disable the collision detection for the geometries in a link
            for(auto q = p + 1; q != accessor.end(); ++q){
                collisionDetector->setNonInterfarenceGeometyrPair(*p, *q);
            }
        }
    }
    
    for(Link* child = link->child(); child; child = child->sibling()){
        addLinkRecursively(child, linkIndexToGeometryIdSetMap, exclusions, isStatic);
    }
}


void BodyCollisionDetectorImpl::setNonInterfarenceLinkPair
(int link1Index, int link2Index, vector<GeometryIdSet>& linkIndexToGeometryIdSetMap)
{
    GeometryIdSetAccessor accessor1(linkIndexToGeometryIdSetMap[link1Index]);
    GeometryIdSetAccessor accessor2(linkIndexToGeometryIdSetMap[link2Index]);

    for(auto p = accessor1.begin(); p != accessor1.end(); ++p){
        for(auto q = accessor2.begin(); q != accessor2.end(); ++q){
            collisionDetector->setNonInterfarenceGeometyrPair(*p, *q);
        }
    }
}


void BodyCollisionDetectorImpl::setNonInterfarenceLinkPairs
(Body* body, vector<GeometryIdSet>& linkIndexToGeometryIdSetMap,
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
                                setNonInterfarenceLinkPair(i, j, linkIndexToGeometryIdSetMap);
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
                    setNonInterfarenceLinkPair(index1, index2, linkIndexToGeometryIdSetMap);
                }
            }
        }
    }
}


bool BodyCollisionDetector::makeReady()
{
    return impl->collisionDetector->makeReady();
}


void BodyCollisionDetector::updatePositions()
{
    impl->collisionDetector->updatePositions(
        [](Referenced* object, Position*& out_Position){
            Link* link = static_cast<Link*>(object);
            out_Position = &link->position();
        });
}


void BodyCollisionDetector::detectCollisions(std::function<void(const CollisionPair& collisionPair)> callback)
{
    impl->collisionDetector->detectCollisions(callback);
}


bool BodyCollisionDetector::isFindClosestPointsAvailable() const
{
    return impl->collisionDetector->isFindClosestPointsAvailable();
}


double BodyCollisionDetector::findClosestPoints(Link* link1, Link* link2, Vector3& out_point1, Vector3& out_point2)
{
    return impl->findClosestPoints(link1, link2, out_point1, out_point2);
}


double BodyCollisionDetectorImpl::findClosestPoints(Link* link1, Link* link2, Vector3& out_point1, Vector3& out_point2)
{
    auto pids1 = linkToGeometryIdSetMap.find(link1);
    if(pids1 != linkToGeometryIdSetMap.end()){
        auto pids2 = linkToGeometryIdSetMap.find(link2);
        if(pids2 != linkToGeometryIdSetMap.end()){
            GeometryIdSet& ids1 = pids1->second;
            GeometryIdSet& ids2 = pids2->second;
            if(ids1.which() == 0 && ids2.which() == 0){
                return collisionDetector->findClosestPoints(
                    boost::get<int>(ids1), boost::get<int>(ids2), out_point1, out_point2);
            } else {
                double minDistance = std::numeric_limits<double>::max();
                GeometryIdSetAccessor accessor1(ids1);
                GeometryIdSetAccessor accessor2(ids2);
                for(auto pid1 = accessor1.begin(); pid1 != accessor1.end(); ++pid1){
                    for(auto pid2 = accessor2.begin(); pid2 != accessor2.end(); ++pid2){
                        Vector3 point1, point2;
                        double distance = collisionDetector->findClosestPoints(*pid1, *pid2, point1, point2);
                        if(distance < minDistance){
                            minDistance = distance;
                            out_point1 = point1;
                            out_point2 = point2;
                        }
                    }
                }
                return minDistance;
            }
        }
    }
    return -1.0;
}


