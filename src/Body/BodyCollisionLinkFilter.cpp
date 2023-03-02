#include "BodyCollisionLinkFilter.h"
#include "Body.h"
#include <cnoid/ValueTree>
#include <cnoid/IdPair>
#include <vector>
#include <unordered_set>

using namespace std;
using namespace cnoid;

namespace cnoid {

class BodyCollisionLinkFilter::Impl
{
public:
    function<void(int linkIndex1, int linkIndex2)> funcToDisableLinkPair;
    vector<bool> linkExclusionFlags;
    unordered_set<IdPair<int>> ignoredLinkPairs;
    int numLinks;
    bool isSelfCollisionDetectionEnabled;

    Impl();
    void setTargetBody(Body* body, bool isSelfCollisionDetectionEnabled);
    void checkCollisionDetectionTargets(Body* body, Listing* rules, bool isSelfCollisionDetectionEnabled);
    void checkCollisionDetectionTargets_OldFormat(Body* body, Mapping* info, bool isSelfCollisionDetectionEnabled);
    void setIgnoredLinkPairsWithinLinkChainLevel(Body* body, int distance);
    void setIgnoredLinkPairsWithinLinkChainLevelIter(Link* link, Link* currentLink, Link* prevLink, int distance);
    void apply();
};

}


BodyCollisionLinkFilter::BodyCollisionLinkFilter()
{
    impl = new Impl;
}


BodyCollisionLinkFilter::Impl::Impl()
{
    numLinks = 0;
    isSelfCollisionDetectionEnabled = false;
}


BodyCollisionLinkFilter::~BodyCollisionLinkFilter()
{
    delete impl;
}


void BodyCollisionLinkFilter::setFuncToDisableLinkPair(FuncToDisableLinkPair func)
{
    impl->funcToDisableLinkPair = func;
}


void BodyCollisionLinkFilter::setTargetBody(Body* body, bool isSelfCollisionDetectionEnabled)
{
    impl->setTargetBody(body, isSelfCollisionDetectionEnabled);
}


void BodyCollisionLinkFilter::Impl::setTargetBody(Body* body, bool isSelfCollisionDetectionEnabled)
{
    this->isSelfCollisionDetectionEnabled = isSelfCollisionDetectionEnabled;
    
    numLinks = body->numLinks();
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
}


//! \todo Implement all the rules
void BodyCollisionLinkFilter::Impl::checkCollisionDetectionTargets
(Body* body, Listing* rules, bool isSelfCollisionDetectionEnabled)
{
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
                        if(auto link1 = body->link(enabledLinks[i].toString())){
                            int index1 = link1->index();
                            linkExclusionFlags[index1] = false;
                            for(int j = i + 1; j < enabledLinks.size(); ++j){
                                if(auto link2 = body->link(enabledLinks[j].toString())){
                                    int index2 = link2->index();
                                    ignoredLinkPairs.erase(IdPair<int>(index1, index2));
                                }
                            }
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


void BodyCollisionLinkFilter::Impl::checkCollisionDetectionTargets_OldFormat
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


void BodyCollisionLinkFilter::Impl::setIgnoredLinkPairsWithinLinkChainLevel(Body* body, int distance)
{
    if(distance > 0){
        for(auto& link : body->links()){
            setIgnoredLinkPairsWithinLinkChainLevelIter(
                link, link, nullptr, distance);
        }
    }
}


void BodyCollisionLinkFilter::Impl::setIgnoredLinkPairsWithinLinkChainLevelIter
(Link* link, Link* currentLink, Link* prevLink, int distance)
{
    if(!currentLink->isFreeJoint()){
        auto parent = currentLink->parent();
        if(parent && parent != prevLink){
            ignoredLinkPairs.emplace(link->index(), parent->index());
            if(distance >= 2){
                setIgnoredLinkPairsWithinLinkChainLevelIter(
                    link, parent, currentLink, distance - 1);
            }
        }
    }
    for(auto child = currentLink->child(); child; child = child->sibling()){
        if(child != prevLink && !child->isFreeJoint()){
            ignoredLinkPairs.emplace(link->index(), child->index());
            if(distance >= 2){
                setIgnoredLinkPairsWithinLinkChainLevelIter(
                    link, child, currentLink, distance - 1);
            }
        }
    }
}


bool BodyCollisionLinkFilter::checkIfEnabledLinkIndex(int linkIndex) const
{
    return !impl->linkExclusionFlags[linkIndex];
}


void BodyCollisionLinkFilter::apply()
{
    if(impl->funcToDisableLinkPair){
        impl->apply();
    }
}


void BodyCollisionLinkFilter::apply(FuncToDisableLinkPair func)
{
    impl->funcToDisableLinkPair = func;
    impl->apply();
}


void BodyCollisionLinkFilter::Impl::apply()
{
    if(isSelfCollisionDetectionEnabled){
        for(auto& pair : ignoredLinkPairs){
            if(!linkExclusionFlags[pair[0]] && !linkExclusionFlags[pair[1]]){
                funcToDisableLinkPair(pair[0], pair[1]);
            }
        }
    } else {
        // exclude all the self link pairs
        for(int i = 0; i < numLinks; ++i){
            if(!linkExclusionFlags[i]){
                for(int j = i + 1; j < numLinks; ++j){
                    if(!linkExclusionFlags[j]){
                        funcToDisableLinkPair(i, j);
                    }
                }
            }
        }
    }
}
