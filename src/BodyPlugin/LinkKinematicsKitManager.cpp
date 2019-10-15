#include "LinkKinematicsKitManager.h"
#include <cnoid/Link>
#include <cnoid/LinkKinematicsKit>
#include <cnoid/CoordinateFrameSet>
#include <cnoid/CoordinateFrameSetPairItem>
#include <cnoid/BodyItem>
#include <cnoid/WorldItem>
#include <cnoid/ItemList>
#include <map>

using namespace std;
using namespace cnoid;

namespace cnoid {

class LinkKinematicsKitManager::Impl
{
public:
    BodyItem* bodyItem;
    map<int, LinkKinematicsKitPtr> linkIndexToKinematicsKitMap;

    Impl(BodyItem* bodyItem);
    LinkKinematicsKit* getOrCreateKinematicsKit(Link* targetLink);
    CoordinateFrameSetPairPtr extractCoordinateFrameSets();
    CoordinateFrameSetPairPtr extractWorldCoordinateFrameSets(Item* item);
};

}


LinkKinematicsKitManager::LinkKinematicsKitManager(BodyItem* bodyItem)
{
    impl = new Impl(bodyItem);
}


LinkKinematicsKitManager::Impl::Impl(BodyItem* bodyItem)
    : bodyItem(bodyItem)
{

}


LinkKinematicsKitManager::~LinkKinematicsKitManager()
{
    delete impl;
}


LinkKinematicsKit* LinkKinematicsKitManager::getOrCreateKinematicsKit(Link* targetLink)
{
    return impl->getOrCreateKinematicsKit(targetLink);
}


LinkKinematicsKit* LinkKinematicsKitManager::Impl::getOrCreateKinematicsKit(Link* targetLink)
{
    auto iter = linkIndexToKinematicsKitMap.find(targetLink->index());
    if(iter != linkIndexToKinematicsKitMap.end()){
        auto kit = iter->second;
        if(kit->link() == targetLink){
            return kit;
        }
    }
    
    auto kit = new LinkKinematicsKit(targetLink);

    if(auto frameSets = extractCoordinateFrameSets()){
        kit->setFrameSetPair(frameSets);
    }
    
    linkIndexToKinematicsKitMap[targetLink->index()] = kit;

    return kit;
}
    

LinkKinematicsKit* LinkKinematicsKitManager::getOrCreateKinematicsKit(Link* targetLink, Link* baseLink)
{
    auto kit = impl->getOrCreateKinematicsKit(targetLink);
    if(kit){
        kit->setBaseLink(baseLink);
    }
    return kit;
}


LinkKinematicsKit* LinkKinematicsKitManager::getOrCreateKinematicsKit
(Link* targetLink, std::shared_ptr<InverseKinematics> ik)
{
    auto kit = impl->getOrCreateKinematicsKit(targetLink);
    if(kit){
        kit->setInversetKinematics(ik);
    }
    return kit;
}



CoordinateFrameSetPairPtr LinkKinematicsKitManager::Impl::extractCoordinateFrameSets()
{
    CoordinateFrameSetPairPtr extracted;
    
    ItemList<CoordinateFrameSetPairItem> lowerItems;
    if(lowerItems.extractSubTreeItems(bodyItem)){
        extracted = lowerItems.toSingle()->frameSetPair();
    } else {
        auto upperItem = bodyItem->parentItem();
        while(upperItem){
            if(auto fspairItem = dynamic_cast<CoordinateFrameSetPairItem*>(upperItem)){
                extracted = fspairItem->frameSetPair();
                break;
            }
            if(auto worldItem = dynamic_cast<WorldItem*>(upperItem)){
                extracted = extractWorldCoordinateFrameSets(worldItem);
                break;
            }
            upperItem = upperItem->parentItem();
        }
    }

    return extracted;
}


CoordinateFrameSetPairPtr LinkKinematicsKitManager::Impl::extractWorldCoordinateFrameSets(Item* item)
{
    if(auto frameSetPairItem = dynamic_cast<CoordinateFrameSetPairItem*>(item)){
        return frameSetPairItem->frameSetPair();
    } else if(auto bodyItem = dynamic_cast<BodyItem*>(item)){
        return nullptr;
    }
    for(Item* child = item->childItem(); child; child = child->nextItem()){
        if(auto frameSetPair = extractWorldCoordinateFrameSets(child)){
            return frameSetPair;
        }
    }
    return nullptr;
}
