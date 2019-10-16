#include "LinkKinematicsKitManager.h"
#include "BodySelectionManager.h"
#include <cnoid/Link>
#include <cnoid/LinkKinematicsKit>
#include <cnoid/CoordinateFrameSet>
#include <cnoid/CoordinateFrameListPairItem>
#include <cnoid/RootItem>
#include <cnoid/BodyItem>
#include <cnoid/WorldItem>
#include <cnoid/ItemList>
#include <cnoid/PositionEditManager>
#include <cnoid/PositionDragger>
#include <cnoid/ConnectionSet>
#include <map>

using namespace std;
using namespace cnoid;

namespace cnoid {

class LinkKinematicsKitManager::Impl
{
public:
    BodyItem* bodyItem;
    map<int, LinkKinematicsKitPtr> linkIndexToKinematicsKitMap;

    ScopedConnection treeChangeConnection;
    CoordinateFrameSetPairPtr commonFrameSetPair;

    BodySelectionManager* bodySelectionManager;
    AbstractPositionEditTarget* frameEditTarget;
    Link* frameEditLink;
    PositionDraggerPtr positionDragger;
    SgUpdate update;
    ScopedConnection positionEditManagerConnection;
    ScopedConnectionSet frameEditConnections;

    Impl(BodyItem* bodyItem);
    LinkKinematicsKit* getOrCreateKinematicsKit(Link* targetLink);
    CoordinateFrameSetPairPtr extractCoordinateFrameSets();
    CoordinateFrameSetPairPtr extractWorldCoordinateFrameSets(Item* item);
    void onTreeChanged();
    void setupPositionDragger();
    bool onPositionEditRequest(AbstractPositionEditTarget* target);
    bool startBaseFrameEditing(AbstractPositionEditTarget* target, CoordinateFrame* frame);
    bool startLocalFrameEditing(AbstractPositionEditTarget* target, CoordinateFrame* frame);
    void setFrameEditTarget(AbstractPositionEditTarget* target, Link* link);
    void onFrameEditPositionChanged(const Position& T);
    void onDraggerPositionChanged();
};

}


LinkKinematicsKitManager::LinkKinematicsKitManager(BodyItem* bodyItem)
{
    impl = new Impl(bodyItem);
}


LinkKinematicsKitManager::Impl::Impl(BodyItem* bodyItem)
    : bodyItem(bodyItem)
{
    commonFrameSetPair = new CoordinateFrameSetPair;
    
    treeChangeConnection =
        RootItem::instance()->sigTreeChanged().connect([&](){ onTreeChanged(); });

    bodySelectionManager = BodySelectionManager::instance();
    
    setupPositionDragger();
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

    

    kit->setFrameSetPair(commonFrameSetPair);
    
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
    
    ItemList<CoordinateFrameListPairItem> lowerItems;
    if(lowerItems.extractSubTreeItems(bodyItem)){
        extracted = lowerItems.toSingle()->frameSetPair();
    } else {
        auto upperItem = bodyItem->parentItem();
        while(upperItem){
            if(auto listPairItem = dynamic_cast<CoordinateFrameListPairItem*>(upperItem)){
                extracted = listPairItem->frameSetPair();
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
    if(auto listPairItem = dynamic_cast<CoordinateFrameListPairItem*>(item)){
        return listPairItem->frameSetPair();
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


void LinkKinematicsKitManager::Impl::onTreeChanged()
{
    if(auto fsets = extractCoordinateFrameSets()){
        *commonFrameSetPair = *fsets;
    }
}


SgNode* LinkKinematicsKitManager::scene()
{
    return impl->positionDragger;
}


void LinkKinematicsKitManager::Impl::setupPositionDragger()
{
    positionDragger = new PositionDragger;
    positionDragger->setRadius(0.05);
    positionDragger->setDisplayMode(PositionDragger::DisplayInEditMode);
    positionDragger->setContainerMode(true);

    positionDragger->sigPositionDragged().connect(
        [&](){ onDraggerPositionChanged(); });

    frameEditTarget = nullptr;

    positionEditManagerConnection =
        PositionEditManager::instance()->sigPositionEditRequest().connect(
            [&](AbstractPositionEditTarget* target){
                return onPositionEditRequest(target); });
}    


bool LinkKinematicsKitManager::Impl::onPositionEditRequest(AbstractPositionEditTarget* target)
{
    bool accepted = false;
    if(auto frame = dynamic_cast<CoordinateFrame*>(target->getPositionObject())){
        if(bodyItem == bodySelectionManager->currentBodyItem()){
            if(auto frameSet = frame->ownerFrameSet()){
                if(commonFrameSetPair->baseFrameSet()->contains(frameSet)){
                    accepted = startBaseFrameEditing(target, frame);
                } else if(commonFrameSetPair->localFrameSet()->contains(frameSet)){
                    accepted = startLocalFrameEditing(target, frame);
                }
            }
        }
    }
    return accepted;
}


bool LinkKinematicsKitManager::Impl::startBaseFrameEditing
(AbstractPositionEditTarget* target, CoordinateFrame* frame)
{
    if(auto link = bodySelectionManager->currentLink()){
        auto kit = bodyItem->getLinkKinematicsKit(link);
        if(auto baseLink = kit->baseLink()){
            setFrameEditTarget(target, baseLink);
            return true;
        }
    }
    return false;
}


bool LinkKinematicsKitManager::Impl::startLocalFrameEditing
(AbstractPositionEditTarget* target, CoordinateFrame* frame)
{
    if(auto link = bodySelectionManager->currentLink()){
        setFrameEditTarget(target, link);
        return true;
    }
    return false;
}


void LinkKinematicsKitManager::Impl::setFrameEditTarget(AbstractPositionEditTarget* target, Link* link)
{
    frameEditConnections.disconnect();

    frameEditTarget = target;
    frameEditLink = link;

    if(target){
        frameEditConnections.add(
            target->sigPositionChanged().connect(
                [&](const Position& T){ onFrameEditPositionChanged(T); }));

        frameEditConnections.add(
            target->sigPositionEditTargetExpired().connect(
                [=](){ setFrameEditTarget(nullptr, link); }));

        frameEditConnections.add(
            bodyItem->sigKinematicStateChanged().connect(
                [&](){ onFrameEditPositionChanged(frameEditTarget->getPosition()); }));

        onFrameEditPositionChanged(target->getPosition());
    }
}


void LinkKinematicsKitManager::Impl::onFrameEditPositionChanged(const Position& T)
{
    Position F;
    F.linear() = frameEditLink->attitude();
    F.translation() = frameEditLink->translation();
    positionDragger->setPosition(F * T);
    positionDragger->notifyUpdate(update);
}


void LinkKinematicsKitManager::Impl::onDraggerPositionChanged()
{
    if(frameEditTarget){
        frameEditConnections.block();
        Position F;
        F.linear() = frameEditLink->attitude();
        F.translation() = frameEditLink->translation();
        Position T = F.inverse(Eigen::Isometry) * positionDragger->T();
        frameEditTarget->setPosition(T);
        frameEditConnections.unblock();
    }
}
