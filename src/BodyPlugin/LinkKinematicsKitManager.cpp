#include "LinkKinematicsKitManager.h"
#include "BodySelectionManager.h"
#include "KinematicsBar.h"
#include <cnoid/Link>
#include <cnoid/LinkKinematicsKit>
#include <cnoid/JointPath>
#include <cnoid/CompositeIK>
#include <cnoid/PinDragIK>
#include <cnoid/CoordinateFrameList>
#include <cnoid/CoordinateFrameListItem>
#include <cnoid/RootItem>
#include <cnoid/BodyItem>
#include <cnoid/WorldItem>
#include <cnoid/ItemList>
#include <cnoid/PositionEditManager>
#include <cnoid/PositionDragger>
#include <cnoid/ConnectionSet>
#include <cnoid/ValueTree>
#include <map>

using namespace std;
using namespace cnoid;

namespace {

enum BaseLinkId {
    PresetBaseLink = -1,
    FreeBaseLink = -2
};

}
    
namespace cnoid {

class LinkKinematicsKitManager::Impl
{
public:
    BodyItem* bodyItem;

    // Key is pair(target link index, base link index);
    // Use an integer index value as a key to keep the number of instances growing
    map<std::pair<int, int>, LinkKinematicsKitPtr> linkPairToKinematicsKitMap;

    ScopedConnection bodyItemConnection;
    ScopedConnection frameListConnection;
    CoordinateFrameListPtr baseFrames;
    CoordinateFrameListPtr offsetFrames;
    CoordinateFrameListPtr defaultBaseFrames;
    CoordinateFrameListPtr defaultOffsetFrames;

    BodySelectionManager* bodySelectionManager;
    AbstractPositionEditTarget* frameEditTarget;
    Link* frameEditLink;
    PositionDraggerPtr positionDragger;
    SgUpdate update;
    ScopedConnection positionEditManagerConnection;
    ScopedConnectionSet frameEditConnections;

    Impl(BodyItem* bodyItem);
    void onBodyItemPositionChanged();
    LinkKinematicsKit* findKinematicsKit(Link* targetLink, bool isPresetOnly);
    std::shared_ptr<InverseKinematics> findPresetIK(Link* targetLink);
    bool updateCoordinateFramesOf(LinkKinematicsKit* kit, bool forceUpdate);
    void findFrameListItems();
    void onFrameListItemAddedOrUpdated(CoordinateFrameListItem* frameListItem);
    void setupPositionDragger();
    bool onPositionEditRequest(AbstractPositionEditTarget* target);
    bool startBaseFrameEditing(AbstractPositionEditTarget* target, CoordinateFrame* frame);
    bool startOffsetFrameEditing(AbstractPositionEditTarget* target, CoordinateFrame* frame);
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
    bodySelectionManager = BodySelectionManager::instance();

    bodyItemConnection =
        bodyItem->sigPositionChanged().connect(
            [&](){ onBodyItemPositionChanged(); });

    onBodyItemPositionChanged();
    
    setupPositionDragger();
}


LinkKinematicsKitManager::~LinkKinematicsKitManager()
{
    delete impl;
}


void LinkKinematicsKitManager::Impl::onBodyItemPositionChanged()
{
    if(bodyItem->isConnectedToRoot()){
        if(!frameListConnection.connected()){
            frameListConnection =
                CoordinateFrameListItem::sigInstanceAddedOrUpdated().connect(
                    [&](CoordinateFrameListItem* frameListItem){
                        onFrameListItemAddedOrUpdated(frameListItem);
                    });
        }
        findFrameListItems();
    } else {
        if(frameListConnection.connected()){
            frameListConnection.disconnect();
            baseFrames.reset();
            offsetFrames.reset();
        }
    }
}


LinkKinematicsKit* LinkKinematicsKitManager::getCurrentKinematicsKit(Link* targetLink)
{
    return impl->findKinematicsKit(targetLink, false);
}


LinkKinematicsKit* LinkKinematicsKitManager::findPresetKinematicsKit(Link* targetLink)
{
    return impl->findKinematicsKit(targetLink, true);
}


LinkKinematicsKit* LinkKinematicsKitManager::Impl::findKinematicsKit(Link* targetLink, bool isPresetOnly)
{
    LinkKinematicsKit* kit = nullptr;

    if(isPresetOnly && !targetLink){
        targetLink = bodyItem->body()->findUniqueEndLink();
    }
    if(!targetLink){
        return nullptr;
    }

    bool isPresetMode = KinematicsBar::instance()->mode() == KinematicsBar::PresetKinematics;
    Link* baseLink = nullptr;
    int baseLinkIndex;
    shared_ptr<InverseKinematics> presetIK;
    bool hasNoPresetIK = false;
    
    if(isPresetOnly){
        baseLinkIndex = PresetBaseLink;
    } else {
        baseLink = bodyItem->currentBaseLink();
        if(baseLink){
            baseLinkIndex = baseLink->index();
        } else {
            baseLinkIndex = FreeBaseLink;
            if(isPresetMode){
                presetIK = findPresetIK(targetLink);
                if(presetIK){
                    baseLinkIndex = PresetBaseLink;
                } else {
                    hasNoPresetIK = true;
                }
            }
        }
    }
    auto key = make_pair(targetLink->index(), baseLinkIndex);
    
    auto iter = linkPairToKinematicsKitMap.find(key);
    if(iter != linkPairToKinematicsKitMap.end()){
        auto foundKit = iter->second;
        if(foundKit->link() == targetLink){
            kit = foundKit;
        }
    }

    if(!kit){
        if(isPresetOnly || isPresetMode){
            if(!presetIK && !hasNoPresetIK){
                presetIK = findPresetIK(targetLink);
            }
            if(presetIK){
                kit = new LinkKinematicsKit(targetLink);
                kit->setInversetKinematics(presetIK);
                updateCoordinateFramesOf(kit, true);
            }
        }
        if(!kit && !isPresetOnly){
            kit = new LinkKinematicsKit(targetLink);

            auto pinDragIK = bodyItem->pinDragIK(); // create if not created
            if(pinDragIK->numPinnedLinks() > 0){
                pinDragIK->setTargetLink(targetLink, true);
                if(pinDragIK->initialize()){
                    kit->setInversetKinematics(pinDragIK);
                }
            }
            if(!kit->inverseKinematics()){
                auto body = bodyItem->body();
                if(!baseLink){
                    baseLink = body->rootLink();
                }
                kit->setInversetKinematics(JointPath::getCustomPath(body, baseLink, targetLink));
            }
        }

        if(kit){
            linkPairToKinematicsKitMap[key] = kit;
        }
    }

    return kit;        
}


std::shared_ptr<InverseKinematics> LinkKinematicsKitManager::Impl::findPresetIK(Link* targetLink)
{
    std::shared_ptr<InverseKinematics> ik;
    auto body = bodyItem->body();
    const Mapping& setupMap = *body->info()->findMapping("defaultIKsetup");
    if(setupMap.isValid()){
        const Listing& setup = *setupMap.findListing(targetLink->name());
        if(setup.isValid() && !setup.empty()){
            Link* baseLink = body->link(setup[0].toString());
            if(baseLink){
                if(setup.size() == 1){
                    ik = JointPath::getCustomPath(body, baseLink, targetLink);
                } else {
                    auto compositeIK = make_shared<CompositeIK>(body, targetLink);
                    ik = compositeIK;
                    for(int i=0; i < setup.size(); ++i){
                        Link* baseLink = body->link(setup[i].toString());
                        if(baseLink){
                            if(!compositeIK->addBaseLink(baseLink)){
                                ik.reset();
                                break;
                            }
                        }
                    }
                }
            }
        }
    }
    return ik;
}


bool LinkKinematicsKitManager::Impl::updateCoordinateFramesOf(LinkKinematicsKit* kit, bool forceUpdate)
{
    bool updated = false;
    
    auto prevBaseFrames = kit->baseFrames();
    if(forceUpdate || prevBaseFrames){
        CoordinateFrameList* newBaseFrames = nullptr;
        if(baseFrames){
            newBaseFrames = baseFrames;
        } else {
            if(!defaultBaseFrames){
                defaultBaseFrames = new CoordinateFrameList;
                defaultBaseFrames->setFrameTypeEnabled(CoordinateFrame::Local);
            }
            newBaseFrames = defaultBaseFrames;
        }
        if(newBaseFrames != prevBaseFrames){
            kit->setBaseFrames(newBaseFrames);
            updated = true;
        }
    }

    auto prevOffsetFrames = kit->offsetFrames();
    if(forceUpdate || prevOffsetFrames){
        CoordinateFrameList* newOffsetFrames = nullptr;
        if(offsetFrames){
            newOffsetFrames = offsetFrames;
        } else {
            if(!defaultOffsetFrames){
                defaultOffsetFrames = new CoordinateFrameList;
                defaultOffsetFrames->setFrameTypeEnabled(CoordinateFrame::Offset);
            }
            newOffsetFrames = defaultOffsetFrames;
        }
        if(newOffsetFrames != prevOffsetFrames){
            kit->setOffsetFrames(newOffsetFrames);
            updated = true;
        }
    }

    return updated;
}
        

void LinkKinematicsKitManager::Impl::findFrameListItems()
{
    baseFrames.reset();
    offsetFrames.reset();
    auto items = bodyItem->descendantItems<CoordinateFrameListItem>();
    for(auto& item : items){
        auto frameList = item->frameList();
        if(!baseFrames){
            if(frameList->isFrameTypeEnabled(CoordinateFrame::Global) ||
               frameList->isFrameTypeEnabled(CoordinateFrame::Local)){
                baseFrames = frameList;
            }
        }
        if(!offsetFrames){
            if(frameList->isFrameTypeEnabled(CoordinateFrame::Offset)){
                offsetFrames = frameList;
            }
        }
        if(baseFrames && offsetFrames){
            break;
        }
    }
}


void LinkKinematicsKitManager::Impl::onFrameListItemAddedOrUpdated
(CoordinateFrameListItem* frameListItem)
{
    bool isTargetFrameList = false;
    auto upperItem = frameListItem->parentItem();
    while(upperItem){
        if(auto upperBodyItem = dynamic_cast<BodyItem*>(upperItem)){
            if(upperBodyItem == bodyItem){
                isTargetFrameList = true;
            }
            break;
        }
        upperItem = upperItem->parentItem();
    }
    if(isTargetFrameList){
        bool updated = false;
        auto frameList = frameListItem->frameList();
        if(frameList->isFrameTypeEnabled(CoordinateFrame::Global) ||
           frameList->isFrameTypeEnabled(CoordinateFrame::Local)){
            baseFrames = frameList;
            updated = true;
        }
        if(frameList->isFrameTypeEnabled(CoordinateFrame::Offset)){
            offsetFrames = frameList;
            updated = true;
        }
        if(updated){
            for(auto& kv : linkPairToKinematicsKitMap){
                auto& kit = kv.second;
                if(updateCoordinateFramesOf(kit, false)){
                    kit->notifyFrameUpdate();
                }
            }
        }
    }
}


SgNode* LinkKinematicsKitManager::scene()
{
    return impl->positionDragger;
}


void LinkKinematicsKitManager::Impl::setupPositionDragger()
{
    positionDragger = new PositionDragger(PositionDragger::AllAxes, PositionDragger::PositiveOnlyHandle);
    positionDragger->setOverlayMode(true);
    positionDragger->setConstantPixelSizeMode(true, 92.0);
    positionDragger->setDisplayMode(PositionDragger::DisplayNever);

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
        if(auto frameList = frame->ownerFrameList()){
            if(frameList == baseFrames){
                accepted = startBaseFrameEditing(target, frame);
            } else if(frameList == offsetFrames){
                accepted = startOffsetFrameEditing(target, frame);
            }
        }
    }
    return accepted;
}


bool LinkKinematicsKitManager::Impl::startBaseFrameEditing
(AbstractPositionEditTarget* target, CoordinateFrame* frame)
{
    if(auto link = bodySelectionManager->currentLink()){
        if(auto kit = bodyItem->findPresetLinkKinematicsKit(link)){
            if(auto baseLink = kit->baseLink()){
                setFrameEditTarget(target, baseLink);
                return true;
            }
        }
    }
    return false;
}


bool LinkKinematicsKitManager::Impl::startOffsetFrameEditing
(AbstractPositionEditTarget* target, CoordinateFrame* frame)
{
    auto endLink = bodyItem->body()->findUniqueEndLink();
    if(!endLink){
        endLink = bodySelectionManager->currentLink();
    }
    if(endLink){
        setFrameEditTarget(target, endLink);
        return true;
    }
    return false;
}


void LinkKinematicsKitManager::Impl::setFrameEditTarget(AbstractPositionEditTarget* target, Link* link)
{
    frameEditConnections.disconnect();

    frameEditTarget = target;
    frameEditLink = link;

    if(!target){
        positionDragger->setDisplayMode(PositionDragger::DisplayNever);

    } else {
        positionDragger->setDisplayMode(PositionDragger::DisplayAlways);
        positionDragger->setDragEnabled(target->isEditable());

        frameEditConnections.add(
            target->sigPositionChanged().connect(
                [&](const Position& T){ onFrameEditPositionChanged(T); }));

        frameEditConnections.add(
            target->sigPositionEditTargetExpired().connect(
                [=](){ setFrameEditTarget(nullptr, nullptr); }));

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


bool LinkKinematicsKitManager::storeState(Mapping& archive) const
{
    auto body = impl->bodyItem->body();
    archive.setKeyQuoteStyle(DOUBLE_QUOTED);
    for(auto& kv : impl->linkPairToKinematicsKitMap){
        auto& linkIndexPair = kv.first;
        int baseLinkIndex = linkIndexPair.second;

        // This function only stores the states of preset link kinematics objects
        if(baseLinkIndex == PresetBaseLink){
            int linkIndex = linkIndexPair.first;
            if(auto link = body->link(linkIndex)){
                auto& kit = kv.second;
                MappingPtr state = new Mapping;
                if(!kit->storeState(*state)){
                    return false;
                }
                if(!state->empty()){
                    archive.insert(link->name(), state);
                }
            }
        }
    }
    return true;
}


bool LinkKinematicsKitManager::restoreState(const Mapping& archive)
{
    auto body = impl->bodyItem->body();
    for(auto& kv : archive){
        auto& linkName = kv.first;
        if(auto link = body->link(linkName)){
            if(auto kit = findPresetKinematicsKit(link)){
                kit->restoreState(*kv.second->toMapping());
            }
        }
    }
    return true;
}
