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
#include <cnoid/ValueTree>
#include <map>

using namespace std;
using namespace cnoid;

namespace {

enum BaseLinkId {
    PresetBaseLink = -1,
    UnspecifiedBaseLinkForPinDragIK = -2
};

}
    
namespace cnoid {

class LinkKinematicsKitManager::Impl
{
public:
    BodyItem* bodyItem;
    Body* body;

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

    Impl(BodyItem* bodyItem);
    void onBodyItemPositionChanged();
    LinkKinematicsKit* findKinematicsKit(Link* targetLink, bool isPresetOnly);
    std::shared_ptr<InverseKinematics> findPresetIK(Link* targetLink);
    bool updateCoordinateFramesOf(LinkKinematicsKit* kit, bool forceUpdate);
    void findFrameListItems();
    void onFrameListAssociation(CoordinateFrameListItem* frameListItem, bool on);
};

}


LinkKinematicsKitManager::LinkKinematicsKitManager(BodyItem* bodyItem)
{
    impl = new Impl(bodyItem);
}


LinkKinematicsKitManager::Impl::Impl(BodyItem* bodyItem)
    : bodyItem(bodyItem),
      body(bodyItem->body())
{
    bodySelectionManager = BodySelectionManager::instance();

    bodyItemConnection =
        bodyItem->sigTreePositionChanged().connect(
            [&](){ onBodyItemPositionChanged(); });

    onBodyItemPositionChanged();
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
                CoordinateFrameListItem::sigListAssociationWith(bodyItem).connect(
                    [&](CoordinateFrameListItem* frameListItem, bool on){
                        onFrameListAssociation(frameListItem, on);
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
    if(!targetLink){
        if(!isPresetOnly){
            return nullptr;
        } else {
            targetLink = body->findUniqueEndLink();
            if(!targetLink){
                return nullptr;
            }
        }
    }
    bool isPresetMode = false;
    if(!isPresetOnly && KinematicsBar::instance()->mode() == KinematicsBar::PresetKinematics){
        isPresetMode = true;
    }

    Link* baseLink = nullptr;
    int baseLinkIndex;

    if(isPresetOnly || isPresetMode){
        baseLinkIndex = PresetBaseLink;
    } else {
        auto pinDragIK = bodyItem->checkPinDragIK();
        if(pinDragIK && pinDragIK->numPinnedLinks() > 0){
            baseLinkIndex = UnspecifiedBaseLinkForPinDragIK;
        } else {
            baseLink = bodyItem->currentBaseLink();
            if(!baseLink){
                baseLink = body->rootLink();
            }
            baseLinkIndex = baseLink->index();
        }
    }

    auto key = make_pair(targetLink->index(), baseLinkIndex);

    LinkKinematicsKit* kit = nullptr;
    auto iter = linkPairToKinematicsKitMap.find(key);
    if(iter != linkPairToKinematicsKitMap.end()){
        kit = iter->second;
    }

    if(!kit){
        bool needToRegistration = false;
        
        if(isPresetOnly || isPresetMode){
            auto presetIK = findPresetIK(targetLink);
            if(presetIK){
                kit = new LinkKinematicsKit(targetLink);
                kit->setInverseKinematics(presetIK);
                needToRegistration = true;
            }
        }
        if(!kit && !isPresetOnly){
            // Special case
            if(baseLinkIndex == PresetBaseLink && targetLink->isFixedToRoot()){
                baseLink = targetLink;
                baseLinkIndex = targetLink->index();
                key.second = baseLinkIndex;
                auto iter = linkPairToKinematicsKitMap.find(key);
                if(iter != linkPairToKinematicsKitMap.end()){
                    kit = iter->second;
                }
            }
        }
        if(!kit && (baseLinkIndex != PresetBaseLink)){
            kit = new LinkKinematicsKit(targetLink);
            if(baseLinkIndex != UnspecifiedBaseLinkForPinDragIK){
                kit->setInverseKinematics(JointPath::getCustomPath(body, baseLink, targetLink));
            }
            needToRegistration = true;
        }
        if(needToRegistration && kit){
            linkPairToKinematicsKitMap[key] = kit;
            updateCoordinateFramesOf(kit, true);
        }
    }

    if(baseLinkIndex == UnspecifiedBaseLinkForPinDragIK){
        auto pinDragIK = bodyItem->checkPinDragIK();
        pinDragIK->setTargetLink(targetLink, true);
        if(pinDragIK->initialize()){
            kit->setInverseKinematics(pinDragIK);
        }
    }
    
    return kit;
}


std::shared_ptr<InverseKinematics> LinkKinematicsKitManager::Impl::findPresetIK(Link* targetLink)
{
    std::shared_ptr<InverseKinematics> ik;
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
    } else {
        // If the link is an end link that has more than five degrees of freedom,
        // the path between the link and the root link is treated as an IK link
        // when there is no IK link specification
        if(!targetLink->child()){
            int dof = 0;
            auto link = targetLink->parent();
            while(link){
                if(++dof >= 6){
                    ik = JointPath::getCustomPath(body, body->rootLink(), targetLink);
                    break;
                }
                link = link->parent();
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
                defaultBaseFrames->setFrameType(CoordinateFrameList::Base);
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
                defaultOffsetFrames->setFrameType(CoordinateFrameList::Offset);
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
        if(!baseFrames && frameList->isForBaseFrames()){
            baseFrames = frameList;
        }
        if(!offsetFrames && frameList->isForOffsetFrames()){
            offsetFrames = frameList;
        }
        if(baseFrames && offsetFrames){
            break;
        }
    }
}


/**
   \todo Currently only one frame list item can be associated with a body item for each frame type
   at the same time, but multiple frame lists should be associated with the item as a composite list.
*/
void LinkKinematicsKitManager::Impl::onFrameListAssociation(CoordinateFrameListItem* frameListItem, bool on)
{
    bool updated = false;
    
    auto frameList = frameListItem->frameList();
    if(frameList->isForBaseFrames()){
        if(on){
            if(!baseFrames){ // Keep the existing base frames by this condition
                baseFrames = frameList;
                updated = true;
            }
        } else {
            if(frameList == baseFrames){
                baseFrames.reset();
                updated = true;
            }
        }
    } else if(frameList->isForOffsetFrames()){
        if(on){
            if(!offsetFrames){ // Keep the existing base frames by this condition
                offsetFrames = frameList;
                updated = true;
            }
        } else {
            if(frameList == offsetFrames){
                offsetFrames.reset();
                updated = true;
            }
        }
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


bool LinkKinematicsKitManager::storeState(Mapping& archive) const
{
    archive.setKeyQuoteStyle(DOUBLE_QUOTED);
    for(auto& kv : impl->linkPairToKinematicsKitMap){
        auto& linkIndexPair = kv.first;
        int baseLinkIndex = linkIndexPair.second;

        // This function only stores the states of preset link kinematics objects
        if(baseLinkIndex == PresetBaseLink){
            int linkIndex = linkIndexPair.first;
            if(auto link = impl->body->link(linkIndex)){
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
    for(auto& kv : archive){
        auto& linkName = kv.first;
        if(auto link = impl->body->link(linkName)){
            if(auto kit = findPresetKinematicsKit(link)){
                kit->restoreState(*kv.second->toMapping());
            }
        }
    }
    return true;
}
