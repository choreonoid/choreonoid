#include "KinematicBodyItemSet.h"
#include "BodyItem.h"
#include <cnoid/Body>
#include <cnoid/Link>
#include <stdexcept>

using namespace std;
using namespace cnoid;


KinematicBodyItemSet::KinematicBodyItemSet()
{

}


KinematicBodyItemSet::KinematicBodyItemSet(const KinematicBodyItemSet& org, CloneMap* cloneMap)
    : KinematicBodySet(org, cloneMap)
{

}


Referenced* KinematicBodyItemSet::doClone(CloneMap* cloneMap) const
{
    return new KinematicBodyItemSet(*this, cloneMap);
}


void KinematicBodyItemSet::setBodyPart(int index, BodyKinematicsKit* kinematicsKit)
{
    auto kit = dynamic_cast<BodyItemKinematicsKit*>(kinematicsKit);
    if(!kit){
        throw std::invalid_argument("Type mismatch in the KinematicBodyItemSet::setBodyPart function");
    }
    auto& entry = bodyItemEntryMap[index];
    entry.connections.disconnect();
    auto bodyItem = kit->bodyItem();
    entry.connections.add(
        bodyItem->sigDisconnectedFromRoot().connect(
            [this, index]{
                removeBodyPart(index);
                notifyBodySetChange();
            }));
    entry.connections.add(
        bodyItem->sigModelUpdated().connect(
            [this, index](int flags){ onBodyItemModelUpdate(index, flags); }));
    entry.jointSignature = captureJointSignature(kit);
    KinematicBodySet::setBodyPart(index, kit);
}


void KinematicBodyItemSet::removeBodyPart(int index)
{
    bodyItemEntryMap.erase(index);
    KinematicBodySet::removeBodyPart(index);
}


/*
   The kit holds Link pointers and reads per-joint parameters (jointId,
   jointType, jointRange, jointAxis, ...) live from each Link. Parameter
   changes therefore propagate through the kit on their own and do not
   invalidate it. What does invalidate it is a change in the membership
   of body->joints() -- e.g. clearing a jointId removes the Link from the
   joint list while the kit still points at it. Capturing the pointer
   sequence is sufficient to detect exactly the cases that require a kit
   refresh; other consequences of model updates (UI redraw, collision
   shape refresh, etc.) are handled by other BodyItem::sigModelUpdated()
   subscribers on their own, so this class does not need to chase them.
*/
std::vector<LinkPtr> KinematicBodyItemSet::captureJointSignature(BodyItemKinematicsKit* kit)
{
    std::vector<LinkPtr> signature;
    if(kit){
        if(auto body = kit->body()){
            signature.reserve(body->numJoints());
            for(auto& joint : body->joints()){
                signature.emplace_back(joint);
            }
        }
    }
    return signature;
}


void KinematicBodyItemSet::onBodyItemModelUpdate(int index, int flags)
{
    if(!(flags & (BodyItem::LinkSetUpdate | BodyItem::LinkSpecUpdate))){
        return;
    }
    auto it = bodyItemEntryMap.find(index);
    if(it == bodyItemEntryMap.end()){
        return;
    }
    auto kit = bodyItemPart(index);
    auto bodyItem_ = bodyItem(index);
    if(!kit || !bodyItem_){
        return;
    }
    auto current = captureJointSignature(kit);
    if(current == it->second.jointSignature){
        return;
    }

    bool bodySetChanged = onBodyItemModelUpdated(index, bodyItem_, flags);
    /*
       onBodyItemModelUpdated may have replaced or removed the kit
       via setBodyItemPart / removeBodyPart. Re-fetch and update signature.
    */
    auto entryIt = bodyItemEntryMap.find(index);
    if(entryIt != bodyItemEntryMap.end()){
        if(auto updatedKit = bodyItemPart(index)){
            entryIt->second.jointSignature = captureJointSignature(updatedKit);
        }
    }
    if(bodySetChanged){
        notifyBodySetChange();
    }
}


bool KinematicBodyItemSet::onBodyItemModelUpdated
(int /* index */, BodyItem* /* bodyItem */, int /* flags */)
{
    return false;
}


int KinematicBodyItemSet::indexOf(const BodyItem* item) const
{
    int index = -1;
    if(item){
        int n = maxIndex() + 1;
        for(int i=0; i < n; ++i){
            if(bodyItem(i) == item){
                index = i;
                break;
            }
        }
    }
    return index;
}


bool KinematicBodyItemSet::contains(const BodyItem* bodyItem) const
{
    return indexOf(bodyItem) >= 0;
}
