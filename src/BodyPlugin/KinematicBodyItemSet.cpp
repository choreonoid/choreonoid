#include "KinematicBodyItemSet.h"
#include "BodyItem.h"

using namespace std;
using namespace cnoid;

namespace {

class BodyPartEx : public KinematicBodySet::BodyPart
{
public:
    BodyItem* bodyItem;
    ScopedConnectionSet connections;
    BodyPartEx() : bodyItem(nullptr) { }
};

}


KinematicBodyItemSet::KinematicBodyItemSet()
    : KinematicBodySet(
        [](){
            return new BodyPartEx;
        },
        [this](BodyPart* newBodyPart, BodyPart* orgBodyPart, CloneMap* cloneMap){
            copyBodyPart(newBodyPart, orgBodyPart, cloneMap);
        })
{

}


KinematicBodyItemSet::KinematicBodyItemSet(const KinematicBodyItemSet& org, CloneMap* cloneMap)
    : KinematicBodySet(org, cloneMap)
{

}


void KinematicBodyItemSet::copyBodyPart(BodyPart* newBodyPart, BodyPart* orgBodyPart, CloneMap* cloneMap)
{
    KinematicBodySet::copyBodyPart(newBodyPart, orgBodyPart, cloneMap);

    initializeBodyPart(newBodyPart, static_cast<BodyPartEx*>(orgBodyPart)->bodyItem);
}


Referenced* KinematicBodyItemSet::doClone(CloneMap* cloneMap) const
{
    return new KinematicBodyItemSet(*this, cloneMap);
}


void KinematicBodyItemSet::setBodyItem(const GeneralId& partId, BodyItem* bodyItem)
{
    if(bodyItem){
        if(auto bodyPart = findOrCreateBodyPart(partId)){
            initializeBodyPart(bodyPart, bodyItem);
        }
    } else {
        clearBodyPart(partId);
    }
}


void KinematicBodyItemSet::initializeBodyPart(BodyPart* bodyPart, BodyItem* bodyItem)
{
    auto bodyPartEx = static_cast<BodyPartEx*>(bodyPart);
    if(bodyItem && bodyItem != bodyPartEx->bodyItem){
        bodyPartEx->bodyItem = bodyItem;
        auto kinematicsKit = bodyItem->findPresetLinkKinematicsKit();
        KinematicBodySet::initializeBodyPart(bodyPart, kinematicsKit);
    }
}


BodyItem* KinematicBodyItemSet::bodyItem(const GeneralId& partId)
{
    if(auto bodyPart = dynamic_cast<BodyPartEx*>(findBodyPart(partId))){
        return bodyPart->bodyItem;
    }
    return nullptr;
}
