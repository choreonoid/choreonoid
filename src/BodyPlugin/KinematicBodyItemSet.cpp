#include "KinematicBodyItemSet.h"

using namespace std;
using namespace cnoid;


KinematicBodyItemSet::KinematicBodyItemSet()
    : KinematicBodySet(
        [](){
            return new BodyItemPart;
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

    auto newBodyItemPart = static_cast<BodyItemPart*>(newBodyPart);
    auto orgBodyItemPart = static_cast<BodyItemPart*>(orgBodyPart);
    newBodyItemPart->bodyItemConnections.disconnect();
    newBodyItemPart->bodyItem_ = orgBodyItemPart->bodyItem_;
}


Referenced* KinematicBodyItemSet::doClone(CloneMap* cloneMap) const
{
    return new KinematicBodyItemSet(*this, cloneMap);
}


void KinematicBodyItemSet::setBodyItemPart
(int index, BodyItem* bodyItem, LinkKinematicsKit* linkKinematicsKit)
{
    if(bodyItem){
        if(auto bodyPart = static_cast<BodyItemPart*>(findOrCreateBodyPart(index))){
            initializeBodyPart(bodyPart, linkKinematicsKit);
            bodyPart->bodyItem_ = bodyItem;
        }
    } else {
        clearBodyPart(index);
    }
}


void KinematicBodyItemSet::setBodyItemPart
(int index, BodyItem* bodyItem, std::shared_ptr<JointTraverse> jointTraverse)
{
    if(bodyItem){
        if(auto bodyPart = static_cast<BodyItemPart*>(findOrCreateBodyPart(index))){
            initializeBodyPart(bodyPart, jointTraverse);
            bodyPart->bodyItem_ = bodyItem;
        }
    } else {
        clearBodyPart(index);
    }
}
