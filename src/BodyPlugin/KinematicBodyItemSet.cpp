#include "KinematicBodyItemSet.h"

using namespace std;
using namespace cnoid;


KinematicBodyItemSet::BodyItemPart::BodyItemPart()
{

}


KinematicBodyItemSet::BodyItemPart::BodyItemPart(const BodyItemPart& org, CloneMap* cloneMap)
    : KinematicBodyPart(org, cloneMap),
      bodyItem_(org.bodyItem_)
{

}


Referenced* KinematicBodyItemSet::BodyItemPart::doClone(CloneMap* cloneMap) const
{
    return new BodyItemPart(*this, cloneMap);
}


KinematicBodyItemSet::KinematicBodyItemSet()
    : KinematicBodySet([](){ return new BodyItemPart; })
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


void KinematicBodyItemSet::setBodyItemPart(int index, BodyItem* bodyItem, std::shared_ptr<JointTraverse> traverse)
{
    auto bodyPart = static_cast<BodyItemPart*>(findOrCreateBodyPart(index));
    bodyPart->setJointTraverse(traverse);
    bodyPart->bodyItem_ = bodyItem;
}


void KinematicBodyItemSet::setBodyItemPart(int index, BodyItem* bodyItem, LinkKinematicsKit* kit)
{
    auto bodyPart = static_cast<BodyItemPart*>(findOrCreateBodyPart(index));
    bodyPart->setLinkKinematicsKit(kit);
    bodyPart->bodyItem_ = bodyItem;
}
