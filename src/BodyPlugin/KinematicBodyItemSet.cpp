#include "KinematicBodyItemSet.h"
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
    if(auto kit = dynamic_cast<BodyItemKinematicsKit*>(kinematicsKit)){
        KinematicBodySet::setBodyPart(index, kit);
    } else {
        throw std::invalid_argument("Type mismatch in the KinematicBodyItemSet::setBodyPart function");
    }
}
