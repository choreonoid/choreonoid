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
        bodyItemConnectionMap[index] =
            kit->bodyItem()->sigDisconnectedFromRoot().connect(
                [this, index]{
                    removeBodyPart(index);
                    notifyBodySetChange();
                });
        KinematicBodySet::setBodyPart(index, kit);
        
    } else {
        throw std::invalid_argument("Type mismatch in the KinematicBodyItemSet::setBodyPart function");
    }
}


void KinematicBodyItemSet::removeBodyPart(int index)
{
    bodyItemConnectionMap.erase(index);
    KinematicBodySet::removeBodyPart(index);
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
