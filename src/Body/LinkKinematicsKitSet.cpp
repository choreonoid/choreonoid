#include "LinkKinematicsKitSet.h"
#include <cnoid/CloneMap>

using namespace std;
using namespace cnoid;


LinkKinematicsKitSet::LinkKinematicsKitSet()
{
    mainKinematicsKitIndex_ = -1;
}


LinkKinematicsKitSet::LinkKinematicsKitSet(const LinkKinematicsKitSet& org, CloneMap* cloneMap)
{
    for(auto element : org.elements){
        auto kit = element.kinematicsKit;
        if(cloneMap){
            kit = cloneMap->getClone(kit);
        }
        addKinematicsKit(kit);
    }

    mainKinematicsKitIndex_ = org.mainKinematicsKitIndex_;
}


Referenced* LinkKinematicsKitSet::doClone(CloneMap* cloneMap) const
{
    return new LinkKinematicsKitSet(*this, cloneMap);
}


LinkKinematicsKitSet::~LinkKinematicsKitSet()
{
    clearKinematicsKits();
}


void LinkKinematicsKitSet::clearKinematicsKits()
{
    for(auto& element : elements){
        element.connections.disconnect();
    }
    elements.clear();
}


void LinkKinematicsKitSet::addKinematicsKit(LinkKinematicsKit* kit)
{
    elements.emplace_back();
    auto& element = elements.back();

    element.kinematicsKit = kit;

    element.connections.add(
        kit->sigFrameSetChange().connect(
            [this](){ sigFrameSetChange_(); }));
    element.connections.add(
        kit->sigPositionError().connect(
            [this](const Isometry3& T_frameCoordinate){ sigPositionError_(T_frameCoordinate); }));
}
