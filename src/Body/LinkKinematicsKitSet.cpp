#include "LinkKinematicsKitSet.h"
#include <cnoid/CloneMap>

using namespace std;
using namespace cnoid;


LinkKinematicsKitSet::LinkKinematicsKitSet()
{

}


LinkKinematicsKitSet::LinkKinematicsKitSet(const LinkKinematicsKitSet& org, CloneMap* cloneMap)
{
    for(auto& kv : org.kitMap){
        auto& id = kv.first;
        auto& info = kv.second;
        auto kit = info.kinematicsKit;
        if(cloneMap){
            kit = cloneMap->getClone(kit);
        }
        setKinematicsKit(id, kit);
    }

    mainPartId_ = org.mainPartId_;
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
    kitMap.clear();
}


bool LinkKinematicsKitSet::setKinematicsKit(const GeneralId& id, LinkKinematicsKit* kit)
{
    if(!id.isValid()){
        return false;
    }

    if(!kit){
        kitMap.erase(id);
        if(mainPartId_ == id){
            mainPartId_.reset();
        }
    } else {
        auto& info = kitMap[id];
        if(info.kinematicsKit){
            info.connections.disconnect();
        }
        info.kinematicsKit = kit;
        info.connections.add(
            kit->sigFrameSetChange().connect(
                [this](){ sigFrameSetChange_(); }));
        info.connections.add(
            kit->sigPositionError().connect(
                [this](const Isometry3& T_frameCoordinate){ sigPositionError_(T_frameCoordinate); }));
    }
    
    return true;
}


LinkKinematicsKit* LinkKinematicsKitSet::kinematicsKit(const GeneralId& partId)
{
    LinkKinematicsKit* kit;
    auto it = kitMap.find(partId);
    if(it == kitMap.end()){
        kit = nullptr;
    } else {
        kit = it->second.kinematicsKit;
    }
    return kit;
}


LinkKinematicsKit* LinkKinematicsKitSet::mainKinematicsKit()
{
    return mainPartId_.isValid() ? kinematicsKit(mainPartId_) : nullptr;
}
