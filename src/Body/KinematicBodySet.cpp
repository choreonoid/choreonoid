#include "KinematicBodySet.h"
#include <cnoid/CloneMap>

using namespace std;
using namespace cnoid;

namespace cnoid {

class KinematicBodySet::Impl
{
public:
    std::unordered_map<GeneralId, ref_ptr<BodyPart>> bodyPartMap;
    GeneralId mainBodyPartId;

    // Function objects are used instead of virtual functions so that the functions can be used in the constructor.
    CreateBodyPartFunc createBodyPart;
    CopyBodyPartFunc copyBodyPart;
    
    Signal<void()> sigFrameSetChange;
    Signal<void(const Isometry3& T_frameCoordinate)> sigPositionError;

    Impl(const CreateBodyPartFunc& createBodyPart, const CopyBodyPartFunc& copyBodyPart);
    void clearBodyPart(const GeneralId& partId);
};

}


KinematicBodySet::KinematicBodySet()
{
    impl = new Impl(
        [](){
            return new BodyPart;
        },
        [this](BodyPart* newBodyPart, BodyPart* orgBodyPart, CloneMap* cloneMap){
            copyBodyPart(newBodyPart, orgBodyPart, cloneMap);
        });
}


KinematicBodySet::KinematicBodySet(CreateBodyPartFunc createBodyPart, CopyBodyPartFunc copyBodyPart)
{
    impl = new Impl(createBodyPart, copyBodyPart);
}


KinematicBodySet::Impl::Impl(const CreateBodyPartFunc& createBodyPart, const CopyBodyPartFunc& copyBodyPart)
    : createBodyPart(createBodyPart),
      copyBodyPart(copyBodyPart)
{

}


KinematicBodySet::KinematicBodySet(const KinematicBodySet& org, CloneMap* cloneMap)
{
    impl = new Impl(org.impl->createBodyPart, org.impl->copyBodyPart);
    
    for(auto& kv : org.impl->bodyPartMap){
        auto& id = kv.first;
        auto part = kv.second;
        auto kinematicsKit = part->kinematicsKit;
        if(cloneMap){
            kinematicsKit = cloneMap->getClone(kinematicsKit);
        }
        setBodyPart(id, kinematicsKit);
    }

    impl->mainBodyPartId = org.impl->mainBodyPartId;
}


Referenced* KinematicBodySet::doClone(CloneMap* cloneMap) const
{
    return new KinematicBodySet(*this, cloneMap);
}


KinematicBodySet::~KinematicBodySet()
{
    delete impl;
}


void KinematicBodySet::copyBodyPart(BodyPart* newBodyPart, BodyPart* orgBodyPart, CloneMap* cloneMap)
{
    auto kinematicsKit = orgBodyPart->kinematicsKit;
    if(cloneMap){
        kinematicsKit = cloneMap->getClone(kinematicsKit);
    }
    initializeBodyPart(newBodyPart, kinematicsKit);
}


void KinematicBodySet::initializeBodyPart(BodyPart* bodyPart, LinkKinematicsKit* kinematicsKit)
{
    if(kinematicsKit != bodyPart->kinematicsKit){
        bodyPart->kinematicsKit = kinematicsKit;
        bodyPart->connections.disconnect();
        bodyPart->connections.add(
            kinematicsKit->sigFrameSetChange().connect(
                [this](){ impl->sigFrameSetChange(); }));
        bodyPart->connections.add(
            kinematicsKit->sigPositionError().connect(
                [this](const Isometry3& T_frameCoordinate){ impl->sigPositionError(T_frameCoordinate); }));
    }
}


KinematicBodySet::BodyPart* KinematicBodySet::findBodyPart(const GeneralId& partId)
{
    auto it = impl->bodyPartMap.find(partId);
    if(it != impl->bodyPartMap.end()){
        return it->second;
    }
    return nullptr;
}


KinematicBodySet::BodyPart* KinematicBodySet::findOrCreateBodyPart(const GeneralId& partId)
{
    auto& part = impl->bodyPartMap[partId];
    if(!part){
        part = impl->createBodyPart();
    }
    return part;
}


void KinematicBodySet::setBodyPart(const GeneralId& partId, LinkKinematicsKit* kinematicsKit)
{
    if(partId.isValid()){
        if(kinematicsKit){
            auto bodyPart = findOrCreateBodyPart(partId);
            initializeBodyPart(bodyPart, kinematicsKit);
        } else {
            clearBodyPart(partId);
        }
    }
}


void KinematicBodySet::clear()
{
    impl->bodyPartMap.clear();
    impl->mainBodyPartId.reset();
}


void KinematicBodySet::clearBodyPart(const GeneralId& partId)
{
    impl->clearBodyPart(partId);
}


void KinematicBodySet::Impl::clearBodyPart(const GeneralId& partId)
{
    bodyPartMap.erase(partId);
    if(mainBodyPartId == partId){
        mainBodyPartId.reset();
    }
}


int KinematicBodySet::numKinematicBodyParts() const
{
    return impl->bodyPartMap.size();
}


LinkKinematicsKit* KinematicBodySet::kinematicsKit(const GeneralId& partId)
{
    if(auto bodyPart = findBodyPart(partId)){
        return bodyPart->kinematicsKit;
    }
    return nullptr;
}


void KinematicBodySet::setMainBodyPartId(const GeneralId& partId)
{
    impl->mainBodyPartId = partId;
}


const GeneralId& KinematicBodySet::mainBodyPartId() const
{
    return impl->mainBodyPartId;
}


LinkKinematicsKit* KinematicBodySet::mainKinematicsKit()
{
    return impl->mainBodyPartId.isValid() ? kinematicsKit(impl->mainBodyPartId) : nullptr;
}


SignalProxy<void()> KinematicBodySet::sigFrameSetChange()
{
    return impl->sigFrameSetChange;
}


SignalProxy<void(const Isometry3& T_frameCoordinate)> KinematicBodySet::sigPositionError()
{
    return impl->sigPositionError;
}
