#ifndef CNOID_BODY_KINEMATIC_BODY_SET_H
#define CNOID_BODY_KINEMATIC_BODY_SET_H

#include "BodyKinematicsKit.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT KinematicBodySet : public ClonableReferenced
{
public:
    KinematicBodySet();
    KinematicBodySet(const KinematicBodySet& org, CloneMap* cloneMap);

    KinematicBodySet* clone(CloneMap* cloneMap){
        return static_cast<KinematicBodySet*>(doClone(cloneMap));
    }

    virtual void setBodyPart(int index, BodyKinematicsKit* kinematicsKit);
    void clearBodyPart(int index);
    void clear();
    void setMainBodyPartIndex(int index) { mainBodyPartIndex_ = index; }
    int mainBodyPartIndex() const { return mainBodyPartIndex_; }

    bool empty() const { return bodyParts_.empty(); }
    int maxIndex() const { return bodyParts_.size() - 1; }

    bool hasMultiBodyParts() const {
        return numValidBodyParts_ >= 2 && mainBodyPartIndex_ >= 0;
    }
    
    std::vector<int> validBodyPartIndices() const;
    
    BodyKinematicsKit* bodyPart(int index) {
        return index < static_cast<int>(bodyParts_.size()) ? bodyParts_[index] : nullptr;
    }
    const BodyKinematicsKit* bodyPart(int index) const {
        return const_cast<KinematicBodySet*>(this)->bodyPart(index);
    }
    BodyKinematicsKit* mainBodyPart() {
        return (mainBodyPartIndex_ >= 0) ? bodyParts_[mainBodyPartIndex_] : nullptr;
    }
    const BodyKinematicsKit* mainBodyPart() const {
        return const_cast<KinematicBodySet*>(this)->mainBodyPart();
    }

    int indexOf(const BodyKinematicsKit* kit) const;

    SignalProxy<void()> sigBodySetChanged() { return sigBodySetChanged_; }
    void notifyBodySetChange() { sigBodySetChanged_(); }

protected:
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    std::vector<BodyKinematicsKitPtr> bodyParts_;
    int numValidBodyParts_;
    int mainBodyPartIndex_;
    Signal<void()> sigBodySetChanged_;
};

typedef ref_ptr<KinematicBodySet> KinematicBodySetPtr;
    
}

#endif
