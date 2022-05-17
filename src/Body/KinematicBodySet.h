#ifndef CNOID_BODY_KINEMATIC_BODY_SET_H
#define CNOID_BODY_KINEMATIC_BODY_SET_H

#include "KinematicBodyPart.h"
#include <functional>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT KinematicBodySet : public ClonableReferenced
{
public:
    KinematicBodySet();
    KinematicBodySet(const KinematicBodySet& org, CloneMap* cloneMap);

    void setBodyPart(int index, std::shared_ptr<JointTraverse> jointTraverse);
    void setBodyPart(int index, LinkKinematicsKit* kit);
    void clearBodyPart(int index);
    void clear();
    void setMainBodyPartIndex(int index) { mainBodyPartIndex_ = index; }
    int mainBodyPartIndex() const { return mainBodyPartIndex_; }

    bool empty() const { return bodyParts_.empty(); }
    int maxIndex() const { return bodyParts_.size() - 1; }
    std::vector<int> validBodyPartIndices() const;
    
    KinematicBodyPart* bodyPart(int index) { return bodyParts_[index]; }
    const KinematicBodyPart* bodyPart(int index) const { return bodyParts_[index]; }
    KinematicBodyPart* mainBodyPart() {
        return (mainBodyPartIndex_ >= 0) ? bodyParts_[mainBodyPartIndex_] : nullptr;
    }
    const KinematicBodyPart* mainBodyPart() const {
        return const_cast<KinematicBodySet*>(this)->mainBodyPart();
    }

    SignalProxy<void()> sigUpdated() { return sigUpdated_; }
    void notifyUpdate() { sigUpdated_(); }
    
protected:
    typedef std::function<KinematicBodyPart*()> CreateBodyPartFunc;
    KinematicBodySet(CreateBodyPartFunc createBodyPart);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;
    KinematicBodyPart* findOrCreateBodyPart(int index);

private:
    std::vector<KinematicBodyPartPtr> bodyParts_;
    int mainBodyPartIndex_;

    // Function objects are used instead of virtual functions so that the functions can be used in the constructor.
    CreateBodyPartFunc createBodyPartFunc;

    Signal<void()> sigUpdated_;
};

typedef ref_ptr<KinematicBodySet> KinematicBodySetPtr;
    
}

#endif
