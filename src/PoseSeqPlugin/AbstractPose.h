#ifndef CNOID_POSE_SEQ_PLUGIN_ABSTRACT_POSE_H
#define CNOID_POSE_SEQ_PLUGIN_ABSTRACT_POSE_H

#include <cnoid/ClonableReferenced>
#include "exportdecl.h"

namespace cnoid {

class Body;
class Mapping;
    
class CNOID_EXPORT AbstractPose : public ClonableReferenced
{
public:
    AbstractPose();
    AbstractPose(const AbstractPose& org);
    virtual ~AbstractPose();

    AbstractPose* clone(CloneMap* cloneMap = nullptr) const { return static_cast<AbstractPose*>(doClone(cloneMap)); }

    virtual bool hasSameParts(AbstractPose* pose) const;
    virtual bool restore(const Mapping& archive, const Body* body) = 0;
    virtual void store(Mapping& archive, const Body* body) const = 0;

private:
    friend class PoseSeq;
    friend class SequentialPose;
};

typedef ref_ptr<AbstractPose> AbstractPosePtr;

}

#endif
