#ifndef CNOID_BODY_LINKED_JOINT_HANDLER_H
#define CNOID_BODY_LINKED_JOINT_HANDLER_H

#include "BodyHandler.h"
#include "exportdecl.h"

namespace cnoid {

class Link;

class CNOID_EXPORT LinkedJointHandler : public virtual BodyHandler
{
public:
    virtual bool updateLinkedJointDisplacements(Link* masterJoint = nullptr) = 0;

    // Experimental
    virtual bool limitLinkedJointDisplacementsWithinMovableRanges(Link* masterJoint = nullptr);
};

typedef ref_ptr<LinkedJointHandler> LinkedJointHandlerPtr;

}

#endif
