#ifndef CNOID_BODY_LINKED_JOINT_HANDLER_H
#define CNOID_BODY_LINKED_JOINT_HANDLER_H

#include "BodyHandler.h"
#include "exportdecl.h"

namespace cnoid {

class Link;

class CNOID_EXPORT LinkedJointHandler : public virtual BodyHandler
{
public:
    static LinkedJointHandler* findOrCreateLinkedJointHandler(Body* body);

    /**
       \return true when the displacement of any joint is modified by a movement linked to another joint.
       Note that the return value is false when only the displacement of the master joint is modified to
       be the argument value,
    */
    virtual bool updateLinkedJointDisplacements(Link* masterJoint, double masterJointDisplacement) = 0;
    
    bool updateLinkedJointDisplacements(){
        return updateLinkedJointDisplacements(nullptr, 0.0);
    }
    virtual bool limitLinkedJointDisplacementsWithinMovableRanges(Link* masterJoint = nullptr);
};

typedef ref_ptr<LinkedJointHandler> LinkedJointHandlerPtr;

}

#endif
