#include "LinkedJointHandler.h"
#include "Body.h"

using namespace cnoid;


bool LinkedJointHandler::limitLinkedJointDisplacementsWithinMovableRanges(Link* masterJoint)
{
    bool updated = false;
    for(auto& joint : body()->allJoints()){
        if(joint != masterJoint){
            double q = joint->q();
            bool isLimitOver = false;
            if(q < joint->q_lower()){
                q = joint->q_lower();
                isLimitOver = true;
            } else if(q > joint->q_upper()){
                q = joint->q_upper();
                isLimitOver = true;
            }
            if(isLimitOver){
                joint->q() = q;
                updateLinkedJointDisplacements(joint);
                updated = true;
            }
        }
    }
    return updated;
}
