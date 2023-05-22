#include "LinkedJointHandler.h"
#include "Body.h"

using namespace cnoid;

namespace {

class NonLinkedJointHandler : public LinkedJointHandler
{
public:
    virtual BodyHandler* clone() override;
    virtual bool updateLinkedJointDisplacements(Link* masterJoint, double masterJointDisplacement) override;
};

}

    
BodyHandler* NonLinkedJointHandler::clone()
{
    return new NonLinkedJointHandler;
}


LinkedJointHandler* LinkedJointHandler::findOrCreateLinkedJointHandler(Body* body)
{
    auto handler = body->findHandler<LinkedJointHandler>();
    if(!handler){
        handler = new NonLinkedJointHandler;
    }
    return handler;
}


bool NonLinkedJointHandler::updateLinkedJointDisplacements(Link* masterJoint, double masterJointDisplacement)
{
    if(masterJoint){
        masterJoint->q() = masterJointDisplacement;
    }
    return false;
}


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
                updateLinkedJointDisplacements(joint, q);
                updated = true;
            }
        }
    }
    return updated;
}
