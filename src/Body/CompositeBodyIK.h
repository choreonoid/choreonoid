#ifndef CNOID_BODY_COMPOSITE_BODY_IK_H
#define CNOID_BODY_COMPOSITE_BODY_IK_H

#include <memory>

namespace cnoid {

class CompositeBodyIK : public InverseKinematics
{
public:
    virtual std::shared_ptr<InverseKinematics> getParentBodyIK() = 0;
};

}

#endif

