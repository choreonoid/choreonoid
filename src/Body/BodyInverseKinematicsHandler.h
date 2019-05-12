#ifndef CNOID_BODY_BODY_INVERSE_KINEMATICS_HANDLER_H
#define CNOID_BODY_BODY_INVERSE_KINEMATICS_HANDLER_H

#include "BodyHandler.h"
#include "InverseKinematics.h"

namespace cnoid {

class Link;

class BodyInverseKinematicsHandler : public virtual BodyHandler
{
public:
    virtual std::shared_ptr<InverseKinematics> getInverseKinematics(Link* baseLink, Link* endLink) = 0;
};

typedef ref_ptr<BodyInverseKinematicsHandler> BodyInverseKinematicsHandlerPtr;

}

#endif
