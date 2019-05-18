#ifndef CNOID_BODY_CUSTOM_JOINT_PATH_HANDLER_H
#define CNOID_BODY_CUSTOM_JOINT_PATH_HANDLER_H

#include "BodyHandler.h"
#include "exportdecl.h"

namespace cnoid {

class Link;
class JointPath;

class CNOID_EXPORT CustomJointPathHandler : public virtual BodyHandler
{
public:
    virtual std::shared_ptr<JointPath> getCustomJointPath(Link* baseLink, Link* endLink) = 0;
};

typedef ref_ptr<CustomJointPathHandler> CustomJointPathHandlerPtr;

}

#endif
