#ifndef CNOID_BODY_BODY_CLONE_MAP_H
#define CNOID_BODY_BODY_CLONE_MAP_H

#include <cnoid/CloneMap>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT BodyCloneMap : public CloneMap
{
public:
    BodyCloneMap();
    BodyCloneMap(const BodyCloneMap& org) = delete;
};

}

#endif
