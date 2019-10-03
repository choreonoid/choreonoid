#ifndef CNOID_UTIL_CLONE_MAPPABLE_REFERENCED_H
#define CNOID_UTIL_CLONE_MAPPABLE_REFERENCED_H

#include "Referenced.h"

namespace cnoid {

class CloneMap;

class CloneMappableReferenced : public Referenced
{
public:
    virtual Referenced* doClone(CloneMap* cloneMap) const = 0;
};

}

#endif
