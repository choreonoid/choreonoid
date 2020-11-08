#ifndef CNOID_UTIL_CLONABLE_REFERENCED_H
#define CNOID_UTIL_CLONABLE_REFERENCED_H

#include "Referenced.h"

namespace cnoid {

class CloneMap;

class ClonableReferenced : public Referenced
{
public:
    virtual Referenced* doClone(CloneMap* cloneMap) const = 0;
};

}

#endif
