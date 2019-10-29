#ifndef CNOID_UTIL_CLONEABLE_REFERENCED_H
#define CNOID_UTIL_CLONEABLE_REFERENCED_H

#include "Referenced.h"

namespace cnoid {

class CloneMap;

class CloneableReferenced : public Referenced
{
public:
    virtual Referenced* doClone(CloneMap* cloneMap) const = 0;
};

}

#endif
