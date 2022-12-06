#ifndef CNOID_UTIL_COLLISION_H
#define CNOID_UTIL_COLLISION_H

#include "EigenTypes.h"

namespace cnoid {

struct Collision {
    Vector3 point;
    Vector3 normal;
    double depth;
    union {
        struct {
            unsigned int id1;
            unsigned int id2;
        };
        unsigned long long int id;
    };
};

}

#endif
