/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_INVERSE_DYNAMICS_H
#define CNOID_BODY_INVERSE_DYNAMICS_H

#include <cnoid/EigenTypes>
#include "exportdecl.h"

namespace cnoid {

class Link;

/**
   @return force being applied to the root link
*/
CNOID_EXPORT Vector6 calcInverseDynamics(Link* link);

}

#endif
