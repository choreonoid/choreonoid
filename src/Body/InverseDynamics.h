/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_INVERSE_DYNAMICS_H_INCLUDED
#define CNOID_BODY_INVERSE_DYNAMICS_H_INCLUDED

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
