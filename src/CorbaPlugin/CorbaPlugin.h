/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_CORBAPLUGIN_CORBAPLUGIN_H_INCLUDED
#define CNOID_CORBAPLUGIN_CORBAPLUGIN_H_INCLUDED

#include <cnoid/CorbaUtil>
#include "exportdecl.h"

namespace cnoid {
CNOID_EXPORT void checkOrInvokeCorbaNameServer();
CNOID_EXPORT bool takeOverCorbaPluginInitialization(CORBA::ORB_ptr orb);
}

#endif
