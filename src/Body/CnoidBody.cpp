/*! @file
  @author Shin'ichiro Nakaoka
  @note This file must be linked lastly because the constructors of the static class instances
  define in this file must be executed after all the other static instances are constructed.
*/

#include <cnoid/GettextUtil>
#include "gettext.h"

CNOID_BIND_GETTEXT_DOMAN()
