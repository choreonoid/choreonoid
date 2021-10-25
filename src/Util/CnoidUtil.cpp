/*! @file
  @note This file must be linked lastly because the constructors of the static class instances
  define in this file must be executed after all the other static instances are constructed.
*/

#include "GettextUtil.h"

CNOID_BIND_MODULE_TEXT_DOMAIN(Util)
