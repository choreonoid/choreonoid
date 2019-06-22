/*! @file
  @author Shin'ichiro Nakaoka
  @note This file must be linked lastly because the constructors of the static class instances
  define in this file must be executed after all the other static instances are constructed.
*/

#include <cnoid/GettextUtil>
#include "CnoidBody.h"
#include "gettext.h"

void cnoid::setCnoidBodyTextDomainCodeset()
{
#ifdef CNOID_ENABLE_GETTEXT
    bind_textdomain_codeset(CNOID_GETTEXT_DOMAIN_NAME, "utf-8");
#endif
}

CNOID_BIND_GETTEXT_DOMAN()
