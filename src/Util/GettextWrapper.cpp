/**
   @author Shin'ichiro Nakaoka
*/

#include "gettext.h"
#include "exportdecl.h"

#if CNOID_ENABLE_GETTEXT

#include <libintl.h>

namespace cnoid {
CNOID_EXPORT const char* getText(const char* domainname, const char* msgid)
{
    return dgettext(domainname, msgid);
}
}

#else

namespace cnoid {
CNOID_EXPORT const char* getText(const char* domainname, const char* msgid)
{
    return msgid;
}
}

#endif
