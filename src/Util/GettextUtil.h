/**
   \file
   \author Shin'ichiro Nakaoka
   \note This file can only be included from gettext.h defining CNOID_GETTEXT_DOMAIN_NAME
*/

#ifndef CNOID_UTIL_GETTEXT_UTIL_H
#define CNOID_UTIL_GETTEXT_UTIL_H

#include <cnoid/Config>
#include "exportdecl.h"

#ifdef CNOID_ENABLE_GETTEXT
# include <libintl.h>

# ifdef CNOID_USE_GETTEXT_WRAPPER
#  define _(text) cnoid::getText(CNOID_GETTEXT_DOMAIN_NAME, text)
# else
#  define _(text) dgettext(CNOID_GETTEXT_DOMAIN_NAME, text)
# endif

#else
namespace cnoid {
inline const char* bindtextdomain(const char* domainname, const char* dirname) {
    return dirname;
}
inline const char* dgettext(const char* domainname, const char* msgid){
    return msgid;
}
}
#define _(string) string
#endif

#define N_(string) string

namespace cnoid {

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
CNOID_EXPORT const char* getText(const char* domainname, const char* msgid);
#else
inline const char* getText(const char* domainname, const char* msgid) {
    return dgettext(domainname, msgid);
}
#endif

CNOID_EXPORT void bindGettextDomain(const char* domainname);

class GettextDomainBinder
{
public:
    GettextDomainBinder(const char* domainname){
        bindGettextDomain(domainname);
    }
};

}

/**
   Implement this once in a shared library to bind a gettext domain.
   The "gettext.h" header must be included before using this macro.
*/
#define CNOID_BIND_GETTEXT_DOMAN() \
    namespace { cnoid::GettextDomainBinder cnoidGettextDomainBinder(CNOID_GETTEXT_DOMAIN_NAME); }

#endif
