/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_GETTEXT_UTIL_H
#define CNOID_UTIL_GETTEXT_UTIL_H

#include "exportdecl.h"

namespace cnoid {

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
#define CNOID_BIND_GETTEXT_DOMAN()                                      \
    namespace { cnoid::GettextDomainBinder cnoidGettextDomainBinder(CNOID_GETTEXT_DOMAIN_NAME); }

#endif
