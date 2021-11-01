/**
   \file
   \author Shin'ichiro Nakaoka
   \note This file can only be included from gettext.h defining CNOID_GETTEXT_DOMAIN_NAME
*/

#ifndef CNOID_UTIL_GETTEXT_UTIL_H
#define CNOID_UTIL_GETTEXT_UTIL_H

#include <cnoid/Config>
#include <string>
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

/**
   \param moduleName The target module (library or plugin) name.
   The domain name is based on the module name and is named as "Cnoid" + module name + Choreonoid version suffix.
   \param customSubDirectory If this string is empty, the default message catalog directory is used
   to bind the domain. You can use a special directory to customize messages from an application executable
   by specifying a locale sub directory in the application's share directory.
   \param useUTF8 Set UTF-8 as the encoding of translated messages.
   \return domain name
*/
CNOID_EXPORT std::string bindModuleTextDomain(
    const std::string& moduleName, const std::string& customSubDirectory = "", bool useUTF8 = true);

CNOID_EXPORT void setUTF8ToModuleTextDomain(const std::string& moduleName);

/**
   This function is called from the beginning part of the application main function if necessary.
*/
CNOID_EXPORT void useEnglishMessageCatalogForUnsupportedLocale(const std::string& customLabel = "");

CNOID_EXPORT bool checkCurrentLocaleLanguageSupport();

class ModuleTextDomainBinder
{
public:
    ModuleTextDomainBinder(const std::string& moduleName){
        bindModuleTextDomain(moduleName, "", false);
    }
};

}

/**
   Implement this once in a shared library to bind a gettext domain.
   The "gettext.h" header must be included before using this macro.
*/
#define CNOID_BIND_MODULE_TEXT_DOMAIN(moduleName) \
    namespace { cnoid::ModuleTextDomainBinder cnoidModuleTextDomainBinder(#moduleName); }

#endif
