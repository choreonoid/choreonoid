/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "GettextUtil.h"
#include <cnoid/ExecutablePath>
#include <cnoid/stdx/filesystem>
#include "exportdecl.h"

namespace cnoid {

namespace filesystem = stdx::filesystem;

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
# ifdef CNOID_ENABLE_GETTEXT
const char* getText(const char* domainname, const char* msgid)
{
    return dgettext(domainname, msgid);
}
# else
const char* getText(const char* domainname, const char* msgid)
{
    return msgid;
}
# endif
#endif

void bindGettextDomain(const char* domainname)
{
#ifdef CNOID_ENABLE_GETTEXT
    filesystem::path localePath = executableTopDirPath() / "share" / "locale";
    if(filesystem::is_directory(localePath)){
        bindtextdomain(domainname, localePath.string().c_str());
    } else {
        localePath = shareDirPath() / "locale";
        if(filesystem::is_directory(localePath)){
            bindtextdomain(domainname, localePath.string().c_str());
        }
    }
#endif
}

}
