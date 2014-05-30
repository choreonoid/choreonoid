/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "GettextUtil.h"
#include <cnoid/ExecutablePath>
#include <cnoid/FileUtil>
#include "gettext.h"

using namespace boost;
using namespace cnoid;

void cnoid::bindGettextDomain(const char* domainname)
{
#if CNOID_ENABLE_GETTEXT
    filesystem::path localePath = filesystem::path(executableTopDirectory()) / "share" / "locale";
    if(filesystem::is_directory(localePath)){
        bindtextdomain(domainname, getPathString(localePath).c_str());
    } else {
        localePath = filesystem::path(shareDirectory()) / "locale";
        if(filesystem::is_directory(localePath)){
            bindtextdomain(domainname, getPathString(localePath).c_str());
        }
    }
#endif
}
