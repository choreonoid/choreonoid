#ifndef CNOID_BASE_APP_UTIL_H
#define CNOID_BASE_APP_UTIL_H

#include <cnoid/Signal>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT AppUtil
{
public:
    static SignalProxy<void()> sigAboutToQuit();
    static void updateGui();
};

[[deprecated("Use AppUtil::sigAboutToQuit")]]
CNOID_EXPORT SignalProxy<void()> sigAboutToQuit();
[[deprecated("Use AppUtil::updateGui")]]
CNOID_EXPORT void updateGui();

}

#endif
