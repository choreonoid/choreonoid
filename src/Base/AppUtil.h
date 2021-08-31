/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_APP_UTIL_H
#define CNOID_BASE_APP_UTIL_H

#include <cnoid/Signal>
#include "exportdecl.h"

namespace cnoid {

[[deprecated("Use App::sigAboutToQuit")]]
CNOID_EXPORT SignalProxy<void()> sigAboutToQuit();
[[deprecated("Use App::updateGui")]]
CNOID_EXPORT void updateGui();

}

#endif
