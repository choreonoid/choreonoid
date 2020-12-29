/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_APP_UTIL_H
#define CNOID_BASE_APP_UTIL_H

#include <cnoid/Signal>
#include "exportdecl.h"

namespace cnoid {

CNOID_EXPORT SignalProxy<void()> sigAboutToQuit();

CNOID_EXPORT void updateGui();

}

#endif
