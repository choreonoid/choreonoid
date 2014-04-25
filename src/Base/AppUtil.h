/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_APP_UTIL_H_INCLUDED
#define CNOID_BASE_APP_UTIL_H_INCLUDED

#include <cnoid/SignalProxy>
#include "exportdecl.h"

namespace cnoid {
CNOID_EXPORT SignalProxy< boost::signal<void()> > sigAboutToQuit();
}

#endif
