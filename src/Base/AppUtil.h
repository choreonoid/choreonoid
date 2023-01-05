#ifndef CNOID_BASE_APP_UTIL_H
#define CNOID_BASE_APP_UTIL_H

#include <cnoid/Signal>
#include <QKeyEvent>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT AppUtil
{
public:
    static SignalProxy<void()> sigAboutToQuit();
    static void updateGui();
    static bool isNoWindowMode();
    static SignalProxy<void(QKeyEvent* event)> sigKeyPressed();
    static SignalProxy<void(QKeyEvent* event)> sigKeyReleased();
};

[[deprecated("Use AppUtil::sigAboutToQuit")]]
CNOID_EXPORT SignalProxy<void()> sigAboutToQuit();
[[deprecated("Use AppUtil::updateGui")]]
CNOID_EXPORT void updateGui();

}

#endif
