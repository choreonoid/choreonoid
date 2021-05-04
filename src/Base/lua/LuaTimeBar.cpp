/*!
  @author Shin'ichiro Nakaoka
*/

#include "../TimeBar.h"
#include <cnoid/LuaUtil>
#include <cnoid/LuaSignal>

using namespace cnoid;

namespace cnoid {

void exportLuaTimeBar(sol::table& module)
{
    module.new_usertype<TimeBar>(
        "TimeBar",
        sol::base_classes, sol::bases<ToolBar, QWidget, QObject>(),
        "instance", &TimeBar::instance,
        "sigPlaybackInitialized", &TimeBar::sigPlaybackInitialized,
        "sigPlaybackStarted", &TimeBar::sigPlaybackStarted,
        "sigTimeChanged", &TimeBar::sigTimeChanged,
        "sigPlaybackStopped", &TimeBar::sigPlaybackStopped,
        "time", &TimeBar::time,
        "setTime", &TimeBar::setTime,
        "realPlaybackTime", &TimeBar::realPlaybackTime,
        "minTime", &TimeBar::minTime,
        "maxTime", &TimeBar::maxTime,

        "setTimeRange", &TimeBar::setTimeRange,
        "frameRate", &TimeBar::frameRate,
        "setFrameRate", &TimeBar::setFrameRate,
        "timeStep", &TimeBar::timeStep,
        "playbackSpeedScale", &TimeBar::playbackSpeedScale,
        "setPlaybackSpeedScale", &TimeBar::setPlaybackSpeedScale,
        "playbackFrameRate", &TimeBar::playbackFrameRate,
        "setPlaybackFrameRate", &TimeBar::setPlaybackFrameRate,
        "setRepeatMode", &TimeBar::setRepeatMode,
        "startPlayback", sol::overload(
            [](TimeBar* self){ self->startPlayback(); },
            [](TimeBar* self, double time){ self->startPlayback(time); }),
        "stopPlayback", sol::overload(
            [](TimeBar* self) { self->stopPlayback(); },
            [](TimeBar* self, bool isStoppedManually) { self->stopPlayback(isStoppedManually); }),
        "isDoingPlayback", &TimeBar::isDoingPlayback
        );

    LuaSignal<bool(double time), LogicalProduct>("PlaybackInitializationSignal", module);
    LuaSignal<bool(double time), LogicalSum>("PlaybackTimeSignal", module);
}

}
