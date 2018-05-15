/*!
  @author Shin'ichiro Nakaoka
*/

#include "../ToolBar.h"
#include "../TimeBar.h"
#include <cnoid/PySignal>
#include <cnoid/PyUtil>

using namespace boost::python;
using namespace cnoid;

// for MSVC++2015 Update3
CNOID_PYTHON_DEFINE_GET_POINTER(QWidget)
CNOID_PYTHON_DEFINE_GET_POINTER(ToolButton)
CNOID_PYTHON_DEFINE_GET_POINTER(TimeBar)

namespace {

ToolButton* (ToolBar::*ToolBar_addButton1)(const QString& text, const QString& tooltip) = &ToolBar::addButton;
ToolButton* (ToolBar::*ToolBar_addButton2)(const QIcon& icon, const QString& tooltip) = &ToolBar::addButton;
ToolButton* (ToolBar::*ToolBar_addButton3)(const char* const* xpm, const QString& tooltip) = &ToolBar::addButton;

ToolButton* ToolBar_addToggleButton1(ToolBar& self, const QString& text, const QString& tooltip = QString()){
    return self.addToggleButton(text, tooltip);
}
BOOST_PYTHON_FUNCTION_OVERLOADS(ToolBar_addToggleButton1_overloads, ToolBar_addToggleButton1, 2, 3)

ToolButton* (ToolBar::*ToolBar_addToggleButton2)(const QIcon& icon, const QString& tooltip) = &ToolBar::addToggleButton;
ToolButton* (ToolBar::*ToolBar_addToggleButton3)(const char* const* xpm, const QString& tooltip) = &ToolBar::addToggleButton;
ToolButton* (ToolBar::*ToolBar_addRadioButton1)(const QString& text, const QString& tooltip) = &ToolBar::addRadioButton;
ToolButton* (ToolBar::*ToolBar_addRadioButton2)(const QIcon& icon, const QString& tooltip) = &ToolBar::addRadioButton;
ToolButton* (ToolBar::*ToolBar_addRadioButton3)(const char* const* xpm, const QString& tooltip) = &ToolBar::addRadioButton;


BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(TimeBar_stopPlayback_overloads, stopPlayback, 0, 1)

}

namespace cnoid {

void exportPyToolBars()
{
    class_<ToolBar, ToolBar*, bases<QWidget>, boost::noncopyable>("ToolBar", init<const QString&>())
        .def("addButton", ToolBar_addButton1, return_value_policy<reference_existing_object>())
        .def("addButton", ToolBar_addButton2, return_value_policy<reference_existing_object>())
        .def("addButton", ToolBar_addButton3, return_value_policy<reference_existing_object>())
        .def("addToggleButton", ToolBar_addToggleButton1, ToolBar_addToggleButton1_overloads()[return_value_policy<reference_existing_object>()])
        .def("addToggleButton", ToolBar_addToggleButton2, return_value_policy<reference_existing_object>())
        .def("addToggleButton", ToolBar_addToggleButton3, return_value_policy<reference_existing_object>())
        .def("requestNewRadioGroup", &ToolBar::requestNewRadioGroup)
        //.def("currentRadioGroup", &ToolBar::currentRadioGroup, return_value_policy<reference_existing_object>())
        .def("addRadioButton", ToolBar_addRadioButton1, return_value_policy<reference_existing_object>())
        .def("addRadioButton", ToolBar_addRadioButton2, return_value_policy<reference_existing_object>())
        .def("addRadioButton", ToolBar_addRadioButton3, return_value_policy<reference_existing_object>())
        //.def("addAction", &ToolBar::addAction)
        .def("addWidget", &ToolBar::addWidget)
        //.def("addLabel", &ToolBar::addLabel, return_value_policy<reference_existing_object>())
        //.def("addImage", &ToolBar::addImage, return_value_policy<reference_existing_object>())
        .def("addSeparator", &ToolBar::addSeparator, return_value_policy<reference_existing_object>())
        .def("addSpacing", &ToolBar::addSpacing)
        .def("setVisibleByDefault", &ToolBar::setVisibleByDefault)
        .def("isVisibleByDefault", &ToolBar::isVisibleByDefault)
        .def("setStretchable", &ToolBar::setStretchable)
        .def("isStretchable", &ToolBar::isStretchable, return_value_policy<return_by_value>())
        //.def("toolBarArea", &ToolBar::toolBarArea, return_value_policy<reference_existing_object>());
        ;

    {
        scope timeBarScope =
            class_<TimeBar, TimeBar*, bases<ToolBar>, boost::noncopyable>("TimeBar", no_init)
            .def("instance", &TimeBar::instance, return_value_policy<reference_existing_object>()).staticmethod("instance")
            .def("getInstance", &TimeBar::instance, return_value_policy<reference_existing_object>()).staticmethod("getInstance")
            .def("sigPlaybackInitialized", &TimeBar::sigPlaybackInitialized)
            .def("getSigPlaybackInitialized", &TimeBar::sigPlaybackInitialized)
            .def("sigPlaybackStarted", &TimeBar::sigPlaybackStarted)
            .def("getSigPlaybackStarted", &TimeBar::sigPlaybackStarted)
            .def("sigTimeChanged", &TimeBar::sigTimeChanged)
            .def("getSigTimeChanged", &TimeBar::sigTimeChanged)
            .def("sigPlaybackStopped", &TimeBar::sigPlaybackStopped)        
            .def("getSigPlaybackStopped", &TimeBar::sigPlaybackStopped)        
            .def("time", &TimeBar::time)
            .def("getTime", &TimeBar::time)
            .def("setTime", &TimeBar::setTime)
            .def("realPlaybackTime", &TimeBar::realPlaybackTime)
            .def("getRealPlaybackTime", &TimeBar::realPlaybackTime)
            .def("minTime", &TimeBar::minTime)
            .def("getMinTime", &TimeBar::minTime)
            .def("maxTime", &TimeBar::maxTime)
            .def("getMaxTime", &TimeBar::maxTime)
            .def("setTimeRange", &TimeBar::setTimeRange)
            .def("frameRate", &TimeBar::frameRate)
            .def("getFrameRate", &TimeBar::frameRate)
            .def("setFrameRate", &TimeBar::setFrameRate)
            .def("timeStep", &TimeBar::timeStep)
            .def("getTimeStep", &TimeBar::timeStep)
            .def("playbackSpeedScale", &TimeBar::playbackSpeedScale)
            .def("getPlaybackSpeedScale", &TimeBar::playbackSpeedScale)
            .def("setPlaybackSpeedScale", &TimeBar::setPlaybackSpeedScale)
            .def("playbackFrameRate", &TimeBar::playbackFrameRate)
            .def("getPlaybackFrameRate", &TimeBar::playbackFrameRate)
            .def("setPlaybackFrameRate", &TimeBar::setPlaybackFrameRate)
            .def("setRepeatMode", &TimeBar::setRepeatMode)
            .def("startPlayback", &TimeBar::startPlayback)
            .def("startPlaybackFromFillLevel", &TimeBar::startPlaybackFromFillLevel)
            .def("stopPlayback", &TimeBar::stopPlayback, TimeBar_stopPlayback_overloads())
            .def("isDoingPlayback", &TimeBar::isDoingPlayback)
            .def("startFillLevelUpdate", &TimeBar::startFillLevelUpdate)
            .def("updateFillLevel", &TimeBar::updateFillLevel)
            .def("stopFillLevelUpdate", &TimeBar::stopFillLevelUpdate)
            .def("setFillLevelSync", &TimeBar::setFillLevelSync)
            ;

        PySignal<bool(double time), LogicalProduct>("SigPlaybackInitialized");
        PySignal<bool(double time), LogicalSum>("SigTimeChanged");
    }
}

}
