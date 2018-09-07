/*!
  @author Shin'ichiro Nakaoka
*/

#include "../ToolBar.h"
#include "../TimeBar.h"
#include "PyQString.h"
#include <cnoid/PySignal>

using namespace cnoid;
namespace py = pybind11;

namespace cnoid {

void exportPyToolBars(py::module m)
{
    py::class_<ToolBar, QWidget>(m, "ToolBar")
        .def(py::init<const QString&>())
        .def("addButton", [](ToolBar& self, const char* text){ return self.addButton(text); },
             py::return_value_policy::reference)
        .def("addButton", [](ToolBar& self, const char* text, const char* tooltip){ return self.addButton(text, tooltip); },
             py::return_value_policy::reference)
        .def("addButton", [](ToolBar& self, const QIcon& icon){ return self.addButton(icon); },
             py::return_value_policy::reference)
        .def("addButton", [](ToolBar& self, const QIcon& icon, const char* tooltip){ return self.addButton(icon, tooltip); },
             py::return_value_policy::reference)
        .def("addToggleButton", [](ToolBar& self, const char* text){ return self.addToggleButton(text); },
             py::return_value_policy::reference_internal)
        .def("addToggleButton", [](ToolBar& self, const char* text, const char* tooltip){ return self.addToggleButton(text, tooltip); },
             py::return_value_policy::reference_internal)
        .def("addToggleButton", [](ToolBar& self, const QIcon& icon){ return self.addToggleButton(icon); },
             py::return_value_policy::reference_internal)
        .def("addToggleButton", [](ToolBar& self, const QIcon& icon, const char* tooltip){ return self.addToggleButton(icon, tooltip); },
             py::return_value_policy::reference_internal)
        .def("requestNewRadioGroup", &ToolBar::requestNewRadioGroup)
        .def("addRadioButton", [](ToolBar& self, const char* text){ return self.addRadioButton(text); },
             py::return_value_policy::reference_internal)
        .def("addRadioButton", [](ToolBar& self, const char* text, const char* tooltip){ return self.addRadioButton(text, tooltip); },
             py::return_value_policy::reference_internal)
        .def("addRadioButton", [](ToolBar& self, const QIcon& icon){ return self.addRadioButton(icon); },
             py::return_value_policy::reference_internal)
        .def("addRadioButton", [](ToolBar& self, const QIcon& icon, const char* tooltip){ return self.addRadioButton(icon, tooltip); },
             py::return_value_policy::reference_internal)
        .def("addWidget", &ToolBar::addWidget)
        .def("addSeparator", &ToolBar::addSeparator, py::return_value_policy::reference_internal)
        .def("addSpacing", &ToolBar::addSpacing)
        .def("setVisibleByDefault", &ToolBar::setVisibleByDefault)
        .def("isVisibleByDefault", &ToolBar::isVisibleByDefault)
        .def("setStretchable", &ToolBar::setStretchable)
        .def("isStretchable", &ToolBar::isStretchable)
        ;

    py::class_<TimeBar>(m, "TimeBar")
        .def_property_readonly_static(
            "instance", [](py::object){ return TimeBar::instance(); }, py::return_value_policy::reference)
        .def_property_readonly("sigPlaybackInitialized", &TimeBar::sigPlaybackInitialized)
        .def_property_readonly("sigPlaybackStarted", &TimeBar::sigPlaybackStarted)
        .def_property_readonly("sigTimeChanged", &TimeBar::sigTimeChanged)
        .def_property_readonly("sigPlaybackStopped", &TimeBar::sigPlaybackStopped)
        .def_property("time", &TimeBar::time, &TimeBar::setTime)
        .def("setTime", &TimeBar::setTime)
        .def_property_readonly("realPlaybackTime", &TimeBar::realPlaybackTime)
        .def_property_readonly("minTime", &TimeBar::minTime)
        .def_property_readonly("maxTime", &TimeBar::maxTime)
        .def("setTimeRange", &TimeBar::setTimeRange)
        .def_property("frameRate", &TimeBar::frameRate, &TimeBar::setFrameRate)
        .def("setFrameRate", &TimeBar::setFrameRate)
        .def_property_readonly("timeStep", &TimeBar::timeStep)
        .def_property("playbackSpeedScale", &TimeBar::playbackSpeedScale, &TimeBar::setPlaybackSpeedScale)
        .def("setPlaybackSpeedScale", &TimeBar::setPlaybackSpeedScale)
        .def_property("playbackFrameRate", &TimeBar::playbackFrameRate, &TimeBar::setPlaybackFrameRate)
        .def("setPlaybackFrameRate", &TimeBar::setPlaybackFrameRate)
        .def("setRepeatMode", &TimeBar::setRepeatMode)
        .def("startPlayback", &TimeBar::startPlayback)
        .def("startPlaybackFromFillLevel", &TimeBar::startPlaybackFromFillLevel)
        .def("stopPlayback", [](TimeBar& self){ self.stopPlayback(); })
        .def("stopPlayback", &TimeBar::stopPlayback)
        .def("isDoingPlayback", &TimeBar::isDoingPlayback)
        .def("startFillLevelUpdate", &TimeBar::startFillLevelUpdate)
        .def("updateFillLevel", &TimeBar::updateFillLevel)
        .def("stopFillLevelUpdate", &TimeBar::stopFillLevelUpdate)
        .def("setFillLevelSync", &TimeBar::setFillLevelSync)

        // deprecated
        .def_static("getInstance", &TimeBar::instance, py::return_value_policy::reference)
        .def("getSigPlaybackInitialized", &TimeBar::sigPlaybackInitialized)
        .def("getSigPlaybackStarted", &TimeBar::sigPlaybackStarted)
        .def("getSigTimeChanged", &TimeBar::sigTimeChanged)
        .def("getSigPlaybackStopped", &TimeBar::sigPlaybackStopped)
        .def("getTime", &TimeBar::time)
        .def("getRealPlaybackTime", &TimeBar::realPlaybackTime)
        .def("getMinTime", &TimeBar::minTime)
        .def("getMaxTime", &TimeBar::maxTime)
        .def("getFrameRate", &TimeBar::frameRate)
        .def("getTimeStep", &TimeBar::timeStep)
        .def("getPlaybackSpeedScale", &TimeBar::playbackSpeedScale)
        .def("getPlaybackFrameRate", &TimeBar::playbackFrameRate)
        ;

        PySignal<bool(double time), LogicalProduct>(m, "SigPlaybackInitialized");
        PySignal<bool(double time), LogicalSum>(m, "SigTimeChanged");
    }
}
