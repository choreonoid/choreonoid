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
        .def_static("instance", &TimeBar::instance, py::return_value_policy::reference)
        .def("sigPlaybackInitialized", &TimeBar::sigPlaybackInitialized)
        .def("sigPlaybackStarted", &TimeBar::sigPlaybackStarted)
        .def("sigTimeChanged", &TimeBar::sigTimeChanged)
        .def("sigPlaybackStopped", &TimeBar::sigPlaybackStopped)
        .def("time", &TimeBar::time)
        .def("setTime", &TimeBar::setTime)
        .def("realPlaybackTime", &TimeBar::realPlaybackTime)
        .def("minTime", &TimeBar::minTime)
        .def("maxTime", &TimeBar::maxTime)
        .def("setTimeRange", &TimeBar::setTimeRange)
        .def("frameRate", &TimeBar::frameRate)
        .def("setFrameRate", &TimeBar::setFrameRate)
        .def("timeStep", &TimeBar::timeStep)
        .def("playbackSpeedScale", &TimeBar::playbackSpeedScale)
        .def("setPlaybackSpeedScale", &TimeBar::setPlaybackSpeedScale)
        .def("playbackFrameRate", &TimeBar::playbackFrameRate)
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
        ;

        PySignal<bool(double time), LogicalProduct>(m, "SigPlaybackInitialized");
        PySignal<bool(double time), LogicalSum>(m, "SigTimeChanged");
    }
}
