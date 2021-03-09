/*!
  @author Shin'ichiro Nakaoka
*/

#include "PyQObjectHolder.h"
#include "PyQString.h"
#include "../ToolBar.h"
#include "../TimeBar.h"
#include <cnoid/PyUtil>

using namespace cnoid;
namespace py = pybind11;

namespace cnoid {

void exportPyToolBars(py::module m)
{
    py::class_<ToolBar, PyQObjectHolder<ToolBar>, QWidget>(m, "ToolBar")
        .def(py::init<const QString&>())
        .def("addButton",
             [](ToolBar& self, const char* text, const char* tooltip){ return self.addButton(text, tooltip); },
             py::arg("text"), py::arg("tooltip") = QString())
        .def("addButton",
             [](ToolBar& self, const QIcon& icon, const char* tooltip){ return self.addButton(icon, tooltip); },
             py::arg("icon"), py::arg("tooltip") = QString())
        .def("addToggleButton",
             [](ToolBar& self, const char* text, const char* tooltip){ return self.addToggleButton(text, tooltip); },
             py::arg("text"), py::arg("tooltip") = QString())
        .def("addToggleButton",
            [](ToolBar& self, const QIcon& icon, const char* tooltip){ return self.addToggleButton(icon, tooltip); },
             py::arg("icon"), py::arg("tooltip") = QString())
        .def("requestNewRadioGroup", &ToolBar::requestNewRadioGroup)
        .def("addRadioButton",
             [](ToolBar& self, const char* text, const char* tooltip){ return self.addRadioButton(text, tooltip); },
             py::arg("text"), py::arg("tooltip") = QString())
        .def("addRadioButton",
             [](ToolBar& self, const QIcon& icon, const char* tooltip){ return self.addRadioButton(icon, tooltip); },
             py::arg("icon"), py::arg("tooltip") = QString())
        .def("addWidget", &ToolBar::addWidget)
        .def("addLabel", &ToolBar::addLabel)
        .def("addSeparator", &ToolBar::addSeparator)
        .def("addSpacing", &ToolBar::addSpacing, py::arg("spacing") = -1)
        .def("setInsertionPosition", &ToolBar::setInsertionPosition)
        .def("setVisibleByDefault", &ToolBar::setVisibleByDefault, py::arg("on") = true)
        .def("isVisibleByDefault", &ToolBar::isVisibleByDefault)
        .def("placeOnNewRowByDefault", &ToolBar::placeOnNewRowByDefault, py::arg("on") = true)
        .def("isPlacedOnNewRowByDefault", &ToolBar::isPlacedOnNewRowByDefault)
        .def("setStretchable", &ToolBar::setStretchable)
        .def("isStretchable", &ToolBar::isStretchable)
        .def("setAutoRaiseByDefault", &ToolBar::setAutoRaiseByDefault, py::arg("on") = true)
        .def("isAutoRaiseByDefault", &ToolBar::isAutoRaiseByDefault)
        ;

    py::class_<TimeBar, PyQObjectHolder<TimeBar>, ToolBar>(m, "TimeBar")
        .def_property_readonly_static("instance", [](py::object){ return TimeBar::instance(); })
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
        .def("startPlayback", (void(TimeBar::*)()) &TimeBar::startPlayback)
        .def("startPlayback", (void(TimeBar::*)(double)) &TimeBar::startPlayback)
        .def("stopPlayback", &TimeBar::stopPlayback, py::arg("isStoppedManually") = false)
        .def("isDoingPlayback", &TimeBar::isDoingPlayback)

        // deprecated
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
