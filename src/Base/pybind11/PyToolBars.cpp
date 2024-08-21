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
        .def(py::init<const std::string&>())

        .def("addButton", [](ToolBar& self, const char* text, int id){ return self.addButton(text, id); },
             py::arg("text"), py::arg("id") = -1)
        .def("addButton", [](ToolBar& self, const QIcon& icon, int id){ return self.addButton(icon, id); },
             py::arg("icon"), py::arg("id") = -1)
        .def("addToggleButton", [](ToolBar& self, const char* text, int id){ return self.addToggleButton(text, id); },
             py::arg("text"), py::arg("id") = -1)
        .def("addToggleButton", [](ToolBar& self, const QIcon& icon, int id){ return self.addButton(icon, id); },
             py::arg("icon"), py::arg("id") = -1)
        .def("requestNewRadioGroup", &ToolBar::requestNewRadioGroup)
        .def("addRadioButton", [](ToolBar& self, const char* text, int id){ return self.addRadioButton(text, id); },
             py::arg("text"), py::arg("id") = -1)
        .def("addRadioButton", [](ToolBar& self, const QIcon& icon, int id){ return self.addRadioButton(icon, id); },
             py::arg("icon"), py::arg("id") = -1)
        .def("addWidget", &ToolBar::addWidget, py::arg("widget"), py::arg("id") = -1)
        .def("addLabel", &ToolBar::addLabel, py::arg("text"), py::arg("id") = -1)
        .def("addSeparator", &ToolBar::addSeparator, py::arg("id") = -1)
        .def("addSpacing", &ToolBar::addSpacing, py::arg("spacing") = -1, py::arg("id") = -1)
        .def("setInsertionPosition", &ToolBar::setInsertionPosition)
        .def("elementPosition", &ToolBar::elementPosition)
        .def("numElements", &ToolBar::numElements)
        .def("setVisibleByDefault", &ToolBar::setVisibleByDefault, py::arg("on") = true)
        .def("isVisibleByDefault", &ToolBar::isVisibleByDefault)
        .def("placeOnNewRowByDefault", &ToolBar::placeOnNewRowByDefault, py::arg("on") = true)
        .def("isPlacedOnNewRowByDefault", &ToolBar::isPlacedOnNewRowByDefault)
        .def("setStretchable", &ToolBar::setStretchable)
        .def("isStretchable", &ToolBar::isStretchable)
        .def("setAutoRaiseByDefault", &ToolBar::setAutoRaiseByDefault, py::arg("on") = true)
        .def("isAutoRaiseByDefault", &ToolBar::isAutoRaiseByDefault)

        // deprecated
        .def("addButton",
             [](ToolBar& self, const char* text, const char* tooltip){
                 auto button = self.addButton(text);
                 button->setToolTip(tooltip);
                 return button;
             })
        .def("addButton",
             [](ToolBar& self, const QIcon& icon, const char* tooltip){
                 auto button = self.addButton(icon);
                 button->setToolTip(tooltip);
                 return button;
             })
        .def("addToggleButton",
             [](ToolBar& self, const char* text, const char* tooltip){
                 auto button = self.addToggleButton(text);
                 button->setToolTip(tooltip);
                 return button;
             })
        .def("addToggleButton",
             [](ToolBar& self, const QIcon& icon, const char* tooltip){
                 auto button = self.addToggleButton(icon);
                 button->setToolTip(tooltip);
                 return button;
             })
        .def("addRadioButton",
             [](ToolBar& self, const char* text, const char* tooltip){
                 auto button = self.addRadioButton(text);
                 button->setToolTip(tooltip);
                 return button;
             })
        .def("addRadioButton",
             [](ToolBar& self, const QIcon& icon, const char* tooltip){
                 auto button = self.addRadioButton(icon);
                 button->setToolTip(tooltip);
                 return button;
             })
        ;

    py::class_<TimeBar, PyQObjectHolder<TimeBar>, ToolBar> timeBarClass(m, "TimeBar");

    py::enum_<TimeBar::TimeOption>(timeBarClass, "TimeOption")
        .value("Truncate", TimeBar::Truncate)
        .value("Round", TimeBar::Round)
        .value("Expand", TimeBar::Expand)
        .export_values();
        ;

    timeBarClass
        .def_property_readonly_static("instance", [](py::object){ return TimeBar::instance(); })
        .def_property_readonly("sigPlaybackInitialized", &TimeBar::sigPlaybackInitialized)
        .def_property_readonly("sigPlaybackStarted", &TimeBar::sigPlaybackStarted)
        .def_property_readonly("sigTimeChanged", &TimeBar::sigTimeChanged)
        .def_property_readonly("sigPlaybackStopped", &TimeBar::sigPlaybackStopped)
        .def_property_readonly("sigPlaybackStoppedEx", &TimeBar::sigPlaybackStoppedEx)
        .def_property("time", &TimeBar::time, &TimeBar::setTime)
        .def("setTime", &TimeBar::setTime, py::arg("time"), py::arg("options") = TimeBar::Truncate)
        .def_property_readonly("realPlaybackTime", &TimeBar::realPlaybackTime)
        .def_property_readonly("minTime", &TimeBar::minTime)
        .def_property_readonly("maxTime", &TimeBar::maxTime)
        .def("setTimeRange", &TimeBar::setTimeRange)
        .def_property("frameRate", &TimeBar::frameRate, &TimeBar::setFrameRate)
        .def("setFrameRate", &TimeBar::setFrameRate)
        .def_property_readonly("timeStep", &TimeBar::timeStep)
        .def_property("playbackSpeedRatio", &TimeBar::playbackSpeedRatio, &TimeBar::setPlaybackSpeedRatio)
        .def("setPlaybackSpeedRatio", &TimeBar::setPlaybackSpeedRatio)
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
        .def("getPlaybackSpeedScale", &TimeBar::playbackSpeedRatio)
        .def("getPlaybackFrameRate", &TimeBar::playbackFrameRate)
        ;
    }
}
