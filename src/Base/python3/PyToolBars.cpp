/*!
  @author Shin'ichiro Nakaoka
*/

#include "../ToolBar.h"
#include "../TimeBar.h"
#include "PyQtCore.h"
#include <cnoid/Py3Signal>
#include <cnoid/Py3Util>

namespace py = pybind11;
using namespace cnoid;

// for MSVC++2015 Update3
CNOID_PYTHON_DEFINE_GET_POINTER(QWidget)
CNOID_PYTHON_DEFINE_GET_POINTER(ToolButton)
CNOID_PYTHON_DEFINE_GET_POINTER(TimeBar)

namespace cnoid {

void exportPyToolBars(py::module m)
{
    py::class_<ToolBar, QWidget>(m, "ToolBar")
        .def(py::init<const QString&>())
        .def("addButton", (ToolButton* (ToolBar::*)(const QString&, const QString&)) &ToolBar::addButton,
                py::return_value_policy::reference_internal, py::arg("text"), py::arg("tooltip")=QString())
        .def("addButton", (ToolButton* (ToolBar::*)(const QIcon&, const QString&)) &ToolBar::addButton,
                py::return_value_policy::reference_internal, py::arg("icon"), py::arg("tooltip")=QString())
//       .def("addButton", (ToolButton* (ToolBar::*)(const char* const*, const QString&)) &ToolBar::addButton,
//                py::return_value_policy::reference_internal, py::arg("xpm"), py::arg("tooltip")=QString())
        .def("addToggleButton", (ToolButton* (ToolBar::*)(const QString&, const QString&)) &ToolBar::addToggleButton,
                py::return_value_policy::reference_internal, py::arg("text"), py::arg("tooltip")=QString())
        .def("addToggleButton", (ToolButton* (ToolBar::*)(const QIcon&, const QString&)) &ToolBar::addToggleButton,
                py::return_value_policy::reference_internal, py::arg("icon"), py::arg("tooltip")=QString())
 //       .def("addToggleButton", (ToolButton* (ToolBar::*)(const char* const*, const QString&)) &ToolBar::addToggleButton,
 //               py::return_value_policy::reference_internal, py::arg("xpm"), py::arg("tooltip")=QString())
        .def("requestNewRadioGroup", &ToolBar::requestNewRadioGroup)
        //.def("currentRadioGroup", &ToolBar::currentRadioGroup, return_value_policy<reference_existing_object>())
        .def("addRadioButton", (ToolButton* (ToolBar::*)(const QString&, const QString&)) &ToolBar::addRadioButton,
                py::return_value_policy::reference_internal, py::arg("text"), py::arg("tooltip")=QString())
        .def("addRadioButton", (ToolButton* (ToolBar::*)(const QIcon&, const QString&)) &ToolBar::addRadioButton,
                py::return_value_policy::reference_internal, py::arg("icon"), py::arg("tooltip")=QString())
 //       .def("addRadioButton", (ToolButton* (ToolBar::*)(const char* const*, const QString&)) &ToolBar::addRadioButton,
 //               py::return_value_policy::reference_internal, py::arg("xpm"), py::arg("tooltip")=QString())
        //.def("addAction", &ToolBar::addAction)
        .def("addWidget", &ToolBar::addWidget)
        //.def("addLabel", &ToolBar::addLabel, return_value_policy<reference_existing_object>())
        //.def("addImage", &ToolBar::addImage, return_value_policy<reference_existing_object>())
        .def("addSeparator", &ToolBar::addSeparator, py::return_value_policy::reference_internal)
        .def("addSpacing", &ToolBar::addSpacing)
        .def("setVisibleByDefault", &ToolBar::setVisibleByDefault)
        .def("isVisibleByDefault", &ToolBar::isVisibleByDefault)
        .def("setStretchable", &ToolBar::setStretchable)
        .def("isStretchable", &ToolBar::isStretchable)
        //.def("toolBarArea", &ToolBar::toolBarArea, return_value_policy<reference_existing_object>());
        ;

    py::class_<TimeBar, std::unique_ptr<TimeBar, py::nodelete>>(m, "TimeBar")
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
        .def("stopPlayback", &TimeBar::stopPlayback, py::arg("isStoppedManually")=false)
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

