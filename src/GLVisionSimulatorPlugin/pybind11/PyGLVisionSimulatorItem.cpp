#include "../GLVisionSimulatorItem.h"
#include <cnoid/PyBase>

using namespace cnoid;
namespace py = pybind11;

PYBIND11_MODULE(GLVisionSimulatorPlugin, m)
{
    m.doc() = "Choreonoid GLVisionSimulatorPlugin module";

    py::module::import("cnoid.BodyPlugin");

    py::class_<GLVisionSimulatorItem, GLVisionSimulatorItemPtr, SubSimulatorItem>
        glVisionSimulatorItemClass(m, "GLVisionSimulatorItem");

    glVisionSimulatorItemClass
        .def(py::init<>())
        .def("setTargetBodies", &GLVisionSimulatorItem::setTargetBodies)
        .def("setTargetSensors", &GLVisionSimulatorItem::setTargetSensors)
        .def("setMaxFrameRate", &GLVisionSimulatorItem::setMaxFrameRate)
        .def("setMaxLatency", &GLVisionSimulatorItem::setMaxLatency)
        .def("setVisionDataRecordingEnabled", &GLVisionSimulatorItem::setVisionDataRecordingEnabled)
        .def("setThreadMode", &GLVisionSimulatorItem::setThreadMode)
        .def("setBestEffortMode", &GLVisionSimulatorItem::setBestEffortMode)
        .def("setRangeSensorPrecisionRatio", &GLVisionSimulatorItem::setRangeSensorPrecisionRatio)
        .def("setEveryRenderableItemEnabled", &GLVisionSimulatorItem::setEveryRenderableItemEnabled)
        .def("setHeadLightEnabled", &GLVisionSimulatorItem::setHeadLightEnabled)
        .def("setAdditionalLightSetEnabled", &GLVisionSimulatorItem::setAdditionalLightSetEnabled)

        // deprecated
        .def("setDedicatedSensorThreadsEnabled",
             [](GLVisionSimulatorItem& self, bool on){
                 self.setThreadMode(
                     on ? GLVisionSimulatorItem::SENSOR_THREAD_MODE : GLVisionSimulatorItem::SINGLE_THREAD_MODE);
             })
        .def("setAdditionalLightsEnabled", &GLVisionSimulatorItem::setAdditionalLightSetEnabled)
        .def("setAllSceneObjectsEnabled", &GLVisionSimulatorItem::setEveryRenderableItemEnabled)
        ;

    py::enum_<GLVisionSimulatorItem::ThreadMode>(glVisionSimulatorItemClass, "ThreadMode")
        .value("SINGLE_THREAD_MODE", GLVisionSimulatorItem::SINGLE_THREAD_MODE)
        .value("SENSOR_THREAD_MODE", GLVisionSimulatorItem::SENSOR_THREAD_MODE)
        .value("SCREEN_THREAD_MODE", GLVisionSimulatorItem::SCREEN_THREAD_MODE)
        .value("N_THREAD_MODES", GLVisionSimulatorItem::N_THREAD_MODES)
        .export_values();

    PyItemList<GLVisionSimulatorItem>(m, "GLVisionSimulatorItemList");
}
