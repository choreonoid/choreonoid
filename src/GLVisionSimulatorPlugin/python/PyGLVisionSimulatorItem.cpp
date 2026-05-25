#include "../GLVisionSimulatorItem.h"
#include <cnoid/PyBase>

using namespace cnoid;
namespace nb = nanobind;

NB_MODULE(GLVisionSimulatorPlugin, m)
{
    m.doc() = "Choreonoid GLVisionSimulatorPlugin module";

    nb::module_::import_("cnoid.BodyPlugin");

    nb::class_<GLVisionSimulatorItem, SubSimulatorItem>
        glVisionSimulatorItemClass(m, "GLVisionSimulatorItem");

    glVisionSimulatorItemClass
        .def(nb::init<>())
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
        ;

    nb::enum_<GLVisionSimulatorItem::ThreadMode>(glVisionSimulatorItemClass, "ThreadMode")
        .value("SINGLE_THREAD_MODE", GLVisionSimulatorItem::SINGLE_THREAD_MODE)
        .value("SENSOR_THREAD_MODE", GLVisionSimulatorItem::SENSOR_THREAD_MODE)
        .value("SCREEN_THREAD_MODE", GLVisionSimulatorItem::SCREEN_THREAD_MODE)
        .value("N_THREAD_MODES", GLVisionSimulatorItem::N_THREAD_MODES)
        .export_values();

    PyItemList<GLVisionSimulatorItem>(m, "GLVisionSimulatorItemList");
}
