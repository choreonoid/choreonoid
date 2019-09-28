/*!
 * @author Shin'ichiro Nakaoka
*/

#include "../MessageView.h"
#include "../SceneWidget.h"
#include "../SceneView.h"
#include "../TaskView.h"
#include "../ViewManager.h"
#include "PyQString.h"
#include <cnoid/PySignal>
#include <cnoid/PyEigenTypes>
#include <QWidget>

using namespace cnoid;
namespace py = pybind11;

namespace cnoid {

void exportPyViews(py::module m)
{
    PySignal<void(View*)>(m, "ViewSignal");

    py::class_<View, QWidget> view(m, "View");
    view
        .def_property("name", &View::name, &View::setName)
        .def("setName", &View::setName)
        .def("isActive", &View::isActive)
        .def("bringToFront", &View::bringToFront)
        .def_property_readonly("sigActivated", &View::sigActivated)
        .def_property_readonly("sigDeactivated", &View::sigDeactivated)
        .def_property("defaultLayoutArea", &View::defaultLayoutArea, &View::setDefaultLayoutArea)
        .def("setDefaultLayoutArea", &View::setDefaultLayoutArea)
        .def_property_readonly("indicatorOnInfoBar", &View::indicatorOnInfoBar, py::return_value_policy::reference)
        .def("enableFontSizeZoomKeys", &View::enableFontSizeZoomKeys)
        .def_property_readonly_static("lastFocusView", &View::lastFocusView, py::return_value_policy::reference)
        .def_property_readonly("sigFocusChanged", &View::sigFocusChanged)

        // deprecated
        .def("getName", &View::name)
        .def("getSigActivated", &View::sigActivated)
        .def("getSigDeactivated", &View::sigDeactivated)
        .def("getDefaultLayoutArea", &View::defaultLayoutArea)
        .def("getIndicatorOnInfoBar", &View::indicatorOnInfoBar, py::return_value_policy::reference)
        .def_static("getLastFocusView", &View::lastFocusView, py::return_value_policy::reference)
        .def("getSigFocusChanged", &View::sigFocusChanged)
        ;

    py::enum_<View::LayoutArea>(view, "LayoutArea")
        .value("LEFT", View::LayoutArea::LEFT)
        .value("LEFT_TOP", View::LayoutArea::LEFT_TOP)
        .value("LEFT_BOTTOM", View::LayoutArea::LEFT_BOTTOM)
        .value("CENTER", View::LayoutArea::CENTER)
        .value("RIGHT", View::LayoutArea::RIGHT)
        .value("BOTTOM", View::LayoutArea::BOTTOM)
        .value("NUM_AREAS", View::LayoutArea::NUM_AREAS)
        .export_values();

    py::class_<MessageView, View>(m, "MessageView")
        .def_property_readonly_static(
            "instance", [](py::object){ return MessageView::instance(); }, py::return_value_policy::reference)
        .def("put", (void (MessageView::*)(const std::string&, int)) &MessageView::put)
        .def("putln", (void (MessageView::*)(const std::string&, int)) &MessageView::putln)
        .def("notify", (void (MessageView::*)(const std::string&, int)) &MessageView::notify)
        .def("flush", &MessageView::flush)
        .def("clear", &MessageView::clear)
        .def("beginStdioRedirect", &MessageView::beginStdioRedirect)
        .def("endStdioRedirect", &MessageView::endStdioRedirect)
        .def_static("isFlushing", &MessageView::isFlushing)
        .def_property_readonly_static("sigFlushFinished", &MessageView::sigFlushFinished)

        // deprecated
        .def_static("getInstance", &MessageView::instance, py::return_value_policy::reference)
        .def_static("getSigFlushFinished", &MessageView::sigFlushFinished)
        ;

    py::class_<SceneWidget, QWidget>(m, "SceneWidget")
        .def("draw", &SceneWidget::draw)
        .def_property_readonly("sigStateChanged", &SceneWidget::sigStateChanged)
        .def("setEditMode", &SceneWidget::setEditMode)
        .def_property_readonly("lastClickedPoint", &SceneWidget::lastClickedPoint)
        .def_property_readonly("builtinCameraTransform", &SceneWidget::builtinCameraTransform)
        .def_property("collisionLinesVisible", &SceneWidget::collisionLinesVisible, &SceneWidget::setCollisionLinesVisible)
        .def("setCollisionLinesVisible", &SceneWidget::setCollisionLinesVisible)
        .def("setHeadLightIntensity", &SceneWidget::setHeadLightIntensity)
        .def("setWorldLightIntensity", &SceneWidget::setWorldLightIntensity)
        .def("setWorldLightAmbient", &SceneWidget::setWorldLightAmbient)
        .def("setFloorGridSpan", &SceneWidget::setFloorGridSpan)
        .def("setFloorGridInterval", &SceneWidget::setFloorGridInterval)
        .def("setLineWidth", &SceneWidget::setLineWidth)
        .def("setPointSize", &SceneWidget::setPointSize)
        .def("setNormalLength", &SceneWidget::setNormalLength)
        .def("setHeadLightEnabled", &SceneWidget::setHeadLightEnabled)
        .def("setHeadLightLightingFromBack", &SceneWidget::setHeadLightLightingFromBack)
        .def("setWorldLight", &SceneWidget::setWorldLight)
        .def("setAdditionalLights", &SceneWidget::setAdditionalLights)
        .def("setFloorGrid", &SceneWidget::setFloorGrid)
        .def("setNormalVisualization", &SceneWidget::setNormalVisualization)
        .def("setCoordinateAxes", &SceneWidget::setCoordinateAxes)
        .def("setShowFPS", &SceneWidget::setShowFPS)
        .def("setBackgroundColor", &SceneWidget::setBackgroundColor)
        .def("setColor", &SceneWidget::setBackgroundColor)
        .def("setCameraPosition", &SceneWidget::setCameraPosition)
        .def("setFieldOfView", &SceneWidget::setFieldOfView)
        .def("setHeight", &SceneWidget::setHeight)
        .def("setNear", &SceneWidget::setNear)
        .def("setFar", &SceneWidget::setFar)

        // deprecated
        .def("getSigStateChanged", &SceneWidget::sigStateChanged)
        .def("getCollisionLinesVisible", &SceneWidget::collisionLinesVisible)
        ;

    py::class_<SceneView, View>(m, "SceneView")
        .def_property_readonly_static(
            "instance", [](py::object){ return SceneView::instance(); }, py::return_value_policy::reference)
        .def_property_readonly("sceneWidget", &SceneView::sceneWidget, py::return_value_policy::reference)

        // deprecated
        .def_static("getInstance", &SceneView::instance, py::return_value_policy::reference)
        .def("getSceneWidget", &SceneView::sceneWidget, py::return_value_policy::reference)
        ;

    py::class_<TaskView, View, AbstractTaskSequencer>(m, "TaskView")
        .def_property_readonly_static(
            "instance", [](py::object){ return TaskView::instance(); }, py::return_value_policy::reference)

        // deprecated
        .def_static("getInstance", &TaskView::instance, py::return_value_policy::reference)
        ;

    py::class_<ViewManager>(m, "ViewManager")
        .def_static("getOrCreateView",
                    [](const std::string& moduleName, const std::string& className){
                        return ViewManager::getOrCreateView(moduleName, className);
                    }, py::return_value_policy::reference)
        .def_property_readonly_static("sigViewCreated", &ViewManager::sigViewCreated)
        .def_property_readonly_static("sigViewActivated", &ViewManager::sigViewActivated)
        .def_property_readonly_static("sigViewDeactivated", &ViewManager::sigViewDeactivated)
        .def_property_readonly_static("sigViewRemoved", &ViewManager::sigViewRemoved)

        // deprecated
        .def_static("getSigViewCreated", &ViewManager::sigViewCreated)
        .def_static("getSigViewActivated", &ViewManager::sigViewActivated)
        .def_static("getSigViewDeactivated", &ViewManager::sigViewDeactivated)
        .def_static("getSigViewRemoved", &ViewManager::sigViewRemoved)
        ;
}

}
