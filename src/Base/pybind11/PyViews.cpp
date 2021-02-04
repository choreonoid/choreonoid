/*!
 * @author Shin'ichiro Nakaoka
*/

#include "PyQObjectHolder.h"
#include "PyQString.h"
#include "../MessageView.h"
#include "../SceneWidget.h"
#include "../SceneWidgetEvent.h"
#include "../SceneView.h"
#include "../InteractiveCameraTransform.h"
#include "../TaskView.h"
#include "../ViewManager.h"
#include "../Menu.h"
#include <cnoid/PyUtil>
#include <cnoid/SceneRenderer>
#include <cnoid/SceneCameras>
#include <QWidget>

using namespace cnoid;
namespace py = pybind11;

namespace cnoid {

void exportPyViews(py::module m)
{
    PySignal<void(View*)>(m, "ViewSignal");

    py::class_<View, PyQObjectHolder<View>, QWidget> view(m, "View");
    view
        .def_property("name", &View::name, &View::setName)
        .def("setName", &View::setName)
        .def("isActive", &View::isActive)
        .def("bringToFront", &View::bringToFront)
        .def_property_readonly("sigActivated", &View::sigActivated)
        .def_property_readonly("sigDeactivated", &View::sigDeactivated)
        .def_property("defaultLayoutArea", &View::defaultLayoutArea, &View::setDefaultLayoutArea)
        .def("setDefaultLayoutArea", &View::setDefaultLayoutArea)
        .def_property_readonly("indicatorOnInfoBar", &View::indicatorOnInfoBar)
        .def("enableFontSizeZoomKeys", &View::enableFontSizeZoomKeys)
        .def_property_readonly_static("lastFocusView", &View::lastFocusView)

        // deprecated
        .def("getName", &View::name)
        .def("getSigActivated", &View::sigActivated)
        .def("getSigDeactivated", &View::sigDeactivated)
        .def("getDefaultLayoutArea", &View::defaultLayoutArea)
        .def("getIndicatorOnInfoBar", &View::indicatorOnInfoBar)
        .def_static("getLastFocusView", &View::lastFocusView)
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

    py::class_<MessageView, PyQObjectHolder<MessageView>, View>(m, "MessageView")
        .def_property_readonly_static("instance", [](py::object){ return MessageView::instance(); })
        .def("put", (void (MessageView::*)(const std::string&, int)) &MessageView::put)
        .def("putln", (void (MessageView::*)(const std::string&, int)) &MessageView::putln)
        .def("notify", (void (MessageView::*)(const std::string&, int)) &MessageView::notify)
        .def("flush", &MessageView::flush)
        .def("clear", &MessageView::clear)
        .def("beginStdioRedirect", &MessageView::beginStdioRedirect)
        .def("endStdioRedirect", &MessageView::endStdioRedirect)
        ;

    m.def("showMessageBox", (void(*)(const std::string&)) &showMessageBox);
    m.def("showWarningDialog", (void(*)(const std::string&)) &showWarningDialog);
    m.def("showConfirmDialog", (bool(*)(const std::string&, const std::string&)) &showConfirmDialog);

    py::class_<SceneWidget, PyQObjectHolder<SceneWidget>, QWidget> sceneWidget(m, "SceneWidget");

    py::enum_<SceneWidget::ViewpointOperationMode>(sceneWidget, "ViewpointOperationMode")
        .value("ThirdPersonMode", SceneWidget::ThirdPersonMode)
        .value("FirstPersonMode", SceneWidget::FirstPersonMode)
        .export_values();

    py::enum_<SceneWidget::PolygonElement>(sceneWidget, "PolygonElement", py::arithmetic())
        .value("PolygonFace", SceneWidget::PolygonFace)
        .value("PolygonEdge", SceneWidget::PolygonEdge)
        .value("PolygonVertex", SceneWidget::PolygonVertex)
        .export_values();

    sceneWidget
        .def_property_readonly("sceneRoot", &SceneWidget::sceneRoot)
        .def_property_readonly("scene", &SceneWidget::scene)
        .def_property_readonly("systemNodeGroup", &SceneWidget::systemNodeGroup)
        .def_property_readonly("renderer", &SceneWidget::renderer)
        .def("renderScene", &SceneWidget::renderScene, py::arg("doImmediately") = false)
        .def_property_readonly("sigStateChanged", &SceneWidget::sigStateChanged)
        .def("setEditMode", &SceneWidget::setEditMode)
        .def_property_readonly("latestEvent", &SceneWidget::latestEvent)
        .def_property_readonly("lastClickedPoint", &SceneWidget::lastClickedPoint)
        .def("setViewpointOperationMode", &SceneWidget::setViewpointOperationMode)
        .def_property_readonly("viewpointOperationMode", &SceneWidget::viewpointOperationMode)
        .def_property_readonly("builtinCameraTransform", &SceneWidget::builtinCameraTransform)
        .def_property_readonly("builtinPerspectiveCamera", &SceneWidget::builtinPerspectiveCamera)
        .def_property_readonly("builtinOrthographicCamera", &SceneWidget::builtinOrthographicCamera)
        .def("isBuiltinCameraCurrent", &SceneWidget::isBuiltinCameraCurrent)
        .def("isBuiltinCamera", &SceneWidget::isBuiltinCamera)
        .def("findOwnerInteractiveCameraTransform", &SceneWidget::findOwnerInteractiveCameraTransform)
        .def("startBuiltinCameraViewChange", &SceneWidget::startBuiltinCameraViewChange)
        .def("rotateBuiltinCameraView", &SceneWidget::rotateBuiltinCameraView)
        .def("translateBuiltinCameraView", &SceneWidget::translateBuiltinCameraView)
        .def("unproject",
             [](SceneWidget& self, double x, double y, double z) -> py::object {
                 Vector3 projected;
                 if(self.unproject(x, y, z, projected)){
                     return py::cast(projected);
                 }
                 return py::cast(nullptr);
             })
        .def("viewAll", &SceneWidget::viewAll)
        .def("setVisiblePolygonElements", &SceneWidget::setVisiblePolygonElements)
        .def_property_readonly("visiblePolygonElements", &SceneWidget::visiblePolygonElements)
        .def("setHighlightingEnabled", &SceneWidget::setHighlightingEnabled)
        .def("isHighlightingEnabled", &SceneWidget::isHighlightingEnabled)
        .def_property_readonly("collisionLineVisibility", &SceneWidget::collisionLineVisibility)
        .def("setCollisionLineVisibility", &SceneWidget::setCollisionLineVisibility)
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
        .def_property_readonly("backgroundColor", &SceneWidget::backgroundColor)
        .def("setColor", &SceneWidget::setBackgroundColor)
        .def("setCameraPosition", &SceneWidget::setCameraPosition)
        .def("setFieldOfView", &SceneWidget::setFieldOfView)
        .def("setHeight", &SceneWidget::setHeight)
        .def("setNear", &SceneWidget::setNear)
        .def("setFar", &SceneWidget::setFar)
        .def("setSceneFocus", &SceneWidget::setSceneFocus)
        .def("setCursor", &SceneWidget::setCursor)
        .def("contextMenu", &SceneWidget::contextMenu, py::return_value_policy::reference)
        .def("showContextMenuAtPointerPosition", &SceneWidget::showContextMenuAtPointerPosition)
        .def_property_readonly("sigContextMenuRequest", &SceneWidget::sigContextMenuRequest)
        .def("showConfigDialog", &SceneWidget::showConfigDialog)
        .def_property_readonly("configDialogVBox", &SceneWidget::configDialogVBox)
        .def("saveImage", &SceneWidget::saveImage)
        .def("getImage", &SceneWidget::getImage)
        .def("setScreenSize", &SceneWidget::setScreenSize)
        .def("updateIndicator", &SceneWidget::updateIndicator)
        .def_property_readonly("indicator", &SceneWidget::indicator)
        .def_property_readonly("sigWidgetFocusChanged", &SceneWidget::sigWidgetFocusChanged)
        .def_property_readonly("sigAboutToBeDestroyed", &SceneWidget::sigAboutToBeDestroyed)
        ;

    py::class_<SceneView, PyQObjectHolder<SceneView>, View>(m, "SceneView")
        .def_property_readonly_static(
            "instance", [](py::object){ return releaseFromPythonSideManagement(SceneView::instance()); })
        .def_property_readonly("sceneWidget", &SceneView::sceneWidget)
        ;
    
    /*
      Although TaskView inherits AbstractTaskSequencer, AbstractTaskSequencer is not specified as a base class
      because its holder type is different and pybind11 cannot mix the different holder types.
      The functions defined in AbstractTaskSequencer must be independently defined in the following binding
      if a function included in it is used in a Python script.
    */
    py::class_<TaskView, PyQObjectHolder<TaskView>, View>(m, "TaskView", py::multiple_inheritance())
        .def_property_readonly_static(
            "instance", [](py::object){ return releaseFromPythonSideManagement(TaskView::instance()); })
        ;

    py::class_<ViewManager>(m, "ViewManager")
        .def_static(
            "getOrCreateView",
            [](const std::string& moduleName, const std::string& className){
                return releaseFromPythonSideManagement(ViewManager::getOrCreateView(moduleName, className));
            })
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
