/*!
 * @author Shin'ichiro Nakaoka
*/

#include "PyQObjectHolder.h"
#include "PyQString.h"
#include "../MessageView.h"
#include "../SceneWidget.h"
#include "../SceneWidgetEvent.h"
#include "../SceneView.h"
#include "../GLSceneRenderer.h"
#include "../InteractiveCameraTransform.h"
#include "../TaskView.h"
#include "../ViewManager.h"
#include "../Menu.h"
#include <cnoid/PyUtil>
#include <cnoid/SceneRenderer>
#include <cnoid/SceneCameras>
#include <cnoid/SceneLights>
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
        .def_property_readonly_static("lastFocusView", [](py::object){ return View::lastFocusView(); })

        // deprecated
        .def("getName", &View::name)
        .def("getSigActivated", &View::sigActivated)
        .def("getSigDeactivated", &View::sigDeactivated)
        .def("getDefaultLayoutArea", &View::defaultLayoutArea)
        .def("getIndicatorOnInfoBar", &View::indicatorOnInfoBar)
        .def_static("getLastFocusView", &View::lastFocusView)
        ;

    py::enum_<View::LayoutArea>(view, "LayoutArea")
        .value("TopLeftArea", View::TopLeftArea)
        .value("MiddleLeftArea", View::MiddleLeftArea)
        .value("BottomLeftArea", View::BottomLeftArea)
        .value("TopCenterArea", View::TopCenterArea)
        .value("CenterArea", View::CenterArea)
        .value("BottomCenterArea", View::BottomCenterArea)
        .value("TopRightArea", View::TopRightArea)
        .value("MiddleRightArea", View::MiddleRightArea)
        .value("BottomRightArea", View::BottomRightArea)
        .value("NumLayoutAreas", View::NumLayoutAreas)
        .export_values();

    py::class_<MessageView, PyQObjectHolder<MessageView>, View> messageView(m, "MessageView");

    py::enum_<MessageView::MessageType>(messageView, "MessageType")
        .value("Normal", MessageView::Normal)
        .value("Error", MessageView::Error)
        .value("Warning", MessageView::Warning)
        .value("Highlight", MessageView::Highlight)
        .export_values();

    messageView
        .def_property_readonly_static("instance", [](py::object){ return MessageView::instance(); })
        .def("put", (void (MessageView::*)(const std::string&, int)) &MessageView::put,
             py::arg("message"), py::arg("type") = MessageView::Normal)
        .def("putln", (void (MessageView::*)(const std::string&, int)) &MessageView::putln,
             py::arg("message"), py::arg("type") = MessageView::Normal)
        .def("notify", (void (MessageView::*)(const std::string&, int)) &MessageView::notify,
             py::arg("message"), py::arg("type") = MessageView::Normal)
        .def("flush", &MessageView::flush)
        .def("clear", &MessageView::clear)
        .def("hasErrorMessages", &MessageView::hasErrorMessages)
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

    py::enum_<SceneWidget::GridPlane>(sceneWidget, "GridPlane", py::arithmetic())
        .value("XY_Grid", SceneWidget::XY_Grid)
        .value("XZ_Grid", SceneWidget::XZ_Grid)
        .value("YZ_Grid", SceneWidget::YZ_Grid)
        .export_values();
    
    sceneWidget
        .def_property_readonly("sceneRoot", &SceneWidget::sceneRoot)
        .def_property_readonly("scene", &SceneWidget::scene)
        .def_property_readonly("systemNodeGroup", &SceneWidget::systemNodeGroup)
        .def_property_readonly("renderer", [](SceneWidget& self){ return self.renderer(); })
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
        .def("setLineWidth", &SceneWidget::setLineWidth)
        .def("setPointSize", &SceneWidget::setPointSize)
        .def("setHeadLightEnabled", &SceneWidget::setHeadLightEnabled)
        .def("setHeadLightLightingFromBack", &SceneWidget::setHeadLightLightingFromBack)
        .def("setAdditionalLights", &SceneWidget::setAdditionalLights)
        .def("setCoordinateAxes", &SceneWidget::setCoordinateAxes)
        .def("setShowFPS", &SceneWidget::setShowFPS)
        .def("setBackgroundColor", &SceneWidget::setBackgroundColor)
        .def_property_readonly("backgroundColor", &SceneWidget::backgroundColor)
        .def("setColor", &SceneWidget::setBackgroundColor)
        .def("setCameraPosition", &SceneWidget::setCameraPosition)
        .def("setFieldOfView", &SceneWidget::setFieldOfView)
        .def("setHeight", &SceneWidget::setHeight)
        .def("setClipDistances", &SceneWidget::setClipDistances)
        .def("setGridEnabled", &SceneWidget::setGridEnabled)
        .def("setSceneFocus", &SceneWidget::setSceneFocus)
        .def("setCursor", &SceneWidget::setCursor)
        .def("contextMenu", &SceneWidget::contextMenu, py::return_value_policy::reference)
        .def("showContextMenuAtPointerPosition", &SceneWidget::showContextMenuAtPointerPosition)
        .def_property_readonly("sigContextMenuRequest", &SceneWidget::sigContextMenuRequest)
        .def("saveImage", &SceneWidget::saveImage)
        .def("getImage", &SceneWidget::getImage)
        .def("setScreenSize", &SceneWidget::setScreenSize)
        .def("updateIndicator", &SceneWidget::updateIndicator)
        .def_property_readonly("indicator", &SceneWidget::indicator)
        .def_property_readonly("sigWidgetFocusChanged", &SceneWidget::sigWidgetFocusChanged)
        .def_property_readonly("sigAboutToBeDestroyed", &SceneWidget::sigAboutToBeDestroyed)

        // deprecated
        .def("setWorldLight", [](SceneWidget&){ })
        .def("setWorldLightEnabled", [](SceneWidget&){ })
        .def("setWorldLightIntensity", [](SceneWidget&, double){ })
        .def("setWorldLightAmbient",
             [](SceneWidget& sceneWidget, double ambient){
                 sceneWidget.renderer()->headLight()->setAmbientIntensity(ambient);
             })
        .def("setFloorGridEnabled",
             [](SceneWidget& widget, bool on){
                 widget.setGridEnabled(SceneWidget::XY_Grid, on);
             })
        .def("setNormalVisualization",
             [](SceneWidget& self, bool on){
                 if(auto renderer = self.renderer<GLSceneRenderer>()){
                     renderer->setNormalVisualizationEnabled(on);
                 }
             })
        .def("setNormalLength",
             [](SceneWidget& self, double length){
                 if(auto renderer = self.renderer<GLSceneRenderer>()){
                     renderer->setNormalVisualizationLength(length);
                 }
             })
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

        // Virtual functions defined in AbstractTaskSequencer
        .def("activate", &TaskView::activate, py::arg("on") = true)
        .def("isActive", &TaskView::isActive)
        .def("addTask", &TaskView::addTask)
        .def("updateTask", &TaskView::updateTask)
        .def("removeTask", &TaskView::removeTask)
        .def_property_readonly("sigTaskAdded", &TaskView::sigTaskAdded)
        .def_property_readonly("sigTaskRemoved", &TaskView::sigTaskRemoved)
        .def("clearTasks", &TaskView::clearTasks)
        .def_property_readonly("numTasks", &TaskView::numTasks)
        .def("getTask", &TaskView::task)
        .def_property_readonly("currentTaskIndex", &TaskView::currentTaskIndex)
        .def("setCurrentTask", &TaskView::setCurrentTask)
        .def_property_readonly("sigCurrentTaskChanged", &TaskView::sigCurrentTaskChanged)
        .def_property_readonly("currentPhaseIndex", &TaskView::currentPhaseIndex)
        .def("setCurrentPhase", &TaskView::setCurrentPhase)
        .def_property_readonly("sigCurrentPhaseChanged", &TaskView::sigCurrentPhaseChanged)
        .def_property_readonly("currentCommandIndex", &TaskView::currentCommandIndex)
        .def("executeCommand", &TaskView::executeCommand)
        .def_property_readonly("sigCurrentCommandChanged", &TaskView::sigCurrentCommandChanged)
        .def("isBusy", &TaskView::isBusy)
        .def_property_readonly("sigBusyStateChanged", &TaskView::sigBusyStateChanged)
        .def("cancelCurrentCommand", &TaskView::cancelCurrentCommand)
        .def_property_readonly("sigCurrentCommandCanceled", &TaskView::sigCurrentCommandCanceled)
        .def("isAutoMode", &TaskView::isAutoMode)
        .def("setAutoMode", &TaskView::setAutoMode)
        .def("sigAutoModeToggled", &TaskView::sigAutoModeToggled)
        .def("serializeTasks", &TaskView::serializeTasks)

        // Non-virtual functions defined in TaskView
        .def("setNoExecutionMode", &TaskView::setNoExecutionMode)
        .def("isNoExecutionMode", &TaskView::isNoExecutionMode)
        .def("setCurrentCommand", &TaskView::setCurrentCommand)
        .def("setBusyState", &TaskView::setBusyState)
        .def("executeMenuItem", &TaskView::executeMenuItem)
        .def("checkMenuItem", &TaskView::checkMenuItem)
        .def_property_readonly("menuItemCheckStates", &TaskView::menuItemCheckStates)
        .def_property_readonly("sigMenuRequest", &TaskView::sigMenuRequest)
        .def("showMenu", &TaskView::showMenu)
        .def_property_readonly("sigMenuItemTriggered", &TaskView::sigMenuItemTriggered)
        .def_property_readonly("sigMenuItemToggled", &TaskView::sigMenuItemToggled)
        ;

    py::class_<ViewManager>(m, "ViewManager")
        .def_static(
            "getOrCreateView",
            [](const std::string& moduleName, const std::string& className){
                return releaseFromPythonSideManagement(ViewManager::getOrCreateView(moduleName, className));
            })
        .def_property_readonly_static("sigViewCreated", [](py::object){ return ViewManager::sigViewCreated(); })
        .def_property_readonly_static("sigViewActivated", [](py::object){ return ViewManager::sigViewActivated(); })
        .def_property_readonly_static("sigViewDeactivated", [](py::object){ return ViewManager::sigViewDeactivated(); })
        .def_property_readonly_static("sigViewRemoved", [](py::object){ return ViewManager::sigViewRemoved(); })

        // deprecated
        .def_static("getSigViewCreated", &ViewManager::sigViewCreated)
        .def_static("getSigViewActivated", &ViewManager::sigViewActivated)
        .def_static("getSigViewDeactivated", &ViewManager::sigViewDeactivated)
        .def_static("getSigViewRemoved", &ViewManager::sigViewRemoved)
        ;
}

}
