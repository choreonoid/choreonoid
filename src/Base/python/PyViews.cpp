#include "PyQString.h"
#include "PyQtTrampoline.h"
#include "../MessageView.h"
#include "../SceneWidget.h"
#include "../SceneWidgetEvent.h"
#include "../SceneView.h"
#include "../InteractiveCameraTransform.h"
#include "../TaskView.h"
#include "../ViewManager.h"
#include "../Archive.h"
#include "../Menu.h"
#include <nanobind/trampoline.h>
#include <nanobind/stl/string.h>
#include <cnoid/PyUtil>
#include <cnoid/PyEigenTypes>
#include <cnoid/PySignal>
#include <cnoid/SceneRenderer>
#include <cnoid/GLSceneRenderer>
#include <cnoid/SceneCameras>
#include <cnoid/SceneLights>
#include <QWidget>

using namespace cnoid;
namespace nb = nanobind;

namespace cnoid {

// Trampoline that lets a Python subclass override View's virtual functions.
// Heap-allocated by createQtTrampoline() (see PyQtTrampoline.h) so the overrides
// dispatch into Python while the object stays safely deletable by Qt.
class PyView : public View
{
public:
    NB_TRAMPOLINE(View, 5);

    void onActivated() override {
        NB_OVERRIDE(onActivated);
    }
    void onDeactivated() override {
        NB_OVERRIDE(onDeactivated);
    }
    void onFocusChanged(bool on) override {
        NB_OVERRIDE(onFocusChanged, on);
    }
    bool storeState(Archive& archive) override {
        NB_OVERRIDE(storeState, archive);
    }
    bool restoreState(const Archive& archive) override {
        NB_OVERRIDE(restoreState, archive);
    }
};

void exportPyViews(nb::module_ m)
{
    PySignal<void(View*)>(m, "ViewSignal");

    nb::class_<View, QWidget> view(m, "View");

    view
        // Construct a heap-allocated PyView trampoline so that a Python subclass
        // ("class MyView(View)") can override View's virtual functions. See
        // PyQtTrampoline.h and the ToolBar binding for the rationale.
        .def_static("__new__", [](nb::handle cls, nb::args, nb::kwargs){
            return python::createQtTrampoline<PyView>(cls); })
        // Accept the implicit __init__ call made on instantiation. The name is
        // assigned by the subclass via setName().
        .def("__init__", [](nb::handle, nb::args, nb::kwargs){})
        .def_prop_rw("name", &View::name, &View::setName)
        .def("setName", &View::setName)
        .def("isActive", &View::isActive)
        .def("bringToFront", &View::bringToFront)
        .def_prop_ro("sigActivated", &View::sigActivated)
        .def_prop_ro("sigDeactivated", &View::sigDeactivated)
        .def_prop_rw("defaultLayoutArea", &View::defaultLayoutArea, &View::setDefaultLayoutArea)
        .def("setDefaultLayoutArea", &View::setDefaultLayoutArea)
        .def_prop_ro("indicatorOnInfoBar", &View::indicatorOnInfoBar)
        .def("enableFontSizeZoomKeys", &View::enableFontSizeZoomKeys)
        .def_prop_ro_static("lastFocusView", [](nb::handle){ return View::lastFocusView(); }, nb::rv_policy::reference)
        ;

    nb::enum_<View::LayoutArea>(view, "LayoutArea", nb::is_arithmetic())
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

    nb::class_<MessageView, View> messageView(m, "MessageView");

    // Singleton views are obtained through their "instance" accessor and are
    // never constructed from Python; forbid the inherited View.__new__ so a
    // MessageView() call fails clearly instead of building a plain PyView.
    messageView.def_static("__new__", python::forbidConstruction("MessageView"));

    nb::enum_<MessageView::MessageType>(messageView, "MessageType", nb::is_arithmetic())
        .value("Normal", MessageView::Normal)
        .value("Error", MessageView::Error)
        .value("Warning", MessageView::Warning)
        .value("Highlight", MessageView::Highlight)
        .export_values();

    messageView
        .def_prop_ro_static("instance", [](nb::handle){ return MessageView::instance(); }, nb::rv_policy::reference)
        .def("put", (void (MessageView::*)(const std::string&, int)) &MessageView::put,
             nb::arg("message"), nb::arg("type") = (int)MessageView::Normal)
        .def("putln", (void (MessageView::*)(const std::string&, int)) &MessageView::putln,
             nb::arg("message"), nb::arg("type") = (int)MessageView::Normal)
        .def("notify", (void (MessageView::*)(const std::string&, int)) &MessageView::notify,
             nb::arg("message"), nb::arg("type") = (int)MessageView::Normal)
        .def("flush", &MessageView::flush)
        .def("clear", &MessageView::clear)
        .def("hasErrorMessages", &MessageView::hasErrorMessages)
        ;

    m.def("showMessageBox", (void(*)(const std::string&)) &showMessageBox);
    m.def("showWarningDialog", (bool(*)(const std::string&, bool)) &showWarningDialog);
    m.def("showWarningDialog", (bool(*)(const std::string&, const std::string&, bool)) &showWarningDialog);
    m.def("showConfirmDialog", (bool(*)(const std::string&, const std::string&)) &showConfirmDialog);

    nb::class_<SceneWidget, QWidget> sceneWidget(m, "SceneWidget");

    nb::enum_<SceneWidget::ViewpointOperationMode>(sceneWidget, "ViewpointOperationMode", nb::is_arithmetic())
        .value("ThirdPersonMode", SceneWidget::ThirdPersonMode)
        .value("FirstPersonMode", SceneWidget::FirstPersonMode)
        .export_values();

    nb::enum_<SceneWidget::PolygonElement>(sceneWidget, "PolygonElement", nb::is_arithmetic())
        .value("PolygonFace", SceneWidget::PolygonFace)
        .value("PolygonEdge", SceneWidget::PolygonEdge)
        .value("PolygonVertex", SceneWidget::PolygonVertex)
        .export_values();

    nb::enum_<SceneWidget::GridPlane>(sceneWidget, "GridPlane", nb::is_arithmetic())
        .value("XY_Grid", SceneWidget::XY_Grid)
        .value("XZ_Grid", SceneWidget::XZ_Grid)
        .value("YZ_Grid", SceneWidget::YZ_Grid)
        .export_values();

    sceneWidget
        .def_prop_ro("sceneRoot", &SceneWidget::sceneRoot)
        .def_prop_ro("scene", &SceneWidget::scene)
        .def_prop_ro("systemNodeGroup", &SceneWidget::systemNodeGroup)
        .def_prop_ro("renderer", [](SceneWidget& self){ return self.renderer(); }, nb::rv_policy::reference)
        .def("renderScene", &SceneWidget::renderScene, nb::arg("doImmediately") = false)
        .def_prop_ro("sigStateChanged", &SceneWidget::sigStateChanged)
        .def("setEditMode", &SceneWidget::setEditMode)
        .def_prop_ro("latestEvent", &SceneWidget::latestEvent, nb::rv_policy::reference)
        .def_prop_ro("lastClickedPoint", &SceneWidget::lastClickedPoint)
        .def("setViewpointOperationMode", &SceneWidget::setViewpointOperationMode)
        .def_prop_ro("viewpointOperationMode", &SceneWidget::viewpointOperationMode)
        .def_prop_ro("builtinCameraTransform", &SceneWidget::builtinCameraTransform)
        .def_prop_ro("builtinPerspectiveCamera", &SceneWidget::builtinPerspectiveCamera)
        .def_prop_ro("builtinOrthographicCamera", &SceneWidget::builtinOrthographicCamera)
        .def("isBuiltinCameraCurrent", &SceneWidget::isBuiltinCameraCurrent)
        .def("isBuiltinCamera", &SceneWidget::isBuiltinCamera)
        .def("findOwnerInteractiveCameraTransform", &SceneWidget::findOwnerInteractiveCameraTransform)
        .def("setCameraPosition",
             [](SceneWidget& self, const python::Vector3Arg& position, double transitionTime){
                 self.setCameraPosition(position.value, transitionTime); },
             nb::arg("position"), nb::arg("transitionTime") = 0.0)
        .def("setCameraPositionLookingFor",
             [](SceneWidget& self, const python::Vector3Arg& eye, const python::Vector3Arg& direction,
                const python::Vector3Arg& up, double transitionTime){
                 self.setCameraPositionLookingFor(eye.value, direction.value, up.value, transitionTime); },
             nb::arg("eye"), nb::arg("direction"), nb::arg("up"), nb::arg("transitionTime") = 0.0)
        .def("setCameraPositionLookingAt",
             [](SceneWidget& self, const python::Vector3Arg& eye, const python::Vector3Arg& center,
                const python::Vector3Arg& up, double transitionTime){
                 self.setCameraPositionLookingAt(eye.value, center.value, up.value, transitionTime); },
             nb::arg("eye"), nb::arg("center"), nb::arg("up"), nb::arg("transitionTime") = 0.0)
        .def("startBuiltinCameraViewChange",
             [](SceneWidget& self, const python::Vector3Arg& center){ self.startBuiltinCameraViewChange(center.value); })
        .def("rotateBuiltinCameraView", &SceneWidget::rotateBuiltinCameraView)
        .def("translateBuiltinCameraView",
             [](SceneWidget& self, const python::Vector3Arg& dp){ self.translateBuiltinCameraView(dp.value); })
        .def("unproject",
             [](SceneWidget& self, double x, double y, double z) -> nb::object {
                 Vector3 projected;
                 if(self.unproject(x, y, z, projected)){
                     return nb::cast(projected);
                 }
                 return nb::none();
             })
        .def("fitViewToAll", &SceneWidget::fitViewToAll)
        .def("setVisiblePolygonElements", &SceneWidget::setVisiblePolygonElements)
        .def_prop_ro("visiblePolygonElements", &SceneWidget::visiblePolygonElements)
        .def("setHighlightingEnabled", &SceneWidget::setHighlightingEnabled)
        .def("isHighlightingEnabled", &SceneWidget::isHighlightingEnabled)
        .def_prop_ro("collisionLineVisibility", &SceneWidget::collisionLineVisibility)
        .def("setCollisionLineVisibility", &SceneWidget::setCollisionLineVisibility)
        .def("setHeadLightIntensity", &SceneWidget::setHeadLightIntensity)
        .def("setLineWidth", &SceneWidget::setLineWidth)
        .def("setPointSize", &SceneWidget::setPointSize)
        .def("setHeadLightEnabled", &SceneWidget::setHeadLightEnabled)
        .def("setHeadLightLightingFromBack", &SceneWidget::setHeadLightLightingFromBack)
        .def("setAdditionalLights", &SceneWidget::setAdditionalLights)
        .def("setCoordinateAxes", &SceneWidget::setCoordinateAxes)
        .def("setShowFPS", &SceneWidget::setShowFPS)
        .def("setBackgroundColor", [](SceneWidget& self, const python::Vector3Arg& c){ self.setBackgroundColor(c.value); })
        .def_prop_ro("backgroundColor", &SceneWidget::backgroundColor)
        .def("setColor", [](SceneWidget& self, const python::Vector3Arg& c){ self.setBackgroundColor(c.value); })
        .def("setFieldOfView", &SceneWidget::setFieldOfView)
        .def("setHeight", &SceneWidget::setHeight)
        .def("setClipDistances", &SceneWidget::setClipDistances)
        .def("setGridEnabled", &SceneWidget::setGridEnabled)
        .def("isGridEnabled", &SceneWidget::isGridEnabled)
        .def("setGridGeometry", &SceneWidget::setGridGeometry)
        .def("setGridColor",
             [](SceneWidget& self, SceneWidget::GridPlane plane, const python::Vector3fArg& color){
                 self.setGridColor(plane, color.value); })
        .def("updateGrids", &SceneWidget::updateGrids)
        .def("setSceneFocus", &SceneWidget::setSceneFocus)
        .def("setCursor", &SceneWidget::setCursor)
        .def("contextMenu", &SceneWidget::contextMenu, nb::rv_policy::reference)
        .def("showContextMenuAtPointerPosition", &SceneWidget::showContextMenuAtPointerPosition)
        .def_prop_ro("sigContextMenuRequest", &SceneWidget::sigContextMenuRequest)
        .def("saveImage", &SceneWidget::saveImage)
        .def("getImage", &SceneWidget::getImage)
        .def("setScreenSize", &SceneWidget::setScreenSize)
        .def("updateIndicator", &SceneWidget::updateIndicator)
        .def_prop_ro("indicator", &SceneWidget::indicator, nb::rv_policy::reference)
        .def_prop_ro("sigWidgetFocusChanged", &SceneWidget::sigWidgetFocusChanged)
        .def_prop_ro("sigAboutToBeDestroyed", &SceneWidget::sigAboutToBeDestroyed)
        ;

    nb::class_<SceneView, View> sceneView(m, "SceneView");
    sceneView.def_static("__new__", python::forbidConstruction("SceneView"));
    sceneView
        .def_prop_ro_static(
            "instance", [](nb::handle){ return SceneView::instance(); }, nb::rv_policy::reference)
        .def_prop_ro("sceneWidget", &SceneView::sceneWidget, nb::rv_policy::reference)
        ;

    /*
      Although TaskView inherits AbstractTaskSequencer, AbstractTaskSequencer is
      not specified as a base class. AbstractTaskSequencer is a non-first base of
      TaskView (multiple inheritance with an offset), which nanobind does not
      support as a registered base; the functions defined in it are therefore
      redefined here. (The pybind11 version did the same for a different reason -
      mismatched holder types.)
    */
    nb::class_<TaskView, View> taskView(m, "TaskView");
    taskView.def_static("__new__", python::forbidConstruction("TaskView"));
    taskView
        .def_prop_ro_static(
            "instance", [](nb::handle){ return TaskView::instance(); }, nb::rv_policy::reference)

        // Virtual functions defined in AbstractTaskSequencer
        .def("activate", &TaskView::activate, nb::arg("on") = true)
        .def("isActive", &TaskView::isActive)
        .def("addTask", &TaskView::addTask)
        .def("updateTask", &TaskView::updateTask)
        .def("removeTask", &TaskView::removeTask)
        .def_prop_ro("sigTaskAdded", &TaskView::sigTaskAdded)
        .def_prop_ro("sigTaskRemoved", &TaskView::sigTaskRemoved)
        .def("clearTasks", &TaskView::clearTasks)
        .def_prop_ro("numTasks", &TaskView::numTasks)
        .def("getTask", &TaskView::task)
        .def_prop_ro("currentTaskIndex", &TaskView::currentTaskIndex)
        .def("setCurrentTask", &TaskView::setCurrentTask)
        .def_prop_ro("sigCurrentTaskChanged", &TaskView::sigCurrentTaskChanged)
        .def_prop_ro("currentPhaseIndex", &TaskView::currentPhaseIndex)
        .def("setCurrentPhase", &TaskView::setCurrentPhase)
        .def_prop_ro("sigCurrentPhaseChanged", &TaskView::sigCurrentPhaseChanged)
        .def_prop_ro("currentCommandIndex", &TaskView::currentCommandIndex)
        .def("executeCommand", &TaskView::executeCommand)
        .def_prop_ro("sigCurrentCommandChanged", &TaskView::sigCurrentCommandChanged)
        .def("isBusy", &TaskView::isBusy)
        .def_prop_ro("sigBusyStateChanged", &TaskView::sigBusyStateChanged)
        .def("cancelCurrentCommand", &TaskView::cancelCurrentCommand)
        .def_prop_ro("sigCurrentCommandCanceled", &TaskView::sigCurrentCommandCanceled)
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
        .def_prop_ro("menuItemCheckStates", &TaskView::menuItemCheckStates)
        .def_prop_ro("sigMenuRequest", &TaskView::sigMenuRequest)
        .def("showMenu", &TaskView::showMenu)
        .def_prop_ro("sigMenuItemTriggered", &TaskView::sigMenuItemTriggered)
        .def_prop_ro("sigMenuItemToggled", &TaskView::sigMenuItemToggled)
        ;

    nb::class_<ViewManager>(m, "ViewManager")
        .def_static(
            "getOrCreateView",
            [](const std::string& moduleName, const std::string& className){
                return ViewManager::getOrCreateView(moduleName, className);
            }, nb::rv_policy::reference)
        .def_prop_ro_static("sigViewCreated", [](nb::handle){ return ViewManager::sigViewCreated(); })
        .def_prop_ro_static("sigViewActivated", [](nb::handle){ return ViewManager::sigViewActivated(); })
        .def_prop_ro_static("sigViewDeactivated", [](nb::handle){ return ViewManager::sigViewDeactivated(); })
        .def_prop_ro_static("sigViewRemoved", [](nb::handle){ return ViewManager::sigViewRemoved(); })
        ;
}

}
