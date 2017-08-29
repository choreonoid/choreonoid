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
        .def("setName", &View::setName)
        .def("name", &View::name)
        .def("isActive", &View::isActive)
        .def("bringToFront", &View::bringToFront)
        .def("sigActivated", &View::sigActivated)
        .def("sigDeactivated", &View::sigDeactivated)
        .def("setDefaultLayoutArea", &View::setDefaultLayoutArea)
        .def("defaultLayoutArea", &View::defaultLayoutArea)
        .def("indicatorOnInfoBar", &View::indicatorOnInfoBar, py::return_value_policy::reference)
        .def("enableFontSizeZoomKeys", &View::enableFontSizeZoomKeys)
        .def_static("lastFocusView", &View::lastFocusView, py::return_value_policy::reference)
        .def("sigFocusChanged", &View::sigFocusChanged)
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
        .def_static("instance", &MessageView::instance, py::return_value_policy::reference)
        .def("put", (void (MessageView::*)(const std::string&)) &MessageView::put)
        .def("putln", (void (MessageView::*)(const std::string&)) &MessageView::putln)
        .def("notify", (void (MessageView::*)(const std::string&)) &MessageView::notify)
        .def("flush", &MessageView::flush)
        .def("clear", &MessageView::clear)
        .def("beginStdioRedirect", &MessageView::beginStdioRedirect)
        .def("endStdioRedirect", &MessageView::endStdioRedirect)
        .def_static("isFlushing", &MessageView::isFlushing)
        .def("sigFlushFinished", &MessageView::sigFlushFinished)
        ;

    py::class_<SceneWidget, QWidget>(m, "SceneWidget")
        .def("sigStateChanged", &SceneWidget::sigStateChanged)
        .def("setEditMode", &SceneWidget::setEditMode)
        .def("setCollisionLinesVisible", &SceneWidget::setCollisionLinesVisible)
        .def("collisionLinesVisible", &SceneWidget::collisionLinesVisible)
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
        .def("setNewDisplayListDoubleRenderingEnabled", &SceneWidget::setNewDisplayListDoubleRenderingEnabled)
        .def("setUseBufferForPicking", &SceneWidget::setUseBufferForPicking)
        .def("setBackgroundColor", &SceneWidget::setBackgroundColor)
        .def("setColor", &SceneWidget::setBackgroundColor)
        .def("setCameraPosition", &SceneWidget::setCameraPosition)
        .def("setFieldOfView", &SceneWidget::setFieldOfView)
        .def("setHeight", &SceneWidget::setHeight)
        .def("setNear", &SceneWidget::setNear)
        .def("setFar", &SceneWidget::setFar)
        ;

    py::class_<SceneView, View>(m, "SceneView")
        .def_static("instance", &SceneView::instance, py::return_value_policy::reference)
        .def("sceneWidget", &SceneView::sceneWidget, py::return_value_policy::reference)
        ;

    py::class_<TaskView, View, AbstractTaskSequencer>(m, "TaskView")
        .def_static("instance", &TaskView::instance, py::return_value_policy::reference)
        ;

    py::class_<ViewManager>(m, "ViewManager")
        .def_static("getOrCreateView",
                    [](const std::string& moduleName, const std::string& className){
                        return ViewManager::getOrCreateView(moduleName, className);
                    }, py::return_value_policy::reference)
        .def_static("sigViewCreated", &ViewManager::sigViewCreated)
        .def_static("sigViewActivated", &ViewManager::sigViewActivated)
        .def_static("sigViewDeactivated", &ViewManager::sigViewDeactivated)
        .def_static("sigViewRemoved", &ViewManager::sigViewRemoved)
        ;
}

}
