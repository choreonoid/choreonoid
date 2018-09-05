/*!
 * @author Shin'ichiro Nakaoka
*/

#include "../MessageView.h"
#include "../SceneWidget.h"
#include "../SceneView.h"
#include "../TaskView.h"
#include "../ViewManager.h"
#include <cnoid/PySignal>
#include <QWidget>

using namespace boost::python;
using namespace cnoid;

// for MSVC++2015 Update3
CNOID_PYTHON_DEFINE_GET_POINTER(View)
CNOID_PYTHON_DEFINE_GET_POINTER(TaskView)
CNOID_PYTHON_DEFINE_GET_POINTER(SceneWidget)
CNOID_PYTHON_DEFINE_GET_POINTER(SceneView)
CNOID_PYTHON_DEFINE_GET_POINTER(MessageView)

namespace {

void (MessageView::*MessageView_put)(const std::string& message, int) = &MessageView::put;
void (MessageView::*MessageView_putln)(const std::string& message, int) = &MessageView::putln;
void (MessageView::*MessageView_notify)(const std::string& message, int) = &MessageView::notify;

View* ViewManager_getOrCreateView1(const std::string& moduleName, const std::string& className){
    return ViewManager::getOrCreateView(moduleName, className);
}

}

namespace cnoid {

template<> boost::python::object pyGetSignalArgObject(View*& view){
    return boost::python::object(boost::python::ptr(view));
}


void exportPyViews()
{
    PySignal<void(View*)>("ViewSignal");

    {
        scope viewScope = 
            class_<View, View*, bases<QWidget>, boost::noncopyable>("View", no_init)
            .def("setName", &View::setName)
            .def("name", &View::name)
            .def("getName", &View::name)
            .def("isActive", &View::isActive)
            .def("bringToFront", &View::bringToFront)
            .def("sigActivated", &View::sigActivated)
            .def("getSigActivated", &View::sigActivated)
            .def("sigDeactivated", &View::sigDeactivated)
            .def("getSigDeactivated", &View::sigDeactivated)
            .def("setDefaultLayoutArea", &View::setDefaultLayoutArea)
            .def("defaultLayoutArea", &View::defaultLayoutArea)
            .def("getDefaultLayoutArea", &View::defaultLayoutArea)
            .def("indicatorOnInfoBar", &View::indicatorOnInfoBar, return_value_policy<reference_existing_object>())
            .def("getIndicatorOnInfoBar", &View::indicatorOnInfoBar, return_value_policy<reference_existing_object>())
            .def("enableFontSizeZoomKeys", &View::enableFontSizeZoomKeys)
            .def("lastFocusView", &View::lastFocusView, return_value_policy<reference_existing_object>()).staticmethod("lastFocusView")
            .def("getLastFocusView", &View::lastFocusView, return_value_policy<reference_existing_object>()).staticmethod("getLastFocusView")
            .def("sigFocusChanged", &View::sigFocusChanged)
            .def("getSigFocusChanged", &View::sigFocusChanged)
            ;

        enum_<View::LayoutArea>("LayoutArea")
            .value("LEFT", View::LEFT) 
            .value("LEFT_TOP", View::LEFT_TOP)
            .value("LEFT_BOTTOM", View::LEFT_BOTTOM)
            .value("CENTER", View::CENTER)
            .value("RIGHT", View::RIGHT)
            .value("BOTTOM", View::BOTTOM)
            .value("NUM_AREAS", View::NUM_AREAS);
    }

    class_<MessageView, MessageView*, bases<View>, boost::noncopyable>("MessageView", no_init)
        .def("instance", &MessageView::instance, return_value_policy<reference_existing_object>()).staticmethod("instance")
        .def("getInstance", &MessageView::instance, return_value_policy<reference_existing_object>()).staticmethod("getInstance")
        .def("put", MessageView_put)
        .def("putln", MessageView_putln)
        .def("notify", MessageView_notify)
        .def("flush", &MessageView::flush)
        .def("clear", &MessageView::clear)
        .def("beginStdioRedirect", &MessageView::beginStdioRedirect)
        .def("endStdioRedirect", &MessageView::endStdioRedirect)
        .def("isFlushing", &MessageView::isFlushing).staticmethod("isFlushing")
        .def("sigFlushFinished", &MessageView::sigFlushFinished)
        ;

    class_<SceneWidget, SceneWidget*, boost::noncopyable >("SceneWidget")
        .def("sigStateChanged", &SceneWidget::sigStateChanged)
        .def("getSigStateChanged", &SceneWidget::sigStateChanged)
        .def("setEditMode", &SceneWidget::setEditMode)
        .def("setCollisionLinesVisible", &SceneWidget::setCollisionLinesVisible)
        .def("collisionLinesVisible", &SceneWidget::collisionLinesVisible, return_value_policy<return_by_value>())
        .def("getCollisionLinesVisible", &SceneWidget::collisionLinesVisible, return_value_policy<return_by_value>())
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

    class_<SceneView, SceneView*, bases<View>, boost::noncopyable>("SceneView", no_init)
        .def("instance", &SceneView::instance, return_value_policy<reference_existing_object>()).staticmethod("instance")
        .def("getInstance", &SceneView::instance, return_value_policy<reference_existing_object>()).staticmethod("getInstance")
        .def("sceneWidget", &SceneView::sceneWidget, return_value_policy<reference_existing_object>())
        .def("getSceneWidget", &SceneView::sceneWidget, return_value_policy<reference_existing_object>())
        ;

    class_<TaskView, TaskView*, bases<View, AbstractTaskSequencer>, boost::noncopyable>("TaskView", no_init)
        .def("instance", &TaskView::instance, return_value_policy<reference_existing_object>()).staticmethod("instance")
        .def("getInstance", &TaskView::instance, return_value_policy<reference_existing_object>()).staticmethod("getInstance")
        ;

    class_<ViewManager, boost::noncopyable>("ViewManager", no_init)
        .def("getOrCreateView", ViewManager_getOrCreateView1, return_value_policy<reference_existing_object>())
        .staticmethod("getOrCreateView")
        .def("sigViewCreated", &ViewManager::sigViewCreated).staticmethod("sigViewCreated")
        .def("getSigViewCreated", &ViewManager::sigViewCreated).staticmethod("getSigViewCreated")
        .def("sigViewActivated", &ViewManager::sigViewActivated).staticmethod("sigViewActivated")
        .def("getSigViewActivated", &ViewManager::sigViewActivated).staticmethod("getSigViewActivated")
        .def("sigViewDeactivated", &ViewManager::sigViewDeactivated).staticmethod("sigViewDeactivated")
        .def("getSigViewDeactivated", &ViewManager::sigViewDeactivated).staticmethod("getSigViewDeactivated")
        .def("sigViewRemoved", &ViewManager::sigViewRemoved).staticmethod("sigViewRemoved")
        .def("getSigViewRemoved", &ViewManager::sigViewRemoved).staticmethod("getSigViewRemoved")
        ;
}

}
