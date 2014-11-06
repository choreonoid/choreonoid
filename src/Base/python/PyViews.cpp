/*!
 * @author Shin'ichiro Nakaoka
*/

#include "PyBase.h"
#include <cnoid/View>
#include <cnoid/MessageView>
#include <cnoid/ItemTreeView>
#include <cnoid/SceneWidget>
#include <cnoid/SceneView>

using namespace boost::python;
using namespace cnoid;

namespace {

class PyItemTreeViewVisitor : public def_visitor<PyItemTreeViewVisitor>
{
    friend class def_visitor_access;
    
public:
    template <class classT>
    void visit(classT& c) const {
        c
            .def("isItemSelected", &PyItemTreeViewVisitor::isItemSelected, return_value_policy<return_by_value>())
            .def("selectItem", &PyItemTreeViewVisitor::selectItem, (args("item"), args("select") = true))
            .def("isItemChecked", &PyItemTreeViewVisitor::isItemChecked, return_value_policy<return_by_value>())
            .def("checkItem", &PyItemTreeViewVisitor::checkItem, (args("item"), args("check") = true))
            ;
    }

    // For interface of ItemTreeView (External interface for python)
    static bool isItemSelected(ItemTreeView& self, Item* item) {
        return self.isItemSelected(ItemPtr(item));
    }
    static bool selectItem(ItemTreeView& self, Item* item, bool select = true) {
        return self.selectItem(item, select);
    }
    static bool isItemChecked(ItemTreeView& self, Item* item) {
        return self.isItemChecked(item);
    }
    static bool checkItem(ItemTreeView& self, Item* item, bool check = true) {
        return self.checkItem(item, check);
    }
};


void exportViews()
{
    class_<View, boost::noncopyable>("View", init<>())
        .def("isActive", &View::isActive, return_value_policy<return_by_value>())
        .def("setName", &View::setName)                                   // ...(*1)
        .def("name", &View::name, return_value_policy<return_by_value>()) // ...(*1)
        .def("defaultLayoutArea", &View::defaultLayoutArea, return_value_policy<return_by_value>()) // ... (*2)
        .def("setDefaultLayoutArea", &View::setDefaultLayoutArea)
        .def("enableFontSizeZoomKeys", &View::enableFontSizeZoomKeys)
        .def("lastFocusView", &View::lastFocusView, return_value_policy<reference_existing_object>()).staticmethod("lastFocusView");


    class_<ItemTreeView, bases<View>, boost::noncopyable>("ItemTreeView", no_init)
        .def(PyItemTreeViewVisitor())
        .def("instance", &ItemTreeView::instance, return_value_policy<reference_existing_object>()).staticmethod("instance")
        .def("selectAllItems", &ItemTreeView::selectAllItems)
        .def("clearSelection", &ItemTreeView::clearSelection);

    
    /*!
     * @brief Reference of the output of the message-view.
     * @note  BOOST_PYTHON_FUNCTION_OVERLOADS is available only if the first argument matches.
     *        Otherwise, we must define the wrapped manually.
     */
    void (MessageView::*put)(const std::string& message) = &MessageView::put;
    void (MessageView::*putln)(const std::string& message) = &MessageView::putln;
    void (MessageView::*notify)(const std::string& message) = &MessageView::notify;
 
    class_<MessageView, bases<View>, boost::noncopyable>("MessageView", no_init)
        .def("instance", &MessageView::instance, return_value_policy<reference_existing_object>()).staticmethod("instance")
        .def("put", put)
        .def("putln", putln)
        .def("notify", notify)
        .def("flush", &MessageView::flush)
        .def("clear", &MessageView::clear)
        .def("beginStdioRedirect", &MessageView::beginStdioRedirect)
        .def("endStdioRedirect", &MessageView::endStdioRedirect)
        .def("isFlushing", &MessageView::isFlushing).staticmethod("isFlushing");
 
    class_<SceneWidget, boost::noncopyable >("SceneWidget", no_init)        
        .def("setHeadLightIntensity", &SceneWidget::setHeadLightIntensity)
        .def("setWorldLightIntensity", &SceneWidget::setWorldLightIntensity)
        .def("setWorldLightAmbient", &SceneWidget::setWorldLightAmbient)
        .def("setFloorGridSpan", &SceneWidget::setFloorGridSpan)
        .def("setFloorGridInterval", &SceneWidget::setFloorGridInterval)
        .def("setLineWidth", &SceneWidget::setLineWidth)
        .def("setPointSize", &SceneWidget::setPointSize)
        .def("setNormalLength", &SceneWidget::setNormalLength)
        .def("setCollisionLinesVisible", &SceneWidget::setCollisionLinesVisible)
        .def("collisionLinesVisible", &SceneWidget::collisionLinesVisible, return_value_policy<return_by_value>())
        .def("setHeadLightEnabled", &SceneWidget::setHeadLightEnabled)
        .def("setHeadLightLightingFromBack", &SceneWidget::setHeadLightLightingFromBack)
        .def("setWorldLight", &SceneWidget::setWorldLight)
        .def("setAdditionalLights", &SceneWidget::setAdditionalLights)
        .def("setFloorGrid", &SceneWidget::setFloorGrid)
        .def("setNormalVisualization", &SceneWidget::setNormalVisualization)
        .def("setCoordinateAxes", &SceneWidget::setCoordinateAxes)
        .def("setShowFPS", &SceneWidget::setShowFPS)
        .def("setUseBufferForPicking", &SceneWidget::setUseBufferForPicking)
        .def("setBackgroundColor", &SceneWidget::setBackgroundColor)
        .def("setCameraPosition", &SceneWidget::setCameraPosition)
        .def("setFieldOfView", &SceneWidget::setFieldOfView)
        .def("setHeight", &SceneWidget::setHeight)
        .def("setNear", &SceneWidget::setNear)
        .def("setFar", &SceneWidget::setFar);
    
    /*!
     * @brief Provides a reference the SceneView for operations are add.
     */
    class_<SceneView, bases<View>, boost::noncopyable>("SceneView", no_init)
        .def("instance", &SceneView::instance, return_value_policy<reference_existing_object>()).staticmethod("instance")
        .def("sceneWidget", &SceneView::sceneWidget, return_value_policy<reference_existing_object>());


    /*!
     * @brief Provides following enum value of the View.
     */
    enum_ <View::LayoutArea>("LayoutArea")
        .value("LEFT", View::LEFT) 
        .value("LEFT_TOP", View::LEFT_TOP)
        .value("LEFT_BOTTOM", View::LEFT_BOTTOM)
        .value("CENTER", View::CENTER)
        .value("RIGHT", View::RIGHT)
        .value("BOTTOM", View::BOTTOM)
        .value("NUM_AREAS", View::NUM_AREAS);
    
}

