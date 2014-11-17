/*!
 * @author Shin'ichiro Nakaoka
*/

#include "../View.h"
#include "../MessageView.h"
#include "../ItemTreeView.h"
#include "../SceneWidget.h"
#include "../SceneView.h"
#include "../RootItem.h"
#include <cnoid/PySignal>

using namespace boost::python;
using namespace cnoid;

namespace {

void (MessageView::*MessageView_put)(const std::string& message) = &MessageView::put;
void (MessageView::*MessageView_putln)(const std::string& message) = &MessageView::putln;
void (MessageView::*MessageView_notify)(const std::string& message) = &MessageView::notify;
 
RootItemPtr ItemTreeView_rootItem(ItemTreeView& self) { return self.rootItem(); }
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ItemTreeView_selectItem_overloads, selectItem, 1, 2)
ItemList<Item> ItemTreeView_checkedItems(ItemTreeView& self, int id = 0) { return self.checkedItems<Item>(id); }
BOOST_PYTHON_FUNCTION_OVERLOADS(ItemTreeView_checkedItems_overloads, ItemTreeView_checkedItems, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ItemTreeView_isItemChecked_overloads, isItemChecked, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ItemTreeView_checkItem_overloads, checkItem, 1, 3)
SignalProxy<void(Item*, bool)> (ItemTreeView::*ItemTreeView_sigCheckToggled1)(int) = &ItemTreeView::sigCheckToggled;
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ItemTreeView_sigCheckToggled1_overloads, sigCheckToggled, 0, 1)
SignalProxy<void(bool)> (ItemTreeView::*ItemTreeView_sigCheckToggled2)(Item*, int) = &ItemTreeView::sigCheckToggled;
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ItemTreeView_sigCheckToggled2_overloads, sigCheckToggled, 1, 2)

}

namespace cnoid {

void exportViews()
{
    PySignalProxy<void(View*)>("ViewSignal");

    {
        scope viewScope = 
            class_<View, View*, boost::noncopyable>("View", no_init)
            .def("setName", &View::setName)
            .def("name", &View::name)
            .def("isActive", &View::isActive)
            .def("bringToFront", &View::bringToFront)
            .def("sigActivated", &View::sigActivated)
            .def("sigDeactivated", &View::sigDeactivated)
            .def("setDefaultLayoutArea", &View::setDefaultLayoutArea)
            .def("defaultLayoutArea", &View::defaultLayoutArea)
            .def("indicatorOnInfoBar", &View::indicatorOnInfoBar, return_value_policy<reference_existing_object>())
            .def("enableFontSizeZoomKeys", &View::enableFontSizeZoomKeys)
            .def("lastFocusView", &View::lastFocusView, return_value_policy<reference_existing_object>()).staticmethod("lastFocusView")
            .def("sigFocusChanged", &View::sigFocusChanged)
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

    PySignalProxy<void(const ItemList<>&)>("ItemListSignal");
    PySignalProxy<void(Item* item, bool isChecked)>("ItemBoolSignal");
    
    class_<ItemTreeView, ItemTreeView*, bases<View>, boost::noncopyable>("ItemTreeView", no_init)
        .def("instance", &ItemTreeView::instance, return_value_policy<reference_existing_object>()).staticmethod("instance")
        .def("rootItem", ItemTreeView_rootItem)
        .def("showRoot", &ItemTreeView::showRoot)
        .def("selectedItems", &ItemTreeView::selectedItems<Item>)
        .def("isItemSelected", &ItemTreeView::isItemSelected)
        .def("selectItem", &ItemTreeView::selectItem, ItemTreeView_selectItem_overloads())
        .def("selectAllItems", &ItemTreeView::selectAllItems)
        .def("clearSelection", &ItemTreeView::clearSelection)
        .def("checkedItems", ItemTreeView_checkedItems, ItemTreeView_checkedItems_overloads())
        .def("isItemChecked", &ItemTreeView::isItemChecked, ItemTreeView_isItemChecked_overloads())
        .def("checkItem", &ItemTreeView::checkItem, ItemTreeView_checkItem_overloads())
        .def("sigSelectionChanged", &ItemTreeView::sigSelectionChanged)
        .def("sigSelectionOrTreeChanged", &ItemTreeView::sigSelectionOrTreeChanged)
        .def("sigCheckToggled", ItemTreeView_sigCheckToggled1, ItemTreeView_sigCheckToggled1_overloads())
        .def("sigCheckToggled", ItemTreeView_sigCheckToggled2, ItemTreeView_sigCheckToggled2_overloads())
        .def("cutSelectedItems", &ItemTreeView::cutSelectedItems)
        ;

    class_<SceneWidget, SceneWidget*, boost::noncopyable >("SceneWidget")
        .def("setEditMode", &SceneWidget::setEditMode)
        .def("sigEditModeToggled", &SceneWidget::sigEditModeToggled)
        
        .def("setCollisionLinesVisible", &SceneWidget::setCollisionLinesVisible)
        .def("collisionLinesVisible", &SceneWidget::collisionLinesVisible, return_value_policy<return_by_value>())

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
        .def("sceneWidget", &SceneView::sceneWidget, return_value_policy<reference_existing_object>())
        ;
}

}
