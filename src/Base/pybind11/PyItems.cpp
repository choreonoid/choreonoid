/*!
  @author Shin'ichiro Nakaoka
*/

#include "PyItemList.h"
#include "PyQString.h"
#include "../Item.h"
#include "../RenderableItem.h"
#include "../RootItem.h"
#include "../FolderItem.h"
#include "../SubProjectItem.h"
#include "../AbstractTextItem.h"
#include "../ScriptItem.h"
#include "../ExtCommandItem.h"
#include "../SceneItem.h"
#include "../PointSetItem.h"
#include "../MultiPointSetItem.h"
#include <cnoid/PyUtil>
#include <cnoid/ValueTree>

using namespace std;
using namespace cnoid;
namespace py = pybind11;

namespace cnoid {

void exportPyItems(py::module m)
{
    py::class_<Item, ItemPtr, Referenced> itemClass(m, "Item");

    itemClass
        .def_static("find", [](const string& path){ return ItemPtr(Item::find(path)); })
        .def_property("name", &Item::name, &Item::setName)
        .def("setName", &Item::setName)
        .def("hasAttribute", &Item::hasAttribute)
        .def_property_readonly("childItem", &Item::childItem)
        .def_property_readonly("prevItem", &Item::prevItem)
        .def_property_readonly("nextItem", &Item::nextItem)
        .def_property_readonly("parentItem", &Item::parentItem)
        .def("addChildItem", &Item::addChildItem, py::arg("item"), py::arg("isManualOperation") = false)
        .def("addSubItem", &Item::addSubItem)
        .def("isSubItem", &Item::isSubItem)
        .def("setSubItemAttributes", &Item::setSubItemAttributes)
        .def("removeFromParentItem", &Item::removeFromParentItem)
        .def("insertChild",
             &Item::insertChild, py::arg("position"), py::arg("item"), py::arg("isManualOperation") = false)
        .def("isTemporal", &Item::isTemporal)
        .def("setTemporal", &Item::setTemporal, py::arg("on") = true)
        .def("isSelected", &Item::isSelected)
        .def("setSelected", &Item::setSelected, py::arg("on"), py::arg("isCurrent") = false)
        .def("isChecked", [](Item& self){ return self.isChecked(); })
        .def("setChecked", [](Item& self, bool on){ return self.setChecked(on); })
        .def("findRootItem", &Item::findRootItem)
        .def("findItem", [](Item& self, const string& path){ return self.findItem(path); })
        .def("findItem",
             [](Item& self, py::object itemClass){
                 return self.findItem<Item>(
                     [&](Item* item) -> bool {
                         py::object pyItem(py::cast(item));
                         return (PyObject_IsInstance(pyItem.ptr(), itemClass.ptr()) > 0);
                     });
             })
        .def("findChildItem", [](Item& self, const string& path){ return self.findChildItem(path); })
        .def_property_readonly("headItem", &Item::headItem)
        .def("getDescendantItems", [](Item& self){ return self.descendantItems(); })
        .def("getDescendantItems", [](Item& self, py::object itemClass) {
                return getPyNarrowedItemList(self.descendantItems(), itemClass); })
        .def("duplicate", [](Item& self){ return self.duplicate(); })
        .def("duplicateSubTree", &Item::duplicateSubTree)
        .def("assign", &Item::assign)
        .def("load",
             [](Item& self, const string& filename, const string& format, const Mapping* options){
                 return self.load(filename, format, options); },
             py::arg("filename"), py::arg("format") = std::string(), py::arg("options") = nullptr)
        .def("load",
             [](Item& self, const string& filename, Item* parent, const string& format, const Mapping* options){
                 return self.load(filename, parent, format, options); },
             py::arg("filename"), py::arg("parent"), py::arg("format") = std::string(), py::arg("options") = nullptr)
        .def("save",
             [](Item& self, const string& filename, const string& format, const Mapping* options){
                 return self.save(filename, format, options); },
             py::arg("filename"), py::arg("format") = string(), py::arg("options") = nullptr)
        .def("overwrite", &Item::overwrite, py::arg("forceOverwrite") = false, py::arg("format") = string())
        .def_property_readonly("filePath", &Item::filePath)
        .def_property_readonly("fileFormat", &Item::fileFormat)
        .def("clearFileInformation", &Item::clearFileInformation)
        .def("suggestFileUpdate", &Item::suggestFileUpdate)
        .def("reload", &Item::reload)
        .def("replace", &Item::replace)
        .def("findOriginalItem", &Item::findOriginalItem)
        .def("notifyUpdate", &Item::notifyUpdate)
        .def_property_readonly("sigNameChanged", &Item::sigNameChanged)
        .def_property_readonly("sigUpdated", &Item::sigUpdated)
        .def_property_readonly("sigTreePathChanged", &Item::sigTreePositionChanged)
        .def_property_readonly("sigTreePositionChanged", &Item::sigTreePositionChanged)
        .def_property_readonly("sigDisconnectedFromRoot", &Item::sigDisconnectedFromRoot)
        .def_property_readonly("sigSubTreeChanged", &Item::sigSubTreeChanged)

        // deprecated
        .def_property_readonly("sigPositionChanged", &Item::sigTreePositionChanged)
        .def("detachFromParentItem", &Item::removeFromParentItem)
        .def("insertChildItem",
             [](Item& self, Item* item, Item* nextItem, bool isManualOperation = false){
                 return self.insertChild(nextItem, item, isManualOperation); },
             py::arg("item"), py::arg("nextItem"), py::arg("isManualOperation") = false)
        .def("findSubItem", [](Item& self, const string& path){
                return self.findChildItem(path, [](Item* item){ return item->isSubItem(); }); })
        .def("getName", &Item::name)
        .def("getChildItem", &Item::childItem)
        .def("getPrevItem", &Item::prevItem)
        .def("getNextItem", &Item::nextItem)
        .def("getParentItem", &Item::parentItem)
        .def("getHeadItem", &Item::headItem)
        .def("getDescendantItems", [](Item& self){ return self.descendantItems(); })
        .def("getDescendantItems", [](Item& self, py::object itemClass) {
                return getPyNarrowedItemList(self.descendantItems(), itemClass); })
        .def("getFilePath", &Item::filePath)
        .def("getFileFormat", &Item::fileFormat)
        .def("getSigNameChanged", &Item::sigNameChanged)
        .def("getSigUpdated", &Item::sigUpdated)
        .def("getSigPositionChanged", &Item::sigTreePositionChanged)
        .def("getSigDisconnectedFromRoot", &Item::sigDisconnectedFromRoot)
        .def("getSigSubTreeChanged", &Item::sigSubTreeChanged)
        ;

    py::enum_<Item::CheckId>(itemClass, "CheckId")
        .value("LogicalSumOfAllChecks", Item::LogicalSumOfAllChecks)
        .value("PrimaryCheck", Item::PrimaryCheck)
        ;

    PyItemList<Item>(m, "ItemList");

    py::class_<RootItem, RootItemPtr, Item>(m, "RootItem")
        .def_property_readonly_static("instance", [](py::object){ return RootItem::instance(); })
        .def("selectItem", &RootItem::selectItem)
        .def_property_readonly("sigSelectionChanged", &RootItem::sigSelectionChanged)
        .def_property_readonly("sigSelectedItemsChanged", &RootItem::sigSelectedItemsChanged)
        .def_property_readonly("selectedItems", [](RootItem& self){ return self.selectedItems(); })
        .def_property_readonly("checkedItems", [](RootItem& self){ return self.checkedItems(); })
        ;

    py::class_<RenderableItem>(m, "RenderableItem")
        .def("getScene", (SgNode*(RenderableItem::*)()) &RenderableItem::getScene)
        ;
    
    PyItemList<RootItem>(m, "RootItemList");

    py::class_<FolderItem, FolderItemPtr, Item>(m, "FolderItem")
        .def(py::init<>());

    PyItemList<FolderItem>(m, "FolderItemList");

    py::class_<SubProjectItem, SubProjectItemPtr, Item>(m, "SubProjectItem")
        .def(py::init<>());

    PyItemList<SubProjectItem>(m, "SubProjectItemList");
    
    py::class_<AbstractTextItem, AbstractTextItemPtr, Item>(m, "AbstractTextItem")
        .def_property_readonly("textFilename", &AbstractTextItem::textFilename)

        // deprecated
        .def("getTextFilename", &AbstractTextItem::textFilename)
        ;
            
    //PyItemList<AbstractTextItem>("AbstractTextItemList");
    
    py::class_<ScriptItem, ScriptItemPtr, AbstractTextItem> (m, "ScriptItem")
        .def_property_readonly("scriptFilename", &ScriptItem::scriptFilename)
        .def_property_readonly("identityName", &ScriptItem::identityName)
        .def("setBackgroundMode", &ScriptItem::setBackgroundMode)
        .def("isBackgroundMode", &ScriptItem::isBackgroundMode)
        .def("isRunning", &ScriptItem::isRunning)
        .def("execute", &ScriptItem::execute)
        .def("waitToFinish", &ScriptItem::waitToFinish, py::arg("timeout") = 0.0)
        .def_property_readonly("resultString", &ScriptItem::resultString)
        .def_property_readonly("sigScriptFinished", &ScriptItem::sigScriptFinished)
        .def("terminate", &ScriptItem::terminate)

        // deprecated
        .def("getScriptFilename", &ScriptItem::scriptFilename)
        .def("getIdentityName", &ScriptItem::identityName)
        .def("getResultString", &ScriptItem::resultString)
        .def("getSigScriptFinished", &ScriptItem::sigScriptFinished)
        ;

    //PyItemList<ScriptItem>("ScriptItemList");

    py::class_<ExtCommandItem, ExtCommandItemPtr, Item>(m, "ExtCommandItem")
        .def(py::init<>())
        .def_property("command", &ExtCommandItem::command, &ExtCommandItem::setCommand, py::return_value_policy::reference)
        .def("setCommand", &ExtCommandItem::setCommand)
        .def("waitingTimeAfterStarted", &ExtCommandItem::waitingTimeAfterStarted)
        .def("setWaitingTimeAfterStarted", &ExtCommandItem::setWaitingTimeAfterStarted)
        .def("execute", &ExtCommandItem::execute)
        .def("terminate", &ExtCommandItem::terminate)

        // deprecated
        .def("getCommand", &ExtCommandItem::command, py::return_value_policy::reference)
        ;
    
    PyItemList<ExtCommandItem>(m, "ExtCommandItemList");

    py::class_<SceneItem, SceneItemPtr, Item>(m, "SceneItem", py::multiple_inheritance())
        .def(py::init<>())
        .def_property_readonly("topNode", (SgPosTransform*(SceneItem::*)()) &SceneItem::topNode)
        .def("setTranslation", (void(SceneItem::*)(const Vector3&)) &SceneItem::setTranslation)
        .def("setRotation", (void(SceneItem::*)(const AngleAxis&)) &SceneItem::setRotation)
        .def("setLightweightRenderingEnabled", &SceneItem::setLightweightRenderingEnabled)
        .def("isLightweightRenderingEnabled", &SceneItem::isLightweightRenderingEnabled)

        // deprecated
        .def("getTopNode", (SgPosTransform*(SceneItem::*)()) &SceneItem::topNode)
        ;

    py::class_<PointSetItem, PointSetItemPtr, Item> (m, "PointSetItem", py::multiple_inheritance())
        .def(py::init<>())
        .def_property_readonly("numAttentionPoints", &PointSetItem::numAttentionPoints)
        .def_property_readonly("attentionPoint", (Vector3(PointSetItem::*)(int)const) &PointSetItem::attentionPoint)
        .def("clearAttentionPoints", &PointSetItem::clearAttentionPoints)
        .def("addAttentionPoint", &PointSetItem::addAttentionPoint)
        .def_property_readonly("sigAttentionPointsChanged", &PointSetItem::sigAttentionPointsChanged)
        .def("notifyAttentionPointChange", &PointSetItem::notifyAttentionPointChange)

        // deprecated
        .def_property("offsetTransform", &PointSetItem::offsetPosition, &PointSetItem::setOffsetPosition)
        .def("setOffsetTransform", &PointSetItem::setOffsetPosition)
        .def_property_readonly("sigOffsetTransformChanged", &PointSetItem::sigOffsetPositionChanged)
        .def("notifyOffsetTransformChange", &PointSetItem::notifyOffsetPositionChange)
        
        .def("getOffsetTransform", &PointSetItem::offsetPosition)
        .def("getSigOffsetTransformChanged", &PointSetItem::sigOffsetPositionChanged)
        .def("getNumAttentionPoints", &PointSetItem::numAttentionPoints)
        .def("getAttentionPoint", (Vector3(PointSetItem::*)(int)const) &PointSetItem::attentionPoint)
        .def("getSigAttentionPointsChanged", &PointSetItem::sigAttentionPointsChanged)
        ;

    PyItemList<PointSetItem>(m, "PointSetItemList");

    py::class_<MultiPointSetItem, MultiPointSetItemPtr, Item>(m, "MultiPointSetItem", py::multiple_inheritance())
        .def(py::init<>())
        .def_property_readonly("numPointSetItems", &MultiPointSetItem::numPointSetItems)
        .def_property_readonly("pointSetItem", (PointSetItem*(MultiPointSetItem::*)(int)) &MultiPointSetItem::pointSetItem)
        .def_property_readonly("numActivePointSetItems", &MultiPointSetItem::numActivePointSetItems)
        .def_property_readonly("activePointSetItem", (PointSetItem*(MultiPointSetItem::*)(int)) &MultiPointSetItem::activePointSetItem)
        .def_property_readonly("sigPointSetItemAdded", &MultiPointSetItem::sigPointSetItemAdded)
        .def_property_readonly("sigPointSetUpdated", &MultiPointSetItem::sigPointSetUpdated)
        .def_property("offsetPosition", &MultiPointSetItem::offsetPosition, &MultiPointSetItem::setOffsetPosition)
        .def("setOffsetPosition", &MultiPointSetItem::setOffsetPosition)
        .def_property_readonly("sigOffsetPositionChanged", &MultiPointSetItem::sigOffsetPositionChanged)
        .def("notifyOffsetPositionChange", &MultiPointSetItem::notifyOffsetPositionChange)
        .def_property_readonly("totalOffsetPositionOf", &MultiPointSetItem::totalOffsetPositionOf)
        .def("getTransformedPointSet", &MultiPointSetItem::getTransformedPointSet)
        .def_property_readonly("numAttentionPoints", &MultiPointSetItem::numAttentionPoints)
        .def("attentionPoint", &MultiPointSetItem::attentionPoint)
        .def("clearAttentionPoints", &MultiPointSetItem::clearAttentionPoints)
        .def("addAttentionPoint", &MultiPointSetItem::addAttentionPoint)
        .def_property_readonly("sigAttentionPointsChanged", &MultiPointSetItem::sigAttentionPointsChanged)
        .def("notifyAttentionPointChange", &MultiPointSetItem::notifyAttentionPointChange)
        .def("startAutomaticSave", &MultiPointSetItem::startAutomaticSave)
        .def("stopAutomaticSave", &MultiPointSetItem::stopAutomaticSave)

        // deprecated
        .def("getNumPointSetItems", &MultiPointSetItem::numPointSetItems)
        .def("getPointSetItem", (PointSetItem*(MultiPointSetItem::*)(int)) &MultiPointSetItem::pointSetItem)
        .def("getNumActivePointSetItems", &MultiPointSetItem::numActivePointSetItems)
        .def("getActivePointSetItem", (PointSetItem*(MultiPointSetItem::*)(int)) &MultiPointSetItem::activePointSetItem)
        .def("getSigPointSetItemAdded", &MultiPointSetItem::sigPointSetItemAdded)
        .def("getSigPointSetUpdated", &MultiPointSetItem::sigPointSetUpdated)
        .def("getTopOffsetTransform", &MultiPointSetItem::offsetPosition)
        .def("getSigTopOffsetTransformChanged", &MultiPointSetItem::sigOffsetPositionChanged)
        .def("getOffsetTransform", &MultiPointSetItem::totalOffsetPositionOf)
        .def("getNumAttentionPoints", &MultiPointSetItem::numAttentionPoints)
        .def("getAttentionPoint", &MultiPointSetItem::attentionPoint)
        .def("getSigAttentionPointsChanged", &MultiPointSetItem::sigAttentionPointsChanged)

        .def_property("topOffsetTransform", &MultiPointSetItem::offsetPosition, &MultiPointSetItem::setOffsetPosition)
        .def("setTopOffsetTransform", &MultiPointSetItem::setOffsetPosition)
        .def_property_readonly("sigTopOffsetTransformChanged", &MultiPointSetItem::sigOffsetPositionChanged)
        .def("notifyTopOffsetTransformChange", &MultiPointSetItem::notifyOffsetPositionChange)
        .def_property_readonly("offsetTransform", &MultiPointSetItem::totalOffsetPositionOf)
        ;

    PyItemList<MultiPointSetItem>(m, "MultiPointSetItemList");
}

} // namespace cnoid
