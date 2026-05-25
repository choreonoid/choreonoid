#include "PyItemList.h"
#include "PyQString.h"
#include <cnoid/PyEigenTypes>
#include "../Item.h"
#include "../RenderableItem.h"
#include "../RootItem.h"
#include "../FolderItem.h"
#include "../SubProjectItem.h"
#include "../AbstractTextItem.h"
#include "../ScriptItem.h"
#include "../ExtCommandItem.h"
#include "../SceneItem.h"
#include "../LightingItem.h"
#include "../PointSetItem.h"
#include "../MultiPointSetItem.h"
#include <nanobind/stl/string.h>
#include <cnoid/PyUtil>
#include <cnoid/ValueTree>
#include <cnoid/CloneMap>

using namespace std;
using namespace cnoid;
namespace nb = nanobind;

namespace cnoid {

void exportPyItems(nb::module_ m)
{
    nb::class_<Item, Referenced> itemClass(m, "Item");

    itemClass
        .def_static("find", [](const string& path){ return ItemPtr(Item::find(path)); })
        .def_prop_rw("name", &Item::name, &Item::setName)
        .def("setName", &Item::setName)
        .def("setAttribute", &Item::setAttribute)
        .def("hasAttribute", &Item::hasAttribute)
        .def_prop_ro("childItem", &Item::childItem)
        .def_prop_ro("prevItem", &Item::prevItem)
        .def_prop_ro("nextItem", &Item::nextItem)
        .def_prop_ro("parentItem", [](Item& self){ return self.parentItem(); })
        .def("addChildItem", &Item::addChildItem, nb::arg("item"), nb::arg("isManualOperation") = false)
        .def("addSubItem", &Item::addSubItem)
        .def("isSubItem", &Item::isSubItem)
        .def("removeFromParentItem", &Item::removeFromParentItem)
        .def("insertChild",
             &Item::insertChild, nb::arg("position"), nb::arg("item"), nb::arg("isManualOperation") = false)
        .def("isTemporary", &Item::isTemporary)
        .def("setTemporary", &Item::setTemporary, nb::arg("on") = true)
        .def("isSelected", &Item::isSelected)
        .def("setSelected", &Item::setSelected, nb::arg("on") = true, nb::arg("isCurrent") = false)
        .def("isChecked", [](Item& self){ return self.isChecked(); })
        .def("setChecked", [](Item& self, bool on){ return self.setChecked(on); }, nb::arg("on") = true)
        .def("findRootItem", &Item::findRootItem)
        .def("findItem", [](Item& self, const string& path){ return self.findItem(path); })
        .def("findItem",
             [](Item& self, nb::object itemClass){
                 return self.findItem<Item>(
                     [&](Item* item) -> bool {
                         nb::object pyItem = nb::cast(item);
                         return (PyObject_IsInstance(pyItem.ptr(), itemClass.ptr()) > 0);
                     });
             })
        .def("findChildItem", [](Item& self, const string& path){ return self.findChildItem(path); })
        .def_prop_ro("headItem", &Item::headItem)
        .def("getDescendantItems", [](Item& self){ return self.descendantItems(); })
        .def("getDescendantItems",
             [](Item& self, nb::object itemClass) {
                 return getPyNarrowedItemList(self.descendantItems(), itemClass);
             })
        .def("duplicate", [](Item& self){ return self.clone(); })
        .def("duplicateSubTree", &Item::cloneSubTree)
        .def("assign", &Item::assign)
        .def("load",
             [](Item& self, const string& filename, const string& format, const Mapping* options){
                 return self.load(filename, format, options); },
             nb::arg("filename"), nb::arg("format") = std::string(), nb::arg("options").none() = nullptr)
        .def("load",
             [](Item& self, const string& filename, Item* parent, const string& format, const Mapping* options){
                 return self.load(filename, parent, format, options); },
             nb::arg("filename"), nb::arg("parent"), nb::arg("format") = std::string(), nb::arg("options").none() = nullptr)
        .def("save",
             [](Item& self, const string& filename, const string& format, const Mapping* options){
                 return self.save(filename, format, options); },
             nb::arg("filename"), nb::arg("format") = string(), nb::arg("options").none() = nullptr)
        .def("overwriteOrSaveWithDialog", &Item::overwriteOrSaveWithDialog,
             nb::arg("forceOverwrite") = false, nb::arg("format") = string())
        .def_prop_ro("filePath", &Item::filePath)
        .def_prop_ro("fileFormat", &Item::fileFormat)
        .def("clearFileInformation", &Item::clearFileInformation)
        .def("suggestFileUpdate", &Item::suggestFileUpdate)
        .def("reload", &Item::reload)
        .def("replace", &Item::replace)
        .def("findOriginalItem", &Item::findOriginalItem)
        .def("notifyUpdate", &Item::notifyUpdate)
        .def_prop_ro("sigNameChanged", &Item::sigNameChanged)
        .def_prop_ro("sigUpdated", &Item::sigUpdated)
        .def_prop_ro("sigTreePathChanged", &Item::sigTreePositionChanged)
        .def_prop_ro("sigTreePositionChanged", &Item::sigTreePositionChanged)
        .def_prop_ro("sigDisconnectedFromRoot", &Item::sigDisconnectedFromRoot)
        .def_prop_ro("sigSubTreeChanged", &Item::sigSubTreeChanged)
        ;

    nb::enum_<Item::Attribute>(itemClass, "Attribute", nb::is_arithmetic())
        .value("Attached", Item::Attached)
        .value("SubItemAdditionalAttribute", Item::SubItemAdditionalAttribute)
        .value("SubItem", Item::SubItem)
        .value("Unique", Item::Unique)
        .value("Temporary", Item::Temporary)
        .value("Builtin", Item::Builtin)
        .value("FileImmutable", Item::FileImmutable)
        .value("Reloadable", Item::Reloadable)
        .value("ExcludedFromUnifiedEditHistory", Item::ExcludedFromUnifiedEditHistory)
        .export_values();

    nb::enum_<Item::CheckId>(itemClass, "CheckId", nb::is_arithmetic())
        .value("LogicalSumOfAllChecks", Item::LogicalSumOfAllChecks)
        .value("PrimaryCheck", Item::PrimaryCheck)
        .export_values();

    PyItemList<Item>(m, "ItemList");

    nb::class_<RootItem, Item>(m, "RootItem")
        .def_prop_ro_static("instance", [](nb::handle){ return RootItem::instance(); }, nb::rv_policy::reference)
        .def("selectItem", &RootItem::selectItem)
        .def_prop_ro("sigSelectionChanged", &RootItem::sigSelectionChanged)
        .def_prop_ro("sigSelectedItemsChanged", &RootItem::sigSelectedItemsChanged)
        .def_prop_ro("selectedItems", [](RootItem& self){ return self.selectedItems(); })
        .def("getSelectedItems",
              [](RootItem& self, nb::object itemClass){
                  return getPyNarrowedItemList(self.selectedItems(), itemClass);
              })
        .def_prop_ro("checkedItems", [](RootItem& self){ return self.checkedItems(); })
        .def("getCheckedItems",
              [](RootItem& self, nb::object itemClass){
                  return getPyNarrowedItemList(self.checkedItems(), itemClass);
              })
        ;

    nb::class_<RenderableItem>(m, "RenderableItem")
        .def("getScene", (SgNode*(RenderableItem::*)()) &RenderableItem::getScene)
        ;

    PyItemList<RootItem>(m, "RootItemList");

    nb::class_<FolderItem, Item>(m, "FolderItem")
        .def(nb::init<>());

    PyItemList<FolderItem>(m, "FolderItemList");

    nb::class_<SubProjectItem, Item>(m, "SubProjectItem")
        .def(nb::init<>());

    PyItemList<SubProjectItem>(m, "SubProjectItemList");

    nb::class_<AbstractTextItem, Item>(m, "AbstractTextItem")
        .def_prop_ro("textFilename", &AbstractTextItem::textFilename)
        ;

    nb::class_<ScriptItem, AbstractTextItem> (m, "ScriptItem")
        .def_prop_ro("scriptFilename", &ScriptItem::scriptFilename)
        .def_prop_ro("identityName", &ScriptItem::identityName)
        .def("setBackgroundMode", &ScriptItem::setBackgroundMode)
        .def("isBackgroundMode", &ScriptItem::isBackgroundMode)
        .def("isRunning", &ScriptItem::isRunning)
        .def("execute", &ScriptItem::execute)
        .def("waitToFinish", &ScriptItem::waitToFinish, nb::arg("timeout") = 0.0)
        .def_prop_ro("resultString", &ScriptItem::resultString)
        .def_prop_ro("sigScriptFinished", &ScriptItem::sigScriptFinished)
        .def("terminate", &ScriptItem::terminate)
        ;

    nb::class_<ExtCommandItem, Item>(m, "ExtCommandItem")
        .def(nb::init<>())
        .def_prop_rw("command", &ExtCommandItem::command, &ExtCommandItem::setCommand)
        .def("setCommand", &ExtCommandItem::setCommand)
        .def("waitingTimeAfterStarted", &ExtCommandItem::waitingTimeAfterStarted)
        .def("setWaitingTimeAfterStarted", &ExtCommandItem::setWaitingTimeAfterStarted)
        .def("execute", &ExtCommandItem::execute)
        .def("terminate", &ExtCommandItem::terminate)
        ;

    PyItemList<ExtCommandItem>(m, "ExtCommandItemList");

    nb::class_<SceneItem, Item>(m, "SceneItem")
        .def(nb::init<>())
        .def_prop_ro("topNode", (SgPosTransform*(SceneItem::*)()) &SceneItem::topNode)
        .def("setTranslation",
             [](SceneItem& self, const python::Vector3Arg& p){ self.setTranslation(p.value); })
        .def("setRotation", (void(SceneItem::*)(const AngleAxis&)) &SceneItem::setRotation)
        .def("setLightweightRenderingEnabled", &SceneItem::setLightweightRenderingEnabled)
        .def("isLightweightRenderingEnabled", &SceneItem::isLightweightRenderingEnabled)
        ;

    nb::class_<LightingItem, Item> lightingItemClass(m, "LightingItem");

    lightingItemClass
        .def(nb::init<>())
        .def("setLightType", &LightingItem::setLightType)
        .def("setLightEnabled", &LightingItem::setLightEnabled)
        .def("setTranslation", [](LightingItem& self, const python::Vector3Arg& t){ self.setTranslation(t.value); })
        .def("setDirection", [](LightingItem& self, const python::Vector3Arg& d){ self.setDirection(d.value); })
        .def("setIntensity", &LightingItem::setIntensity)
        .def("setAmbientIntensity", &LightingItem::setAmbientIntensity)
        .def("setColor", [](LightingItem& self, const python::Vector3fArg& c){ self.setColor(c.value); })
        .def("setConstantAttenuation", &LightingItem::setConstantAttenuation)
        .def("setLinearAttenuation", &LightingItem::setLinearAttenuation)
        .def("setQuadraticAttenuation", &LightingItem::setQuadraticAttenuation)
        .def("setBeamWidth", &LightingItem::setBeamWidth)
        .def("setCutOffAngle", &LightingItem::setCutOffAngle)
        .def("setCutOffExponent", &LightingItem::setCutOffExponent)
        .def("setLightMarkerEnabled", &LightingItem::setLightMarkerEnabled)
        ;

    nb::enum_<LightingItem::LightType>(lightingItemClass, "LightType", nb::is_arithmetic())
        .value("DirectionalLight", LightingItem::DirectionalLight)
        .value("PointLight", LightingItem::PointLight)
        .value("SpotLight", LightingItem::SpotLight)
        .export_values();

    nb::class_<PointSetItem, Item> (m, "PointSetItem")
        .def(nb::init<>())
        .def_prop_ro("numAttentionPoints", &PointSetItem::numAttentionPoints)
        .def_prop_ro("attentionPoint", (Vector3(PointSetItem::*)(int)const) &PointSetItem::attentionPoint)
        .def("clearAttentionPoints", &PointSetItem::clearAttentionPoints)
        .def("addAttentionPoint", [](PointSetItem& self, const python::Vector3Arg& p){ self.addAttentionPoint(p.value); })
        .def_prop_ro("sigAttentionPointsChanged", &PointSetItem::sigAttentionPointsChanged)
        .def("notifyAttentionPointChange", &PointSetItem::notifyAttentionPointChange)
        ;

    PyItemList<PointSetItem>(m, "PointSetItemList");

    nb::class_<MultiPointSetItem, Item>(m, "MultiPointSetItem")
        .def(nb::init<>())
        .def_prop_ro("numPointSetItems", &MultiPointSetItem::numPointSetItems)
        .def_prop_ro("pointSetItem", (PointSetItem*(MultiPointSetItem::*)(int)) &MultiPointSetItem::pointSetItem)
        .def_prop_ro("numActivePointSetItems", &MultiPointSetItem::numActivePointSetItems)
        .def_prop_ro("activePointSetItem", (PointSetItem*(MultiPointSetItem::*)(int)) &MultiPointSetItem::activePointSetItem)
        .def_prop_ro("sigPointSetItemAdded", &MultiPointSetItem::sigPointSetItemAdded)
        .def_prop_ro("sigPointSetUpdated", &MultiPointSetItem::sigPointSetUpdated)
        .def_prop_rw("offsetPosition", &MultiPointSetItem::offsetPosition, &MultiPointSetItem::setOffsetPosition)
        .def("setOffsetPosition", &MultiPointSetItem::setOffsetPosition)
        .def_prop_ro("sigOffsetPositionChanged", &MultiPointSetItem::sigOffsetPositionChanged)
        .def("notifyOffsetPositionChange", &MultiPointSetItem::notifyOffsetPositionChange)
        .def_prop_ro("totalOffsetPositionOf", &MultiPointSetItem::totalOffsetPositionOf)
        .def("getTransformedPointSet", &MultiPointSetItem::getTransformedPointSet)
        .def_prop_ro("numAttentionPoints", &MultiPointSetItem::numAttentionPoints)
        .def("attentionPoint", &MultiPointSetItem::attentionPoint)
        .def("clearAttentionPoints", &MultiPointSetItem::clearAttentionPoints)
        .def("addAttentionPoint", [](MultiPointSetItem& self, const python::Vector3Arg& p){ self.addAttentionPoint(p.value); })
        .def_prop_ro("sigAttentionPointsChanged", &MultiPointSetItem::sigAttentionPointsChanged)
        .def("notifyAttentionPointChange", &MultiPointSetItem::notifyAttentionPointChange)
        .def("startAutomaticSave", &MultiPointSetItem::startAutomaticSave)
        .def("stopAutomaticSave", &MultiPointSetItem::stopAutomaticSave)
        ;

    PyItemList<MultiPointSetItem>(m, "MultiPointSetItemList");
}

} // namespace cnoid
