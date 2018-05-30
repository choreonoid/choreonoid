/*!
  @author Shin'ichiro Nakaoka
*/

#include "PyBase.h"
#include "../Item.h"
#include "../RootItem.h"
#include "../FolderItem.h"
#include "../AbstractTextItem.h"
#include "../ScriptItem.h"
#include "../ExtCommandItem.h"
#include "../MultiValueSeqItem.h"
#include "../MultiSE3MatrixSeqItem.h"
#include "../MultiSE3SeqItem.h"
#include "../Vector3SeqItem.h"
#include "../SceneItem.h"
#include "../PointSetItem.h"
#include "../MultiPointSetItem.h"
#include <cnoid/PyUtil>

namespace py = boost::python;
using namespace cnoid;

// for MSVC++2015 Update3
CNOID_PYTHON_DEFINE_GET_POINTER(Item)
CNOID_PYTHON_DEFINE_GET_POINTER(RootItem)

namespace {

template<typename ItemType>
struct ItemList_to_pylist_converter {
    static PyObject* convert(const ItemList<ItemType>& items){
        py::list retval;
        for(size_t i=0; i < items.size(); ++i){
            retval.append(items[i]);
        }
        return py::incref(retval.ptr());
    }
};

ItemPtr Item_childItem(Item& self) { return self.childItem(); }
ItemPtr Item_prevItem(Item& self) { return self.prevItem(); }
ItemPtr Item_nextItem(Item& self) { return self.nextItem(); }
ItemPtr Item_parentItem(Item& self) { return self.parentItem(); }
ItemPtr Item_find(const std::string& path) { return Item::find(path); }
RootItemPtr Item_findRootItem(Item& self) { return self.findRootItem(); }
ItemPtr Item_findItem(Item& self, const std::string& path) { return self.findItem(path); };
ItemPtr Item_findChildItem(Item& self, const std::string& path) { return self.findChildItem(path); }
ItemPtr Item_findSubItem(Item& self, const std::string& path) { return self.findSubItem(path); }
ItemPtr Item_headItem(Item& self) { return self.headItem(); }

ItemList<Item> Item_getDescendantItems1(Item& self){
    ItemList<Item> items;
    items.extractChildItems(&self);
    return items;
}

py::object Item_getDescendantItems2(Item& self, py::object itemClass){
    ItemList<Item> items;
    items.extractChildItems(&self);
    return getPyNarrowedItemList(items, itemClass);
}

ItemPtr Item_duplicate(Item& self) { return self.duplicate(); }
ItemPtr Item_duplicateAll(Item& self) { return self.duplicateAll(); }


BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(Item_addChildItem_overloads, addChildItem, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(Item_insertChildItem, insertChildItem, 2, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(Item_setTemporal, setTemporal, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(Item_load1_overloads, load, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(Item_load2_overloads, load, 2, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(Item_save, load, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(Item_overwrite, overwrite, 0, 2)

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ScriptItem_waitToFinish, waitToFinish, 0, 1)

RootItemPtr RootItem_Instance() { return RootItem::instance(); }


SgPosTransformPtr SceneItem_topNode(SceneItem& self){
    return self.topNode();
}

Affine3 PointSetItem_offsetTransform(const PointSetItem& self){
    return self.offsetTransform();
}

Vector3 PointSetItem_attentionPoint(PointSetItem& self, int index){
    return self.attentionPoint(index);
}

PointSetItemPtr MultiPointSetItem_pointSetItem(MultiPointSetItem& self, int index)
{
    return self.pointSetItem(index);
}

PointSetItemPtr MultiPointSetItem_activePointSetItem(MultiPointSetItem& self, int index)
{
    return self.activePointSetItem(index);
}

} // namespace


namespace cnoid {

void exportPyItems()
{
    py::to_python_converter<ItemList<Item>, ItemList_to_pylist_converter<Item> >();
    py::to_python_converter<ItemList<RootItem>, ItemList_to_pylist_converter<RootItem> >();
    py::to_python_converter<ItemList<FolderItem>, ItemList_to_pylist_converter<FolderItem> >();
    py::to_python_converter<ItemList<ScriptItem>, ItemList_to_pylist_converter<ScriptItem> >();
    py::to_python_converter<ItemList<ExtCommandItem>, ItemList_to_pylist_converter<ExtCommandItem> >();
    py::to_python_converter<ItemList<MultiValueSeqItem>, ItemList_to_pylist_converter<MultiValueSeqItem> >();
    py::to_python_converter<ItemList<MultiSE3MatrixSeqItem>, ItemList_to_pylist_converter<MultiSE3MatrixSeqItem> >();
    py::to_python_converter<ItemList<MultiSE3SeqItem>, ItemList_to_pylist_converter<MultiSE3SeqItem> >();
    py::to_python_converter<ItemList<Vector3SeqItem>, ItemList_to_pylist_converter<Vector3SeqItem> >();

    bool (Item::*Item_load1)(const std::string& filename, const std::string& formatId) = &Item::load;
    bool (Item::*Item_load2)(const std::string& filename, Item* parent, const std::string& formatId) = &Item::load;
    
    py::class_<Item, ItemPtr, boost::noncopyable> itemClass("Item", py::no_init);
    
    itemClass
        .def("find", Item_find).staticmethod("find")
        .def("name", &Item::name, py::return_value_policy<py::copy_const_reference>())
        .def("getName", &Item::name, py::return_value_policy<py::copy_const_reference>())
        .def("setName", &Item::setName)
        .def("hasAttribute", &Item::hasAttribute)
        .def("childItem", Item_childItem)
        .def("getChildItem", Item_childItem)
        .def("prevItem", Item_prevItem)
        .def("getPrevItem", Item_prevItem)
        .def("nextItem", Item_nextItem)
        .def("getNextItem", Item_nextItem)
        .def("parentItem", Item_parentItem)
        .def("getParentItem", Item_parentItem)
        .def("addChildItem", &Item::addChildItem, Item_addChildItem_overloads())
        .def("addSubItem", &Item::addSubItem)
        .def("isSubItem", &Item::isSubItem)
        .def("detachFromParentItem", &Item::detachFromParentItem)
        .def("emitSigDetachedFromRootForSubTree", &Item::emitSigDetachedFromRootForSubTree)
        .def("insertChildItem", &Item::insertChildItem, Item_insertChildItem())
        .def("insertSubItem", &Item::insertSubItem)
        .def("isTemporal", &Item::isTemporal)
        .def("setTemporal", &Item::setTemporal, Item_setTemporal())
        .def("findRootItem", Item_findRootItem)
        .def("findItem", Item_findItem)
        .def("findChildItem", Item_findChildItem)
        .def("findSubItem", Item_findSubItem)
        .def("headItem", Item_headItem)
        .def("getHeadItem", Item_headItem)
        .def("getDescendantItems", Item_getDescendantItems1)
        .def("getDescendantItems", Item_getDescendantItems2)
        .def("duplicate", Item_duplicate)
        .def("duplicateAll", Item_duplicateAll)
        .def("assign", &Item::assign)
        .def("load", Item_load1, Item_load1_overloads())
        .def("load", Item_load2, Item_load2_overloads())
        .def("save", &Item::save, Item_save())
        .def("overwrite", &Item::overwrite, Item_overwrite())
        .def("filePath", &Item::filePath, py::return_value_policy<py::copy_const_reference>())
        .def("getFilePath", &Item::filePath, py::return_value_policy<py::copy_const_reference>())
        .def("fileFormat", &Item::fileFormat, py::return_value_policy<py::copy_const_reference>())
        .def("getFileFormat", &Item::fileFormat, py::return_value_policy<py::copy_const_reference>())
        .def("clearFileInformation", &Item::clearFileInformation)
        .def("suggestFileUpdate", &Item::suggestFileUpdate)
        .def("notifyUpdate", &Item::notifyUpdate)
        .def("sigNameChanged", &Item::sigNameChanged)
        .def("getSigNameChanged", &Item::sigNameChanged)
        .def("sigUpdated", &Item::sigUpdated)
        .def("getSigUpdated", &Item::sigUpdated)
        .def("sigPositionChanged", &Item::sigPositionChanged)
        .def("getSigPositionChanged", &Item::sigPositionChanged)
        .def("sigDisconnectedFromRoot", &Item::sigDisconnectedFromRoot)
        .def("getSigDisconnectedFromRoot", &Item::sigDisconnectedFromRoot)
        .def("sigSubTreeChanged", &Item::sigSubTreeChanged)
        .def("getSigSubTreeChanged", &Item::sigSubTreeChanged);
    
    {
        py::scope itemScope = itemClass;
        
        py::enum_<Item::Attribute>("Attribute")
            .value("SUB_ITEM", Item::SUB_ITEM) 
            .value("TEMPORAL", Item::TEMPORAL)
            .value("LOAD_ONLY", Item::LOAD_ONLY)
            .value("NUM_ATTRIBUTES", Item::NUM_ATTRIBUTES);
    }

    py::implicitly_convertible<ItemPtr, ReferencedPtr>();
    PyItemList<Item>("ItemList");

    py::class_<RootItem, RootItemPtr, py::bases<Item>>("RootItem")
        .def("instance", RootItem_Instance).staticmethod("instance")
        .def("getInstance", RootItem_Instance).staticmethod("getInstance");

    py::implicitly_convertible<RootItemPtr, ItemPtr>();
    PyItemList<RootItem>("RootItemList");

    py::class_<FolderItem, FolderItemPtr, py::bases<Item>>("FolderItem");

    py::implicitly_convertible<FolderItemPtr, ItemPtr>();
    PyItemList<FolderItem>("FolderItemList");

    py::class_< AbstractTextItem, AbstractTextItemPtr, py::bases<Item>, boost::noncopyable >
        ("AbstractTextItem", py::no_init)
        .def("textFilename", &AbstractTextItem::textFilename, py::return_value_policy<py::copy_const_reference>())
        .def("getTextFilename", &AbstractTextItem::textFilename, py::return_value_policy<py::copy_const_reference>());
            
    py::implicitly_convertible<AbstractTextItemPtr, ItemPtr>();
    //PyItemList<AbstractTextItem>("AbstractTextItemList");
    
    py::class_<ScriptItem, ScriptItemPtr, py::bases<AbstractTextItem>, boost::noncopyable>
        ("ScriptItem", py::no_init)
        .def("scriptFilename", &ScriptItem::scriptFilename, py::return_value_policy<py::copy_const_reference>())
        .def("getScriptFilename", &ScriptItem::scriptFilename, py::return_value_policy<py::copy_const_reference>())
        .def("identityName", &ScriptItem::identityName)
        .def("getIdentityName", &ScriptItem::identityName)
        .def("setBackgroundMode", &ScriptItem::setBackgroundMode)
        .def("isBackgroundMode", &ScriptItem::isBackgroundMode)
        .def("isRunning", &ScriptItem::isRunning)
        .def("execute", &ScriptItem::execute)
        .def("waitToFinish", &ScriptItem::waitToFinish, ScriptItem_waitToFinish())
        .def("resultString", &ScriptItem::resultString)
        .def("getResultString", &ScriptItem::resultString)
        .def("sigScriptFinished", &ScriptItem::sigScriptFinished)
        .def("getSigScriptFinished", &ScriptItem::sigScriptFinished)
        .def("terminate", &ScriptItem::terminate)
        ;

    py::implicitly_convertible<ScriptItemPtr, AbstractTextItemPtr>();
    //PyItemList<ScriptItem>("ScriptItemList");

    py::class_<ExtCommandItem, ExtCommandItemPtr, py::bases<Item>>("ExtCommandItem")
        .def("setCommand", &ExtCommandItem::setCommand)
        .def("command", &ExtCommandItem::command, py::return_value_policy<py::copy_const_reference>())
        .def("getCommand", &ExtCommandItem::command, py::return_value_policy<py::copy_const_reference>())
        .def("waitingTimeAfterStarted", &ExtCommandItem::waitingTimeAfterStarted)
        .def("setWaitingTimeAfterStarted", &ExtCommandItem::setWaitingTimeAfterStarted)
        .def("execute", &ExtCommandItem::execute)
        .def("terminate", &ExtCommandItem::terminate);
    
    py::implicitly_convertible<ExtCommandItemPtr, ItemPtr>();
    PyItemList<ExtCommandItem>("ExtCommandItemList");

    // seq items
    py::class_<AbstractSeqItem, AbstractSeqItemPtr, py::bases<Item>, boost::noncopyable >
        abstractSeqItemClass("AbstractSeqItem", py::no_init);
    abstractSeqItemClass
        .def("abstractSeq", &AbstractSeqItem::abstractSeq)
        .def("getAbstractSeq", &AbstractSeqItem::abstractSeq);

    py::implicitly_convertible<AbstractSeqItemPtr, ItemPtr>();    
    PyItemList<AbstractSeqItem>("AbstractSeqItemList", abstractSeqItemClass);

    py::class_<Vector3SeqItem, Vector3SeqItemPtr, py::bases<AbstractSeqItem>>("Vector3SeqItem")
        .def("seq", &Vector3SeqItem::seq)
        .def("getSeq", &Vector3SeqItem::seq);
    
    py::implicitly_convertible<Vector3SeqItemPtr, AbstractSeqItemPtr>();
    PyItemList<Vector3SeqItem>("Vector3SeqItemList");

    // multi seq items
    py::class_<AbstractMultiSeqItem, AbstractMultiSeqItemPtr, py::bases<AbstractSeqItem>, boost::noncopyable>
        abstractMultiSeqItemClass("AbstractMultiSeqItem", py::no_init);
    abstractMultiSeqItemClass
        .def("abstractMultiSeq", &AbstractMultiSeqItem::abstractMultiSeq)
        .def("getAbstractMultiSeq", &AbstractMultiSeqItem::abstractMultiSeq);

    py::implicitly_convertible<AbstractMultiSeqItemPtr, AbstractSeqItemPtr>();
    //PyItemList<AbstractMultiSeqItem>("AbstractMultiSeqItemList", abstractMultiSeqItemClass);
    
    py::class_<MultiValueSeqItem, MultiValueSeqItemPtr, py::bases<AbstractMultiSeqItem>>("MultiValueSeqItem")
        .def("abstractMultiSeq", &MultiValueSeqItem::abstractMultiSeq)
        .def("getAbstractMultiSeq", &MultiValueSeqItem::abstractMultiSeq)
        .def("seq", &MultiValueSeqItem::seq)
        .def("getSeq", &MultiValueSeqItem::seq);

    py::implicitly_convertible<MultiValueSeqItemPtr, AbstractMultiSeqItemPtr>();
    PyItemList<MultiValueSeqItem>("MultiValueSeqItemList");

    py::class_<MultiSE3MatrixSeqItem, MultiSE3MatrixSeqItemPtr, py::bases<AbstractMultiSeqItem>>("MultiSE3MatrixSeqItem")
        .def("abstractMultiSeq", &MultiSE3MatrixSeqItem::abstractMultiSeq)
        .def("getAbstractMultiSeq", &MultiSE3MatrixSeqItem::abstractMultiSeq)
        .def("seq", &MultiSE3MatrixSeqItem::seq)
        .def("getSeq", &MultiSE3MatrixSeqItem::seq);

    py::implicitly_convertible<MultiSE3MatrixSeqItemPtr, AbstractMultiSeqItemPtr>();
    PyItemList<MultiSE3MatrixSeqItem>("MultiSE3MatrixSeqItemList");

    py::class_<MultiSE3SeqItem, MultiSE3SeqItemPtr, py::bases<AbstractMultiSeqItem>>("MultiSE3SeqItem")
        .def("abstractMultiSeq", &MultiSE3SeqItem::abstractMultiSeq)
        .def("getAbstractMultiSeq", &MultiSE3SeqItem::abstractMultiSeq)
        .def("seq", &MultiSE3SeqItem::seq)
        .def("getSeq", &MultiSE3SeqItem::seq);
    
    py::implicitly_convertible<MultiSE3SeqItemPtr, AbstractMultiSeqItemPtr>();
    PyItemList<MultiSE3SeqItem>("MultiSE3SeqItemList");

    py::class_<SceneItem, SceneItemPtr, py::bases<Item, SceneProvider>>("SceneItem")
        .def("topNode", SceneItem_topNode)
        .def("getTopNode", SceneItem_topNode)
        ;

    py::implicitly_convertible<SceneItemPtr, ItemPtr>();
    py::implicitly_convertible<SceneItemPtr, SceneProvider*>();

    py::class_<PointSetItem, PointSetItemPtr, py::bases<Item, SceneProvider>>("PointSetItem")
        .def("offsetTransform", PointSetItem_offsetTransform)
        .def("getOffsetTransform", PointSetItem_offsetTransform)
        .def("setOffsetTransform", &PointSetItem::setOffsetTransform)
        .def("sigOffsetTransformChanged", &PointSetItem::sigOffsetTransformChanged)
        .def("getSigOffsetTransformChanged", &PointSetItem::sigOffsetTransformChanged)
        .def("notifyOffsetTransformChange", &PointSetItem::notifyOffsetTransformChange)
        .def("numAttentionPoints", &PointSetItem::numAttentionPoints)
        .def("getNumAttentionPoints", &PointSetItem::numAttentionPoints)
        .def("attentionPoint", PointSetItem_attentionPoint)
        .def("getAttentionPoint", PointSetItem_attentionPoint)
        .def("clearAttentionPoints", &PointSetItem::clearAttentionPoints)
        .def("addAttentionPoint", &PointSetItem::addAttentionPoint)
        .def("sigAttentionPointsChanged", &PointSetItem::sigAttentionPointsChanged)
        .def("getSigAttentionPointsChanged", &PointSetItem::sigAttentionPointsChanged)
        .def("notifyAttentionPointChange", &PointSetItem::notifyAttentionPointChange)
        ;

    py::implicitly_convertible<PointSetItemPtr, ItemPtr>();
    py::implicitly_convertible<PointSetItemPtr, SceneProvider*>();
    PyItemList<PointSetItem>("PointSetItemList");

    py::class_<MultiPointSetItem, MultiPointSetItemPtr, py::bases<PointSetItem, SceneProvider>>("MultiPointSetItem")
        .def("numPointSetItems", &MultiPointSetItem::numPointSetItems)
        .def("getNumPointSetItems", &MultiPointSetItem::numPointSetItems)
        .def("pointSetItem", MultiPointSetItem_pointSetItem)
        .def("getPointSetItem", MultiPointSetItem_pointSetItem)
        .def("numActivePointSetItems", &MultiPointSetItem::numActivePointSetItems)
        .def("getNumActivePointSetItems", &MultiPointSetItem::numActivePointSetItems)
        .def("activePointSetItem", MultiPointSetItem_activePointSetItem)
        .def("getActivePointSetItem", MultiPointSetItem_activePointSetItem)
        .def("sigPointSetItemAdded", &MultiPointSetItem::sigPointSetItemAdded)
        .def("getSigPointSetItemAdded", &MultiPointSetItem::sigPointSetItemAdded)
        .def("sigPointSetUpdated", &MultiPointSetItem::sigPointSetUpdated)
        .def("getSigPointSetUpdated", &MultiPointSetItem::sigPointSetUpdated)
        .def("topOffsetTransform", &MultiPointSetItem::topOffsetTransform, py::return_value_policy<py::copy_const_reference>())
        .def("getTopOffsetTransform", &MultiPointSetItem::topOffsetTransform, py::return_value_policy<py::copy_const_reference>())
        .def("setTopOffsetTransform", &MultiPointSetItem::setTopOffsetTransform)
        .def("sigTopOffsetTransformChanged", &MultiPointSetItem::sigTopOffsetTransformChanged)
        .def("getSigTopOffsetTransformChanged", &MultiPointSetItem::sigTopOffsetTransformChanged)
        .def("notifyTopOffsetTransformChange", &MultiPointSetItem::notifyTopOffsetTransformChange)
        .def("offsetTransform", &MultiPointSetItem::offsetTransform)
        .def("getOffsetTransform", &MultiPointSetItem::offsetTransform)
        .def("getTransformedPointSet", &MultiPointSetItem::getTransformedPointSet)
        .def("numAttentionPoints", &MultiPointSetItem::numAttentionPoints)
        .def("getNumAttentionPoints", &MultiPointSetItem::numAttentionPoints)
        .def("attentionPoint", &MultiPointSetItem::attentionPoint)
        .def("getAttentionPoint", &MultiPointSetItem::attentionPoint)
        .def("clearAttentionPoints", &MultiPointSetItem::clearAttentionPoints)
        .def("addAttentionPoint", &MultiPointSetItem::addAttentionPoint)
        .def("sigAttentionPointsChanged", &MultiPointSetItem::sigAttentionPointsChanged)
        .def("getSigAttentionPointsChanged", &MultiPointSetItem::sigAttentionPointsChanged)
        .def("notifyAttentionPointChange", &MultiPointSetItem::notifyAttentionPointChange)
        .def("startAutomaticSave", &MultiPointSetItem::startAutomaticSave)
        .def("stopAutomaticSave", &MultiPointSetItem::stopAutomaticSave);
    
    py::implicitly_convertible<MultiPointSetItemPtr, ItemPtr>();
    py::implicitly_convertible<MultiPointSetItemPtr, SceneProvider*>();
    PyItemList<MultiPointSetItem>("MultiPointSetItemList");

#ifdef _MSC_VER
    py::register_ptr_to_python<ItemPtr>();
    py::register_ptr_to_python<RootItemPtr>();
#endif

}

} // namespace cnoid
