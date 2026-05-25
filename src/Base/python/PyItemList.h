#ifndef CNOID_BASE_PYTHON_PYITEMLIST_H
#define CNOID_BASE_PYTHON_PYITEMLIST_H

#include "../ItemList.h"
#include <cnoid/PyUtil>
#include "exportdecl.h"

/*
   A cnoid::ItemList<ItemType> is exposed to Python as a plain list of bound
   item objects (C++ -> Python only; there is no Python -> C++ direction).

   For each concrete item type a module-level function (e.g. "RootItemList") is
   bound that narrows a list of items down to those that are instances of that
   type. The narrowing relies on the Python type registered for the item type
   rather than on nanobind internals: getPyNarrowedItemList/getPyNarrowedFirstItem
   take the item class as an nb::object and use PyObject_IsInstance, while the
   PyItemList template obtains the class via nb::type<ItemType>(). This avoids
   the get_internals() access that the pybind11 version relied on.
*/

namespace nanobind {
namespace detail {

template<typename ItemType>
struct type_caster<cnoid::ItemList<ItemType>>
{
    NB_TYPE_CASTER(cnoid::ItemList<ItemType>, const_name("ItemList"))

    bool from_python(handle, uint8_t, cleanup_list*) noexcept {
        return false;
    }

    static handle from_cpp(const cnoid::ItemList<ItemType>& src, rv_policy, cleanup_list*) noexcept {
        list retval;
        for(size_t i = 0; i < src.size(); ++i){
            retval.append(cast(src[i]));
        }
        return retval.release();
    }
};

} // namespace detail
} // namespace nanobind

namespace cnoid {

CNOID_EXPORT nanobind::list getPyNarrowedItemList(const ItemList<>& orgItemList, nanobind::object itemClass);
CNOID_EXPORT nanobind::list getPyNarrowedItemList(nanobind::list orgItemList, nanobind::object itemClass);
CNOID_EXPORT nanobind::object getPyNarrowedFirstItem(const ItemList<>& orgItemList, nanobind::object itemClass);

template<typename ItemType>
class PyItemList
{
    static nanobind::object itemType;

    static nanobind::list construct1(nanobind::list items){
        return getPyNarrowedItemList(items, nanobind::borrow(nanobind::type<ItemType>()));
    }
    static nanobind::list construct2(nanobind::list items){
        return getPyNarrowedItemList(items, itemType);
    }
public:
    PyItemList(nanobind::module_& m, const char* name){
        m.def(name, &PyItemList::construct1);
    }
    /**
       Use this constructor to avoid a compile error for a pure abstract class
    */
    PyItemList(nanobind::module_& m, const char* name, nanobind::object itemType_){
        itemType = itemType_;
        m.def(name, &PyItemList::construct2);
    }
};

template<typename ItemType> nanobind::object PyItemList<ItemType>::itemType;

}

#endif
