/*!
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_PYBASE_H
#define CNOID_BASE_PYBASE_H

#include "../ItemList.h"
#include <cnoid/Py3Util>
#include "exportdecl.h"

namespace cnoid {

CNOID_EXPORT pybind11::list getPyNarrowedItemList(const ItemList<>& orgItemList, pybind11::object itemClass);
CNOID_EXPORT pybind11::list getPyNarrowedItemList(pybind11::list orgItemList, const PyTypeObject* itemClass);
CNOID_EXPORT pybind11::list getPyNarrowedItemList(pybind11::list orgItemList, pybind11::detail::type_info* itemClass);
CNOID_EXPORT pybind11::object getPyNarrowedFirstItem(const ItemList<>& orgItemList, pybind11::object itemClass);

template<typename ItemType>
class PyItemList
{
    static pybind11::object itemType;
    
    static pybind11::list construct1(pybind11::list items){
        auto &types = pybind11::detail::get_internals().registered_types_cpp;
        auto it = types.find(std::type_index(typeid(ItemType)));
        pybind11::detail::type_info* itemClass = 0;
        if (it != types.end())
            itemClass = (pybind11::detail::type_info *) it->second;

        return getPyNarrowedItemList(items, itemClass);
    }
    static pybind11::list construct2(pybind11::list items){
        return getPyNarrowedItemList(items, (PyTypeObject*)itemType.ptr());
    }
public:
    PyItemList(pybind11::module m, const char* name){
        m.def(name, &PyItemList::construct1);
    }
    /**
       Use this constructor to avoid a compile error for a pure abstract class
    */
    PyItemList(pybind11::module m, const char* name, pybind11::object itemType_){
        itemType = itemType_;
        m.def(name, &PyItemList::construct2);
    }
};

template<typename ItemType> pybind11::object PyItemList<ItemType>::itemType;

}

#endif
