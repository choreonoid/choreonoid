/*!
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_PYITEMLIST_H
#define CNOID_BASE_PYITEMLIST_H

#include "../ItemList.h"
#include <cnoid/PyUtil>
#include "exportdecl.h"

namespace pybind11 { namespace detail {
    template<typename ItemType> struct type_caster<cnoid::ItemList<ItemType>> {
    public:
        PYBIND11_TYPE_CASTER(cnoid::ItemList<ItemType>, _("ItemList"));

        //Conversion part 1 (Python->C++)
        bool load(handle src, bool) {
            return false;
        }

        //Conversion part 2 (C++ -> Python)
        static handle cast(cnoid::ItemList<ItemType> src, return_value_policy, handle ) {
            pybind11::list retval;
            for(size_t i=0; i < src.size(); i++){
                retval.append(src[i]);
            }
            return  retval.inc_ref();
        }
    };
}}


namespace cnoid {

CNOID_EXPORT pybind11::list getPyNarrowedItemList(const ItemList<>& orgItemList, pybind11::object itemClass);
CNOID_EXPORT pybind11::list getPyNarrowedItemList(pybind11::list orgItemList, const PyTypeObject* itemClass);
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

        return getPyNarrowedItemList(items, itemClass->type);
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
