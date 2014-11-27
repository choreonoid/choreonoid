/*!
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_PYBASE_H
#define CNOID_BASE_PYBASE_H

#include "../ItemList.h"
#include <cnoid/PyUtil>
#include "exportdecl.h"

namespace cnoid {

CNOID_EXPORT boost::python::list getPyNarrowedItemList(const ItemList<>& orgItemList, boost::python::object itemClass);
CNOID_EXPORT boost::python::list getPyNarrowedItemList(boost::python::list orgItemList, const PyTypeObject* itemClass);
CNOID_EXPORT boost::python::object getPyNarrowedFirstItem(const ItemList<>& orgItemList, boost::python::object itemClass);

template<typename ItemType>
class PyItemList
{
    static boost::python::object itemType;
    
    static boost::python::list construct1(boost::python::list items){
        return getPyNarrowedItemList(items, boost::python::converter::registered_pytype<ItemType>::get_pytype());
    }
    static boost::python::list construct2(boost::python::list items){
        return getPyNarrowedItemList(items, (PyTypeObject*)itemType.ptr());
    }
public:
    PyItemList(const char* name){
        boost::python::def(name, &PyItemList::construct1);
    }
    /**
       Use this constructor to avoid a compile error for a pure abstract class
    */
    PyItemList(const char* name, boost::python::object itemType_){
        itemType = itemType_;
        boost::python::def(name, &PyItemList::construct2);
    }
};

template<typename ItemType> boost::python::object PyItemList<ItemType>::itemType;

}

#endif
