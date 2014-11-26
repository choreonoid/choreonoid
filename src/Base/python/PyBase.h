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
    static boost::python::list construct(boost::python::list items){
        return getPyNarrowedItemList(items, boost::python::converter::registered_pytype<ItemType>::get_pytype());
    }
public:
    PyItemList(const char* name){
        boost::python::def(name, &PyItemList::construct);
    }
};

}

#endif
