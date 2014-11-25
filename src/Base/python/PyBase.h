/*!
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_PYBASE_H
#define CNOID_BASE_PYBASE_H

#include "../ItemList.h"
#include <cnoid/PyUtil>

namespace cnoid {

boost::python::object getPyItemClass();

boost::python::list getPyNarrowedItemList(const ItemList<>& orgItemList, boost::python::object itemClass);
boost::python::object getPyNarrowedFirstItem(const ItemList<>& orgItemList, boost::python::object itemClass);

}

#endif
