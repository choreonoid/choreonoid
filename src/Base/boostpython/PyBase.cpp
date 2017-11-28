/*!
  @author Shin'ichiro Nakaoka
*/

#include "PyBase.h"

using namespace cnoid;

namespace {

void checkIfItemSubclass(PyObject* typeObject)
{
    PyObject* itemType = (PyObject*)python::converter::registered_pytype<Item>::get_pytype();
    int isSubclass = PyObject_IsSubclass(typeObject, itemType);
    if(isSubclass <= 0){
        PyErr_SetString(PyExc_TypeError, "argument for specifying the item list type must be an Item class");
        python::throw_error_already_set();
    }
}


} // namespace

namespace cnoid {

boost::python::list getPyNarrowedItemList(const ItemList<>& orgItemList, boost::python::object itemClass)
{
    checkIfItemSubclass(itemClass.ptr());

    python::list narrowedItemList;
    for(size_t i=0; i < orgItemList.size(); ++i){
        python::object item(orgItemList[i]);
        if(PyObject_IsInstance(item.ptr(), itemClass.ptr()) > 0){
            narrowedItemList.append(item);
        }
    }
    return narrowedItemList;
}

boost::python::list getPyNarrowedItemList(boost::python::list orgItemList, const PyTypeObject* itemClass)
{
    checkIfItemSubclass((PyObject*)itemClass);
    
    python::list narrowedItemList;
    const int n = python::len(orgItemList);
    for(int i=0; i < n; ++i){
        python::object item = orgItemList[i];
        if(PyObject_IsInstance(item.ptr(), (PyObject*)itemClass) > 0){
            narrowedItemList.append(item);
        }
    }
    return narrowedItemList;
}

boost::python::object getPyNarrowedFirstItem(const ItemList<>& orgItemList, boost::python::object itemClass)
{
    checkIfItemSubclass(itemClass.ptr());

    for(size_t i=0; i < orgItemList.size(); ++i){
        python::object item(orgItemList[i]);
        if(PyObject_IsInstance(item.ptr(), itemClass.ptr()) > 0){
            return item;
        }
    }
    return python::object();
}

} // namespace cnoid
