#include "PyItemList.h"

using namespace cnoid;
namespace nb = nanobind;

namespace {

// Verifies that the given Python object is a subclass of the bound Item type.
// The Item class object is obtained via nb::type<Item>() instead of reaching
// into nanobind internals (the pybind11 version used get_internals()).
void checkIfItemSubclass(nb::handle typeObject)
{
    nb::handle itemClass = nb::type<Item>();
    int isSubclass = PyObject_IsSubclass(typeObject.ptr(), itemClass.ptr());
    if(isSubclass <= 0){
        PyErr_SetString(PyExc_TypeError, "argument for specifying the item list type must be an Item class");
        throw nb::python_error();
    }
}

}

namespace cnoid {

nb::list getPyNarrowedItemList(const ItemList<>& orgItemList, nb::object itemClass)
{
    checkIfItemSubclass(itemClass);

    nb::list narrowedItemList;
    for(size_t i=0; i < orgItemList.size(); ++i){
        nb::object item = nb::cast(orgItemList[i]);
        if(PyObject_IsInstance(item.ptr(), itemClass.ptr()) > 0){
            narrowedItemList.append(item);
        }
    }
    return narrowedItemList;
}

nb::list getPyNarrowedItemList(nb::list orgItemList, nb::object itemClass)
{
    checkIfItemSubclass(itemClass);

    nb::list narrowedItemList;
    for(auto item : orgItemList){
        if(PyObject_IsInstance(item.ptr(), itemClass.ptr()) > 0){
            narrowedItemList.append(item);
        }
    }
    return narrowedItemList;
}

nb::object getPyNarrowedFirstItem(const ItemList<>& orgItemList, nb::object itemClass)
{
    checkIfItemSubclass(itemClass);

    for(size_t i=0; i < orgItemList.size(); ++i){
        nb::object item = nb::cast(orgItemList[i]);
        if(PyObject_IsInstance(item.ptr(), itemClass.ptr()) > 0){
            return item;
        }
    }
    return nb::none();
}

}
