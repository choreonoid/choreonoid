#include "PyUtil.h"

void cnoid::python::handleException()
{
    if(PyErr_Occurred()){
        PyObject* ptype;
        PyObject* pvalue;
        PyObject* ptraceback;
        PyErr_Fetch(&ptype, &pvalue, &ptraceback);
        PyErr_Restore(ptype, pvalue, ptraceback);
        PyErr_Print();
    }
}
