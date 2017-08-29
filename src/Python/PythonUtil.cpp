/*!
  @author Shin'ichiro Nakaoka
*/

#include "PythonUtil.h"

#ifdef CNOID_USE_BOOST_PYTHON

#include <Python.h>

void cnoid::handlePythonException()
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

#endif
