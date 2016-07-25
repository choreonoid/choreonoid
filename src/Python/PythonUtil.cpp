/*!
  @author Shin'ichiro Nakaoka
*/

#include <Python.h>
#include "PythonUtil.h"

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
