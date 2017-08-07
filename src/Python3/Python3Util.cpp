/*!
  @author Shin'ichiro Nakaoka
*/

#include <Python.h>
#include "Python3Util.h"

void cnoid::python3::handlePythonException()
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
