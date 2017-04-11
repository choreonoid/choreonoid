/**
  @author Shin'ichiro Nakaoka
*/

#include <pybind11/pybind11.h>
#include "omniORBpy.h"
#include <cnoid/CorbaUtil>

using namespace std;
using namespace cnoid;
namespace py =pybind11;

namespace {

    omniORBpyAPI* api;

    PyObject* getPyORB()
    {
        CORBA::ORB_ptr orb = getORB();
        PyObject* pyOrb = api->cxxObjRefToPyObjRef(orb, true);
        return pyOrb;
    }
}


PYBIND11_PLUGIN(Corba)
{
    py::module m("Corba", "Corba Python Module");

    //PyObject* omnipy = PyImport_ImportModule((char*)"_omnipy");
    //PyObject* pyapi = PyObject_GetAttrString(omnipy, (char*)"API");
    //PyObject* pyapi = PyCapsule_Import((char*)"_omnipy.API", false);
    //api = (omniORBpyAPI*) PyCapsule_GetPointer(pyapi, (char*)"_omnipy.API");
    //Py_DECREF(pyapi);
    api = (omniORBpyAPI*) PyCapsule_Import((char*)"_omnipy.API", false);

    m.def("getORB", &getPyORB);
}
