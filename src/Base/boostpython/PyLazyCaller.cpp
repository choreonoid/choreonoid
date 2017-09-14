/*!
  @author Shin'ichiro Nakaoka
*/

#include "../LazyCaller.h"
#include <cnoid/PyUtil>

using namespace cnoid;

namespace {

struct PyFunc
{
    python::object func;
    PyFunc(python::object f) : func(f) {
        if(!PyFunction_Check(f.ptr()) && !PyMethod_Check(f.ptr())){
            PyErr_SetString(PyExc_TypeError, "Task command must be a function type object");
            python::throw_error_already_set();
        }
    }
    void operator()() {
        python::gil_scoped_acquire lock;
        try {
            func();
        } catch(python::error_already_set const& ex) {
            python::handleException();
        }
    }
};


void cnoid_callLater(python::object func)
{
    cnoid::callLater(PyFunc(func));
}


void cnoid_callSynchronously(python::object func)
{
    cnoid::callSynchronously(PyFunc(func));
}


} // namespace

namespace cnoid {

void exportPyLazyCaller()
{
    python::def("callLater", cnoid_callLater);
    python::def("callSynchronously", cnoid_callSynchronously);
}

}
