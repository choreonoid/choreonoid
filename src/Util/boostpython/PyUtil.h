/*!
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_PYUTIL_H
#define CNOID_UTIL_PYUTIL_H

#include "../Referenced.h"
#include <cnoid/Config>
#include <boost/python.hpp>

namespace cnoid {

class PyGILock
{
    PyGILState_STATE gstate;
public:
    PyGILock(){
        gstate = PyGILState_Ensure();
    }
    ~PyGILock() {
        PyGILState_Release(gstate);
    }
};

template <typename T>
T* get_pointer(cnoid::ref_ptr<T> const& p)
{
    return p.get();
}

} // namespace cnoid

namespace boost { namespace python {

template <typename T>
struct pointee< cnoid::ref_ptr<T> >
{
    typedef T type;
};

}} // namespace boost::python

/**
   The following macro is used to avoid compile errors caused by a bug of VC++ 2015 Update 3
   http://stackoverflow.com/questions/38261530/unresolved-external-symbols-since-visual-studio-2015-update-3-boost-python-link
*/
#ifdef _MSC_VER
#define CNOID_PYTHON_DEFINE_GET_POINTER(CLASS) namespace boost { template <> CLASS const volatile * get_pointer<class CLASS const volatile >(class CLASS const volatile *c){ return c; } } 
#else
#define CNOID_PYTHON_DEFINE_GET_POINTER(classname)
#endif

#endif
