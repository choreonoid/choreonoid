/*!
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_PYUTIL_H
#define CNOID_UTIL_PYUTIL_H

#include "../Referenced.h"
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

#endif
