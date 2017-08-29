/*!
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_PYUTIL_H
#define CNOID_UTIL_PYUTIL_H

#include "../Referenced.h"
#include <cnoid/Config>
#include <pybind11/pybind11.h>

PYBIND11_DECLARE_HOLDER_TYPE(T, cnoid::ref_ptr<T>, true);

#endif
