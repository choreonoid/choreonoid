#ifndef CNOID_UTIL_PY_REFERENCED_H
#define CNOID_UTIL_PY_REFERENCED_H

#include "../Referenced.h"
#include <pybind11/pybind11.h>

PYBIND11_DECLARE_HOLDER_TYPE(T, cnoid::ref_ptr<T>, true);

#endif
