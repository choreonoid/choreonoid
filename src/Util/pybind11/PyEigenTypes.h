#ifndef CNOID_UTIL_PY_EIGEN_TYPES_H
#define CNOID_UTIL_PY_EIGEN_TYPES_H

#include "../EigenTypes.h"
#include <pybind11/eigen.h>

namespace pybind11 { namespace detail {

/*
template<> struct type_caster<cnoid::Affine3>
{
    PYBIND11_TYPE_CASTER(cnoid::Affine3, _("Affine3"));
    
    bool load(handle src, bool) {
        try{
            value = src.cast<cnoid::Matrix4>();
        } catch(pybind11::cast_error error){
            return false;
        }
        return true;
    }
    
    static handle cast(const cnoid::Affine3& src, return_value_policy, handle) {
        return pybind11::cast(src.matrix()).release();
    }
};

template<> struct type_caster<cnoid::Isometry3>
{
    PYBIND11_TYPE_CASTER(cnoid::Isometry3, _("Isometry3"));
    
    bool load(handle src, bool) {
        try{  value = src.cast<cnoid::Matrix4>();
        } catch(pybind11::cast_error error){
            return false;
        }
        return true;
    }
    
    static handle cast(const cnoid::Isometry3& src, return_value_policy, handle) {
        return pybind11::cast(src.matrix()).release();
    }
};
*/

}} // namespace pybind11::detail

#endif
