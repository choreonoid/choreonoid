/*!
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_PYUTIL_H
#define CNOID_UTIL_PYUTIL_H

#include "../Referenced.h"
#include "../EigenTypes.h"
#include <cnoid/Config>
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

PYBIND11_DECLARE_HOLDER_TYPE(T, cnoid::ref_ptr<T>, true);

namespace pybind11 {

class scoped_gil_release
{
    PyThreadState* state;
    scoped_gil_release(const scoped_gil_release& org){ }
    scoped_gil_release& operator=(const scoped_gil_release& org){ }
public:
    scoped_gil_release(){
        state = PyEval_SaveThread();
    }
    ~scoped_gil_release(){
        PyEval_RestoreThread(state);
    }
};

typedef call_guard<scoped_gil_release> release_gil;

namespace detail {

    template<> struct type_caster<cnoid::Position> {
    public:
        PYBIND11_TYPE_CASTER(cnoid::Position, _("Position"));

        //Conversion part 1 (Python->C++)
        bool load(handle src, bool) {
            try{
                Eigen::Matrix4d mat = src.cast<Eigen::Matrix4d>();
                value.linear() = mat.block(0, 0, 3, 3);
                value.translation() = mat.block(0, 3, 3, 1);
            }catch(pybind11::cast_error error){
                return false;
            }

            return true;
        }

        //Conversion part 2 (C++ -> Python)
        static handle cast(cnoid::Position src, return_value_policy, handle ) {
            Eigen::Matrix4d retval = Eigen::Matrix4d::Identity();
            retval.block(0, 0, 3, 3) = src.linear();
            retval.block(0, 3, 3, 1) = src.translation();
            return  pybind11::cast(&retval, pybind11::return_value_policy::reference_internal).inc_ref();
        }
    };

    template<> struct type_caster<cnoid::Affine3> {
    public:
        PYBIND11_TYPE_CASTER(cnoid::Affine3, _("Affine3"));

        //Conversion part 1 (Python->C++)
        bool load(handle src, bool) {
            try{
                Eigen::Matrix4d mat = src.cast<Eigen::Matrix4d>();
                value.linear() = mat.block(0, 0, 3, 3);
                value.translation() = mat.block(0, 3, 3, 1);
            }catch(pybind11::cast_error error){
                return false;
            }

            return true;
        }

        //Conversion part 2 (C++ -> Python)
        static handle cast(cnoid::Position src, return_value_policy, handle ) {
            Eigen::Matrix4d retval = Eigen::Matrix4d::Identity();
            retval.block(0, 0, 3, 3) = src.linear();
            retval.block(0, 3, 3, 1) = src.translation();
            return  pybind11::cast(&retval, pybind11::return_value_policy::reference_internal).inc_ref();
        }
    };
}} // namespace pybind11::detail

#endif
