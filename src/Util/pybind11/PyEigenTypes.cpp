/*!
  @author Shin'ichiro Nakaoka
*/

#include "PyUtil.h"
#include "PySignal.h"
#include "../EigenTypes.h"
#include "../EigenUtil.h"

namespace py = pybind11;
using namespace cnoid;

namespace {

Matrix3 angleAxis(double angle, const Vector3& vec){
    return Matrix3(AngleAxis(angle, vec));
}

Affine3 angleAxis44(double angle, const Vector3& vec){
    return Affine3(AngleAxis(angle, vec));
}

Vector3 getNormalized(const Vector3& vec){
    return vec.normalized();
}

Affine3 rotFromRpy44(const Vector3& vec){
    return Affine3(rotFromRpy(vec));
}

Vector3 getUnitX(){
    return Vector3::UnitX();
}

Vector3 getUnitY(){
    return Vector3::UnitY();
}

Vector3 getUnitZ(){
    return Vector3::UnitZ();
}

}

namespace cnoid {

void exportPyEigenTypes(py::module& m)
{
    m.def("rpyFromRot", &cnoid::rpyFromRot);
    m.def("rotFromRpy", (Matrix3 (*)(const Vector3&)) &cnoid::rotFromRpy);
    m.def("rotFromRpy44", &rotFromRpy44);
    m.def("omegaFromRot", &cnoid::omegaFromRot);
    m.def("angleAxis", &angleAxis);
    m.def("angleAxis44", &angleAxis44);
    m.def("normalized", &getNormalized);
    m.def("unitX", &getUnitX);
    m.def("unitY", &getUnitY);
    m.def("unitZ", &getUnitZ);

    PySignal<void(const Affine3&)>(m, "Affine3Signal");
}

}
