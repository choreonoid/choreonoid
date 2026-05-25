#include "PyUtil.h"
#include "PyEigenTypes.h"
#include "PySignal.h"
#include "../EigenTypes.h"
#include "../EigenUtil.h"

namespace nb = nanobind;
using namespace cnoid;

namespace cnoid {

void exportPyEigenTypes(nb::module_& m)
{
    m.def("rpyFromRot", [](const python::Matrix3Arg& R){ return rpyFromRot(R.value); });
    m.def("rotFromRpy", [](const python::Vector3Arg& rpy){ return rotFromRpy(rpy.value); });
    m.def("rotFromRpy", (Matrix3 (*)(double, double, double)) &cnoid::rotFromRpy);
    m.def("rotFromRpy44", [](const python::Vector3Arg& v){ return Affine3(rotFromRpy(v.value)); });
    m.def("omegaFromRot", [](const python::Matrix3Arg& R){ return omegaFromRot(R.value); });
    m.def("normalized", [](const python::Vector3Arg& v){ return v.value.normalized(); });
    m.attr("UnitX") = Vector3::UnitX();
    m.attr("UnitY") = Vector3::UnitY();
    m.attr("UnitZ") = Vector3::UnitZ();

    nb::class_<AngleAxis>(m, "AngleAxis")
        .def(nb::init<>())
        .def("__init__", [](AngleAxis* self, const python::Matrix3Arg& R){ new(self) AngleAxis(R.value); })
        .def("__init__", [](AngleAxis* self, const AngleAxis::Scalar& angle, const python::Vector3Arg& axis){
            new(self) AngleAxis(angle, axis.value); })
        .def_prop_rw(
            "axis",
            [](AngleAxis& self){ return self.axis(); },
            [](AngleAxis& self, const python::Vector3Arg& a){ self.axis() = a.value; })
        .def_prop_rw(
            "angle",
            [](AngleAxis& self){ return self.angle(); },
            [](AngleAxis& self, double a){ self.angle() = a; })
        .def("toRotationMatrix", &AngleAxis::toRotationMatrix)
        .def("inverse", &AngleAxis::inverse)
        ;

    nb::class_<AngleAxisf>(m, "AngleAxisf")
        .def(nb::init<>())
        .def("__init__", [](AngleAxisf* self, const python::Matrix3fArg& R){ new(self) AngleAxisf(R.value); })
        .def("__init__", [](AngleAxisf* self, const AngleAxisf::Scalar& angle, const python::Vector3fArg& axis){
            new(self) AngleAxisf(angle, axis.value); })
        .def_prop_rw(
            "axis",
            [](AngleAxisf& self){ return self.axis(); },
            [](AngleAxisf& self, const python::Vector3fArg& a){ self.axis() = a.value; })
        .def_prop_rw(
            "angle",
            [](AngleAxisf& self){ return self.angle(); },
            [](AngleAxisf& self, float a){ self.angle() = a; })
        .def("toRotationMatrix", &AngleAxisf::toRotationMatrix)
        .def("inverse", &AngleAxisf::inverse)
        ;

    PySignal<void(const Vector3&)>(m, "Vector33Signal");
    PySignal<void(const Vector4&)>(m, "Vector43Signal");
    PySignal<void(const Matrix3&)>(m, "Matrix3Signal");
    PySignal<void(const Matrix4&)>(m, "Matrix4Signal");
    PySignal<void(const Affine3&)>(m, "Affine3Signal");
}

}
