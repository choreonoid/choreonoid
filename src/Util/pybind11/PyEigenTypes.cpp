/*!
  @author Shin'ichiro Nakaoka
*/

#include "PyUtil.h"
#include "../EigenTypes.h"
#include "../EigenUtil.h"

namespace py = pybind11;
using namespace cnoid;

namespace cnoid {

void exportPyEigenTypes(py::module& m)
{
    m.def("rpyFromRot", [](Eigen::Ref<const Matrix3> R){ return rpyFromRot(R); });
    m.def("rotFromRpy", [](Eigen::Ref<const Vector3> rpy){ return rotFromRpy(rpy); });
    m.def("rotFromRpy", (Matrix3 (*)(double, double, double)) &cnoid::rotFromRpy);
    m.def("rotFromRpy44", [](Eigen::Ref<const Vector3> v){ return Affine3(rotFromRpy(v)); });
    m.def("omegaFromRot", [](Eigen::Ref<const Matrix3> R){ return omegaFromRot(R); });
    m.def("normalized", [](Eigen::Ref<const Vector3> v){ return v.normalized(); });
    m.attr("UnitX") = Vector3::UnitX();
    m.attr("UnitY") = Vector3::UnitY();
    m.attr("UnitZ") = Vector3::UnitZ();

    py::class_<AngleAxis>(m, "AngleAxis")
        .def(py::init<>())
        .def(py::init<Eigen::Ref<const Matrix3>>())
        .def(py::init<const AngleAxis::Scalar&, Eigen::Ref<const AngleAxis::Vector3>>())
        .def_property(
            "axis",
            [](AngleAxis& self){ return self.axis(); },
            [](AngleAxis& self, Eigen::Ref<const Vector3> a){ self.axis() = a; })
        .def_property(
            "angle",
            [](AngleAxis& self){ return self.angle(); },
            [](AngleAxis& self, double a){ self.angle() = a; })
        .def("toRotationMatrix", &AngleAxis::toRotationMatrix)
        .def("inverse", &AngleAxis::inverse)
        ;

    py::class_<AngleAxisf>(m, "AngleAxisf")
        .def(py::init<>())
        .def(py::init<Eigen::Ref<const Matrix3f>>())
        .def(py::init<const AngleAxisf::Scalar&, Eigen::Ref<const Vector3f>>())
        .def_property(
            "axis",
            [](AngleAxisf& self){ return self.axis(); },
            [](AngleAxisf& self, Eigen::Ref<const Vector3f> a){ self.axis() = a; })
        .def_property(
            "angle",
            [](AngleAxisf& self){ return self.angle(); },
            [](AngleAxisf& self, float a){ self.angle() = a; })
        .def("toRotationMatrix", &AngleAxisf::toRotationMatrix)
        .def("inverse", &AngleAxisf::inverse)
        ;
    
    // deprecated
    m.def("getUnitX", Vector3::UnitX);
    m.def("getUnitY", Vector3::UnitY);
    m.def("getUnitZ", Vector3::UnitZ);
    m.def("angleAxis", [](double angle, Eigen::Ref<const Vector3> axis){ return Matrix3(AngleAxis(angle, axis)); });
    m.def("angleAxis44", [](double angle, Eigen::Ref<const Vector3> axis){ return Affine3(AngleAxis(angle, axis)); });

    PySignal<void(const Vector3&)>(m, "Vector33Signal");
    PySignal<void(const Vector4&)>(m, "Vector43Signal");
    PySignal<void(const Matrix3&)>(m, "Matrix3Signal");
    PySignal<void(const Matrix4&)>(m, "Matrix4Signal");
    PySignal<void(const Affine3&)>(m, "Affine3Signal");
}

}
