/*!
  @author Shin'ichiro Nakaoka
*/

#include "PyEigenTypes.h"
#include "PySignal.h"
#include "../EigenTypes.h"
#include "../EigenUtil.h"

namespace py = pybind11;
using namespace cnoid;

namespace cnoid {

void exportPyEigenTypes(py::module& m)
{
    m.def("rpyFromRot", (Vector3 (*)(const Matrix3&)) &cnoid::rpyFromRot);
    m.def("rotFromRpy", (Matrix3 (*)(const Vector3&)) &cnoid::rotFromRpy);
    m.def("rotFromRpy", (Matrix3 (*)(double, double, double)) &cnoid::rotFromRpy);
    m.def("rotFromRpy44", [](const Vector3& v){ return Affine3(rotFromRpy(v)); });
    m.def("omegaFromRot", &cnoid::omegaFromRot);
    m.def("angleAxis", [](double angle, const Vector3& axis){ return Matrix3(AngleAxis(angle, axis)); });
    m.def("angleAxis44", [](double angle, const Vector3& axis){ return Affine3(AngleAxis(angle, axis)); });
    m.def("normalized", [](const Vector3& v){ return v.normalized(); });
    m.attr("UnitX") = Vector3::UnitX();
    m.attr("UnitY") = Vector3::UnitY();
    m.attr("UnitZ") = Vector3::UnitZ();

    py::class_<AngleAxis>(m, "AngleAxis")
        .def(py::init<>())
        .def(py::init<const Matrix3&>())
        .def("axis", [](AngleAxis& self){ return self.axis(); })
        .def("angle", [](AngleAxis& self){ return self.angle(); })
        ;

    // deprecated
    m.def("getUnitX", Vector3::UnitX);
    m.def("getUnitY", Vector3::UnitY);
    m.def("getUnitZ", Vector3::UnitZ);

    PySignal<void(const Vector3&)>(m, "Vector33Signal");
    PySignal<void(const Vector4&)>(m, "Vector43Signal");
    PySignal<void(const Matrix3&)>(m, "Matrix3Signal");
    PySignal<void(const Matrix4&)>(m, "Matrix4Signal");
    PySignal<void(const Affine3&)>(m, "Affine3Signal");
}

}
