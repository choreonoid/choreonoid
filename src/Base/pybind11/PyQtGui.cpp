/*!
  @author Shin'ichiro Nakaoka
*/

#include "PyQObjectHolder.h"
#include "PyQString.h"
#include "PyQtSignal.h"
#include <QRegion>

namespace py = pybind11;
using namespace cnoid;

PYBIND11_MODULE(QtGui, m)
{
    m.doc() = "Choreonoid QtGui module";

    py::module::import("cnoid.QtCore");

    py::class_<QRegion>(m, "QRegion")
        .def(py::init<>())
        ;
}
