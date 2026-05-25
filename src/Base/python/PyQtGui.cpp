#include "PyQString.h"
#include "PyQtSignal.h"
#include <QRegion>

namespace nb = nanobind;
using namespace cnoid;

NB_MODULE(QtGui, m)
{
    m.doc() = "Choreonoid QtGui module";

    nb::module_::import_("cnoid.QtCore");

    nb::class_<QRegion>(m, "QRegion")
        .def(nb::init<>())
        ;
}
