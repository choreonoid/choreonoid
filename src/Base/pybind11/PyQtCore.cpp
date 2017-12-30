/*!
  @author Shin'ichiro Nakaoka
*/

#include "PyQString.h"
#include <QObject>
#include <QTimer>
#include <pybind11/pybind11.h>

namespace py = pybind11;

PYBIND11_MODULE(QtCore, m)
{
    m.doc() = "Choreonoid QtCore module";

    py::class_<QObject>(m,"QObject")
        .def("blockSignals", &QObject::blockSignals)
        .def("inherits", &QObject::inherits)
        .def("isWidgetType", &QObject::isWidgetType)
        .def("killTimer", &QObject::killTimer)
        .def("objectName", &QObject::objectName)
        .def("parent", &QObject::parent, py::return_value_policy::reference)
        .def("setObjectName", &QObject::setObjectName)
        .def("setParent", &QObject::setParent)
        .def("startTimer", (int (QObject::*)(int, Qt::TimerType)) &QObject::startTimer)
        .def("deleteLater", &QObject::deleteLater);

    py::class_<QTimer>(m, "QTimer")
        .def("interval", &QTimer::interval)
        .def("isActive", &QTimer::isActive)
        .def("isSingleShot", &QTimer::isSingleShot)
        .def("setInterval", (void (QTimer::*)(int)) &QTimer::setInterval)
        .def("setSingleShot", &QTimer::setSingleShot)
        .def("timerId", &QTimer::timerId)
        .def("start", (void (QTimer::*)()) &QTimer::start)
        .def("start", (void (QTimer::*)(int)) &QTimer::start)
        .def("stop", &QTimer::stop)
#if QT_VERSION < QT_VERSION_CHECK(5, 0, 0)
        .def_static("singleShot", &QTimer::singleShot)
#else
        .def_static("singleShot", (void(*)(int, const QObject*, const char*)) &QTimer::singleShot)
#endif
        ;
}
