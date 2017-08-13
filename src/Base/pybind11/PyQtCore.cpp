/*!
  @author Shin'ichiro Nakaoka
*/

#include "PyQtCore.h"
#include <cnoid/PyUtil>
#include <QObject>
#include <QTimer>

namespace py = pybind11;

PYBIND11_PLUGIN(QtCore)
{
    py::module m("QtCore", "QtCore Python Module");

    py::class_<QObject>( m,"QObject" )
        .def("blockSignals", &QObject::blockSignals)
        .def("inherits", &QObject::inherits)
        .def("isWidgetType", &QObject::isWidgetType)
        .def("killTimer", &QObject::killTimer)
        .def("objectName", &QObject::objectName)
        .def("parent", &QObject::parent, py::return_value_policy::reference)
        .def("setObjectName", &QObject::setObjectName)
        .def("setParent", &QObject::setParent)
        .def("startTimer", &QObject::startTimer)
        .def("deleteLater", &QObject::deleteLater);

    py::class_<QTimer>(m, "QTimer")
        .def("interval", &QTimer::interval)
        .def("isActive", &QTimer::isActive)
        .def("isSingleShot", &QTimer::isSingleShot)
        .def("setInterval", &QTimer::setInterval)
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

    return m.ptr();
}
