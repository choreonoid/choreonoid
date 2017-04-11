/*!
  @author Shin'ichiro Nakaoka
*/

#include <cnoid/Py3Util>
#include <QObject>
#include <QTimer>

namespace py = pybind11;

// for MSVC++2015 Update3
CNOID_PYTHON_DEFINE_GET_POINTER(QObject)

namespace pybind11 { namespace detail {
    template <> struct type_caster<QString> {
    public:
        PYBIND11_TYPE_CASTER(QString, _("QString"));

        //Conversion part 1 (Python->C++)
        bool load(handle src, bool) {
            PyObject *source = src.ptr();
            if (PyBytes_Check(source)) {
                value = PyBytes_AS_STRING(source);
            }
            return !PyErr_Occurred();
        }

        //Conversion part 2 (C++ -> Python)
        static handle cast(QString src, return_value_policy, handle ) {
            QByteArray ba = src.toUtf8();
            return  PyByteArray_FromStringAndSize( ba.constData(), ba.size() );
        }
    };
}}

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
        .def_static("singleShot", (void (*singleShotPtr) (int, const QObject*, const char*)) &QTimer::singleShot)
#endif
        ;

    return m.ptr();
}

