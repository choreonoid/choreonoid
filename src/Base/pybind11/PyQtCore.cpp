/*!
  @author Shin'ichiro Nakaoka
*/

#include "PyQString.h"
#include "PyQtSignal.h"
#include <QObject>
#include <QTimer>
#include <QTime>
#include <QVariant>
#include <QMargins>
#include <QPoint>
#include <QSize>
#include <QRect>

namespace py = pybind11;

namespace cnoid {

void exportPyQtCoreQtNamespace(py::module m);

}

PYBIND11_MODULE(QtCore, m)
{
    m.doc() = "Choreonoid QtCore module";

    py::class_<QObject>(m,"QObject")
        .def("blockSignals", &QObject::blockSignals)
        .def("inherits", &QObject::inherits)
        .def("isWidgetType", &QObject::isWidgetType)
        .def("killTimer", &QObject::killTimer)
        .def_property("objectName", &QObject::objectName, &QObject::setObjectName)
        .def_property_readonly("setObjectName", &QObject::setObjectName)
        .def_property("parent", &QObject::parent, &QObject::setParent, py::return_value_policy::reference)
        .def("setParent", &QObject::setParent)
        .def("deleteLater", &QObject::deleteLater)
        .def("startTimer", (int (QObject::*)(int, Qt::TimerType)) &QObject::startTimer)
        .def_static(
            "disconnect",
            [](const QMetaObject::Connection& connection){ return QObject::disconnect(connection); })

        // deprecated
        .def("getObjectName", &QObject::objectName)
        .def("getParent", &QObject::parent, py::return_value_policy::reference)
        ;

    auto qMetaObject = m.def_submodule("QMetaObject");
    
    py::class_<QMetaObject::Connection>(qMetaObject, "Connection")
        .def(py::init<>())
        .def(py::init<const QMetaObject::Connection&>())
        ;

    py::class_<QTimer, QObject> qTimer(m, "QTimer");

    typedef cnoid::QtSignal<decltype(&QTimer::timeout), void()> TimerSignal;
    cnoid::PyQtSignal<TimerSignal>(qTimer, "Signal");

    qTimer
        .def(py::init<>())
        .def_property("interval", &QTimer::interval, (void (QTimer::*)(int)) &QTimer::setInterval)
        .def("setInterval", (void (QTimer::*)(int)) &QTimer::setInterval)
        .def("isActive", &QTimer::isActive)
        .def("isSingleShot", &QTimer::isSingleShot)
        .def("setSingleShot", &QTimer::setSingleShot)
        .def_property_readonly("timerId", &QTimer::timerId)
        .def("start", (void (QTimer::*)()) &QTimer::start)
        .def("start", (void (QTimer::*)(int)) &QTimer::start)
        .def("stop", &QTimer::stop)
        .def_static("singleShot", (void(*)(int, const QObject*, const char*)) &QTimer::singleShot)
        .def_property_readonly("timeout", [](QTimer* self){ return TimerSignal(self, &QTimer::timeout); })

        // deprecated
        .def("getInterval", &QTimer::interval)
        .def("getTimerId", &QTimer::timerId)
        ;

    py::class_<QTime>(m, "QTime")
        .def(py::init<>())
        .def_property_readonly("elapsed", &QTime::elapsed)
        .def_property_readonly("hour", &QTime::hour)
        .def_property_readonly("minute", &QTime::minute)
        .def_property_readonly("second", &QTime::second)
        .def("start", &QTime::start)
        ;

    py::class_<QVariant>(m, "QVariant")
        .def(py::init<>())
        .def("clear", &QVariant::clear)
        .def("isNull", &QVariant::isNull)
        .def("isValid", &QVariant::isValid)
        .def("setValue", [](QVariant& self, bool v){ self.setValue(v); })
        .def("setValue", [](QVariant& self, int v){ self.setValue(v); })
        .def("setValue", [](QVariant& self, double v){ self.setValue(v); })
        .def("toBool", &QVariant::toBool)
        .def("toDouble", [](QVariant& self){ return self.toDouble(); })
        .def("toFloat",  [](QVariant& self){ return self.toFloat(); })
        .def("toString", &QVariant::toString)
        .def_property_readonly("typeName", &QVariant::typeName)
        ;

    py::class_<QMargins>(m, "QMargins")
        .def(py::init<>())
        .def(py::init<int,int,int,int>())
        .def_property("bottom", &QMargins::bottom, &QMargins::setBottom)
        .def("isNull", &QMargins::isNull)
        .def_property("left", &QMargins::left, &QMargins::setLeft)
        .def_property("right", &QMargins::right, &QMargins::setRight)
        .def("setBottom", &QMargins::setBottom)
        .def("setLeft", &QMargins::setLeft)
        .def("setRight", &QMargins::setRight)
        .def("setTop", &QMargins::setTop)
        .def_property("top", &QMargins::top, &QMargins::setTop)
        ;

    py::class_<QPoint>(m, "QPoint")
        .def(py::init<>())
        .def(py::init<int,int>())
        .def("isNull", &QPoint::isNull)
        .def_property_readonly("manhattanLength", &QPoint::manhattanLength)
        .def_property("x", &QPoint::x, &QPoint::setX)
        .def_property("y", &QPoint::y, &QPoint::setY)
        .def("setX", &QPoint::setX)
        .def("setY", &QPoint::setY)
        ;

    py::class_<QSize>(m, "QSize")
        .def(py::init<>())
        .def(py::init<int,int>())
        .def("getBoundedTo", &QSize::boundedTo)
        .def("getExpandedTo", &QSize::expandedTo)
        .def_property("height", &QSize::height, &QSize::setHeight)
        .def_property("width", &QSize::width, &QSize::setWidth)
        .def("isEmpty", &QSize::isEmpty)
        .def("isNull", &QSize::isNull)
        .def("isValid", &QSize::isValid)
        .def("scale", (void(QSize::*)(int, int, Qt::AspectRatioMode)) &QSize::scale)
        .def("scale", (void(QSize::*)(const QSize&, Qt::AspectRatioMode)) &QSize::scale)
        .def("getScaled", (QSize(QSize::*)(int, int, Qt::AspectRatioMode)const) &QSize::scaled)
        .def("getScaled", (QSize(QSize::*)(const QSize&, Qt::AspectRatioMode)const) &QSize::scaled)
        .def("setHeight", &QSize::setHeight)
        .def("setWidth", &QSize::setWidth)
        .def("transpose", &QSize::transpose)
        .def("getTransposed", &QSize::transposed)
        ;
    
    py::class_<QRect>(m, "QRect")
        .def(py::init<>())
        .def(py::init<int,int,int,int>())
        .def("adjust", &QRect::adjust)
        .def("getAdjusted", &QRect::adjusted)
        .def_property("bottom", &QRect::bottom, &QRect::setBottom)
        .def_property("bottomLeft", &QRect::bottomLeft, &QRect::setBottomLeft)
        .def_property("bottomRight", &QRect::bottomRight, &QRect::setBottomRight)
        .def_property_readonly("center", &QRect::center)
        .def("contains",
             (bool(QRect::*)(const QPoint&, bool)const) &QRect::contains,
             py::arg("point"), py::arg("proper") = false)
        .def("contains",
             (bool(QRect::*)(const QRect&, bool)const) &QRect::contains,
             py::arg("rectangle"), py::arg("proper") = false)
        .def("contains", (bool(QRect::*)(int, int)const) &QRect::contains)
        .def("contains", (bool(QRect::*)(int, int, bool)const) &QRect::contains)
        .def_property("height", &QRect::height, &QRect::setHeight)
        .def("getIntersected", &QRect::intersected)
        .def("intersects", &QRect::intersects)
        .def("isEmpty", &QRect::isEmpty)
        .def("isNull", &QRect::isNull)
        .def("isValid", &QRect::isValid)
        .def_property("left", &QRect::left, &QRect::setLeft)
        .def("getMarginsAdded", &QRect::marginsAdded)
        .def("getMarginsRemoved", &QRect::marginsRemoved)
        .def("moveBottom", &QRect::moveBottom)
        .def("moveBottomLeft", &QRect::moveBottomLeft)
        .def("moveBottomRight", &QRect::moveBottomRight)
        .def("moveCenter", &QRect::moveCenter)
        .def("moveLeft", &QRect::moveLeft)
        .def("moveRight", &QRect::moveRight)
        .def("moveTo", (void(QRect::*)(int, int)) &QRect::moveTo)
        .def("moveTo", (void(QRect::*)(const QPoint &position)) &QRect::moveTo)
        .def("moveTop", &QRect::moveTop)
        .def("moveTopLeft", &QRect::moveTopLeft)
        .def("moveTopRight", &QRect::moveTopRight)
        .def("getNormalized", &QRect::normalized)
        .def_property("right", &QRect::right, &QRect::setRight)
        .def("setBottom", &QRect::setBottom)
        .def("setBottomLeft", &QRect::setBottomLeft)
        .def("setBottomRight", &QRect::setBottomRight)
        .def("setCoords", &QRect::setCoords)
        .def("setHeight", &QRect::setHeight)
        .def("setLeft", &QRect::setLeft)
        .def("setRect", &QRect::setRect)
        .def("setRight", &QRect::setRight)
        .def("setSize", &QRect::setSize)
        .def("setTop", &QRect::setTop)
        .def("setTopLeft", &QRect::setTopLeft)
        .def("setTopRight", &QRect::setTopRight)
        .def("setWidth", &QRect::setWidth)
        .def("setX", &QRect::setX)
        .def("setY", &QRect::setY)
        .def_property_readonly("size", &QRect::size)
        .def_property("top", &QRect::top, &QRect::setTop)
        .def_property("topLeft", &QRect::topLeft, &QRect::setTopLeft)
        .def_property("topRight", &QRect::topRight, &QRect::setTopRight)
        .def("translate", (void(QRect::*)(int, int)) &QRect::translate)
        .def("translate", (void(QRect::*)(const QPoint&)) &QRect::translate)
        .def("getTranslated", (QRect(QRect::*)(int, int)const) &QRect::translated)
        .def("getTranslated", (QRect(QRect::*)(const QPoint&)const) &QRect::translated)
        //.def("getTransposed", &QRect::transposed) // Not supported by Qt 5.5
        .def("getUnited", &QRect::united)
        .def_property("width", &QRect::width, &QRect::setWidth)
        .def_property("x", &QRect::x, &QRect::setX)
        .def_property("y", &QRect::y, &QRect::setY)
        ;
    
    cnoid::exportPyQtCoreQtNamespace(m);
}
