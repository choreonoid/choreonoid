/*!
  @author Shin'ichiro Nakaoka
*/

#include "PyQObjectHolder.h"
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

using namespace cnoid;
namespace py = pybind11;

namespace cnoid {

void exportPyQtCoreQtNamespace(py::module m);

}

PYBIND11_MODULE(QtCore, m)
{
    m.doc() = "Choreonoid QtCore module";

    py::class_<QObject, PyQObjectHolder<QObject>>(m,"QObject")
        .def("blockSignals", &QObject::blockSignals)
        .def("inherits", &QObject::inherits)
        .def("isWidgetType", &QObject::isWidgetType)
        .def("killTimer", &QObject::killTimer)
        .def("objectName", &QObject::objectName)
        .def("setObjectName", &QObject::setObjectName)
        .def("parent", &QObject::parent)
        .def("setParent", &QObject::setParent)
        .def("deleteLater", &QObject::deleteLater)
        .def("startTimer", (int (QObject::*)(int, Qt::TimerType)) &QObject::startTimer)
        .def_static(
            "disconnect",
            [](const QMetaObject::Connection& connection){ return QObject::disconnect(connection); })

        // This is used to delete the object when the object cannot be deleted from the Python side
        .def("delete", [](QObject* self){ delete self; })
        ;

    auto qMetaObject = m.def_submodule("QMetaObject");
    
    py::class_<QMetaObject::Connection>(qMetaObject, "Connection")
        .def(py::init<>())
        .def(py::init<const QMetaObject::Connection&>())
        ;

    py::class_<QTimer, PyQObjectHolder<QTimer>, QObject> qTimer(m, "QTimer");

    typedef cnoid::QtSignal<decltype(&QTimer::timeout), void()> TimerSignal;
    cnoid::PyQtSignal<TimerSignal>(qTimer, "Signal");

    qTimer
        .def(py::init<>())
        .def_property("interval", &QTimer::interval, (void (QTimer::*)(int)) &QTimer::setInterval)
        .def("setInterval", (void (QTimer::*)(int)) &QTimer::setInterval)
        .def("isActive", &QTimer::isActive)
        .def("isSingleShot", &QTimer::isSingleShot)
        .def("setSingleShot", &QTimer::setSingleShot)
        .def("timerId", &QTimer::timerId)
        .def("start", (void (QTimer::*)()) &QTimer::start)
        .def("start", (void (QTimer::*)(int)) &QTimer::start)
        .def("stop", &QTimer::stop)
        .def_static("singleShot", (void(*)(int, const QObject*, const char*)) &QTimer::singleShot)
        .def_property_readonly("timeout", [](QTimer* self){ return TimerSignal(self, &QTimer::timeout); })
        ;

    py::class_<QTime>(m, "QTime")
        .def(py::init<>())
        .def("elapsed", &QTime::elapsed)
        .def("hour", &QTime::hour)
        .def("minute", &QTime::minute)
        .def("second", &QTime::second)
        .def("start", &QTime::start)
        ;

    py::class_<QVariant>(m, "QVariant")
        .def(py::init<>())
        .def(py::init<int>())
        .def(py::init<bool>())
        .def(py::init<double>())
        .def(py::init<const char*>())
        //.def(py::init([](std::nullptr_t){ return std::unique_ptr<QVariant>(new QVariant()); }))
        .def(py::init([](py::none){ return std::unique_ptr<QVariant>(new QVariant()); }))
        .def("clear", &QVariant::clear)
        .def("isNull", &QVariant::isNull)
        .def("isValid", &QVariant::isValid)
        .def("setValue", [](QVariant& self, bool v){ self.setValue(v); })
        .def("setValue", [](QVariant& self, int v){ self.setValue(v); })
        .def("setValue", [](QVariant& self, double v){ self.setValue(v); })
        .def("toBool", &QVariant::toBool)
        .def("toDouble", [](QVariant& self){ return self.toDouble(); })
        .def("toFloat",  [](QVariant& self){ return self.toFloat(); })
        .def("toInt",  [](QVariant& self){ return self.toInt(); })
        .def("toString", &QVariant::toString)
        .def("typeName", &QVariant::typeName)
        ;

    py::implicitly_convertible<int, QVariant>();
    py::implicitly_convertible<bool, QVariant>();
    py::implicitly_convertible<double, QVariant>();
    py::implicitly_convertible<const char*, QVariant>();
    py::implicitly_convertible<py::none, QVariant>();
    

    py::class_<QMargins>(m, "QMargins")
        .def(py::init<>())
        .def(py::init<int,int,int,int>())
        .def("bottom", &QMargins::bottom)
        .def("isNull", &QMargins::isNull)
        .def("left", &QMargins::left)
        .def("right", &QMargins::right)
        .def("setBottom", &QMargins::setBottom)
        .def("setLeft", &QMargins::setLeft)
        .def("setRight", &QMargins::setRight)
        .def("setTop", &QMargins::setTop)
        .def("top", &QMargins::top)
        ;

    py::class_<QPoint>(m, "QPoint")
        .def(py::init<>())
        .def(py::init<int,int>())
        .def("isNull", &QPoint::isNull)
        .def("manhattanLength", &QPoint::manhattanLength)
        .def("x", &QPoint::x)
        .def("y", &QPoint::y)
        .def("setX", &QPoint::setX)
        .def("setY", &QPoint::setY)
        ;

    py::class_<QSize>(m, "QSize")
        .def(py::init<>())
        .def(py::init<int,int>())
        .def("boundedTo", &QSize::boundedTo)
        .def("expandedTo", &QSize::expandedTo)
        .def("height", &QSize::height)
        .def("width", &QSize::width)
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
        .def("transposed", &QSize::transposed)
        ;
    
    py::class_<QRect>(m, "QRect")
        .def(py::init<>())
        .def(py::init<int,int,int,int>())
        .def("adjust", &QRect::adjust)
        .def("getAdjusted", &QRect::adjusted)
        .def("bottom", &QRect::bottom)
        .def("bottomLeft", &QRect::bottomLeft)
        .def("bottomRight", &QRect::bottomRight)
        .def("center", &QRect::center)
        .def("contains",
             (bool(QRect::*)(const QPoint&, bool)const) &QRect::contains,
             py::arg("point"), py::arg("proper") = false)
        .def("contains",
             (bool(QRect::*)(const QRect&, bool)const) &QRect::contains,
             py::arg("rectangle"), py::arg("proper") = false)
        .def("contains", (bool(QRect::*)(int, int)const) &QRect::contains)
        .def("contains", (bool(QRect::*)(int, int, bool)const) &QRect::contains)
        .def("height", &QRect::height)
        .def("intersected", &QRect::intersected)
        .def("intersects", &QRect::intersects)
        .def("isEmpty", &QRect::isEmpty)
        .def("isNull", &QRect::isNull)
        .def("isValid", &QRect::isValid)
        .def("left", &QRect::left)
        .def("marginsAdded", &QRect::marginsAdded)
        .def("marginsRemoved", &QRect::marginsRemoved)
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
        .def("right", &QRect::right)
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
        .def("size", &QRect::size)
        .def("top", &QRect::top)
        .def("topLeft", &QRect::topLeft)
        .def("topRight", &QRect::topRight)
        .def("translate", (void(QRect::*)(int, int)) &QRect::translate)
        .def("translate", (void(QRect::*)(const QPoint&)) &QRect::translate)
        .def("translated", (QRect(QRect::*)(int, int)const) &QRect::translated)
        .def("translated", (QRect(QRect::*)(const QPoint&)const) &QRect::translated)
        //.def("transposed", &QRect::transposed) // Not supported by Qt 5.5
        .def("united", &QRect::united)
        .def("width", &QRect::width)
        .def("x", &QRect::x)
        .def("y", &QRect::y)
        ;
    
    cnoid::exportPyQtCoreQtNamespace(m);
}
