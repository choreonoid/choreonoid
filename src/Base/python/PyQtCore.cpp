#include "PyQString.h"
#include "PyQtSignal.h"
#include <QObject>
#include <QTimer>
#include <QElapsedTimer>
#include <QTime>
#include <QVariant>
#include <QMargins>
#include <QPoint>
#include <QSize>
#include <QRect>

using namespace cnoid;
namespace nb = nanobind;

namespace cnoid {

void exportPyQtCoreQtNamespace(nb::module_ m);

}

/*
   QObject lifetime management (nanobind).

   The pybind11 binding used a custom holder (PyQObjectHolder) that stored a
   reference count in the QObject's "pyref" dynamic property and deleted the
   object on destruction only when it had no Qt parent. nanobind has no custom
   holder mechanism, so the strategy here is different:

   - QObject-derived types are registered with nb::class_<T, Base> (no holder).
   - A C++ pointer returned to Python (e.g. QObject::parent) is bound with
     nb::rv_policy::reference so that nanobind treats the wrapper as a
     non-owning view (it never calls the destructor or operator delete). This
     leaves ownership with C++/Qt, which matches the pybind11 behavior of not
     deleting an object that still has a Qt parent. Forgetting the reference
     policy would make nanobind take ownership (automatic -> take_ownership for
     pointer returns) and delete the object when the wrapper is collected.
   - Objects created from Python via nb::init are owned by nanobind and their
     destructor runs when the wrapper is collected. This fits the common usage
     of holding e.g. a QTimer in a Python variable.
   - The explicit "delete" method deletes the C++ object and then invalidates
     the wrapper (inst_set_state with destruct = false) so that a later access
     through the now-dangling wrapper fails with a RuntimeWarning instead of
     touching freed memory.

   Transferring ownership to a Qt parent (e.g. when a Python-created widget is
   added to a layout or tool bar) is handled at those add methods in the widget
   bindings, not here.
*/

NB_MODULE(QtCore, m)
{
    m.doc() = "Choreonoid QtCore module";

    nb::class_<QObject>(m, "QObject")
        .def("blockSignals", &QObject::blockSignals)
        .def("inherits", &QObject::inherits)
        .def("isWidgetType", &QObject::isWidgetType)
        .def("killTimer", &QObject::killTimer)
        .def("objectName", &QObject::objectName)
        .def("setObjectName", (void (QObject::*)(const QString&)) &QObject::setObjectName)
        .def("parent", &QObject::parent, nb::rv_policy::reference)
        .def("setParent", &QObject::setParent)
        .def("deleteLater", &QObject::deleteLater)
        .def("startTimer", (int (QObject::*)(int, Qt::TimerType)) &QObject::startTimer)
        .def_static(
            "disconnect",
            [](const QMetaObject::Connection& connection){ return QObject::disconnect(connection); })

        // Delete the object when it cannot be deleted by Qt's parent ownership,
        // then invalidate the wrapper so subsequent access fails safely.
        .def("delete", [](nb::pointer_and_handle<QObject> self){
            delete self.p;
            nb::inst_set_state(self.h, /* ready = */ false, /* destruct = */ false);
        })
        ;

    auto qMetaObject = m.def_submodule("QMetaObject");

    nb::class_<QMetaObject::Connection>(qMetaObject, "Connection")
        .def(nb::init<>())
        .def(nb::init<const QMetaObject::Connection&>())
        ;

    nb::class_<QTimer, QObject> qTimer(m, "QTimer");

    typedef cnoid::QtSignal<decltype(&QTimer::timeout), void()> TimerSignal;
    cnoid::PyQtSignal<TimerSignal>(qTimer, "Signal");

    qTimer
        // Created on the heap (nb::new_) like the widgets: a QObject reparented
        // to a Qt parent must be a real heap object so that Qt can delete it.
        .def(nb::new_([]{ return new QTimer(); }))
        .def_prop_rw("interval", &QTimer::interval, (void (QTimer::*)(int)) &QTimer::setInterval)
        .def("setInterval", (void (QTimer::*)(int)) &QTimer::setInterval)
        .def("isActive", &QTimer::isActive)
        .def("isSingleShot", &QTimer::isSingleShot)
        .def("setSingleShot", &QTimer::setSingleShot)
        .def("timerId", &QTimer::timerId)
        .def("start", (void (QTimer::*)()) &QTimer::start)
        .def("start", (void (QTimer::*)(int)) &QTimer::start)
        .def("stop", &QTimer::stop)
        .def_static("singleShot", (void(*)(int, const QObject*, const char*)) &QTimer::singleShot)
        .def_prop_ro("timeout", [](QTimer* self){ return TimerSignal(self, &QTimer::timeout); })
        ;

    nb::class_<QElapsedTimer>(m, "QElapsedTimer")
        .def(nb::init<>())
        .def("elapsed", &QElapsedTimer::elapsed)
        .def("hasExpired", &QElapsedTimer::hasExpired)
        .def("invalidate", &QElapsedTimer::invalidate)
        .def("isValid", &QElapsedTimer::isValid)
        .def("msecsSinceReference", &QElapsedTimer::msecsSinceReference)
        .def("msecsTo", &QElapsedTimer::msecsTo)
        .def("nsecsElapsed", &QElapsedTimer::nsecsElapsed)
        .def("restart", &QElapsedTimer::start)
        .def("secsTo", &QElapsedTimer::secsTo)
        .def("start", &QElapsedTimer::start)
        .def_static("clockType", &QElapsedTimer::clockType)
        .def_static("isMonotonic", &QElapsedTimer::isMonotonic)
        ;

    nb::class_<QTime>(m, "QTime")
        .def(nb::init<>())
        .def("hour", &QTime::hour)
        .def("minute", &QTime::minute)
        .def("second", &QTime::second)
        ;

    nb::class_<QVariant>(m, "QVariant")
        .def(nb::init<>())
        .def(nb::init<int>())
        .def(nb::init<bool>())
        .def(nb::init<double>())
        .def(nb::init<const char*>())
        // Construct an invalid QVariant from Python's None.
        .def("__init__", [](QVariant* self, nb::handle src){
            if(!src.is_none()){
                throw nb::type_error("QVariant() accepts only None here");
            }
            new(self) QVariant();
        }, nb::arg("none").none())
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

    nb::implicitly_convertible<int, QVariant>();
    nb::implicitly_convertible<bool, QVariant>();
    nb::implicitly_convertible<double, QVariant>();
    nb::implicitly_convertible<const char*, QVariant>();

    nb::class_<QMargins>(m, "QMargins")
        .def(nb::init<>())
        .def(nb::init<int,int,int,int>())
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

    nb::class_<QPoint>(m, "QPoint")
        .def(nb::init<>())
        .def(nb::init<int,int>())
        .def("isNull", &QPoint::isNull)
        .def("manhattanLength", &QPoint::manhattanLength)
        .def("x", &QPoint::x)
        .def("y", &QPoint::y)
        .def("setX", &QPoint::setX)
        .def("setY", &QPoint::setY)
        ;

    nb::class_<QSize>(m, "QSize")
        .def(nb::init<>())
        .def(nb::init<int,int>())
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

    nb::class_<QRect>(m, "QRect")
        .def(nb::init<>())
        .def(nb::init<int,int,int,int>())
        .def("adjust", &QRect::adjust)
        .def("getAdjusted", &QRect::adjusted)
        .def("bottom", &QRect::bottom)
        .def("bottomLeft", &QRect::bottomLeft)
        .def("bottomRight", &QRect::bottomRight)
        .def("center", &QRect::center)
        .def("contains",
             (bool(QRect::*)(const QPoint&, bool)const) &QRect::contains,
             nb::arg("point"), nb::arg("proper") = false)
        .def("contains",
             (bool(QRect::*)(const QRect&, bool)const) &QRect::contains,
             nb::arg("rectangle"), nb::arg("proper") = false)
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
        .def("united", &QRect::united)
        .def("width", &QRect::width)
        .def("x", &QRect::x)
        .def("y", &QRect::y)
        ;

    cnoid::exportPyQtCoreQtNamespace(m);
}
