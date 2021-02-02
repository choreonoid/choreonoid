#include "PyQObjectHolder.h"
#include "PyQString.h"
#include "PyQtSignal.h"
#include <QBoxLayout>
#include <QLayoutItem>
#include <QWidget>

namespace py = pybind11;

namespace cnoid {

void exportPyQtLayoutClasses(py::module m)
{
    py::class_<QLayoutItem, std::unique_ptr<QLayoutItem, py::nodelete>>(m, "QLayoutItem")
        .def_property_readonly("alignment", &QLayoutItem::alignment)
        .def_property_readonly("controlTypes", &QLayoutItem::controlTypes)
        .def_property_readonly("expandingDirections", &QLayoutItem::expandingDirections)
        .def_property_readonly("geometry", &QLayoutItem::geometry)
        .def("hasHeightForWidth", &QLayoutItem::hasHeightForWidth)
        .def("getHeightForWidth", &QLayoutItem::heightForWidth)
        .def("invalidate", &QLayoutItem::invalidate)
        .def("isEmpty", &QLayoutItem::isEmpty)
        .def_property_readonly("layout", &QLayoutItem::layout)
        .def_property_readonly("maximumSize", &QLayoutItem::maximumSize)
        .def("getMinimumHeightForWidth", &QLayoutItem::minimumHeightForWidth)
        .def_property_readonly("minimumSize", &QLayoutItem::minimumSize)
        .def("setAlignment", &QLayoutItem::setAlignment)
        .def("setGeometry", &QLayoutItem::setGeometry)
        .def_property_readonly("sizeHint", &QLayoutItem::sizeHint)
        .def_property_readonly("widget", &QLayoutItem::widget)
        ;

    /*
      Although QLayout inherits QLayoutItem, QLayoutItem is not specified as a base class
      because its holder type is different and pybind11 cannot mix the different holder types.
      The functions defined in QLayoutItem must be independently defined in the following binding
      if a function included in it is used in a Python script.
    */
    py::class_<QLayout, PyQObjectHolder<QLayout>, QObject>(m, "QLayout", py::multiple_inheritance())
        .def("activate", &QLayout::activate)
        .def("addItem", &QLayout::addItem)
        .def("addWidget", &QLayout::addWidget)
        .def_property_readonly("contentsMargins", &QLayout::contentsMargins)
        .def_property_readonly("contentsRect", &QLayout::contentsRect)
        .def_property_readonly("count", &QLayout::count)
        .def("getIndexOf", (int(QLayout::*)(QWidget*)const) &QLayout::indexOf)
        .def("getIndexOf", (int(QLayout::*)(QLayoutItem*)const) &QLayout::indexOf)
        .def("isEnabled", &QLayout::isEnabled)
        .def("itemAt", &QLayout::itemAt)
        .def_property_readonly("menuBar", &QLayout::menuBar)
        .def_property_readonly("parentWidget", &QLayout::parentWidget)
        .def("removeItem", &QLayout::removeItem)
        .def("removeWidget", &QLayout::removeWidget)
        .def("replaceWidget", &QLayout::replaceWidget)
        .def("setAlignment", (bool(QLayout::*)(QWidget*, Qt::Alignment)) &QLayout::setAlignment)
        .def("setAlignment", (bool(QLayout::*)(QLayout*, Qt::Alignment)) &QLayout::setAlignment)
        .def("setContentsMargins", (void(QLayout::*)(int,int,int,int)) &QLayout::setContentsMargins)
        .def("setContentsMargins", (void(QLayout::*)(const QMargins&)) &QLayout::setContentsMargins)
        .def("setEnabled", &QLayout::setEnabled)
        .def("setMenuBar", &QLayout::setMenuBar)
        .def("setSizeConstraint", &QLayout::setSizeConstraint)
        .def("setSpacing", &QLayout::setSpacing)
        .def_property_readonly("sizeConstraint", &QLayout::sizeConstraint)
        .def_property_readonly("spacing", &QLayout::spacing)
        .def("takeAt", &QLayout::takeAt)
        .def("update", &QLayout::update)
        ;

    py::class_<QBoxLayout, PyQObjectHolder<QBoxLayout>, QLayout>(m, "QBoxLayout")
        .def("addLayout", &QBoxLayout::addLayout, py::arg("layout"), py::arg("stretch") = 0)
        .def("addSpacerItem", &QBoxLayout::addSpacerItem)
        .def("addSpacing", &QBoxLayout::addSpacing)
        .def("addStretch", &QBoxLayout::addStretch, py::arg("stretch") = 0)
        .def("addStrut", &QBoxLayout::addStrut)
        .def("addWidget", &QBoxLayout::addWidget,
             py::arg("widget"), py::arg("stretch") = 0, py::arg("alignment") = Qt::Alignment())
        .def_property_readonly("direction", &QBoxLayout::direction)
        .def("insertItem", &QBoxLayout::insertItem)
        .def("insertLayout", &QBoxLayout::insertLayout,
             py::arg("index"), py::arg("layout"), py::arg("stretch") = 0)
        .def("insertSpacerItem", &QBoxLayout::insertSpacerItem)
        .def("insertSpacing", &QBoxLayout::insertSpacing)
        .def("insertStretch", &QBoxLayout::insertStretch, py::arg("index"), py::arg("stretch") = 0)
        .def("insertWidget", &QBoxLayout::insertWidget,
             py::arg("index"), py::arg("widget"), py::arg("stretch") = 0, py::arg("alignment") = Qt::Alignment())
        .def("setDirection", &QBoxLayout::setDirection)
        .def("setSpacing", &QBoxLayout::setSpacing)
        .def("setStretch", &QBoxLayout::setStretch)
        .def("setStretchFactor", (bool(QBoxLayout::*)(QWidget*,int)) &QBoxLayout::setStretchFactor)
        .def("setStretchFactor", (bool(QBoxLayout::*)(QLayout*,int)) &QBoxLayout::setStretchFactor)
        .def_property_readonly("spacing", &QBoxLayout::spacing)
        .def("getStretch", &QBoxLayout::stretch)
        ;
    
    py::class_<QHBoxLayout, PyQObjectHolder<QHBoxLayout>, QBoxLayout>(m, "QHBoxLayout")
        .def(py::init<>())
        .def(py::init<QWidget*>())
        ;
    
    py::class_<QVBoxLayout, PyQObjectHolder<QVBoxLayout>, QBoxLayout>(m, "QVBoxLayout")
        .def(py::init<>())
        .def(py::init<QWidget*>())
        ;
}

}
