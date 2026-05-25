#include "PyQString.h"
#include "PyQtSignal.h"
#include <cnoid/PyUtil>
#include <QBoxLayout>
#include <QLayoutItem>
#include <QWidget>

namespace nb = nanobind;

namespace cnoid {

void exportPyQtLayoutClasses(nb::module_ m)
{
    // QLayoutItem is not owned by Python (it is owned by the layout it belongs
    // to), so it is registered without an init and never deleted by nanobind.
    nb::class_<QLayoutItem>(m, "QLayoutItem")
        .def_prop_ro("alignment", &QLayoutItem::alignment)
        .def_prop_ro("controlTypes", &QLayoutItem::controlTypes)
        .def_prop_ro("expandingDirections", &QLayoutItem::expandingDirections)
        .def_prop_ro("geometry", &QLayoutItem::geometry)
        .def("hasHeightForWidth", &QLayoutItem::hasHeightForWidth)
        .def("getHeightForWidth", &QLayoutItem::heightForWidth)
        .def("invalidate", &QLayoutItem::invalidate)
        .def("isEmpty", &QLayoutItem::isEmpty)
        .def_prop_ro("layout", &QLayoutItem::layout, nb::rv_policy::reference)
        .def_prop_ro("maximumSize", &QLayoutItem::maximumSize)
        .def("getMinimumHeightForWidth", &QLayoutItem::minimumHeightForWidth)
        .def_prop_ro("minimumSize", &QLayoutItem::minimumSize)
        .def("setAlignment", &QLayoutItem::setAlignment)
        .def("setGeometry", &QLayoutItem::setGeometry)
        .def_prop_ro("sizeHint", &QLayoutItem::sizeHint)
        .def_prop_ro("widget", &QLayoutItem::widget, nb::rv_policy::reference)
        ;

    /*
      Although QLayout inherits QLayoutItem, QLayoutItem is not specified as a
      base class here. In the pybind11 version this was because the holder types
      differed; in nanobind there is no holder, but QLayoutItem is a non-first
      base of QLayout (multiple inheritance with an offset), which nanobind does
      not support as a registered base. The QLayoutItem functions are therefore
      redefined below where needed.
    */
    nb::class_<QLayout, QObject>(m, "QLayout")
        .def("activate", &QLayout::activate)
        .def("addItem", &QLayout::addItem)
        .def("addWidget",
             [](QLayout& self, python::OwnershipReleased<QWidget> w){
                 self.addWidget(w);
             }, nb::arg("widget"))
        .def_prop_ro("contentsMargins", &QLayout::contentsMargins)
        .def_prop_ro("contentsRect", &QLayout::contentsRect)
        .def_prop_ro("count", &QLayout::count)

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
        .def("getIndexOf", (int(QLayout::*)(const QWidget*)const) &QLayout::indexOf)
        .def("getIndexOf", (int(QLayout::*)(const QLayoutItem*)const) &QLayout::indexOf)
#else
        .def("getIndexOf", (int(QLayout::*)(QWidget*)const) &QLayout::indexOf)
        .def("getIndexOf", (int(QLayout::*)(QLayoutItem*)const) &QLayout::indexOf)
#endif

        .def("isEnabled", &QLayout::isEnabled)
        .def("itemAt", &QLayout::itemAt, nb::rv_policy::reference)
        .def_prop_ro("menuBar", &QLayout::menuBar, nb::rv_policy::reference)
        .def_prop_ro("parentWidget", &QLayout::parentWidget, nb::rv_policy::reference)
        .def("removeItem", &QLayout::removeItem)
        .def("removeWidget", &QLayout::removeWidget)
        .def("replaceWidget", &QLayout::replaceWidget, nb::rv_policy::reference)
        .def("setAlignment", (bool(QLayout::*)(QWidget*, Qt::Alignment)) &QLayout::setAlignment)
        .def("setAlignment", (bool(QLayout::*)(QLayout*, Qt::Alignment)) &QLayout::setAlignment)
        .def("setContentsMargins", (void(QLayout::*)(int,int,int,int)) &QLayout::setContentsMargins)
        .def("setContentsMargins", (void(QLayout::*)(const QMargins&)) &QLayout::setContentsMargins)
        .def("setEnabled", &QLayout::setEnabled)
        .def("setMenuBar", &QLayout::setMenuBar)
        .def("setSizeConstraint", &QLayout::setSizeConstraint)
        .def("setSpacing", &QLayout::setSpacing)
        .def_prop_ro("sizeConstraint", &QLayout::sizeConstraint)
        .def_prop_ro("spacing", &QLayout::spacing)
        .def("takeAt", &QLayout::takeAt, nb::rv_policy::reference)
        .def("update", &QLayout::update)
        ;

    nb::class_<QBoxLayout, QLayout>(m, "QBoxLayout")
        .def("addLayout",
             [](QBoxLayout& self, python::OwnershipReleased<QLayout> l, int stretch){
                 self.addLayout(l, stretch);
             }, nb::arg("layout"), nb::arg("stretch") = 0)
        .def("addSpacerItem", &QBoxLayout::addSpacerItem)
        .def("addSpacing", &QBoxLayout::addSpacing)
        .def("addStretch", &QBoxLayout::addStretch, nb::arg("stretch") = 0)
        .def("addStrut", &QBoxLayout::addStrut)
        .def("addWidget",
             [](QBoxLayout& self, python::OwnershipReleased<QWidget> w, int stretch, Qt::Alignment alignment){
                 self.addWidget(w, stretch, alignment);
             }, nb::arg("widget"), nb::arg("stretch") = 0, nb::arg("alignment") = Qt::Alignment())
        .def_prop_ro("direction", &QBoxLayout::direction)
        .def("insertItem", &QBoxLayout::insertItem)
        .def("insertLayout",
             [](QBoxLayout& self, int index, python::OwnershipReleased<QLayout> l, int stretch){
                 self.insertLayout(index, l, stretch);
             }, nb::arg("index"), nb::arg("layout"), nb::arg("stretch") = 0)
        .def("insertSpacerItem", &QBoxLayout::insertSpacerItem)
        .def("insertSpacing", &QBoxLayout::insertSpacing)
        .def("insertStretch", &QBoxLayout::insertStretch, nb::arg("index"), nb::arg("stretch") = 0)
        .def("insertWidget",
             [](QBoxLayout& self, int index, python::OwnershipReleased<QWidget> w, int stretch, Qt::Alignment alignment){
                 self.insertWidget(index, w, stretch, alignment);
             }, nb::arg("index"), nb::arg("widget"), nb::arg("stretch") = 0, nb::arg("alignment") = Qt::Alignment())
        .def("setDirection", &QBoxLayout::setDirection)
        .def("setSpacing", &QBoxLayout::setSpacing)
        .def("setStretch", &QBoxLayout::setStretch)
        .def("setStretchFactor", (bool(QBoxLayout::*)(QWidget*,int)) &QBoxLayout::setStretchFactor)
        .def("setStretchFactor", (bool(QBoxLayout::*)(QLayout*,int)) &QBoxLayout::setStretchFactor)
        .def_prop_ro("spacing", &QBoxLayout::spacing)
        .def("getStretch", &QBoxLayout::stretch)
        ;

    // Layouts are created on the heap (nb::new_) for the same reason as widgets:
    // a layout set on a widget (setLayout) or added to another layout (addLayout)
    // is owned and deleted by Qt, which requires a real heap object.
    nb::class_<QHBoxLayout, QBoxLayout>(m, "QHBoxLayout")
        .def(nb::new_([]{ return new QHBoxLayout(); }))
        .def(nb::new_([](QWidget* parent){ return new QHBoxLayout(parent); }))
        ;

    nb::class_<QVBoxLayout, QBoxLayout>(m, "QVBoxLayout")
        .def(nb::new_([]{ return new QVBoxLayout(); }))
        .def(nb::new_([](QWidget* parent){ return new QVBoxLayout(parent); }))
        ;
}

}
