#include "PyQString.h"
#include "PyQtSignal.h"
#include <Qt>

namespace py = pybind11;

namespace cnoid {

void exportPyQtCoreQtNamespace(py::module m)
{
    auto qt = m.def_submodule("Qt");
    
    py::enum_<Qt::AlignmentFlag>(qt, "AlignmentFlag")
        .value("AlignLeft", Qt::AlignLeft)
        .value("AlignRight", Qt::AlignRight)
        .value("AlignHCenter", Qt::AlignHCenter)
        .value("AlignJustify", Qt::AlignJustify)
        .value("AlignTop", Qt::AlignTop)
        .value("AlignBottom", Qt::AlignBottom)
        .value("AlignVCenter", Qt::AlignVCenter)
        .value("AlignBaseline", Qt::AlignBaseline)
        .value("AlignCenter", Qt::AlignCenter)
        .export_values();

    py::class_<QFlags<Qt::AlignmentFlag>>(qt, "Alignment")
        .def(py::init<>())
        .def(py::init<Qt::AlignmentFlag>())
        ;

    py::implicitly_convertible<Qt::AlignmentFlag, QFlags<Qt::AlignmentFlag>>();
    py::implicitly_convertible<QFlags<Qt::AlignmentFlag>, Qt::AlignmentFlag>();

    py::enum_<Qt::Orientation>(qt, "Orientation")
        .value("Horizontal", Qt::Horizontal)
        .value("Vertical", Qt::Vertical)
        .export_values();

    py::enum_<Qt::CheckState>(qt, "CheckState")
        .value("Unchecked", Qt::Unchecked)
        .value("PartiallyChecked", Qt::PartiallyChecked)
        .value("Checked", Qt::Checked)
        .export_values();

    py::enum_<Qt::ItemFlag>(qt, "ItemFlag")
        .value("NoItemFlags", Qt::NoItemFlags)
        .value("ItemIsSelectable", Qt::ItemIsSelectable)
        .value("ItemIsEditable", Qt::ItemIsEditable)
        .value("ItemIsDragEnabled", Qt::ItemIsDragEnabled)
        .value("ItemIsDropEnabled", Qt::ItemIsDropEnabled)
        .value("ItemIsUserCheckable", Qt::ItemIsUserCheckable)
        .value("ItemIsEnabled", Qt::ItemIsEnabled)
        //.value("ItemIsAutoTristate", Qt::ItemIsAutoTristate) // No supprted by Qt 5.5
        .value("ItemIsTristate", Qt::ItemIsTristate)
        .value("ItemNeverHasChildren", Qt::ItemNeverHasChildren)
        .value("ItemIsUserTristate", Qt::ItemIsUserTristate)
        .export_values();

    py::class_<QFlags<Qt::ItemFlag>>(qt, "ItemFlags")
        .def(py::init<>())
        .def(py::init<Qt::ItemFlag>())
        ;
    
    py::implicitly_convertible<Qt::ItemFlag, QFlags<Qt::ItemFlag>>();
    py::implicitly_convertible<QFlags<Qt::ItemFlag>, Qt::ItemFlag>();

    py::enum_<Qt::SortOrder>(qt, "SortOrder")
        .value("AscendingOrder", Qt::AscendingOrder)
        .value("DescendingOrder", Qt::DescendingOrder)
        .export_values();

    py::enum_<Qt::ScrollBarPolicy>(qt, "ScrollBarPolicy")
        .value("ScrollBarAsNeeded", Qt::ScrollBarAsNeeded)
        .value("ScrollBarAlwaysOff", Qt::ScrollBarAlwaysOff)
        .value("ScrollBarAlwaysOn", Qt::ScrollBarAlwaysOn)
        .export_values();
}

}
