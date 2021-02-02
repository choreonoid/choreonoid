#include "PyQString.h"
#include "PyQtSignal.h"
#include <pybind11/stl.h>
#include <Qt>

namespace py = pybind11;

namespace cnoid {

void exportPyQtCoreQtNamespace(py::module m)
{
    auto qt = m.def_submodule("Qt");
    
    py::enum_<Qt::AlignmentFlag>(qt, "AlignmentFlag", py::arithmetic())
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

    py::enum_<Qt::ItemFlag>(qt, "ItemFlag", py::arithmetic())
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
        .def(py::init(
                 [](const std::vector<Qt::ItemFlag>& flags){
                     int vflags = 0;
                     for(auto& flag : flags){
                         vflags |= flag;
                     }
                     return std::unique_ptr<QFlags<Qt::ItemFlag>>(new QFlags<Qt::ItemFlag>(vflags));
                 }))
        ;

    py::enum_<Qt::ItemDataRole>(qt, "ItemDataRole")
        .value("DisplayRole", Qt::DisplayRole)
        .value("DecorationRole", Qt::DecorationRole)
        .value("EditRole", Qt::EditRole)
        .value("ToolTipRole", Qt::ToolTipRole)
        .value("StatusTipRole", Qt::StatusTipRole)
        .value("WhatsThisRole", Qt::WhatsThisRole)
        .value("SizeHintRole", Qt::SizeHintRole)
        .value("FontRole", Qt::FontRole)
        .value("TextAlignmentRole", Qt::TextAlignmentRole)
        .value("BackgroundRole", Qt::BackgroundRole)
        .value("BackgroundColorRole", Qt::BackgroundColorRole)
        .value("ForegroundRole", Qt::ForegroundRole)
        .value("TextColorRole", Qt::TextColorRole)
        .value("CheckStateRole", Qt::CheckStateRole)
        .value("InitialSortOrderRole", Qt::InitialSortOrderRole)
        .value("AccessibleTextRole", Qt::AccessibleTextRole)
        .value("AccessibleDescriptionRole", Qt::AccessibleDescriptionRole)
        .value("UserRole", Qt::UserRole)
        .export_values();
    
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

    py::enum_<Qt::GestureType>(qt, "GestureType")
        .value("TapGesture", Qt::TapGesture)
        .value("TapAndHoldGesture", Qt::TapAndHoldGesture)
        .value("PanGesture", Qt::PanGesture)
        .value("PinchGesture", Qt::PinchGesture)
        .value("SwipeGesture", Qt::SwipeGesture)
        .value("CustomGesture", Qt::CustomGesture)
        .export_values();
    
    py::enum_<Qt::GestureFlag>(qt, "GestureFlag")
        .value("DontStartGestureOnChildren", Qt::DontStartGestureOnChildren)
        .value("ReceivePartialGestures", Qt::ReceivePartialGestures)
        .value("IgnoredGesturesPropagateToParent", Qt::IgnoredGesturesPropagateToParent)
        .export_values();

    py::class_<QFlags<Qt::GestureFlag>>(qt, "GestureFlags")
        .def(py::init<>())
        .def(py::init<Qt::ItemFlag>())
        .def(py::init(
                 [](const std::vector<Qt::GestureFlag>& flags){
                     int vflags = 0;
                     for(auto& flag : flags){
                         vflags |= flag;
                     }
                     return std::unique_ptr<QFlags<Qt::GestureFlag>>(new QFlags<Qt::GestureFlag>(vflags));
                 }))
        ;

    py::implicitly_convertible<Qt::GestureFlag, QFlags<Qt::GestureFlag>>();
    py::implicitly_convertible<QFlags<Qt::GestureFlag>, Qt::GestureFlag>();

    py::enum_<Qt::ShortcutContext>(qt, "ShortcutContext")
        .value("WidgetShortcut", Qt::WidgetShortcut)
        .value("WidgetWithChildrenShortcut", Qt::WidgetWithChildrenShortcut)
        .value("WindowShortcut", Qt::WindowShortcut)
        .value("ApplicationShortcut", Qt::ApplicationShortcut)
        .export_values();
}

}
