#include "PyQString.h"
#include "PyQtSignal.h"
#include <nanobind/stl/vector.h>
#include <Qt>

namespace nb = nanobind;

namespace cnoid {

void exportPyQtCoreQtNamespace(nb::module_ m)
{
    auto qt = m.def_submodule("Qt");

    nb::enum_<Qt::AlignmentFlag>(qt, "AlignmentFlag", nb::is_arithmetic())
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

    nb::class_<QFlags<Qt::AlignmentFlag>>(qt, "Alignment")
        .def(nb::init<>())
        .def("__init__", [](QFlags<Qt::AlignmentFlag>* self, Qt::AlignmentFlag f){
            new(self) QFlags<Qt::AlignmentFlag>(f);
        })
        ;

    // Only the enum -> QFlags direction is registered so that an AlignmentFlag
    // (e.g. Qt.AlignCenter) can be passed where an Alignment is expected. The
    // bidirectional implicitly_convertible used in the pybind11 version causes an
    // infinite recursion (segfault) in nanobind.
    nb::implicitly_convertible<Qt::AlignmentFlag, QFlags<Qt::AlignmentFlag>>();

    nb::enum_<Qt::Orientation>(qt, "Orientation")
        .value("Horizontal", Qt::Horizontal)
        .value("Vertical", Qt::Vertical)
        .export_values();

    nb::enum_<Qt::CheckState>(qt, "CheckState")
        .value("Unchecked", Qt::Unchecked)
        .value("PartiallyChecked", Qt::PartiallyChecked)
        .value("Checked", Qt::Checked)
        .export_values();

    nb::enum_<Qt::ItemFlag>(qt, "ItemFlag", nb::is_arithmetic())
        .value("NoItemFlags", Qt::NoItemFlags)
        .value("ItemIsSelectable", Qt::ItemIsSelectable)
        .value("ItemIsEditable", Qt::ItemIsEditable)
        .value("ItemIsDragEnabled", Qt::ItemIsDragEnabled)
        .value("ItemIsDropEnabled", Qt::ItemIsDropEnabled)
        .value("ItemIsUserCheckable", Qt::ItemIsUserCheckable)
        .value("ItemIsEnabled", Qt::ItemIsEnabled)
        .value("ItemIsAutoTristate", Qt::ItemIsAutoTristate)
        .value("ItemIsUserTristate", Qt::ItemIsUserTristate)
        .value("ItemNeverHasChildren", Qt::ItemNeverHasChildren)
        .export_values();

    nb::class_<QFlags<Qt::ItemFlag>>(qt, "ItemFlags")
        .def(nb::init<>())
        .def("__init__", [](QFlags<Qt::ItemFlag>* self, Qt::ItemFlag f){
            new(self) QFlags<Qt::ItemFlag>(f);
        })
        .def("__init__",
             [](QFlags<Qt::ItemFlag>* self, const std::vector<Qt::ItemFlag>& flags){
                 int vflags = 0;
                 for(auto& flag : flags){
                     vflags |= flag;
                 }
                 new(self) QFlags<Qt::ItemFlag>(vflags);
             })
        ;

    nb::enum_<Qt::ItemDataRole>(qt, "ItemDataRole")
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
        .value("ForegroundRole", Qt::ForegroundRole)
        .value("CheckStateRole", Qt::CheckStateRole)
        .value("InitialSortOrderRole", Qt::InitialSortOrderRole)
        .value("AccessibleTextRole", Qt::AccessibleTextRole)
        .value("AccessibleDescriptionRole", Qt::AccessibleDescriptionRole)
        .value("UserRole", Qt::UserRole)
        .export_values();


    nb::enum_<Qt::SortOrder>(qt, "SortOrder")
        .value("AscendingOrder", Qt::AscendingOrder)
        .value("DescendingOrder", Qt::DescendingOrder)
        .export_values();

    nb::enum_<Qt::ScrollBarPolicy>(qt, "ScrollBarPolicy")
        .value("ScrollBarAsNeeded", Qt::ScrollBarAsNeeded)
        .value("ScrollBarAlwaysOff", Qt::ScrollBarAlwaysOff)
        .value("ScrollBarAlwaysOn", Qt::ScrollBarAlwaysOn)
        .export_values();

    nb::enum_<Qt::GestureType>(qt, "GestureType")
        .value("TapGesture", Qt::TapGesture)
        .value("TapAndHoldGesture", Qt::TapAndHoldGesture)
        .value("PanGesture", Qt::PanGesture)
        .value("PinchGesture", Qt::PinchGesture)
        .value("SwipeGesture", Qt::SwipeGesture)
        .value("CustomGesture", Qt::CustomGesture)
        .export_values();

    nb::enum_<Qt::GestureFlag>(qt, "GestureFlag")
        .value("DontStartGestureOnChildren", Qt::DontStartGestureOnChildren)
        .value("ReceivePartialGestures", Qt::ReceivePartialGestures)
        .value("IgnoredGesturesPropagateToParent", Qt::IgnoredGesturesPropagateToParent)
        .export_values();

    nb::class_<QFlags<Qt::GestureFlag>>(qt, "GestureFlags")
        .def(nb::init<>())
        .def("__init__", [](QFlags<Qt::GestureFlag>* self, Qt::GestureFlag f){
            new(self) QFlags<Qt::GestureFlag>(f);
        })
        .def("__init__",
             [](QFlags<Qt::GestureFlag>* self, const std::vector<Qt::GestureFlag>& flags){
                 int vflags = 0;
                 for(auto& flag : flags){
                     vflags |= flag;
                 }
                 new(self) QFlags<Qt::GestureFlag>(vflags);
             })
        ;

    nb::enum_<Qt::ShortcutContext>(qt, "ShortcutContext")
        .value("WidgetShortcut", Qt::WidgetShortcut)
        .value("WidgetWithChildrenShortcut", Qt::WidgetWithChildrenShortcut)
        .value("WindowShortcut", Qt::WindowShortcut)
        .value("ApplicationShortcut", Qt::ApplicationShortcut)
        .export_values();
}

}
