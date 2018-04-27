/*!
  @author Shin'ichiro Nakaoka
*/

#include "PyQString.h"
#include <QWidget>
#include <QMainWindow>
#include <QToolButton>

namespace py = pybind11;

PYBIND11_MODULE(QtGui, m)
{
    m.doc() = "Choreonoid QtGui module";

    py::module::import("cnoid.QtCore");

    py::class_<QWidget, QObject>(m, "QWidget")
        .def("hasFocus", &QWidget::hasFocus)
        .def("isActiveWindow", &QWidget::isActiveWindow)
        .def("isAncestorOf", &QWidget::isAncestorOf)
        .def("isEnabled", &QWidget::isEnabled)
        .def("isEnabledTo", &QWidget::isEnabledTo)
        .def("isFullScreen", &QWidget::isFullScreen)
        .def("isHidden", &QWidget::isHidden)
        .def("isMaximized", &QWidget::isMaximized)
        .def("isMinimized", &QWidget::isMinimized)
        .def("isModal", &QWidget::isModal)
        .def("isVisible", &QWidget::isVisible)
        .def("isVisibleTo", &QWidget::isVisibleTo)
        .def("isWindow", &QWidget::isWindow)
        .def("isWindowModified", &QWidget::isWindowModified)
        .def_property("parentWidget",
                      &QWidget::parentWidget,
                      (void (QWidget::*)(QWidget* parent)) &QWidget::setParent,
                      py::return_value_policy::reference)
        .def("setParent",  (void (QWidget::*)(QWidget* parent)) &QWidget::setParent)
        .def_property("toolTip", &QWidget::toolTip, &QWidget::setToolTip)
        .def("setToolTip", &QWidget::setToolTip)
        .def_property("whatsThis", &QWidget::whatsThis, &QWidget::setWhatsThis)
        .def("setWhatsThis", &QWidget::setWhatsThis)
        .def_property("windowIconText", &QWidget::windowIconText, &QWidget::setWindowIconText)
        .def("setWindowIconText", &QWidget::setWindowIconText)
        .def_property_readonly("window", &QWidget::window, py::return_value_policy::reference)
        .def_property_readonly("windowFilePath", &QWidget::windowFilePath)
        .def_property_readonly("windowRole", &QWidget::windowRole)
        .def_property_readonly("windowTitle", &QWidget::windowTitle)

        .def("close", &QWidget::close)
        .def("hide", &QWidget::hide)
        .def("lower", &QWidget::lower)
        .def("raise", &QWidget::raise)
        .def("repaint", (void (QWidget::*)()) &QWidget::repaint)
        .def("setDisabled", &QWidget::setDisabled)
        .def("setEnabled", &QWidget::setEnabled)
        .def("setFocus", (void (QWidget::*)()) &QWidget::setFocus)
        .def("setHidden", &QWidget::setHidden)
        .def("setVisible", &QWidget::setVisible)
        .def("setWindowModified", &QWidget::setWindowModified)
        .def("setWindowTitle", &QWidget::setWindowTitle)
        .def("show", &QWidget::show)
        .def("showFullScreen", &QWidget::showFullScreen)
        .def("showMaximized", &QWidget::showMaximized)
        .def("showMinimized", &QWidget::showMinimized)
        .def("showNormal", &QWidget::showNormal)
        .def("update",  (void (QWidget::*)()) &QWidget::update)

        // deprecated
        .def("getParentWidget", &QWidget::parentWidget, py::return_value_policy::reference)
        .def("getToolTip", &QWidget::toolTip)
        .def("getWhatsThis", &QWidget::whatsThis)
        .def("getWindowIconText", &QWidget::windowIconText)
        .def("getWindow", &QWidget::window, py::return_value_policy::reference)
        .def("getWindowFilePath", &QWidget::windowFilePath)
        .def("getWindowRole", &QWidget::windowRole)
        .def("getWindowTitle", &QWidget::windowTitle)
        ;

    py::class_<QMainWindow, QWidget>(m, "QMainWindow");

    py::class_<QAbstractButton, QWidget>(m, "QAbstractButton")
        .def_property_readonly("autoExclusive", &QAbstractButton::autoExclusive)
        .def("setAutoExclusive", &QAbstractButton::setAutoExclusive)
        .def_property("autoRepeat", &QAbstractButton::autoRepeat, &QAbstractButton::setAutoRepeat)
        .def("setAutoRepeat", &QAbstractButton::setAutoRepeat)
        .def_property("autoRepeatDelay", &QAbstractButton::autoRepeatDelay, &QAbstractButton::setAutoRepeatDelay)
        .def("setAutoRepeatDelay", &QAbstractButton::setAutoRepeatDelay)
        .def_property("autoRepeatInterval", &QAbstractButton::autoRepeatInterval, &QAbstractButton::setAutoRepeatInterval)
        .def("setAutoRepeatInterval", &QAbstractButton::setAutoRepeatInterval)
        //.def("group", &QAbstractButton::group)
        //.def("icon", &QAbstractButton::icon)
        //.def("iconSize", &QAbstractButton::iconSize)
        .def("isCheckable", &QAbstractButton::isCheckable)
        .def("isChecked", &QAbstractButton::isChecked)
        .def("isDown", &QAbstractButton::isDown)
        .def("setCheckable", &QAbstractButton::setCheckable)
        .def("setDown", &QAbstractButton::setDown)
        //.def("setIcon", &QAbstractButton::setIcon)
        //.def("setShortcut", &QAbstractButton::setShortcut)
        //.def("shortcut", &QAbstractButton::shortcut)
        .def_property("text", &QAbstractButton::text, &QAbstractButton::setText)
        .def("getText", &QAbstractButton::text)
        .def("setText", &QAbstractButton::setText)
        .def("animateClick", [](QAbstractButton& self){ self.animateClick(); })
        .def("animateClick", &QAbstractButton::animateClick)
        .def("click", &QAbstractButton::click)
        .def("setChecked", &QAbstractButton::setChecked)
        //.def("setIconSize", QAbstractButton::setIconSize)
        .def("toggle", &QAbstractButton::toggle)
        
        // deprecated
        .def("getAutoExclusive", &QAbstractButton::autoExclusive)
        .def("getAutoRepeat", &QAbstractButton::autoRepeat)
        .def("getAutoRepeatDelay", &QAbstractButton::autoRepeatDelay)
        .def("getAutoRepeatInterval", &QAbstractButton::autoRepeatInterval)
        ;
    
    py::class_<QToolButton, QAbstractButton>(m, "QToolButton")
        .def_property_readonly("autoRaise", &QToolButton::autoRaise)

        // deprecated
        .def("getAutoRaise", &QToolButton::autoRaise)
        ;
}
