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
        .def("parentWidget", &QWidget::parentWidget, py::return_value_policy::reference)
        .def("setParent",  (void (QWidget::*)(QWidget* parent)) &QWidget::setParent)
        .def("setToolTip", &QWidget::setToolTip)
        .def("setWhatsThis", &QWidget::setWhatsThis)
        .def("setWindowIconText", &QWidget::setWindowIconText)
        
        .def("toolTip", &QWidget::toolTip)
        .def("whatsThis", &QWidget::whatsThis)
        .def("window", &QWidget::window, py::return_value_policy::reference)
        .def("windowFilePath", &QWidget::windowFilePath)
        .def("windowIconText", &QWidget::windowIconText)
        .def("windowRole", &QWidget::windowRole)
        .def("windowTitle", &QWidget::windowTitle)

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
        ;

    py::class_<QMainWindow, QWidget>(m, "QMainWindow");

    py::class_<QAbstractButton, QWidget>(m, "QAbstractButton")
        .def("autoExclusive", &QAbstractButton::autoExclusive)
        .def("autoRepeat", &QAbstractButton::autoRepeat)
        .def("autoRepeatDelay", &QAbstractButton::autoRepeatDelay)
        .def("autoRepeatInterval", &QAbstractButton::autoRepeatInterval)
        //.def("group", &QAbstractButton::group)
        //.def("icon", &QAbstractButton::icon)
        //.def("iconSize", &QAbstractButton::iconSize)
        .def("isCheckable", &QAbstractButton::isCheckable)
        .def("isChecked", &QAbstractButton::isChecked)
        .def("isDown", &QAbstractButton::isDown)
        .def("setAutoExclusive", &QAbstractButton::setAutoExclusive)
        .def("setAutoRepeat", &QAbstractButton::setAutoRepeat)
        .def("setAutoRepeatDelay", &QAbstractButton::setAutoRepeatDelay)
        .def("setAutoRepeatInterval", &QAbstractButton::setAutoRepeatInterval)
        .def("setCheckable", &QAbstractButton::setCheckable)
        .def("setDown", &QAbstractButton::setDown)
        //.def("setIcon", &QAbstractButton::setIcon)
        //.def("setShortcut", &QAbstractButton::setShortcut)
        .def("setText", &QAbstractButton::setText)
        //.def("shortcut", &QAbstractButton::shortcut)
        .def("text", &QAbstractButton::text)
        .def("animateClick", [](QAbstractButton& self){ self.animateClick(); })
        .def("animateClick", &QAbstractButton::animateClick)
        .def("click", &QAbstractButton::click)
        .def("setChecked", &QAbstractButton::setChecked)
        //.def("setIconSize", QAbstractButton::setIconSize)
        .def("toggle", &QAbstractButton::toggle);
    
    py::class_<QToolButton, QAbstractButton>(m, "QToolButton")
        .def("autoRaise", &QToolButton::autoRaise);
}
