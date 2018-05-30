/*!
  @author Shin'ichiro Nakaoka
*/

#include <QWidget>
#include <QMainWindow>
#include <QToolButton>
#include <cnoid/PyUtil>

using namespace boost;
using namespace boost::python;

// for MSVC++2015 Update3
CNOID_PYTHON_DEFINE_GET_POINTER(QWidget)

namespace {

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(QAbstractButton_animateClick_overloads, animateClick, 0, 1)

}

BOOST_PYTHON_MODULE(QtGui)
{
    boost::python::import("cnoid.QtCore");
    
    void (QWidget::*QWidget_setParent1)(QWidget* parent) = &QWidget::setParent;
    void (QWidget::*QWidget_repaint1)() = &QWidget::repaint;
    void (QWidget::*QWidget_setFocus1)() = &QWidget::setFocus;
    void (QWidget::*QWidget_update1)() = &QWidget::update;
    
    class_<QWidget, QWidget*, bases<QObject>, boost::noncopyable>("QWidget")
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
        .def("parentWidget", &QWidget::parentWidget, return_value_policy<reference_existing_object>())
        .def("getParentWidget", &QWidget::parentWidget, return_value_policy<reference_existing_object>())
        .def("setParent", QWidget_setParent1)
        .def("setToolTip", &QWidget::setToolTip)
        .def("setWhatsThis", &QWidget::setWhatsThis)
        .def("setWindowIconText", &QWidget::setWindowIconText)
        
        .def("toolTip", &QWidget::toolTip)
        .def("getToolTip", &QWidget::toolTip)
        .def("whatsThis", &QWidget::whatsThis)
        .def("getWhatsThis", &QWidget::whatsThis)
        .def("window", &QWidget::window, return_value_policy<reference_existing_object>())
        .def("getWindow", &QWidget::window, return_value_policy<reference_existing_object>())
        .def("windowFilePath", &QWidget::windowFilePath)
        .def("getWindowFilePath", &QWidget::windowFilePath)
        .def("windowIconText", &QWidget::windowIconText)
        .def("getWindowIconText", &QWidget::windowIconText)
        .def("windowRole", &QWidget::windowRole)
        .def("getWindowRole", &QWidget::windowRole)
        .def("windowTitle", &QWidget::windowTitle)
        .def("getWindowTitle", &QWidget::windowTitle)

        .def("close", &QWidget::close)
        .def("hide", &QWidget::hide)
        .def("lower", &QWidget::lower)
        .def("raise", &QWidget::raise)
        .def("repaint", QWidget_repaint1)
        .def("setDisabled", &QWidget::setDisabled)
        .def("setEnabled", &QWidget::setEnabled)
        .def("setFocus", QWidget_setFocus1)
        .def("setHidden", &QWidget::setHidden)
        .def("setVisible", &QWidget::setVisible)
        .def("setWindowModified", &QWidget::setWindowModified)
        .def("setWindowTitle", &QWidget::setWindowTitle)
        .def("show", &QWidget::show)
        .def("showFullScreen", &QWidget::showFullScreen)
        .def("showMaximized", &QWidget::showMaximized)
        .def("showMinimized", &QWidget::showMinimized)
        .def("showNormal", &QWidget::showNormal)
        .def("update", QWidget_update1);


    class_<QMainWindow, QMainWindow*, bases<QWidget>, boost::noncopyable>("QMainWindow");

    //class_ < QButtonGroup, boost::noncopyable >("QButtonGroup", init<>());

    class_<QAbstractButton, QAbstractButton*, bases<QWidget>, boost::noncopyable>("QAbstractButton", no_init)
        .def("autoExclusive", &QAbstractButton::autoExclusive)
        .def("getAutoExclusive", &QAbstractButton::autoExclusive)
        .def("autoRepeat", &QAbstractButton::autoRepeat)
        .def("getAutoRepeat", &QAbstractButton::autoRepeat)
        .def("autoRepeatDelay", &QAbstractButton::autoRepeatDelay)
        .def("getAutoRepeatDelay", &QAbstractButton::autoRepeatDelay)
        .def("autoRepeatInterval", &QAbstractButton::autoRepeatInterval)
        .def("getAutoRepeatInterval", &QAbstractButton::autoRepeatInterval)
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
        .def("getText", &QAbstractButton::text)
        .def("animateClick", &QAbstractButton::animateClick, QAbstractButton_animateClick_overloads())
        .def("click", &QAbstractButton::click)
        .def("setChecked", &QAbstractButton::setChecked)
        //.def("setIconSize", QAbstractButton::setIconSize)
        .def("toggle", &QAbstractButton::toggle);
    
    class_<QToolButton, QToolButton*, bases<QAbstractButton>, boost::noncopyable>("QToolButton")
        .def("autoRaise", &QToolButton::autoRaise)
        .def("getAutoRaise", &QToolButton::autoRaise);
}
