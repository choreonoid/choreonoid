/*!
  @author Shin'ichiro Nakaoka
*/

#include "PyQString.h"
#include "PyQtSignal.h"
#include <QWidget>
#include <QLayout>
#include <QMainWindow>
#include <QPushButton>
#include <QToolButton>
#include <QCheckBox>
#include <QLabel>
#include <QSpinBox>
#include <QDialog>
#include <QFrame>
#include <QAbstractScrollArea>
#include <QMenu>

namespace py = pybind11;

namespace cnoid {

void exportPyQtGuiLayoutClasses(py::module m);
void exportPyQtGuiModelViewClasses(py::module m);

}

using namespace cnoid;

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
        .def("setLayout", &QWidget::setLayout)
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

        // Public slots
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

    py::class_<QAbstractButton, QWidget> qAbstractButton(m, "QAbstractButton");

    typedef cnoid::QtSignal<void(QAbstractButton::*)(bool), void()> ButtonClickSignal;
    cnoid::PyQtSignal<ButtonClickSignal>(qAbstractButton, "ClickSignal");
    typedef cnoid::QtSignal<void(QAbstractButton::*)(bool), void(bool)> ButtonBoolSignal;
    cnoid::PyQtSignal<ButtonBoolSignal>(qAbstractButton, "BoolSignal");

    qAbstractButton
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
        .def_property_readonly(
            "clicked",
            [](QAbstractButton* self){ return ButtonClickSignal(self, &QAbstractButton::clicked); })
        .def_property_readonly(
            "toggled",
            [](QAbstractButton* self){ return ButtonBoolSignal(self, &QAbstractButton::toggled); })
        
        // deprecated
        .def("getAutoExclusive", &QAbstractButton::autoExclusive)
        .def("getAutoRepeat", &QAbstractButton::autoRepeat)
        .def("getAutoRepeatDelay", &QAbstractButton::autoRepeatDelay)
        .def("getAutoRepeatInterval", &QAbstractButton::autoRepeatInterval)
        ;

    py::class_<QPushButton, std::unique_ptr<QPushButton, py::nodelete>, QAbstractButton>(m, "QPushButton")
        .def(py::init<QWidget*>(), py::arg("parent") = nullptr)
        .def(py::init<const QString&, QWidget*>(), py::arg("text"), py::arg("parent") = nullptr)
        .def_property_readonly("autoDefault", &QPushButton::autoDefault)
        .def("isDefault", &QPushButton::isDefault)
        .def("isFlat", &QPushButton::isFlat)
        .def_property_readonly("menu", &QPushButton::menu)
        .def("setAutoDefault", &QPushButton::setAutoDefault)
        .def("setDefault", &QPushButton::setDefault)
        .def("setFlat", &QPushButton::setFlat)
        .def("setMenu", &QPushButton::setMenu)
        .def("showMenu", &QPushButton::showMenu)
        ;
    
    py::class_<QToolButton, std::unique_ptr<QToolButton, py::nodelete>, QAbstractButton>(m, "QToolButton")
        .def_property_readonly("autoRaise", &QToolButton::autoRaise)

        // deprecated
        .def("getAutoRaise", &QToolButton::autoRaise)
        ;

    py::class_<QCheckBox, std::unique_ptr<QCheckBox, py::nodelete>, QAbstractButton>(m, "QCheckBox")
        .def(py::init<>())
        .def(py::init<const QString&>())
        ;

    py::class_<QLabel, std::unique_ptr<QLabel, py::nodelete>, QWidget>(m, "QLabel")
        .def(py::init<>())
        .def(py::init<const QString&>())
        .def("setText", &QLabel::setText)
        ;

    py::class_<QAbstractSpinBox, std::unique_ptr<QAbstractSpinBox, py::nodelete>, QWidget>(m, "QAbstractSpinBox")
        .def_property("alignment", &QAbstractSpinBox::alignment, &QAbstractSpinBox::setAlignment)
        .def("setAlignment", &QAbstractSpinBox::setAlignment)
        ;

    py::class_<QSpinBox, std::unique_ptr<QSpinBox, py::nodelete>, QAbstractSpinBox> qSpinBox(m, "QSpinBox");

    typedef cnoid::QtSignal<void(QSpinBox::*)(int), void(int)> SpinBoxIntSignal;
    cnoid::PyQtSignal<SpinBoxIntSignal>(qSpinBox, "IntSignal");

    qSpinBox
        .def(py::init<>())
        .def("setMaximum", &QSpinBox::setMaximum)
        .def("setMinimum", &QSpinBox::setMinimum)
        .def("setRange", &QSpinBox::setRange)
        .def_property("maximum", &QSpinBox::maximum, &QSpinBox::setMaximum)
        .def_property("minimum", &QSpinBox::minimum, &QSpinBox::setMinimum)
        .def("setSingleStep", &QSpinBox::setSingleStep)
        .def_property("value", &QSpinBox::value, &QSpinBox::setValue)
        .def("setValue", &QSpinBox::setValue)
        .def_property_readonly(
            "valueChanged",
            [](QSpinBox* self){ return SpinBoxIntSignal(self, &QSpinBox::valueChanged); })
        ;

    py::class_<QDialog, QWidget> qDialog(m, "QDialog");

    typedef cnoid::QtSignal<void(QDialog::*)(), void()> DialogSignal;
    cnoid::PyQtSignal<DialogSignal>(qDialog, "Signal");
    typedef cnoid::QtSignal<void(QDialog::*)(int), void(int)> DialogIntSignal;
    cnoid::PyQtSignal<DialogIntSignal>(qDialog, "IntSignal");

    qDialog
        .def(py::init<>())
        .def_property_readonly("result", &QDialog::result)
        .def("setResult", &QDialog::setResult)
        .def("setModal", &QDialog::setModal)
        .def("accept", &QDialog::accept)
        .def("done", &QDialog::done)
        .def("exec", &QDialog::exec)
        .def("open", &QDialog::open)
        .def("reject", &QDialog::reject)
        .def_property_readonly(
            "accepted",
            [](QDialog* self){ return DialogSignal(self, &QDialog::accepted); })
        .def_property_readonly(
            "finished",
            [](QDialog* self){ return DialogIntSignal(self, &QDialog::finished); })
        .def_property_readonly(
            "rejected",
            [](QDialog* self){ return DialogSignal(self, &QDialog::rejected); })
        ;

    py::class_<QFrame, QWidget>(m, "QFrame");

    py::enum_<Qt::ScrollBarPolicy>(m, "ScrollBarPolicy")
        .value("ScrollBarAsNeeded", Qt::ScrollBarAsNeeded)
        .value("ScrollBarAlwaysOff", Qt::ScrollBarAlwaysOff)
        .value("ScrollBarAlwaysOn", Qt::ScrollBarAlwaysOn)
        .export_values();

    py::class_<QAbstractScrollArea, QFrame>(m, "QAbstractScrollArea")
        .def("horizontalScrollBarPolicy", &QAbstractScrollArea::horizontalScrollBarPolicy)
        .def("setHorizontalScrollBarPolicy", &QAbstractScrollArea::setHorizontalScrollBarPolicy)
        .def("verticalScrollBarPolicy", &QAbstractScrollArea::verticalScrollBarPolicy)
        .def("setVerticalScrollBarPolicy", &QAbstractScrollArea::setVerticalScrollBarPolicy)
        ;

    exportPyQtGuiLayoutClasses(m);
    exportPyQtGuiModelViewClasses(m);
}
