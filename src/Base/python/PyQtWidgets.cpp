#include "PyQString.h"
#include "PyQtSignal.h"
#include <QMainWindow>
#include <QPushButton>
#include <QToolButton>
#include <QCheckBox>
#include <QLabel>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QDialog>
#include <QFrame>
#include <QAbstractScrollArea>
#include <QMenu>

using namespace std;
namespace nb = nanobind;

namespace cnoid {

void exportPyQWidget(nb::module_ m);
void exportPyQtLayoutClasses(nb::module_ m);
void exportPyQtModelViewClasses(nb::module_ m);

}

using namespace cnoid;

NB_MODULE(QtWidgets, m)
{
    m.doc() = "Choreonoid QtWidgets module";

    nb::module_::import_("cnoid.QtCore");
    nb::module_::import_("cnoid.QtGui");

    exportPyQWidget(m);

    nb::class_<QMainWindow, QWidget>(m, "QMainWindow");

    nb::class_<QAbstractButton, QWidget> qAbstractButton(m, "QAbstractButton");

    typedef cnoid::QtSignal<void(QAbstractButton::*)(bool), void()> ButtonClickSignal;
    cnoid::PyQtSignal<ButtonClickSignal>(qAbstractButton, "ClickSignal");
    typedef cnoid::QtSignal<void(QAbstractButton::*)(bool), void(bool)> ButtonBoolSignal;
    cnoid::PyQtSignal<ButtonBoolSignal>(qAbstractButton, "BoolSignal");

    qAbstractButton
        .def("autoExclusive", &QAbstractButton::autoExclusive)
        .def("setAutoExclusive", &QAbstractButton::setAutoExclusive)
        .def("autoRepeat", &QAbstractButton::autoRepeat)
        .def("setAutoRepeat", &QAbstractButton::setAutoRepeat)
        .def("autoRepeatDelay", &QAbstractButton::autoRepeatDelay)
        .def("setAutoRepeatDelay", &QAbstractButton::setAutoRepeatDelay)
        .def("autoRepeatInterval", &QAbstractButton::autoRepeatInterval)
        .def("setAutoRepeatInterval", &QAbstractButton::setAutoRepeatInterval)
        .def("isCheckable", &QAbstractButton::isCheckable)
        .def("isChecked", &QAbstractButton::isChecked)
        .def("isDown", &QAbstractButton::isDown)
        .def("setCheckable", &QAbstractButton::setCheckable)
        .def("setDown", &QAbstractButton::setDown)
        .def("text", &QAbstractButton::text)
        .def("setText", &QAbstractButton::setText)
        .def("animateClick", [](QAbstractButton& self){ self.animateClick(); })
        .def("animateClick", &QAbstractButton::animateClick)
        .def("click", &QAbstractButton::click)
        .def("setChecked", &QAbstractButton::setChecked)
        .def("toggle", &QAbstractButton::toggle)
        .def_prop_ro(
            "clicked",
            [](QAbstractButton* self){ return ButtonClickSignal(self, &QAbstractButton::clicked); })
        .def_prop_ro(
            "toggled",
            [](QAbstractButton* self){ return ButtonBoolSignal(self, &QAbstractButton::toggled); })
        ;

    nb::class_<QPushButton, QAbstractButton>(m, "QPushButton")
        // Widgets are created on the heap (via nb::new_, not nb::init) so that
        // the C++ object is not embedded in the Python wrapper. An embedded
        // object cannot be deleted by Qt when the widget is reparented to a Qt
        // parent (operator delete on embedded memory crashes). A heap object can.
        .def(nb::new_([](QWidget* parent){ return new QPushButton(parent); }), nb::arg("parent") = nullptr)
        .def(nb::new_([](const QString& text, QWidget* parent){ return new QPushButton(text, parent); }),
             nb::arg("text"), nb::arg("parent") = nullptr)
        .def("autoDefault", &QPushButton::autoDefault)
        .def("isDefault", &QPushButton::isDefault)
        .def("isFlat", &QPushButton::isFlat)
        .def("menu", &QPushButton::menu, nb::rv_policy::reference)
        .def("setAutoDefault", &QPushButton::setAutoDefault)
        .def("setDefault", &QPushButton::setDefault)
        .def("setFlat", &QPushButton::setFlat)
        .def("setMenu", &QPushButton::setMenu)
        .def("showMenu", &QPushButton::showMenu)
        ;

    nb::class_<QToolButton, QAbstractButton>(m, "QToolButton")
        .def("autoRaise", &QToolButton::autoRaise)
        ;

    nb::class_<QCheckBox, QAbstractButton>(m, "QCheckBox")
        .def(nb::new_([]{ return new QCheckBox(); }))
        .def(nb::new_([](const QString& text){ return new QCheckBox(text); }))
        ;

    nb::class_<QLabel, QWidget>(m, "QLabel")
        .def(nb::new_([]{ return new QLabel(); }))
        .def(nb::new_([](const QString& text){ return new QLabel(text); }))
        .def("setText", &QLabel::setText)
        ;

    nb::class_<QAbstractSpinBox, QWidget>(m, "QAbstractSpinBox")
        .def("alignment", &QAbstractSpinBox::alignment)
        .def("setAlignment", &QAbstractSpinBox::setAlignment)
        ;

    nb::class_<QSpinBox, QAbstractSpinBox> qSpinBox(m, "QSpinBox");

    typedef cnoid::QtSignal<void(QSpinBox::*)(int), void(int)> SpinBoxIntSignal;
    cnoid::PyQtSignal<SpinBoxIntSignal>(qSpinBox, "IntSignal");

    qSpinBox
        .def(nb::new_([]{ return new QSpinBox(); }))
        .def("maximum", &QSpinBox::maximum)
        .def("minimum", &QSpinBox::minimum)
        .def("setMaximum", &QSpinBox::setMaximum)
        .def("setMinimum", &QSpinBox::setMinimum)
        .def("setRange", &QSpinBox::setRange)
        .def("setSingleStep", &QSpinBox::setSingleStep)
        .def("singleStep", &QSpinBox::singleStep)
        .def("value", &QSpinBox::value)
        .def("setValue", &QSpinBox::setValue)
        .def_prop_ro(
            "valueChanged",
            [](QSpinBox* self){ return SpinBoxIntSignal(self, &QSpinBox::valueChanged); })
        ;

    nb::class_<QDoubleSpinBox, QAbstractSpinBox> qDoubleSpinBox(m, "QDoubleSpinBox");

    typedef cnoid::QtSignal<void(QDoubleSpinBox::*)(double), void(double)> DoubleSpinBoxIntSignal;
    cnoid::PyQtSignal<DoubleSpinBoxIntSignal>(qDoubleSpinBox, "IntSignal");

    qDoubleSpinBox
        .def(nb::new_([]{ return new QDoubleSpinBox(); }))
        .def("decimals", &QDoubleSpinBox::decimals)
        .def("maximum", &QDoubleSpinBox::maximum)
        .def("minimum", &QDoubleSpinBox::minimum)
        .def("setDecimals", &QDoubleSpinBox::setDecimals)
        .def("setMaximum", &QDoubleSpinBox::setMaximum)
        .def("setMinimum", &QDoubleSpinBox::setMinimum)
        .def("setRange", &QDoubleSpinBox::setRange)
        .def("setSingleStep", &QDoubleSpinBox::setSingleStep)
        .def("singleStep", &QDoubleSpinBox::singleStep)
        .def("value", &QDoubleSpinBox::value)
        .def("setValue", &QDoubleSpinBox::setValue)
        .def_prop_ro(
            "valueChanged",
            [](QDoubleSpinBox* self){ return DoubleSpinBoxIntSignal(self, &QDoubleSpinBox::valueChanged); })
        ;

    nb::class_<QDialog, QWidget> qDialog(m, "QDialog");

    typedef cnoid::QtSignal<void(QDialog::*)(), void()> DialogSignal;
    cnoid::PyQtSignal<DialogSignal>(qDialog, "Signal");
    typedef cnoid::QtSignal<void(QDialog::*)(int), void(int)> DialogIntSignal;
    cnoid::PyQtSignal<DialogIntSignal>(qDialog, "IntSignal");

    qDialog
        .def(nb::new_([]{ return new QDialog(); }))
        .def("result", &QDialog::result)
        .def("setResult", &QDialog::setResult)
        .def("setModal", &QDialog::setModal)
        .def("accept", &QDialog::accept)
        .def("done", &QDialog::done)
        .def("exec", &QDialog::exec)
        .def("open", &QDialog::open)
        .def("reject", &QDialog::reject)
        .def_prop_ro(
            "accepted",
            [](QDialog* self){ return DialogSignal(self, &QDialog::accepted); })
        .def_prop_ro(
            "finished",
            [](QDialog* self){ return DialogIntSignal(self, &QDialog::finished); })
        .def_prop_ro(
            "rejected",
            [](QDialog* self){ return DialogSignal(self, &QDialog::rejected); })
        ;

    nb::class_<QFrame, QWidget>(m, "QFrame");

    nb::class_<QAbstractScrollArea, QFrame>(m, "QAbstractScrollArea")
        .def("horizontalScrollBarPolicy", &QAbstractScrollArea::horizontalScrollBarPolicy)
        .def("setHorizontalScrollBarPolicy", &QAbstractScrollArea::setHorizontalScrollBarPolicy)
        .def("verticalScrollBarPolicy", &QAbstractScrollArea::verticalScrollBarPolicy)
        .def("setVerticalScrollBarPolicy", &QAbstractScrollArea::setVerticalScrollBarPolicy)
        ;

    exportPyQtLayoutClasses(m);
    exportPyQtModelViewClasses(m);
}
