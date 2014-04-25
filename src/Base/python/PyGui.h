/*
 * @brief Definition of the interface for the gui(PySide) created.
 * @note Reason to provide the interface of the item.
 * 
 * @reference http://goo.gl/KQsx1O
 *            Please show this. This site has many good interfaces.
 *     
 * @author Hisashi Ikari
 */

#include <boost/python.hpp>
#include <shiboken.h>
#include <pyside.h>
#include <QtGui/pyside_qtgui_python.h>
#include <QtCore/pyside_qtcore_python.h>

#include <QtCore>
// !!! IMPORTANT !!! QtGui include order is must be last. ;/

using namespace boost::python;

/*!
 * @brief Holding all types of basic and gui of PySide
 */
PyTypeObject** SbkPySide_QtCoreTypes = NULL;
PyTypeObject** SbkPySide_QtGuiTypes = NULL;

 
/*!
 * @brief Converting the object of Qt4 of C++ to Python. 
 */
template <class QtGuiClass, int SBK_BOGAN_IDX>
struct QtGui_to_python
{
    static PyObject* convert(const QtGuiClass& source)
    {
        // All QtObjects are managed by the smart-point on qt4, I guess. 
        SbkObjectType* result = reinterpret_cast<SbkObjectType*>(SbkPySide_QtGuiTypes[SBK_BOGAN_IDX]);
        return Shiboken::Conversions::pointerToPython(result, &source);
    }

    static PyObject* convert(QtGuiClass* source)
    {
        // @note We can use this interface for pointer arguments too.
        SbkObjectType* result = reinterpret_cast<SbkObjectType*>(SbkPySide_QtGuiTypes[SBK_BOGAN_IDX]);
        return Shiboken::Conversions::pointerToPython(result, source);
    }

};


/*!
 * @brief Converting the object of PySide of Python to C++.
 */
template <class QtGuiClass, int SBK_BOGAN_IDX>
struct QtGui_from_python
{
    QtGui_from_python()
    {
        boost::python::converter::registry::push_back(&QtGui_from_python::convertible, 
            &QtGui_from_python::construct, boost::python::type_id<QtGuiClass>());
    }

    static void* convertible(PyObject* obj_ptr)
    {
        // In this case, we were not able to load the QtGui.
        if (SbkPySide_QtGuiTypes == NULL) return NULL;
        // If object is not QtGui. it is through.
        if (!PyObject_TypeCheck(obj_ptr, (PyTypeObject*)SbkPySide_QtGuiTypes[SBK_BOGAN_IDX])) return NULL; 
        return obj_ptr;
    }

    static void construct(PyObject* obj_ptr, boost::python::converter::rvalue_from_python_stage1_data* data)
    {
        void* storage = ((boost::python::converter::rvalue_from_python_storage<QtGuiClass>*)data)->storage.bytes;
        SbkObject* result = reinterpret_cast<SbkObject*>(obj_ptr);
        // @note We do not create the new object from python object. "JUST CONVERT TYPE".
        //       Coz new object has new address, and there are not same.
        //       (1. Function address is different), (2. All attributes are same)
        QtGuiClass* _ptr = (QtGuiClass*) (Shiboken::Object::cppPointer(result, Py_TYPE(result)));
        data->convertible = _ptr;
    }

};


/*!
 * @brief Registing the QObject an PySide converter (type list).
 */
static void initialize() {
    if (!SbkPySide_QtCoreTypes) {
        Shiboken::AutoDecRef requiredModule(Shiboken::Module::import("PySide.QtCore"));
        if (requiredModule.isNull()) { 
            // "It could not be loaded PySide.QtCore. Please load again this cnoid.Base in reload."
        }
        SbkPySide_QtCoreTypes = Shiboken::Module::getTypes(requiredModule);
    }
    if (!SbkPySide_QtGuiTypes) {
        Shiboken::AutoDecRef requiredModule(Shiboken::Module::import("PySide.QtGui"));
        if (requiredModule.isNull()) {
            // "It could not be loaded PySide.QtGui. Please load again this cnoid.Base in reload."
        }
        SbkPySide_QtGuiTypes = Shiboken::Module::getTypes(requiredModule);
    }
}


/*!
 * @brief Converting the object of PySide of Python to C++.
 */
static void calling(PyObject* callable)
{
    try {
        // We are calling the python-function (only static function and no arguments now) on c++.
        // !!! SORRY !!! 
        // Now, I do not know how to accept (take or put) the argument.
        //
        // Please add the following processing if you add the new calling method.
        //   http://docs.python.jp/2/c-api/object.html
        //
        // !!! INPORTANT !!!
        // If you define AGAIN THE SAME(NAME) FUNCTION.
        // The previous function will be lost. For this purpose, a function that was given to connect also invalid.
        // Further, there is no way to review this invalid function.
        // In other words, that is referred to as an invalid function, it causes a crash. Please please watch out!
        //
        if (PyCallable_Check(callable)) {
            PyObject_CallObject(callable, NULL);
        }
    } catch (...) { /* ignore the exception for keep running */ }
}



/*
 * @brief This class is a wrapper to the SignalProxy.
 *        This wrapper will receive the arguments and self on python(def a(self)). 
 *        It is completely independent to SignalProxy for that.
 */
template <class QtGuiClass, class SignalClass>
class PySignalClickVisitor : public def_visitor< PySignalClickVisitor<QtGuiClass, SignalClass> >
{
friend class def_visitor_access;
public:
    template <class T>
    void visit(T& self) const {
        // Visitor find to match signature on c++.
        // We will be included in the period method. And this appears to be a recursive function. :)
        self.def("clicked", &PySignalClickVisitor< QtGuiClass, SignalClass >::clicked, 
            return_value_policy<return_by_value>(), (args("args") = 0));	
    }

    static void clicked(QtGuiClass& self, boost::python::object& callback, boost::python::object& args) {
        PyObject *pfunc = callback.ptr(), *pargs = args.ptr();
        cnoid::SignalProxy<SignalClass> sig = self.sigClicked();
        sig.connect(boost::bind(calling, pfunc));
    }

};


/*
 * @brief This class is a wrapper to the SignalProxy.
 *        This wrapper will receive the arguments and self on python(def a(self)). 
 *        It is completely independent to SignalProxy for that.
 */
template <class QtGuiClass, class SignalClass>
class PySignalToggleVisitor : public def_visitor< PySignalToggleVisitor<QtGuiClass, SignalClass> >
{
friend class def_visitor_access;
public:
    template <class T>
    void visit(T& self) const {
        // Visitor find to match signature on c++.
        // We will be included in the period method. And this appears to be a recursive function. :)
        self.def("toggled", &PySignalToggleVisitor< QtGuiClass, SignalClass >::toggled, 
            return_value_policy<return_by_value>(), (args("args") = 0));
    }

    static void toggled(QtGuiClass& self, boost::python::object& callback, boost::python::object& args) {
        PyObject *pfunc = callback.ptr(), *pargs = args.ptr();
        cnoid::SignalProxy<SignalClass> sig = self.sigToggled();
        sig.connect(boost::bind(calling, pfunc));
    }

};
