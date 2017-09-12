/**
  @author Shin'ichiro Nakaoka
*/

#include "../GrxUIPlugin.h"
#include "../GrxUIMenuView.h"
#include <cnoid/LazyCaller>

#ifdef CNOID_USE_PYBIND11
#include <pybind11/eval.h>
void throw_error_already_set(){ throw pybind11::error_already_set(); }
#else
void throw_error_already_set(){ boost::python::throw_error_already_set(); }
#endif

using namespace cnoid;

namespace {

    void checkGrxUIPlugin()
    {
        if(!GrxUIPlugin::isActive()){
            PyErr_SetString(PyExc_RuntimeError, "GrxUI Plugin is inactive.");
            throw_error_already_set();
        }
    }

    python::object cancelExceptionType;
    
    void waitInputMenu(const python::list& menu)
    {
        checkGrxUIPlugin();
        callSynchronously([&](){ GrxUIMenuView::instance()->setMenu(menu, false, !isRunningInMainThread()); });
    }

    void waitInputSequentialMenu(const python::list& menu)
    {
        checkGrxUIPlugin();
        callSynchronously([&](){ GrxUIMenuView::instance()->setMenu(menu, true, !isRunningInMainThread()); });
    }

    void waitInputSelectMain(const std::string& message, QMessageBox::StandardButton& out_result)
    {
        if(GrxUIPlugin::isActive()){
            out_result = GrxUIMenuView::waitInputSelect(message);
        }
    }
            
    bool waitInputSelect(const std::string& message)
    {
        checkGrxUIPlugin();
        QMessageBox::StandardButton result = QMessageBox::Cancel;

        Py_BEGIN_ALLOW_THREADS        
        callSynchronously([&](){ waitInputSelectMain(message, result); });
        Py_END_ALLOW_THREADS
        
        if(result == QMessageBox::Cancel){
            PyErr_SetObject(cancelExceptionType.ptr(), 0);
            throw_error_already_set();
        }
        return (result == QMessageBox::Yes);
    }

    void waitInputConfirmMain(const std::string& message, bool& out_result)
    {
        if(GrxUIPlugin::isActive()){
            out_result = GrxUIMenuView::waitInputConfirm(message);
        }
    }

    bool waitInputConfirm(const std::string& message)
    {
        checkGrxUIPlugin();

        bool result = false;

        Py_BEGIN_ALLOW_THREADS        
        callSynchronously([&](){ waitInputConfirmMain(message, result); });
        Py_END_ALLOW_THREADS
            
        if(!result){
            PyErr_SetObject(cancelExceptionType.ptr(), 0);
            throw_error_already_set();
        }

        return true;
    }

    void waitInputMessageMain(const std::string& message, std::string& out_result)
    {
        if(GrxUIPlugin::isActive()){
            out_result = GrxUIMenuView::waitInputMessage(message);
        }
    }

    std::string waitInputMessage(const std::string& message)
    {
        checkGrxUIPlugin();

        std::string result;

        Py_BEGIN_ALLOW_THREADS        
        callSynchronously([&](){ waitInputMessageMain(message, result); });
        Py_END_ALLOW_THREADS

        /*
        if(!result){
            PyErr_SetObject(cancelExceptionType.ptr(), 0);
            throw_error_already_set();
        }
        */

       return result;
    }
}


#ifdef CNOID_USE_PYBIND11

PYBIND11_MODULE(grxui, m)
{
    m.doc() = "Choreonoid GrxUI module";

    if(!GrxUIPlugin::isActive()){
        PyErr_SetString(PyExc_ImportError, "GrxUI Plugin is not loaded.");
        throw_error_already_set();
    }
    
    m.def("waitInputMenu", waitInputMenu);
    m.def("waitInputSequentialMenu", waitInputSequentialMenu);
    m.def("waitInputSelect", waitInputSelect);
    m.def("waitInputConfirm", waitInputConfirm);
    m.def("waitInputMessage", waitInputMessage);

    // define the GrxUICancelException class which inherits the built-in Exception class
    pybind11::object mainModule = pybind11::module::import("__main__");
    pybind11::object mainName = mainModule.attr("__name__");
    mainModule.attr("__name__") = m.attr("__name__");
    pybind11::eval<pybind11::eval_single_statement>(
        "class GrxUICancelException (RuntimeError): pass\n",
        mainModule.attr("__dict__"), m.attr("__dict__"));
    cancelExceptionType = m.attr("GrxUICancelException");
    GrxUIMenuView::setCancelExceptionType(cancelExceptionType);
    mainModule.attr("__name__") = mainName;
}

#else 

BOOST_PYTHON_MODULE(grxui)
{
    if(!GrxUIPlugin::isActive()){
        PyErr_SetString(PyExc_ImportError, "GrxUI Plugin is not loaded.");
        throw_error_already_set();
    }

    boost::python::def("waitInputMenu", waitInputMenu);
    boost::python::def("waitInputSequentialMenu", waitInputSequentialMenu);
    boost::python::def("waitInputSelect", waitInputSelect);
    boost::python::def("waitInputConfirm", waitInputConfirm);
    boost::python::def("waitInputMessage", waitInputMessage);

    // define the GrxUICancelException class which inherits the built-in Exception class
    boost::python::object mainModule = boost::python::import("__main__");
    boost::python::object mainName = mainModule.attr("__name__");
    mainModule.attr("__name__") = boost::python::scope().attr("__name__");
    boost::python::exec(
        "class GrxUICancelException (RuntimeError): pass\n",
        mainModule.attr("__dict__"), boost::python::scope().attr("__dict__"));
    cancelExceptionType = boost::python::scope().attr("GrxUICancelException");
    GrxUIMenuView::setCancelExceptionType(cancelExceptionType);
    mainModule.attr("__name__") = mainName;
}

#endif
