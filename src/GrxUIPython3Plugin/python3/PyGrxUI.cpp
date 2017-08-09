/**
  @author Shin'ichiro Nakaoka
*/

#include "../GrxUIPlugin.h"
#include "../GrxUIMenuView.h"
#include <cnoid/LazyCaller>
#include <pybind11/eval.h>

using namespace cnoid;
namespace py = pybind11;

namespace {

    void checkGrxUIPlugin()
    {
        if(!GrxUIPlugin::isActive()){
            PyErr_SetString(PyExc_RuntimeError, "GrxUI Plugin is inactive.");
            throw py::error_already_set();
        }
    }

    py::object cancelExceptionType;
    
    void waitInputMenu(const py::list& menu)
    {
        checkGrxUIPlugin();
        callSynchronously([&](){ GrxUIMenuView::instance()->setMenu(menu, false, !isRunningInMainThread()); });
    }

    void waitInputSequentialMenu(const py::list& menu)
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
           throw py::error_already_set();
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
            throw py::error_already_set();
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
            python::throw_error_already_set();
        }
        */

       return result;
    }
}


PYBIND11_PLUGIN(grxui)
{
    py::module m("grxui", "grxui Python Module");

    if(!GrxUIPlugin::isActive()){
        PyErr_SetString(PyExc_ImportError, "GrxUI Plugin is not loaded.");
        throw py::error_already_set();
    }
    
    m.def("waitInputMenu", waitInputMenu);
    m.def("waitInputSequentialMenu", waitInputSequentialMenu);
    m.def("waitInputSelect", waitInputSelect);
    m.def("waitInputConfirm", waitInputConfirm);
    m.def("waitInputMessage", waitInputMessage);

    // define the GrxUICancelException class which inherits the built-in Exception class
    py::object mainModule = py::module::import("__main__");
    py::object mainName = mainModule.attr("__name__");
    mainModule.attr("__name__") = m.attr("__name__");
    py::eval<py::eval_single_statement>("class GrxUICancelException (RuntimeError): pass\n",
                 mainModule.attr("__dict__"), m.attr("__dict__"));
    cancelExceptionType = m.attr("GrxUICancelException");
    GrxUIMenuView::setCancelExceptionType(cancelExceptionType);
    mainModule.attr("__name__") = mainName;

    return m.ptr();
}
