#include "PyCApiUtil.h"

using namespace std;

namespace cnoid {

std::string fetchPythonExceptionText(
    PyObjectHandle& type, PyObjectHandle& value, PyObjectHandle& traceback)
{
    PyObject* rawType = nullptr;
    PyObject* rawValue = nullptr;
    PyObject* rawTraceback = nullptr;

    // PyErr_Fetch clears the error indicator and returns new references. This
    // matches the behavior of pybind11::error_already_set, whose constructor
    // also fetches and clears the current exception.
    PyErr_Fetch(&rawType, &rawValue, &rawTraceback);
    PyErr_NormalizeException(&rawType, &rawValue, &rawTraceback);
    if(rawTraceback && rawValue){
        PyException_SetTraceback(rawValue, rawTraceback);
    }

    string text;

    if(rawType){
        PyObject* tracebackModule = PyImport_ImportModule("traceback");
        if(tracebackModule){
            PyObject* lines = PyObject_CallMethod(
                tracebackModule, "format_exception", "OOO",
                rawType, rawValue ? rawValue : Py_None, rawTraceback ? rawTraceback : Py_None);
            if(lines){
                PyObject* empty = PyUnicode_FromString("");
                PyObject* joined = PyObject_CallMethod(empty, "join", "O", lines);
                if(joined){
                    const char* s = PyUnicode_AsUTF8(joined);
                    if(s){
                        text = s;
                    }
                    Py_DECREF(joined);
                }
                Py_XDECREF(empty);
                Py_DECREF(lines);
            }
            Py_DECREF(tracebackModule);
        }
        if(text.empty()){
            // Fallback to the string representation of the exception value.
            PyObject* str = PyObject_Str(rawValue ? rawValue : rawType);
            if(str){
                const char* s = PyUnicode_AsUTF8(str);
                if(s){
                    text = s;
                }
                Py_DECREF(str);
            }
        }
    }

    // Clear any error raised while formatting the traceback above.
    PyErr_Clear();

    // The fetched references are handed over to the output handles, which take
    // ownership (steal) of them. The error indicator stays cleared so that the
    // caller can match the exception type without an active error state.
    type = PyObjectHandle::steal(rawType);
    value = PyObjectHandle::steal(rawValue);
    traceback = PyObjectHandle::steal(rawTraceback);

    return text;
}

}
