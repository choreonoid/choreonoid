#ifndef CNOID_PYTHON_PLUGIN_PY_C_API_UTIL_H
#define CNOID_PYTHON_PLUGIN_PY_C_API_UTIL_H

#include <Python.h>
#include <string>
#include <utility>

namespace cnoid {

/**
   A minimal RAII wrapper for a PyObject pointer used by the nanobind backend of
   the PythonPlugin. The wrapper owns a strong reference and must be accessed only
   while the GIL is held, which is the same convention as the original pybind11
   implementation.
*/
class PyObjectHandle
{
public:
    PyObjectHandle() : ptr_(nullptr) { }

    //! Steals the given reference (does not increment the reference count).
    static PyObjectHandle steal(PyObject* p) {
        PyObjectHandle h;
        h.ptr_ = p;
        return h;
    }

    //! Borrows the given reference (increments the reference count).
    static PyObjectHandle borrow(PyObject* p) {
        PyObjectHandle h;
        Py_XINCREF(p);
        h.ptr_ = p;
        return h;
    }

    PyObjectHandle(const PyObjectHandle& org) {
        ptr_ = org.ptr_;
        Py_XINCREF(ptr_);
    }

    PyObjectHandle(PyObjectHandle&& org) noexcept {
        ptr_ = org.ptr_;
        org.ptr_ = nullptr;
    }

    PyObjectHandle& operator=(const PyObjectHandle& rhs) {
        if(this != &rhs){
            Py_XINCREF(rhs.ptr_);
            Py_XDECREF(ptr_);
            ptr_ = rhs.ptr_;
        }
        return *this;
    }

    PyObjectHandle& operator=(PyObjectHandle&& rhs) noexcept {
        if(this != &rhs){
            Py_XDECREF(ptr_);
            ptr_ = rhs.ptr_;
            rhs.ptr_ = nullptr;
        }
        return *this;
    }

    ~PyObjectHandle() {
        Py_XDECREF(ptr_);
    }

    void reset() {
        Py_XDECREF(ptr_);
        ptr_ = nullptr;
    }

    PyObject* get() const { return ptr_; }

    //! Releases ownership and returns the raw pointer without decrementing.
    PyObject* release() {
        PyObject* p = ptr_;
        ptr_ = nullptr;
        return p;
    }

    explicit operator bool() const { return ptr_ != nullptr; }

private:
    PyObject* ptr_;
};


/**
   A RAII helper that acquires the GIL for the current scope. It corresponds to
   pybind11::gil_scoped_acquire.
*/
class GilScopedAcquire
{
public:
    GilScopedAcquire() { state_ = PyGILState_Ensure(); }
    ~GilScopedAcquire() { PyGILState_Release(state_); }
    GilScopedAcquire(const GilScopedAcquire&) = delete;
    GilScopedAcquire& operator=(const GilScopedAcquire&) = delete;
private:
    PyGILState_STATE state_;
};


/**
   Fetches the current Python exception and formats it into a traceback text
   similar to the string returned by pybind11::error_already_set::what(). The
   exception is restored so that the caller can still inspect or match it. The
   normalized exception type, value and traceback are returned via the output
   handles. The GIL must be held when this function is called.
*/
std::string fetchPythonExceptionText(
    PyObjectHandle& type, PyObjectHandle& value, PyObjectHandle& traceback);

}

#endif
