#ifndef CNOID_BASE_PYTHON_PYQSTRING_H
#define CNOID_BASE_PYTHON_PYQSTRING_H

#include <nanobind/nanobind.h>
#include <QString>

namespace nanobind {
namespace detail {

template<>
struct type_caster<QString>
{
    NB_TYPE_CASTER(QString, const_name("str"))

    bool from_python(handle src, uint8_t, cleanup_list*) noexcept {
        if(!src.ptr()){
            return false;
        }
        object tmp;
        handle src2 = src;
        if(PyUnicode_Check(src2.ptr())){
            tmp = steal(PyUnicode_AsUTF8String(src2.ptr()));
            if(!tmp.is_valid()){ // UnicodeEncodeError
                PyErr_Clear();
                return false;
            }
            src2 = tmp;
        }
        char* buffer = nullptr;
        Py_ssize_t length = 0;
        if(PyBytes_AsStringAndSize(src2.ptr(), &buffer, &length) == -1){ // TypeError
            PyErr_Clear();
            return false;
        }
        value = QString::fromUtf8(buffer, static_cast<int>(length));
        return true;
    }

    static handle from_cpp(const QString& src, rv_policy, cleanup_list*) noexcept {
        static_assert(sizeof(QChar) == 2);
        return PyUnicode_FromKindAndData(PyUnicode_2BYTE_KIND, src.constData(), src.length());
    }
};

} // namespace detail
} // namespace nanobind

#endif
