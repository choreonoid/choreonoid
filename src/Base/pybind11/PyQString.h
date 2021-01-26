#ifndef CNOID_BASE_PYQSTRING_H
#define CNOID_BASE_PYQSTRING_H

#include <pybind11/pybind11.h>
#include <QString>

namespace pybind11 { namespace detail {

template <> struct type_caster<QString>
{
public:
    PYBIND11_TYPE_CASTER(QString, _("QString"));

    bool load(handle src, bool){
        if(!src){
            return false;
        }
        object tmp;
        handle src2 = src;
        if(PyUnicode_Check(src2.ptr())){
            tmp = reinterpret_steal<object>(PyUnicode_AsUTF8String(src2.ptr()));
            if(!tmp) { // UnicodeEncodeError
                PyErr_Clear();
                return false;
            }
            src2 = tmp;
        }
        char* buffer = nullptr;
        ssize_t length = 0;
        int err = PYBIND11_BYTES_AS_STRING_AND_SIZE(src2.ptr(), &buffer, &length);
        if(err == -1) { // TypeError
            PyErr_Clear();
            return false;
        }
        value = QString::fromUtf8(buffer, static_cast<int>(length));
        return true;
    }

    static handle cast(const QString& src, return_value_policy /* policy */, handle /* parent */){
#if PY_VERSION_HEX >= 0x03030000 // Python 3.3
        assert(sizeof(QChar) == 2);
        return PyUnicode_FromKindAndData(PyUnicode_2BYTE_KIND, src.constData(), src.length());
#else
        QByteArray a = src.toUtf8();
        return PyUnicode_FromStringAndSize(a.data(), (ssize_t)a.length());
#endif
    }
};


/* Old implementation

template <> struct type_caster<QString> {
public:
    PYBIND11_TYPE_CASTER(QString, _("QString"));
    
    bool load(handle src, bool) {
        PyObject* source = src.ptr();
        if(PyUnicode_Check(source)){
            Py_ssize_t len;
#if PY_MAJOR_VERSION >= 3
            const char* data = PyUnicode_AsUTF8AndSize(source, &len);
#else
            char* data = PyString_AsString(PyUnicode_AsUTF8String(source));
#endif
            value = QString(data);
        }
        return !PyErr_Occurred();
    }
    
    static handle cast(QString src, return_value_policy, handle ) {
        QByteArray ba = src.toUtf8();
        return  PyUnicode_FromStringAndSize(ba.constData(), ba.size());
    }
};

*/

} } // namespace pybind11::detail

#endif
