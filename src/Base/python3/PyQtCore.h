/*
 *  Created on: 2017/05/16
 *      Author: Shizuko Hattori
 */

#ifndef CNOID_BASE_PYTHON3_PYQTCORE_H
#define CNOID_BASE_PYTHON3_PYQTCORE_H

#include <cnoid/Py3Util>
#include <QString>
#include <iostream>

namespace pybind11 { namespace detail {
    template <> struct type_caster<QString> {
    public:
        PYBIND11_TYPE_CASTER(QString, _("QString"));

        //Conversion part 1 (Python->C++)
        bool load(handle src, bool) {
            PyObject *source = src.ptr();
            if (PyUnicode_Check(source)) {
                Py_ssize_t len;
                char *data = PyUnicode_AsUTF8AndSize(source, &len);
                value = QString(data);
            }
            return !PyErr_Occurred();
        }

        //Conversion part 2 (C++ -> Python)
        static handle cast(QString src, return_value_policy, handle ) {
            QByteArray ba = src.toUtf8();
            return  PyUnicode_FromStringAndSize( ba.constData(), ba.size() );
        }
    };
}}

#endif /*  CNOID_BASE_PYTHON3_PYQTCORE_H_ */
