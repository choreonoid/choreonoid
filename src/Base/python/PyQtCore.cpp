/*!
  @author Shin'ichiro Nakaoka
*/

#include <boost/python.hpp>
#include <QObject>

using namespace boost;
using namespace boost::python;

namespace {

struct QString_to_python_str {
    static PyObject* convert(QString const& s){
        return boost::python::incref(
            boost::python::object(
                s.toUtf8().constData()).ptr());
    }
};

struct QString_from_python_str {
    QString_from_python_str(){
        python::converter::registry::push_back(
            &convertible, &construct, python::type_id<QString>());
    }
    static void* convertible(PyObject* obj_ptr){
        if(!PyString_Check(obj_ptr)) return 0;
        return obj_ptr;
    }
    static void construct(PyObject* obj_ptr, python::converter::rvalue_from_python_stage1_data* data){
        const char* value = PyString_AsString(obj_ptr);
        assert(value);
        void* storage = ((python::converter::rvalue_from_python_storage<QString>*)data)->storage.bytes;
        new (storage) QString(value);
        data->convertible = storage;
    }
};

}

BOOST_PYTHON_MODULE(QtCore)
{
    to_python_converter<QString, QString_to_python_str>();
    QString_from_python_str();
    
    class_<QObject, QObject*, boost::noncopyable>("QObject")
        .def("blockSignals", &QObject::blockSignals)
        .def("inherits", &QObject::inherits)
        .def("isWidgetType", &QObject::isWidgetType)
        .def("killTimer", &QObject::killTimer)
        .def("objectName", &QObject::objectName)
        .def("parent", &QObject::parent, return_value_policy<reference_existing_object>())
        .def("setObjectName", &QObject::setObjectName)
        .def("setParent", &QObject::setParent)
        .def("startTimer", &QObject::startTimer)
        .def("deleteLater", &QObject::deleteLater);
}
