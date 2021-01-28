#ifndef CNOID_BASE_PY_QOBJECT_HOLDER_H
#define CNOID_BASE_PY_QOBJECT_HOLDER_H

#include <pybind11/pybind11.h>
#include <QObject>
#include <QVariant>

namespace cnoid {

/**
   \note The 'pyref' property of the QObject is used as the reference counter.
   The pointer management can be disabled by setting -1 to the property in the C++ side in advance.
*/
template<class QObjectType>
class PyQObjectHolder
{
public:
    QObjectType* object;
    QMetaObject::Connection connection;

    PyQObjectHolder()
    {
        object = nullptr;
    }

    PyQObjectHolder(QObjectType* object) : object(object)
    {
        refObject();
    }

    PyQObjectHolder(const PyQObjectHolder& org)
    {
        (*this) = org;
    }

    PyQObjectHolder& operator=(PyQObjectHolder const & rhs)
    {
        unrefObject();
        object = rhs.object;
        refObject();
        return *this;
    }

    void refObject()
    {
        if(object){
            int refCount = 0;
            auto pyref = object->property("pyref");
            if(pyref.isValid()){
                refCount = pyref.toInt();
            }
            if(refCount >= 0){
                object->setProperty("pyref", ++refCount);
                connection = QObject::connect(
                    object, &QObject::destroyed, [this](){ this->object = nullptr; });
            }
        }
    }            

    void unrefObject()
    {
        if(object){
            int refCount = object->property("pyref").toInt();
            if(refCount > 0){
                object->setProperty("pyref", --refCount);
                QObject::disconnect(connection);
                if(refCount == 0){
                    if(!object->parent()){
                        delete object;
                    }
                }
            }
            object = nullptr;
        }
    }

    ~PyQObjectHolder()
    {
        unrefObject();
    }

    QObjectType* get() { return object; }
};

template<class QObjectType>
QObjectType* releaseFromPythonSideManagement(QObjectType* object)
{
    // Prevent this instance from deleting when the python variable is freed
    object->setProperty("pyref", -1);
    return object;
}

}
    
PYBIND11_DECLARE_HOLDER_TYPE(T, cnoid::PyQObjectHolder<T>, true);

#endif
