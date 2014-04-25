/*
 * @brief Definition of the interface for the item created.
 * @note Reason to provide the interface of the item.
 *       ---
 *         1. Smart pointer manages the item of choreonoid all.
 *         2. Python does not have a smart pointer. (Raw pointer only)
 *         3. Item of choreonoid receive a smart pointer on any interfaces.
 *         4. <<Python can not pass a raw-pointer to choreonoid of smart-pointer.>> This is a problem (type miss match error).
 *       ---
 *       But this interface provides that "CONVERSION". the smart-pointer in choreonoid and the raw-poiner in python.
 *       This interface will convert it properly. As a result, when it is passed through the interface of smart-pointer of choreonoid, 
 *       usually, the address of the object of a python will change the address.
 *       But it will be the same address by the correct conversion.   
 *
 * @reference http://goo.gl/qETVPF
 * @reference http://goo.gl/JD51ZC
 *
 * @note This mechanism is unstable. Therefore, I have replaced all to the Visitor(Boost::python::def_visitor).
 *       Until the transition is over, I leave for reference.
 *     
 * @author Hisashi Ikari
 */

#include <QString>
#include <cnoid/Referenced>
#include <cnoid/Item>

/*!
 * @brief Acquisition of raw-pointer.
 */
namespace cnoid
{
    // @note This function is used by the boost::python when converting the raw-point from smart pointers.
    //       However, it is not a namespace boost::python(or boost namespace).
    //       The correct answer is the name space of the "target class". (e.g. Item, FolderItem, etc)
    template <typename T>
    T* get_pointer(cnoid::ref_ptr<T> const& p)
    {
        return p.get();
    }
}


/*!
 * @brief We use the ref_ptr of choreonoid at all times on boost::python modules.
 *        Smart pointer of the item is ref_ptr. And smart pointer standard of boost::python is shared_ptr.
 *        These are different. Therefore, we overwrite the ref_ptr to shared_ptr for that on following.
 */
namespace boost
{
    namespace python
    {
        template <typename T>
        struct pointee< cnoid::ref_ptr<T> >
        {
            typedef T type;
        };

    };

};


/*!
 * @brief (1way) Convert raw-pointer from the smart-pointer. (2way) It will convert to smart-pointer from raw-pointer.
 */
template <typename S, typename T>
struct two_way_converter
{
    two_way_converter()
    {
        // Subscribe the converter to boost::python. 
        boost::python::implicitly_convertible<S, T>();
        boost::python::converter::registry::push_back(
            &two_way_converter::convertible,
            &two_way_converter::construct,
            boost::python::type_id<S>());
    }

    static void* convertible(PyObject* object)
    {
        // Check a object type of c++ from python-object. (Python to C++)
        typedef typename boost::python::pointee<S>::type pointee;
        return boost::python::extract<T>(object).check() &&
               boost::python::extract<pointee>(object).check()
            ? object
            : NULL;
        // !!! IMPORTANT !!!
        // If a freeze occurs in addChildItem called, object type of c++ has not been found.
        // Please watch namespace (Namespace a different ... Object of the same name. It also be different.-
        // - So it is not found. Therefore endless loop is occured, WE USE "cnoid" NAMESPACE ABOUT ALL OBJECT.
        // Please be careful.) or inheritance ("extract<T>" is derived object, extract<pointee> is parent object.
        // Object not found this relationship is not correct.)
    }

    static void construct(PyObject* object, boost::python::converter::rvalue_from_python_stage1_data* data)
    {
        // Create actual object of c++ by python. Following is creation of c++ object (python-extract). 
        namespace python = boost::python;
        typedef python::converter::rvalue_from_python_storage<S> storage_type;
        void* storage = reinterpret_cast<storage_type*>(data)->storage.bytes;
        // !!! IMPOTANT!!! Direct, we will get the concrete class.
        //                 And we do not type conversions of ref_ptr<FolderItem> from ref_ptr<Item>(concrete from abstract).
        //                 True twoway_converter to solve this problem.
        //                 However, in this case, it is necessary to change the following to the Referenced.h.
        //                 This is not good.
        //                 To do this, use Visitor and this process (on PyBase.cpp), we solve these problems.
        //
        // (e.g. Referenced.h)
        // template <class U>
        // explicit ref_ptr(U* p) : px(static_cast<T*>(p))
        // {
        //     if(px != 0){
        //         px->addRef();
        //     }
        // }
        S target = boost::python::extract<S>(object);
        data->convertible = target;
    } 

};


/*!
 * @brief (1way) Convert python_str from the QString. 
 */
struct QString_to_python_str
{
    static PyObject* convert(QString const& s)
    {
        return boost::python::incref(boost::python::object(s.toLatin1().constData()).ptr());
    }
};


/*!
 * @brief (2way) Convert QString from the python_str. 
 */
struct QString_from_python_str
{
    QString_from_python_str()
    {
        boost::python::converter::registry::push_back(
            &convertible,
            &construct,
            boost::python::type_id<QString>());
    }
 
    static void* convertible(PyObject* obj_ptr)
    {
        // If there is an interface to receive at QString of c++ from a string of python, I would convert.
        // However, because of the string, QString is handled separately from the QtObject other.
        if (!PyString_Check(obj_ptr)) return 0;
        return obj_ptr;
    }
 
    static void construct(PyObject* obj_ptr, boost::python::converter::rvalue_from_python_stage1_data* data)
    {
        const char* value = PyString_AsString(obj_ptr);
        assert(value);
        void* storage = ( (boost::python::converter::rvalue_from_python_storage<QString>*)data )->storage.bytes;
        new (storage) QString(value);
        data->convertible = storage;
    }

};


namespace cnoid
{
    /*!
     * @brief This class is a wrapper to operate "ITEM" in the python-script. 
     * @note  These classes are NOT visible from python.
     *        This is a wrapper of create all kind of item objects. 
     *        <<< WE MUST USE SMART POINTER WHENERVER THE ITEM IS CREATED. >>>
     */
    template <class I>
    static ref_ptr<I> createInstance()
    {
        // We will make a instance of class directly, sorry about no use the factories.
        // Or we must make new item using ItemManager::create(moduleName, className).
        // Now it is not problem when making new object of item by the this function.
        return ref_ptr<I>(new I());
    }

}; // end of namespace



