/*!
  @author Shin'ichiro Nakaoka
*/

#include "PyUtil.h"
#include "../ValueTree.h"

using namespace std;
using namespace boost;
using namespace boost::python;
using namespace cnoid;

namespace {

python::object ValueNode_value(ValueNode& self)
{

}

ValueNodePtr Mapping_find(Mapping& self, const std::string& key)
{
    return self.find(key);
}

MappingPtr Mapping_findMapping(Mapping& self, const std::string& key)
{
    return self.findMapping(key);
}

ListingPtr Mapping_findListing(Mapping& self, const std::string& key)
{
    return self.findListing(key);
}

ValueNodePtr Mapping_get(Mapping& self, const std::string& key)
{
    return &(self.get(key));
}

MappingPtr Mapping_openMapping(Mapping& self, const std::string& key)
{
    return self.openMapping(key);
}

MappingPtr Mapping_openFlowStyleMapping(Mapping& self, const std::string& key)
{
    return self.openFlowStyleMapping(key);
}

MappingPtr Mapping_createMapping(Mapping& self, const std::string& key)
{
    return self.createMapping(key);
}

MappingPtr Mapping_createFlowStyleMapping(Mapping& self, const std::string& key)
{
    return self.createFlowStyleMapping(key);
}

ListingPtr Mapping_openListing(Mapping& self, const std::string& key)
{
    return self.openListing(key);
}

ListingPtr Mapping_openFlowStyleListing(Mapping& self, const std::string& key)
{
    return self.openFlowStyleListing(key);
}

ListingPtr Mapping_createListing(Mapping& self, const std::string& key)
{
    return self.createListing(key);
}

ListingPtr Mapping_createFlowStyleListing(Mapping& self, const std::string& key)
{
    return self.createFlowStyleListing(key);
}

python::object Mapping_readString(Mapping& self, const std::string& key)
{
    string value;
    if(self.read(key, value)){
        return python::object(value);
    }
    return python::object();
}

python::object Mapping_readBool(Mapping& self, const std::string& key)
{
    bool value;
    if(self.read(key, value)){
        return python::object(value);
    }
    return python::object();
}

python::object Mapping_readInt(Mapping& self, const std::string& key)
{
    int value;
    if(self.read(key, value)){
        return python::object(value);
    }
    return python::object();
}

python::object Mapping_readFloat(Mapping& self, const std::string& key)
{
    double value;
    if(self.read(key, value)){
        return python::object(value);
    }
    return python::object();
}

python::object Mapping_get2(Mapping& self, const std::string& key, PyObject* defaultValue)
{
    if(PyBool_Check(defaultValue)){
        return python::object(true);
    }
    if(PyInt_Check(defaultValue)){
        return python::object(1);
    }
    if(PyFloat_Check(defaultValue)){
        return python::object(0.1);
    }
    if(PyString_Check(defaultValue)){
        return python::object("hoge");
    }
    return python::object();
}

void Mapping_writeString1(Mapping& self, const std::string &key, const std::string& value)
{
    self.write(key, value);
}

void Mapping_writeString2(Mapping& self, const std::string &key, const std::string& value, StringStyle style)
{
    self.write(key, value, style);
}

void Mapping_write(Mapping& self, const std::string &key, python::object value)
{
    if(PyBool_Check(value.ptr())){
        self.write(key, python::extract<bool>(value));
    } else if(PyInt_Check(value.ptr())){
        self.write(key, python::extract<int>(value));
    } else if(PyFloat_Check(value.ptr())){
        self.write(key, python::extract<double>(value));
    }
}

}

namespace cnoid {

void exportPyValueTree()
{
    enum_<StringStyle>("StringStyle")
        .value("PLAING_STRING", PLAIN_STRING)
        .value("SINGLE_QUOTED", SINGLE_QUOTED)
        .value("DOUBLE_QUOTED", DOUBLE_QUOTED)
        .value("LITERAL_STRING", LITERAL_STRING)
        .value("FOLDED_STRING", FOLDED_STRING);

    {
        scope valueNodeScope =
            class_<ValueNode, ValueNodePtr, bases<Referenced>, boost::noncopyable>("ValueNode", no_init)
            .def("isValid", &ValueNode::isValid)
            .def("type", &ValueNode::type)
            .def("value", ValueNode_value)
            .def("hasLineInfo", &ValueNode::hasLineInfo)
            .def("line", &ValueNode::line)
            .def("column", &ValueNode::column)
            ;

        enum_<ValueNode::Type>("Type")
            .value("INVALID_NODE", ValueNode::INVALID_NODE)
            .value("SCALAR", ValueNode::SCALAR)
            .value("MAPPING", ValueNode::MAPPING)
            .value("LISTING", ValueNode::LISTING)
            .value("LF_NODE", ValueNode::LF_NODE);
    }

    implicitly_convertible<ValueNodePtr, ReferencedPtr>();

    class_< Mapping, MappingPtr, bases<ValueNode>, boost::noncopyable >("Mapping")
        .def("empty", &Mapping::empty)
        .def("size", &Mapping::size)
        .def("clear", &Mapping::clear)
        .def("setFlowStyle", &Mapping::setFlowStyle)
        .def("isFlowStyle", &Mapping::isFlowStyle)
        .def("setDoubleFormat", &Mapping::setDoubleFormat)
        .def("doubleFormat", &Mapping::doubleFormat, return_value_policy<return_by_value>())
        .def("setKeyQuoteStyle", &Mapping::setKeyQuoteStyle)
        .def("find", Mapping_find)
        .def("findMapping", Mapping_findMapping)
        .def("findListing", Mapping_findListing)
        .def("get", Mapping_get)
        .def("__getitem__", Mapping_get)
        .def("insert", &Mapping::insert)
        .def("openMapping", Mapping_openMapping)
        .def("openFlowStyleMapping", Mapping_openFlowStyleMapping)
        .def("createMapping", Mapping_createMapping)
        .def("createFlowStyleMapping", Mapping_createFlowStyleMapping)
        .def("openListing", Mapping_openListing)
        .def("openFlowStyleListing", Mapping_openFlowStyleListing)
        .def("createListing", Mapping_createListing)
        .def("createFlowStyleListing", Mapping_createFlowStyleListing)
        .def("remove", &Mapping::remove)
        .def("readString", Mapping_readString)
        .def("readBool", Mapping_readBool)
        .def("readInt", Mapping_readInt)
        .def("readFloat", Mapping_readFloat)
        .def("get", Mapping_get2)
        .def("write", Mapping_writeString1)
        .def("write", Mapping_writeString2)
        .def("write", Mapping_write)
        ;

    implicitly_convertible<MappingPtr, ReferencedPtr>();
}

}
