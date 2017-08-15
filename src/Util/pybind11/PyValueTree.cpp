/*!
  @author Shin'ichiro Nakaoka
*/

#include "PyUtil.h"
#include "../ValueTree.h"

using namespace std;
using namespace cnoid;
namespace py = pybind11;

namespace {

template<class ValueType>
py::object ValueNode_read(ValueNode& self){
    ValueType value;
    if(self.read(value)){
        return py::object(py::cast(value));
    }
    return py::object();
}

template<class ValueType>
py::object Mapping_read(Mapping& self, const string& key){
    ValueType value;
    if(self.read(key, value)){
        return py::object(py::cast(value));
    }
    return py::object();
}

py::object Mapping_get(Mapping& self, const std::string& key, py::object defaultValue){
    if(PyBool_Check(defaultValue.ptr())){
        bool value;
        if(self.read(key, value)){
            return py::object(py::cast(value));
        }
    } else if(PyLong_Check(defaultValue.ptr())){
        int value;
        if(self.read(key, value)){
            return py::object(py::cast(value));
        }
    } else if(PyFloat_Check(defaultValue.ptr())){
        double value;
        if(self.read(key, value)){
            return py::object(py::cast(value));
        }
    } else if(PyBytes_Check(defaultValue.ptr())){
        string value;
        if(self.read(key, value)){
            return py::object(py::cast(value));
        }
    } else {
        return py::object();
    }
    return defaultValue;
}

void Mapping_write(Mapping& self, const std::string &key, py::object value){
    if(PyBool_Check(value.ptr())){
        self.write(key, value.cast<bool>());
    } else if(PyLong_Check(value.ptr())){
        self.write(key, value.cast<int>());
    } else if(PyFloat_Check(value.ptr())){
        self.write(key, value.cast<double>());
    } else {
        PyErr_SetString(PyExc_TypeError, "The argument type is not supported");
        throw py::error_already_set();
    }
}

}

namespace cnoid {

void exportPyValueTree(py::module& m)
{
    py::enum_<StringStyle>(m, "StringStyle")
        .value("PLAING_STRING", PLAIN_STRING)
        .value("SINGLE_QUOTED", SINGLE_QUOTED)
        .value("DOUBLE_QUOTED", DOUBLE_QUOTED)
        .value("LITERAL_STRING", LITERAL_STRING)
        .value("FOLDED_STRING", FOLDED_STRING)
        .export_values();

    py::class_<ValueNode, ValueNodePtr, Referenced>(m, "ValueNode")
        .def("isValid", &ValueNode::isValid)
        .def("toInt", &ValueNode::toInt)
        .def("toFloat", &ValueNode::toDouble)
        .def("toBool", &ValueNode::toBool)
        .def("isScalar", &ValueNode::isScalar)
        .def("isString", &ValueNode::isString)
        .def("toString", &ValueNode::toString)
        .def("isMapping", &ValueNode::isMapping)
        .def("toMapping", [](ValueNode& self){ return MappingPtr(self.toMapping()); })
        .def("isListing", &ValueNode::isListing)
        .def("toListing", [](ValueNode& self){ return ListingPtr(self.toListing()); })
        .def("readInt", [](ValueNode& self){ return ValueNode_read<int>(self); })
        .def("readFloat", [](ValueNode& self){ return ValueNode_read<double>(self); })
        .def("readBool",  [](ValueNode& self){ return ValueNode_read<bool>(self); })
        .def("readString", [](ValueNode& self){ return ValueNode_read<string>(self); })
        .def("hasLineInfo", &ValueNode::hasLineInfo)
        .def("line", &ValueNode::line)
        .def("column", &ValueNode::column)
        ;

    py::class_<Mapping, MappingPtr, ValueNode>(m, "Mapping")
        .def("empty", &Mapping::empty)
        .def("size", &Mapping::size)
        .def("clear", &Mapping::clear)
        .def("setFlowStyle", &Mapping::setFlowStyle)
        .def("isFlowStyle", &Mapping::isFlowStyle)
        .def("setFloatFormat", &Mapping::setDoubleFormat)
        .def("floatFormat", &Mapping::doubleFormat)
        .def("setKeyQuoteStyle", &Mapping::setKeyQuoteStyle)
        .def("find", [](Mapping& self, const string& key){ return ValueNodePtr(self.find(key)); })
        .def("findMapping", [](Mapping& self, const string& key){ return MappingPtr(self.findMapping(key)); })
        .def("findListing", [](Mapping& self, const string& key){ return ListingPtr(self.findListing(key)); })
        .def("__getitem__", [](Mapping& self, const string& key){ return ValueNodePtr(&self.get(key)); })
        .def("insert", [](Mapping& self, const string& key, ValueNodePtr node){ self.insert(key, node); })
        .def("openMapping", [](Mapping& self, const string& key){ return MappingPtr(self.openMapping(key)); })
        .def("openFlowStyleMapping", [](Mapping& self, const string& key){ return MappingPtr(self.openFlowStyleMapping(key)); })
        .def("createMapping", [](Mapping& self, const string& key){ return MappingPtr(self.createMapping(key)); })
        .def("createFlowStyleMapping", [](Mapping& self, const string& key){ return MappingPtr(self.createFlowStyleMapping(key)); })
        .def("openListing", [](Mapping& self, const string& key){ return ListingPtr(self.openListing(key)); })
        .def("openFlowStyleListing", [](Mapping& self, const string& key){ return ListingPtr(self.openFlowStyleListing(key)); })
        .def("createListing", [](Mapping& self, const string& key){ return ListingPtr(self.createListing(key)); })
        .def("createFlowStyleListing", [](Mapping& self, const string& key){ return ListingPtr(self.createFlowStyleListing(key)); })
        .def("remove", &Mapping::remove)
        .def("readString", [](Mapping& self, const string& key){ return Mapping_read<string>(self, key); })
        .def("readBool",  [](Mapping& self, const string& key){ return Mapping_read<bool>(self, key); })
        .def("readInt", [](Mapping& self, const string& key){ return Mapping_read<int>(self, key); })
        .def("readFloat", [](Mapping& self, const string& key){ return Mapping_read<double>(self, key); })
        .def("get", Mapping_get)
        .def("write", [](Mapping& self, const string &key, const string& value){ self.write(key, value); })
        .def("write", [](Mapping& self, const string &key, const string& value, StringStyle style){ self.write(key, value, style); })
        .def("write", Mapping_write)
        ;

    py::class_< Listing, ListingPtr, ValueNode>(m, "Listing")
        .def("empty", &Listing::empty)
        .def("size", &Listing::size)
        .def("clear", &Listing::clear)
        .def("reserve", &Listing::reserve)
        .def("setFlowStyle", &Listing::setFlowStyle)
        .def("isFlowStyle", &Listing::isFlowStyle)
        .def("setFloatFormat", &Listing::setDoubleFormat)
        .def("floatFormat", &Listing::doubleFormat)
        .def("front", [](Listing& self){ return ValueNodePtr(self.front()); })
        .def("back", [](Listing& self){ return ValueNodePtr(self.back()); })
        .def("at", [](Listing& self, int i){ return ValueNodePtr(self.at(i)); })
        .def("write", [](Listing& self, int i, int value){ self.write(i, value); })
        .def("write", [](Listing& self, int i, const string& value){ self.write(i, value); })
        .def("write", [](Listing& self, int i, const string& value, StringStyle style){ self.write(i, value, style); })
        .def("__getitem__", [](Listing& self, int i){ return ValueNodePtr(self.at(i)); })
        .def("newMapping", [](Listing& self){ return MappingPtr(self.newMapping()); })
        .def("append", [](Listing& self, ValueNodePtr node){ self.append(node); })
        .def("append", [](Listing& self, int value){ self.append(value); })
        .def("append", [](Listing& self, double value){ self.append(value); })
        .def("append", [](Listing& self, const string& value){ self.append(value); })
        .def("append", [](Listing& self, const string& value, StringStyle style){ self.append(value, style); })
        .def("appendLF", &Listing::appendLF)
        ;
}

}
