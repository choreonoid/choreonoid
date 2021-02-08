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

} // namespace

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
        .def("toMapping", (Mapping*(ValueNode::*)()) &ValueNode::toMapping)
        .def("isListing", &ValueNode::isListing)
        .def("toListing", (Listing*(ValueNode::*)()) &ValueNode::toListing)
        .def("readInt", [](ValueNode& self){ return ValueNode_read<int>(self); })
        .def("readFloat", [](ValueNode& self){ return ValueNode_read<double>(self); })
        .def("readBool",  [](ValueNode& self){ return ValueNode_read<bool>(self); })
        .def("readString", [](ValueNode& self){ return ValueNode_read<string>(self); })
        .def("hasLineInfo", &ValueNode::hasLineInfo)
        .def_property_readonly("line", &ValueNode::line)
        .def_property_readonly("column", &ValueNode::column)

        // deprecated
        .def("getLine", &ValueNode::line)
        .def("getColumn", &ValueNode::column)
        ;

    py::class_<Mapping, MappingPtr, ValueNode>(m, "Mapping")
        .def(py::init<>())
        .def_property_readonly("empty", &Mapping::empty)
        .def_property_readonly("size", &Mapping::size)
        .def("clear", &Mapping::clear)
        .def("setFlowStyle", &Mapping::setFlowStyle)
        .def("isFlowStyle", &Mapping::isFlowStyle)
        .def("setFloatingNumberFormat", &Mapping::setFloatingNumberFormat)
        .def_property_readonly("floatingNumberFormat", &Mapping::floatingNumberFormat)
        .def("setKeyQuoteStyle", &Mapping::setKeyQuoteStyle)
        .def("find", (ValueNode*(Mapping::*)(const std::string&)const) &Mapping::find)
        .def("findMapping", (Mapping*(Mapping::*)(const std::string&)const) &Mapping::findMapping)
        .def("findListing", (Listing*(Mapping::*)(const std::string&)const) &Mapping::findListing)
        .def("__getitem__", &Mapping::operator[])
        .def("insert", (void(Mapping::*)(const string&, ValueNode*)) &Mapping::insert)
        .def("openMapping", &Mapping::openMapping)
        .def("openFlowStyleMapping", &Mapping::openFlowStyleMapping)
        .def("createMapping", &Mapping::createMapping)
        .def("createFlowStyleMapping", &Mapping::createFlowStyleMapping)
        .def("openListing", &Mapping::openListing)
        .def("openFlowStyleListing", &Mapping::openFlowStyleListing)
        .def("createListing", &Mapping::createListing)
        .def("createFlowStyleListing", &Mapping::createFlowStyleListing)
        .def("remove", &Mapping::remove)
        .def("readString", [](Mapping& self, const string& key){ return Mapping_read<string>(self, key); })
        .def("readBool",  [](Mapping& self, const string& key){ return Mapping_read<bool>(self, key); })
        .def("readInt", [](Mapping& self, const string& key){ return Mapping_read<int>(self, key); })
        .def("readFloat", [](Mapping& self, const string& key){ return Mapping_read<double>(self, key); })
        .def("get", Mapping_get)
        .def("write", [](Mapping& self, const string &key, const string& value){ self.write(key, value); })
        .def("write", [](Mapping& self, const string &key, const string& value, StringStyle style){ self.write(key, value, style); })
        .def("write", Mapping_write)

        // deprecated
        .def("isEmpty", &Mapping::empty)
        .def("getSize", &Mapping::size)
        ;

    py::class_<Listing, ListingPtr, ValueNode>(m, "Listing")
        .def(py::init<>())
        .def(py::init<int>())
        .def_property_readonly("empty", &Listing::empty)
        .def_property_readonly("size", &Listing::size)
        .def("clear", &Listing::clear)
        .def("reserve", &Listing::reserve)
        .def("setFlowStyle", &Listing::setFlowStyle)
        .def("isFlowStyle", &Listing::isFlowStyle)
        .def("setFloatingNumberFormat", &Listing::setFloatingNumberFormat)
        .def_property_readonly("floatingNumberFormat", &Listing::floatingNumberFormat)
        .def_property_readonly("front", &Listing::front)
        .def_property_readonly("back", &Listing::back)
        .def("at", &Listing::at)
        .def("write", (void(Listing::*)(int,int)) &Listing::write)
        .def("write", (void(Listing::*)(int,const string&, StringStyle)) &Listing::write)
        .def("write", [](Listing& self, int i, const string& value){ self.write(i, value); })
        .def("__getitem__", &Listing::at)
        .def("newMapping", &Listing::newMapping)
        .def("append", (void(Listing::*)(ValueNode*)) &Listing::append)
        .def("append", (void(Listing::*)(int)) &Listing::append)
        .def("append", (void(Listing::*)(double)) &Listing::append)
        .def("append", (void(Listing::*)(const string&, StringStyle)) &Listing::append)
        .def("append", [](Listing& self, const string& value){ self.append(value); })
        .def("appendLF", &Listing::appendLF)

        // deprecated
        .def("isEmpty", &Listing::empty)
        .def("getSize", &Listing::size)
        .def("getFront", &Listing::front)
        .def("getBack", &Listing::back)
        ;
}

} // namespace cnoid
