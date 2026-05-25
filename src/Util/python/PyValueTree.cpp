#include "PyUtil.h"
#include "../ValueTree.h"

using namespace std;
using namespace cnoid;
namespace nb = nanobind;

namespace {

template<class ValueType>
nb::object ValueNode_read(ValueNode& self)
{
    ValueType value;
    if(self.read(value)){
        return nb::cast(value);
    }
    return nb::none();
}

template<class ValueType>
nb::object Mapping_read(Mapping& self, const string& key)
{
    ValueType value;
    if(self.read(key, value)){
        return nb::cast(value);
    }
    return nb::none();
}

nb::object Mapping_get(Mapping& self, const std::string& key, nb::object defaultValue)
{
    if(PyBool_Check(defaultValue.ptr())){
        bool value;
        if(self.read(key, value)){
            return nb::cast(value);
        }
    } else if(PyLong_Check(defaultValue.ptr())){
        int value;
        if(self.read(key, value)){
            return nb::cast(value);
        }
    } else if(PyFloat_Check(defaultValue.ptr())){
        double value;
        if(self.read(key, value)){
            return nb::cast(value);
        }
    } else if(PyBytes_Check(defaultValue.ptr())){
        string value;
        if(self.read(key, value)){
            return nb::cast(value);
        }
    } else {
        return nb::none();
    }
    return defaultValue;
}

ValueNodePtr Mapping_getitem(Mapping& self, const std::string& key)
{
    return ValueNodePtr(&self[key]);
}

void Mapping_write(Mapping& self, const std::string &key, nb::object value){
    if(PyBool_Check(value.ptr())){
        self.write(key, nb::cast<bool>(value));
    } else if(PyLong_Check(value.ptr())){
        self.write(key, nb::cast<int>(value));
    } else if(PyFloat_Check(value.ptr())){
        self.write(key, nb::cast<double>(value));
    } else {
        throw nb::type_error("The argument type is not supported");
    }
}

ValueNodePtr Listing_getitem(Listing& self, int index)
{
    return &self[index];
}


} // namespace

namespace cnoid {

void exportPyValueTree(nb::module_& m)
{
    nb::enum_<StringStyle>(m, "StringStyle")
        .value("PLAING_STRING", PLAIN_STRING)
        .value("SINGLE_QUOTED", SINGLE_QUOTED)
        .value("DOUBLE_QUOTED", DOUBLE_QUOTED)
        .value("LITERAL_STRING", LITERAL_STRING)
        .value("FOLDED_STRING", FOLDED_STRING)
        .export_values();

    nb::class_<ValueNode, Referenced>(m, "ValueNode")
        .def("isValid", &ValueNode::isValid)
        .def("__bool__", &ValueNode::isValid)
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
        .def_prop_ro("line", &ValueNode::line)
        .def_prop_ro("column", &ValueNode::column)
        ;

    nb::class_<Mapping, ValueNode>(m, "Mapping")
        .def(nb::init<>())
        .def_prop_ro("empty", &Mapping::empty)
        .def_prop_ro("size", &Mapping::size)
        .def("clear", &Mapping::clear)
        .def("setFlowStyle", &Mapping::setFlowStyle)
        .def("isFlowStyle", &Mapping::isFlowStyle)
        .def("setFloatingNumberFormat", &Mapping::setFloatingNumberFormat)
        .def_prop_ro("floatingNumberFormat", &Mapping::floatingNumberFormat)
        .def("setKeyQuoteStyle", &Mapping::setKeyQuoteStyle)
        .def("find", (ValueNode*(Mapping::*)(const std::string&)const) &Mapping::find)
        .def("findMapping", (Mapping*(Mapping::*)(const std::string&)const) &Mapping::findMapping)
        .def("findListing", (Listing*(Mapping::*)(const std::string&)const) &Mapping::findListing)
        .def("__getitem__", [](Mapping& self, const std::string& key){ return Mapping_getitem(self, key); })
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
        .def("write", (void(Mapping::*)(const string&, ValueNode*)) &Mapping::write)
        .def("__len__", &Mapping::size)
        .def("keys", [](Mapping &self) {
            std::vector<std::string> res;
            for (auto it = self.begin(); it != self.end(); it++) res.push_back(it->first);
            return res; })
        ;

    nb::class_<Listing, ValueNode>(m, "Listing")
        .def(nb::init<>())
        .def(nb::init<int>())
        .def_prop_ro("empty", &Listing::empty)
        .def_prop_ro("size", &Listing::size)
        .def("clear", &Listing::clear)
        .def("reserve", &Listing::reserve)
        .def("setFlowStyle", &Listing::setFlowStyle)
        .def("isFlowStyle", &Listing::isFlowStyle)
        .def("setFloatingNumberFormat", &Listing::setFloatingNumberFormat)
        .def_prop_ro("floatingNumberFormat", &Listing::floatingNumberFormat)
        .def_prop_ro("front", &Listing::front)
        .def_prop_ro("back", &Listing::back)
        .def("at", &Listing::at)
        .def("write", (void(Listing::*)(int,int)) &Listing::write)
        .def("write", (void(Listing::*)(int,const string&, StringStyle)) &Listing::write)
        .def("write", [](Listing& self, int i, const string& value){ self.write(i, value); })
        .def("__getitem__", [](Listing& self, int index){ return Listing_getitem(self, index); })
        .def("newMapping", &Listing::newMapping)
        .def("append", (void(Listing::*)(ValueNode*)) &Listing::append)
        .def("append", (void(Listing::*)(int)) &Listing::append)
        .def("append", (void(Listing::*)(double)) &Listing::append)
        .def("append", (void(Listing::*)(const string&, StringStyle)) &Listing::append)
        .def("append", [](Listing& self, const string& value){ self.append(value); })
        .def("appendLF", &Listing::appendLF)
        .def("__len__", &Listing::size)
        ;
}

} // namespace cnoid
