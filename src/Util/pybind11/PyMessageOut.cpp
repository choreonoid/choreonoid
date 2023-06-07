#include "PyUtil.h"
#include "../MessageOut.h"

namespace py = pybind11;
using namespace cnoid;

namespace cnoid {

void exportPyMessageOut(py::module& m)
{
    py::class_<MessageOut, MessageOutPtr, Referenced>(m, "MessageOut")
        .def_property_readonly_static("master", [](py::object){ return MessageOut::master(); })
        .def_property_readonly_static("interactive", [](py::object){ return MessageOut::interactive(); })
        .def("put", (void(MessageOut::*)(const std::string&))&MessageOut::put)
        .def("putln", (void(MessageOut::*)(const std::string&))&MessageOut::putln)
        .def("putHighlighted", (void(MessageOut::*)(const std::string&))&MessageOut::putHighlighted)
        .def("putHighlightedln", (void(MessageOut::*)(const std::string&))&MessageOut::putHighlightedln)
        .def("putWarning", (void(MessageOut::*)(const std::string&))&MessageOut::putWarning)
        .def("putWarningln", (void(MessageOut::*)(const std::string&))&MessageOut::putWarningln)
        .def("putError", (void(MessageOut::*)(const std::string&))&MessageOut::putError)
        .def("putErrorln", (void(MessageOut::*)(const std::string&))&MessageOut::putErrorln)
        ;
}

}
