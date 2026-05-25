#include "PyUtil.h"
#include "../MessageOut.h"

namespace nb = nanobind;
using namespace cnoid;

namespace cnoid {

void exportPyMessageOut(nb::module_& m)
{
    nb::class_<MessageOut, Referenced>(m, "MessageOut")
        .def_prop_ro_static("master", [](nb::handle){ return MessageOut::master(); })
        .def_prop_ro_static("interactive", [](nb::handle){ return MessageOut::interactive(); })
        .def("put", (void(MessageOut::*)(const std::string&))&MessageOut::put)
        .def("putln", (void(MessageOut::*)(const std::string&))&MessageOut::putln)
        .def("putHighlighted", &MessageOut::putHighlighted)
        .def("putHighlightedln", &MessageOut::putHighlightedln)
        .def("putWarning", &MessageOut::putWarning)
        .def("putWarningln", &MessageOut::putWarningln)
        .def("putError", &MessageOut::putError)
        .def("putErrorln", &MessageOut::putErrorln)
        ;
}

}
