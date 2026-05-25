#include "PySignal.h"
#include "PyUtil.h"
#include "../ConnectionSet.h"

namespace nb = nanobind;
using namespace cnoid;

namespace cnoid {

void exportPySignalTypes(nb::module_& m)
{
    PySignal<void()>(m, "VoidVoidSignal");
    PySignal<void(bool)>(m, "VoidBoolSignal");
    PySignal<void(int)>(m, "VoidIntSignal");
    PySignal<void(double)>(m, "VoidDoubleSignal");
    PySignal<bool(double)>(m, "BoolDoubleSignal");
    PySignal<void(double,bool)>(m, "VoidDoubleBoolSignal");
    PySignal<double(double,bool)>(m, "DoubleDoubleBoolSignal");
    PySignal<void(const std::string&)>(m, "VoidStringSignal");

    nb::class_<Connection>(m, "Connection")
        .def(nb::init<>())
        .def(nb::init<const Connection&>())
        .def("disconnect", &Connection::disconnect)
        .def("connected", &Connection::connected)
        .def("block", &Connection::block)
        .def("unblock", &Connection::unblock);

    nb::class_<ScopedConnection>(m, "ScopedConnection")
        .def(nb::init<>())
        .def(nb::init<const Connection&>())
        .def("reset", (void (ScopedConnection::*)()) &ScopedConnection::reset)
        .def("reset", (void (ScopedConnection::*)(const Connection&)) &ScopedConnection::reset)
        .def("disconnect", &ScopedConnection::disconnect)
        .def("connected", &ScopedConnection::connected)
        .def("block", &ScopedConnection::block)
        .def("unblock", &ScopedConnection::unblock);

    nb::class_<ConnectionSetBase>(m, "ConnectionSetBase")
        .def_prop_ro("empty", &ConnectionSetBase::empty)
        .def_prop_ro("numConnections", &ConnectionSetBase::numConnections)
        .def("add", (int (ConnectionSetBase::*)(const Connection&)) &ConnectionSetBase::add)
        .def("add", (void (ConnectionSetBase::*)(const ConnectionSetBase&)) &ConnectionSetBase::add)
        .def("block", (void (ConnectionSetBase::*)()) &ConnectionSetBase::block)
        .def("block", (void (ConnectionSetBase::*)(int)) &ConnectionSetBase::block)
        .def("unblock", (void (ConnectionSetBase::*)()) &ConnectionSetBase::unblock)
        .def("unblock", (void (ConnectionSetBase::*)(int)) &ConnectionSetBase::unblock)
        .def("disconnect", &ConnectionSetBase::disconnect)
        ;

    nb::class_<ConnectionSet, ConnectionSetBase>(m, "ConnectionSet")
        .def(nb::init<>())
        .def(nb::init<const ConnectionSet&>())
        ;

    nb::class_<ScopedConnectionSet, ConnectionSetBase>(m, "ScopedConnectionSet")
        .def(nb::init<>())
        ;
}

}
