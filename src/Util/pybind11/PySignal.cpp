/*!
  @author Shin'ichiro Nakaoka
*/

#include "PyUtil.h"
#include "../Signal.h"
#include "../ConnectionSet.h"

namespace py = pybind11;
using namespace cnoid;

namespace cnoid {

void exportPySignalTypes(py::module& m)
{
    PySignal<void()>(m, "VoidSignal");
    PySignal<void(bool)>(m,"BoolSignal");
    PySignal<void(int)>(m, "IntSignal");
    PySignal<void(double)>(m, "DoubleSignal");
    PySignal<void(const std::string&)>(m, "StringSignal");

    py::class_<Connection>(m, "Connection")
        .def(py::init<>())
        .def(py::init<const Connection&>())
        .def("disconnect", &Connection::disconnect)
        .def("connected", &Connection::connected)
        .def("block", &Connection::block)
        .def("unblock", &Connection::unblock);

    py::class_<ScopedConnection>(m, "ScopedConnection")
        .def(py::init<>())
        .def(py::init<const Connection&>())
        .def("reset", (void (ScopedConnection::*)()) &ScopedConnection::reset)
        .def("reset", (void (ScopedConnection::*)(const Connection&)) &ScopedConnection::reset)
        .def("disconnect", &ScopedConnection::disconnect)
        .def("connected", &ScopedConnection::connected)
        .def("block", &ScopedConnection::block)
        .def("unblock", &ScopedConnection::unblock);

    py::class_<ConnectionSetBase>(m, "ConnectionSetBase")
        .def_property_readonly("empty", &ConnectionSetBase::empty)
        .def_property_readonly("numConnections", &ConnectionSetBase::numConnections)
        .def("add", (int (ConnectionSetBase::*)(const Connection&)) &ConnectionSetBase::add)
        .def("add", (void (ConnectionSetBase::*)(const ConnectionSetBase&)) &ConnectionSetBase::add)
        .def("block", (void (ConnectionSetBase::*)()) &ConnectionSetBase::block)
        .def("block", (void (ConnectionSetBase::*)(int)) &ConnectionSetBase::block)
        .def("unblock", (void (ConnectionSetBase::*)()) &ConnectionSetBase::unblock)
        .def("unblock", (void (ConnectionSetBase::*)(int)) &ConnectionSetBase::unblock)
        .def("disconnect", &ConnectionSetBase::disconnect)
        ;
    
    py::class_<ConnectionSet, ConnectionSetBase>(m, "ConnectionSet")
        .def(py::init<>())
        .def(py::init<const ConnectionSet&>())
        ;

    py::class_<ScopedConnectionSet, ConnectionSetBase>(m, "ScopedConnectionSet")
        .def(py::init<>())
        ;
}

}
