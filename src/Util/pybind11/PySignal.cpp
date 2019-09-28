/*!
  @author Shin'ichiro Nakaoka
*/

#include "PySignal.h"
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
    
    py::class_<ConnectionSet>(m, "ConnectionSet")
        .def(py::init<>())
        .def(py::init<const ConnectionSet&>())
        .def_property_readonly("empty", &ConnectionSet::empty)
        .def_property_readonly("numConnections", &ConnectionSet::numConnections)
        .def("add", (void (ConnectionSet::*)(const Connection&)) &ConnectionSet::add)
        .def("add", (void (ConnectionSet::*)(const ConnectionSet&)) &ConnectionSet::add)
        .def("block", (void (ConnectionSet::*)()) &ConnectionSet::block)
        .def("block", (void (ConnectionSet::*)(int)) &ConnectionSet::block)
        .def("unblock", (void (ConnectionSet::*)()) &ConnectionSet::unblock)
        .def("unblock", (void (ConnectionSet::*)(int)) &ConnectionSet::unblock)
        .def("disconnect", &ConnectionSet::disconnect)

        // deprecated
        .def("isEmpty", &ConnectionSet::empty)
        .def("getNumConnections", &ConnectionSet::numConnections)
        ;

    py::class_<ScopedConnectionSet>(m, "ScopedConnectionSet")
        .def(py::init<>())
        .def_property_readonly("empty", &ScopedConnectionSet::empty)
        .def_property_readonly("numConnections", &ScopedConnectionSet::numConnections)
        .def("add", &ScopedConnectionSet::add)
        .def("block", (void (ScopedConnectionSet::*)()) &ScopedConnectionSet::block)
        .def("block", (void (ScopedConnectionSet::*)(int)) &ScopedConnectionSet::block)
        .def("unblock", (void (ScopedConnectionSet::*)()) &ScopedConnectionSet::unblock)
        .def("unblock", (void (ScopedConnectionSet::*)(int)) &ScopedConnectionSet::unblock)
        .def("disconnect", &ScopedConnectionSet::disconnect)

        // deprecated
        .def("isEmpty", &ScopedConnectionSet::empty)
        .def("getNumConnections", &ScopedConnectionSet::numConnections)
        ;
}

}
