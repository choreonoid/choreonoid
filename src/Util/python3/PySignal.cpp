/*!
  @author Shin'ichiro Nakaoka
*/

#include "PySignal.h"
#include "../Signal.h"
#include "../ConnectionSet.h"

namespace py = pybind11;
using namespace cnoid;

namespace {

void (ConnectionSet::*ConnectionSet_add1)(const Connection&) = &ConnectionSet::add;
void (ConnectionSet::*ConnectionSet_add2)(const ConnectionSet&) = &ConnectionSet::add;
void (ConnectionSet::*ConnectionSet_block1)() = &ConnectionSet::block;
void (ConnectionSet::*ConnectionSet_block2)(int) = &ConnectionSet::block;
void (ConnectionSet::*ConnectionSet_unblock1)() = &ConnectionSet::unblock;
void (ConnectionSet::*ConnectionSet_unblock2)(int) = &ConnectionSet::unblock;

void (ScopedConnectionSet::*ScopedConnectionSet_block1)() = &ScopedConnectionSet::block;
void (ScopedConnectionSet::*ScopedConnectionSet_block2)(int) = &ScopedConnectionSet::block;
void (ScopedConnectionSet::*ScopedConnectionSet_unblock1)() = &ScopedConnectionSet::unblock;
void (ScopedConnectionSet::*ScopedConnectionSet_unblock2)(int) = &ScopedConnectionSet::unblock;

}

namespace cnoid {

void exportPySignalTypes(py::module& m)
{
    PySignal<void()>(m, "VoidSignal");
    PySignal<void(bool)>(m,"BoolSignal");
    PySignal<void(int)>(m, "IntSignal");
    PySignal<void(double)>(m, "DoubleSignal");
    PySignal<void(const std::string& str)>(m, "StringSignal");

    py::class_<Connection>(m, "Connection")
        .def("disconnect", &Connection::disconnect)
        .def("connected", &Connection::connected)
        .def("block", &Connection::block)
        .def("unblock", &Connection::unblock);

    py::class_<ScopedConnection>(m, "ScopedConnection")
        .def("reset", &ScopedConnection::reset)
        .def("disconnect", &ScopedConnection::disconnect)
        .def("connected", &ScopedConnection::connected)
        .def("block", &ScopedConnection::block)
        .def("unblock", &ScopedConnection::unblock);
    
    py::class_<ConnectionSet>(m, "ConnectionSet")
        .def("empty", &ConnectionSet::empty)
        .def("numConnections", &ConnectionSet::numConnections)
        .def("add", ConnectionSet_add1)
        .def("add", ConnectionSet_add2)
        .def("block", ConnectionSet_block1)
        .def("block", ConnectionSet_block2)
        .def("unblock", ConnectionSet_unblock1)
        .def("unblock", ConnectionSet_unblock2)
        .def("disconnect", &ConnectionSet::disconnect);

    py::class_<ScopedConnectionSet>(m, "ScopedConnectionSet")
        .def("empty", &ScopedConnectionSet::empty)
        .def("numConnections", &ScopedConnectionSet::numConnections)
        .def("add", &ScopedConnectionSet::add)
        .def("block", ScopedConnectionSet_block1)
        .def("block", ScopedConnectionSet_block2)
        .def("unblock", ScopedConnectionSet_unblock1)
        .def("unblock", ScopedConnectionSet_unblock2)
        .def("disconnect", &ScopedConnectionSet::disconnect);
}

}
