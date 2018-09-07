/*!
  @author Shin'ichiro Nakaoka
*/

#include "PySignal.h"
#include "../Signal.h"
#include "../ConnectionSet.h"

using namespace boost::python;
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

void exportPySignalTypes()
{
    PySignal<void()>("VoidSignal");
    PySignal<void(bool)>("BoolSignal");
    PySignal<void(int)>("IntSignal");
    PySignal<void(double)>("DoubleSignal");
    PySignal<void(const std::string& str)>("StringSignal");

    class_<Connection>("Connection")
        .def("disconnect", &Connection::disconnect)
        .def("connected", &Connection::connected)
        .def("block", &Connection::block)
        .def("unblock", &Connection::unblock);

    class_<ScopedConnection, boost::noncopyable>("ScopedConnection")
        .def("reset", &ScopedConnection::reset)
        .def("disconnect", &ScopedConnection::disconnect)
        .def("connected", &ScopedConnection::connected)
        .def("block", &ScopedConnection::block)
        .def("unblock", &ScopedConnection::unblock);
    
    class_<ConnectionSet>("ConnectionSet")
        .def("empty", &ConnectionSet::empty)
        .def("isEmpty", &ConnectionSet::empty)
        .def("numConnections", &ConnectionSet::numConnections)
        .def("getNumConnections", &ConnectionSet::numConnections)
        .def("add", ConnectionSet_add1)
        .def("add", ConnectionSet_add2)
        .def("block", ConnectionSet_block1)
        .def("block", ConnectionSet_block2)
        .def("unblock", ConnectionSet_unblock1)
        .def("unblock", ConnectionSet_unblock2)
        .def("disconnect", &ConnectionSet::disconnect);

    class_<ScopedConnectionSet, boost::noncopyable>("ScopedConnectionSet")
        .def("empty", &ScopedConnectionSet::empty)
        .def("isEmpty", &ScopedConnectionSet::empty)
        .def("numConnections", &ScopedConnectionSet::numConnections)
        .def("getNumConnections", &ScopedConnectionSet::numConnections)
        .def("add", &ScopedConnectionSet::add)
        .def("block", ScopedConnectionSet_block1)
        .def("block", ScopedConnectionSet_block2)
        .def("unblock", ScopedConnectionSet_unblock1)
        .def("unblock", ScopedConnectionSet_unblock2)
        .def("disconnect", &ScopedConnectionSet::disconnect);
}

}
