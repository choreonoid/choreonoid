/*!
  @author Shin'ichiro Nakaoka
*/

#include "PySignal.h"
#include "../Signal.h"

using namespace boost::python;
using namespace cnoid;

namespace cnoid {

void exportSignalTypes()
{
    class_<Connection>("Connection")
        .def("disconnect", &Connection::disconnect)
        .def("connected", &Connection::connected)
        .def("block", &Connection::block)
        .def("unblock", &Connection::unblock);

    PySignalProxy<void()>("VoidSignal");
    PySignalProxy<void(bool)>("BoolSignal");
    PySignalProxy<void(double)>("DoubleSignal");
    PySignalProxy<void(const std::string& str)>("StringSignal");
}

}
