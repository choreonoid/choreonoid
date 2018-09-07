#include "../MulticopterSimulatorItem.h"
#include <cnoid/PyBase>

using namespace cnoid;
namespace py = pybind11;

PYBIND11_MODULE(MulticopterPlugin, m)
{
    m.doc() = "Choreonoid MulticopterPlugin module";

    py::class_<MulticopterSimulatorItem, MulticopterSimulatorItemPtr, SubSimulatorItem>(m, "MulticopterSimulatorItem")
        .def(py::init<>());
    
    PyItemList<MulticopterSimulatorItem>(m, "MulticopterSimulatorItemList");
}
