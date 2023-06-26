#include "PyUtil.h"
#include "../FilePathVariableProcessor.h"

namespace py = pybind11;
using namespace cnoid;

namespace cnoid {

void exportPyFilePathVariableProcessor(py::module& m)
{
    py::class_<FilePathVariableProcessor, FilePathVariableProcessorPtr, Referenced>(m, "FilePathVariableProcessor")
        .def_property_readonly_static("systemInstance", [](py::object){ return FilePathVariableProcessor::systemInstance(); })
        .def_property_readonly_static("currentInstance", [](py::object){ return FilePathVariableProcessor::currentInstance(); })

        .def("parameterize", &FilePathVariableProcessor::parameterize)
        .def("expand", &FilePathVariableProcessor::expand)
        .def("errorMessage", &FilePathVariableProcessor::errorMessage)
        ;
}

}
