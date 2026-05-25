#include "PyUtil.h"
#include "../FilePathVariableProcessor.h"

namespace nb = nanobind;
using namespace cnoid;

namespace cnoid {

void exportPyFilePathVariableProcessor(nb::module_& m)
{
    nb::class_<FilePathVariableProcessor, Referenced>(m, "FilePathVariableProcessor")
        .def_prop_ro_static("systemInstance", [](nb::handle){ return FilePathVariableProcessor::systemInstance(); })
        .def_prop_ro_static("currentInstance", [](nb::handle){ return FilePathVariableProcessor::currentInstance(); })

        .def("parameterize", &FilePathVariableProcessor::parameterize)
        .def("expand", &FilePathVariableProcessor::expand)
        .def("errorMessage", &FilePathVariableProcessor::errorMessage)
        ;
}

}
