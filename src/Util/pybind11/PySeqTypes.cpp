/*!
 * @author Shin'ichiro Nakaoka
*/

#include "PyUtil.h"
#include "../MultiValueSeq.h"
#include "../Vector3Seq.h"
#include "../ValueTree.h"
#include "../YAMLWriter.h"

namespace py = pybind11;
using namespace cnoid;

namespace cnoid {

void exportPySeqTypes(py::module& m)
{
    py::class_<AbstractSeq>(m, "AbstractSeq")
        .def("cloneSeq", &AbstractSeq::cloneSeq)
        .def("copySeqProperties", &AbstractSeq::copySeqProperties)
        .def("seqType", &AbstractSeq::seqType, py::return_value_policy::reference)
        .def("getFrameRate", &AbstractSeq::getFrameRate)
        .def("setFrameRate", &AbstractSeq::setFrameRate)
        .def("getTimeStep", &AbstractSeq::getTimeStep)
        .def("setTimeStep", &AbstractSeq::setTimeStep)
        .def("getTimeOfFrame", &AbstractSeq::getTimeOfFrame)
        .def("getOffsetTimeFrame", &AbstractSeq::getOffsetTimeFrame)
        .def("getOffsetTime", &AbstractSeq::getOffsetTime)
        .def("getNumFrames", &AbstractSeq::getNumFrames)
        .def("setNumFrames", &AbstractSeq::setNumFrames,
                py::arg("n"), py::arg("clearNewElements") = false)
        .def("setTimeLength", &AbstractSeq::setTimeLength,
                py::arg("length"), py::arg("clearNewElements") = false)
        .def("getTimeLength", &AbstractSeq::getTimeLength)
        .def("seqContentName", &AbstractSeq::seqContentName, py::return_value_policy::reference)
        .def("setSeqContentName", &AbstractSeq::setSeqContentName)
        .def("readSeq", &AbstractSeq::readSeq)
        .def("writeSeq", &AbstractSeq::writeSeq)
        .def("seqMessage", &AbstractSeq::seqMessage, py::return_value_policy::reference)
        .def("defaultFrameRate", &AbstractSeq::defaultFrameRate);

    py::class_<AbstractMultiSeq, AbstractSeq>(m, "AbstractMultiSeq")
        .def("copySeqProperties", &AbstractMultiSeq::copySeqProperties)
        .def("setDimension", &AbstractMultiSeq::setDimension,
                py::arg("numFrames"), py::arg("numParts"), py::arg("clearNewElements") = false)
        .def("setNumParts", &AbstractMultiSeq::setNumParts,
                py::arg("numParts"), py::arg("clearNewElements") = false)
        .def("getNumParts", &AbstractMultiSeq::getNumParts)
        .def("partIndex", &AbstractMultiSeq::partIndex)
        .def("partLabel", &AbstractMultiSeq::partLabel, py::return_value_policy::copy);

    py::class_< MultiValueSeq, AbstractMultiSeq >(m, "MultiValueSeq")
        .def("empty", &MultiValueSeq::empty)
        .def("resize", &MultiValueSeq::resize)
        .def("resizeColumn", &MultiValueSeq::resizeColumn)
        .def("rowSize", &MultiValueSeq::rowSize)
        .def("resizeRow", &MultiValueSeq::resizeRow)
        .def("colSize", &MultiValueSeq::colSize)
        .def("clear", &MultiValueSeq::clear)
        .def("at", (const MultiValueSeq::Element& (MultiValueSeq::*)(int, int) const) &MultiValueSeq::at, py::return_value_policy::reference_internal)
        .def("row", (MultiValueSeq::Row (MultiValueSeq::*)(int)) &MultiValueSeq::row)
        .def("row", (const MultiValueSeq::Row (MultiValueSeq::*)(int) const ) &MultiValueSeq::row)
        .def("column", (MultiValueSeq::Column (MultiValueSeq::*)(int)) &MultiValueSeq::column)
        .def("column", (const MultiValueSeq::Column (MultiValueSeq::*)(int) const ) &MultiValueSeq::column)
        .def("append", &MultiValueSeq::append)
        .def("pop_back", &MultiValueSeq::pop_back)
        .def("pop_front", (void (MultiValueSeq::*)(int)) &MultiValueSeq::pop_front)
        .def("pop_front", (void (MultiValueSeq::*)()) &MultiValueSeq::pop_front)
        
        .def("copySeqProperties", &MultiValueSeq::copySeqProperties)
        .def("frameRate", &MultiValueSeq::frameRate)
        .def("timeStep", &MultiValueSeq::timeStep)
        .def("numFrames", &MultiValueSeq::numFrames)
        .def("numParts", &MultiValueSeq::numParts)
        .def("timeLength", &MultiValueSeq::timeLength)
        .def("frameOfTime", &MultiValueSeq::frameOfTime)
        .def("timeOfFrame", &MultiValueSeq::timeOfFrame)
        .def("clampFrameIndex", &MultiValueSeq::clampFrameIndex)
        .def("frame", (MultiValueSeq::Frame (MultiValueSeq::*)(int)) &MultiValueSeq::frame)
        .def("frame", (const MultiValueSeq::Frame (MultiValueSeq::*)(int) const ) &MultiValueSeq::frame)
        .def("part", (MultiValueSeq::Part (MultiValueSeq::*)(int)) &MultiValueSeq::part)
        .def("part", (const MultiValueSeq::Part (MultiValueSeq::*)(int) const ) &MultiValueSeq::part)

        .def("loadPlainFormat", &MultiValueSeq::loadPlainFormat)
        .def("saveAsPlainFormat", &MultiValueSeq::saveAsPlainFormat);

}

}
