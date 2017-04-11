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

// for MSVC++2015 Update3
CNOID_PYTHON_DEFINE_GET_POINTER(AbstractSeq)
CNOID_PYTHON_DEFINE_GET_POINTER(AbstractMultiSeq)
CNOID_PYTHON_DEFINE_GET_POINTER(MultiValueSeq)

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
        .def("partLabel", &AbstractMultiSeq::partLabel, py::return_value_policy::reference);

    py::implicitly_convertible<AbstractMultiSeqPtr, AbstractSeqPtr>();

    const MultiValueSeq::Element& (MultiValueSeq::*MultiValueSeq_at_const)(int, int) const = &MultiValueSeq::at;
    MultiValueSeq::Element& (MultiValueSeq::*MultiValueSeq_at)(int, int) = &MultiValueSeq::at;

    void (MultiValueSeq::*MultiValueSeq_pop_front_int)(int) = &MultiValueSeq::pop_front;
    void (MultiValueSeq::*MultiValueSeq_pop_front)(int) = &MultiValueSeq::pop_front;    
    
    MultiValueSeq::Row (MultiValueSeq::*MultiValueSeq_row)(int) = &MultiValueSeq::row;
    const MultiValueSeq::Row (MultiValueSeq::*MultiValueSeq_row_const)(int) const = &MultiValueSeq::row;

    MultiValueSeq::Column (MultiValueSeq::*MultiValueSeq_column)(int) = &MultiValueSeq::column;
    const MultiValueSeq::Column (MultiValueSeq::*MultiValueSeq_column_const)(int) const = &MultiValueSeq::column;
    
    MultiValueSeq::Frame (MultiValueSeq::*MultiValueSeq_frame)(int) = &MultiValueSeq::frame;
    const MultiValueSeq::Frame (MultiValueSeq::*MultiValueSeq_frame_const)(int) const = &MultiValueSeq::frame;

    MultiValueSeq::Part (MultiValueSeq::*MultiValueSeq_part)(int) = &MultiValueSeq::part;
    const MultiValueSeq::Part (MultiValueSeq::*MultiValueSeq_part_const)(int) const = &MultiValueSeq::part;
    
    py::class_< MultiValueSeq, AbstractMultiSeq >(m, "MultiValueSeq")
        .def("empty", &MultiValueSeq::empty)
        .def("resize", &MultiValueSeq::resize)
        .def("resizeColumn", &MultiValueSeq::resizeColumn)
        .def("rowSize", &MultiValueSeq::rowSize)
        .def("resizeRow", &MultiValueSeq::resizeRow)
        .def("colSize", &MultiValueSeq::colSize)
        .def("clear", &MultiValueSeq::clear)
        .def("at", MultiValueSeq_at_const, py::return_value_policy::reference)
        .def("row", MultiValueSeq_row)
        .def("row", MultiValueSeq_row_const)
        .def("column", MultiValueSeq_column)
        .def("column", MultiValueSeq_column_const)
        .def("append", &MultiValueSeq::append)
        .def("pop_back", &MultiValueSeq::pop_back)
        .def("pop_front", MultiValueSeq_pop_front_int)
        .def("pop_front", MultiValueSeq_pop_front)
        
        .def("copySeqProperties", &MultiValueSeq::copySeqProperties)
        .def("frameRate", &MultiValueSeq::frameRate)
        .def("timeStep", &MultiValueSeq::timeStep)
        .def("numFrames", &MultiValueSeq::numFrames)
        .def("numParts", &MultiValueSeq::numParts)
        .def("timeLength", &MultiValueSeq::timeLength)
        .def("frameOfTime", &MultiValueSeq::frameOfTime)
        .def("timeOfFrame", &MultiValueSeq::timeOfFrame)
        .def("clampFrameIndex", &MultiValueSeq::clampFrameIndex)
        .def("frame", MultiValueSeq_frame)
        .def("frame", MultiValueSeq_frame_const)
        .def("part", MultiValueSeq_part)
        .def("part", MultiValueSeq_part_const)

        .def("loadPlainFormat", &MultiValueSeq::loadPlainFormat)
        .def("saveAsPlainFormat", &MultiValueSeq::saveAsPlainFormat);

    py::implicitly_convertible<MultiValueSeqPtr, AbstractMultiSeqPtr>();
}

}
