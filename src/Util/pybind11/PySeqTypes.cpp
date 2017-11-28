/*!
 * @author Shin'ichiro Nakaoka
*/

#include "../MultiValueSeq.h"
#include "../ValueTree.h"
#include "../YAMLWriter.h"
#include <pybind11/pybind11.h>

namespace py = pybind11;
using namespace cnoid;

namespace cnoid {

void exportPySeqTypes(py::module& m)
{
    py::class_<AbstractSeq>(m, "AbstractSeq")
        .def("cloneSeq", &AbstractSeq::cloneSeq)
        .def("copySeqProperties", &AbstractSeq::copySeqProperties)
        .def("seqType", &AbstractSeq::seqType)
        .def("getFrameRate", &AbstractSeq::getFrameRate)
        .def("setFrameRate", &AbstractSeq::setFrameRate)
        .def("getTimeStep", &AbstractSeq::getTimeStep)
        .def("setTimeStep", &AbstractSeq::setTimeStep)
        .def("getTimeOfFrame", &AbstractSeq::getTimeOfFrame)
        .def("getOffsetTimeFrame", &AbstractSeq::getOffsetTimeFrame)
        .def("getOffsetTime", &AbstractSeq::getOffsetTime)
        .def("getNumFrames", &AbstractSeq::getNumFrames)
        .def("setNumFrames", [](AbstractSeq& self, int n){ self.setNumFrames(n); })
        .def("setNumFrames", [](AbstractSeq& self, int n, bool clearNewElements){ self.setNumFrames(n, clearNewElements); })
        .def("setTimeLength",[](AbstractSeq& self, double length){ self.setTimeLength(length); })
        .def("setTimeLength",[](AbstractSeq& self, double length, bool clearNewElements){ self.setTimeLength(length, clearNewElements); })
        .def("getTimeLength", &AbstractSeq::getTimeLength)
        .def("seqContentName", &AbstractSeq::seqContentName)
        .def("setSeqContentName", &AbstractSeq::setSeqContentName)
        .def("readSeq", &AbstractSeq::readSeq)
        .def("writeSeq", &AbstractSeq::writeSeq)
        .def("seqMessage", &AbstractSeq::seqMessage)
        .def("defaultFrameRate", &AbstractSeq::defaultFrameRate)
        ;

    py::class_<AbstractMultiSeq, AbstractSeq>(m, "AbstractMultiSeq")
        .def("copySeqProperties", &AbstractMultiSeq::copySeqProperties)
        .def("setDimension", [](AbstractMultiSeq& self, int numFrames, int numParts){ self.setDimension(numFrames, numParts); })
        .def("setDimension", [](AbstractMultiSeq& self, int numFrames, int numParts, bool clearNewElements){
                self.setDimension(numFrames, numParts, clearNewElements); })
        .def("setNumParts", [](AbstractMultiSeq& self, int numParts){ self.setNumParts(numParts); })
        .def("setNumParts", [](AbstractMultiSeq& self, int numParts, bool clearNewElements){
                self.setNumParts(numParts, clearNewElements); })
        .def("getNumParts", &AbstractMultiSeq::getNumParts)
        .def("partIndex", &AbstractMultiSeq::partIndex)
        .def("partLabel", &AbstractMultiSeq::partLabel)
        ;

    typedef Deque2D<double, std::allocator<double>> Deque2DDouble;
    
    py::class_<Deque2DDouble::Row>(m, "Deque2DDouble_Row")
        .def("size", &Deque2DDouble::Row::size)
        .def("at", &Deque2DDouble::Row::at, py::return_value_policy::reference_internal)
        .def("__getitem__", [](Deque2DDouble::Row& self, int i){ return self[i]; })
        .def("__setitem__", [](Deque2DDouble::Row& self, int i, double x){ self[i] = x; })
        ;

    py::class_<MultiValueSeq, AbstractMultiSeq>(m, "MultiValueSeq")
        .def("empty", &MultiValueSeq::empty)
        .def("resize", &MultiValueSeq::resize)
        .def("clear", &MultiValueSeq::clear)
        .def("at", (MultiValueSeq::Element& (MultiValueSeq::*)(int, int)) &MultiValueSeq::at, py::return_value_policy::reference_internal)
        .def("append", &MultiValueSeq::append)
        .def("pop_back", &MultiValueSeq::pop_back)
        .def("pop_front", (void (MultiValueSeq::*)(int)) &MultiValueSeq::pop_front)
        .def("pop_front", (void (MultiValueSeq::*)()) &MultiValueSeq::pop_front)
        
        //.def("resizeColumn", &MultiValueSeq::resizeColumn)
        //.def("rowSize", &MultiValueSeq::rowSize)
        //.def("resizeRow", &MultiValueSeq::resizeRow)
        //.def("colSize", &MultiValueSeq::colSize)
        //.def("row", (MultiValueSeq::Row (MultiValueSeq::*)(int)) &MultiValueSeq::row)
        //.def("column", (MultiValueSeq::Column (MultiValueSeq::*)(int)) &MultiValueSeq::column)

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
        .def("part", (MultiValueSeq::Part (MultiValueSeq::*)(int)) &MultiValueSeq::part)

        .def("loadPlainFormat", &MultiValueSeq::loadPlainFormat)
        .def("saveAsPlainFormat", &MultiValueSeq::saveAsPlainFormat);
}

}
