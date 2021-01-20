/*!
 * @author Shin'ichiro Nakaoka
*/

#include "../MultiValueSeq.h"
#include "../ReferencedObjectSeq.h"
#include "../ValueTree.h"
#include "../YAMLWriter.h"
#include "PyUtil.h"

using namespace std;
using namespace cnoid;
namespace py = pybind11;

namespace cnoid {

void exportPySeqTypes(py::module& m)
{
    py::class_<AbstractSeq, shared_ptr<AbstractSeq>>(m, "AbstractSeq")
        .def("cloneSeq", &AbstractSeq::cloneSeq)
        .def("copySeqProperties", &AbstractSeq::copySeqProperties)
        .def_property_readonly("seqType", &AbstractSeq::seqType)
        .def("getSeqType", &AbstractSeq::seqType)
        .def_property("frameRate", &AbstractSeq::getFrameRate, &AbstractSeq::setFrameRate)
        .def("getFrameRate", &AbstractSeq::getFrameRate)
        .def("setFrameRate", &AbstractSeq::setFrameRate)
        .def_property("timeStep", &AbstractSeq::getTimeStep, &AbstractSeq::setTimeStep)
        .def("getTimeStep", &AbstractSeq::getTimeStep)
        .def("setTimeStep", &AbstractSeq::setTimeStep)
        .def("timeOfFrame", &AbstractSeq::getTimeOfFrame)
        .def("getTimeOfFrame", &AbstractSeq::getTimeOfFrame)
        .def("frameOfTime", &AbstractSeq::getFrameOfTime)
        .def("getFrameOfTime", &AbstractSeq::getFrameOfTime)
        .def_property_readonly("offsetTimeFrame", &AbstractSeq::getOffsetTimeFrame)
        .def("getOffsetTimeFrame", &AbstractSeq::getOffsetTimeFrame)
        .def_property_readonly("offsetTime", &AbstractSeq::getOffsetTime)
        .def("getOffsetTime", &AbstractSeq::getOffsetTime)
        .def_property("numFrames", &AbstractSeq::getNumFrames, [](AbstractSeq& self, int n){ self.setNumFrames(n); })
        .def("getNumFrames", &AbstractSeq::getNumFrames)
        .def("setNumFrames", &AbstractSeq::setNumFrames, py::arg("n"), py::arg("fillNewElements") = false)
        .def_property("timeLength", &AbstractSeq::getTimeLength, [](AbstractSeq& self, double length){ self.setTimeLength(length); })
        .def("getTimeLength", &AbstractSeq::getTimeLength)
        .def("setTimeLength", &AbstractSeq::setTimeLength, py::arg("length"), py::arg("clearNewElements") = false)
        .def_property("seqContentName", &AbstractSeq::seqContentName, &AbstractSeq::setSeqContentName)
        .def("setSeqContentName", &AbstractSeq::setSeqContentName)
        .def("readSeq", [](AbstractSeq& self, const Mapping* archive){ return self.readSeq(archive); })
        .def("writeSeq", &AbstractSeq::writeSeq)
        .def_property_readonly("seqMessage", &AbstractSeq::seqMessage)
        .def_property_readonly("defaultFrameRate", &AbstractSeq::defaultFrameRate)

        // deprecated
        .def("getSeqContentName", &AbstractSeq::seqContentName)
        .def("getSeqMessage", &AbstractSeq::seqMessage)
        .def("getDefaultFrameRate", &AbstractSeq::defaultFrameRate)
        ;

    py::class_<AbstractMultiSeq, shared_ptr<AbstractMultiSeq>, AbstractSeq>(m, "AbstractMultiSeq")
        .def("copySeqProperties", &AbstractMultiSeq::copySeqProperties)
        .def("setDimension",
             &AbstractMultiSeq::setDimension,
             py::arg("numFrames"), py::arg("numParts"), py::arg("clearNewElements") = false)
        .def_property(
            "numParts",
            &AbstractMultiSeq::getNumParts,
            [](AbstractMultiSeq& self, int numParts){ self.setNumParts(numParts); })
        .def("setNumParts", &AbstractMultiSeq::setNumParts, py::arg("n"), py::arg("fillNewElements") = false)
        .def("getNumParts", &AbstractMultiSeq::getNumParts)
        .def("partIndex", &AbstractMultiSeq::partIndex)
        .def("partLabel", &AbstractMultiSeq::partLabel)

        // deprecated
        .def("getPartIndex", &AbstractMultiSeq::partIndex)
        .def("getPartLabel", &AbstractMultiSeq::partLabel)
        ;

    typedef Deque2D<double, std::allocator<double>> Deque2DDouble;
    
    py::class_<Deque2DDouble::Row>(m, "Deque2DDouble_Row")
        .def_property_readonly("size", &Deque2DDouble::Row::size)
        .def("at", &Deque2DDouble::Row::at, py::return_value_policy::reference_internal)
        .def("__getitem__", [](Deque2DDouble::Row& self, int i){ return self[i]; })
        .def("__setitem__", [](Deque2DDouble::Row& self, int i, double x){ self[i] = x; })

        // deprecated
        .def("getSize", &Deque2DDouble::Row::size)
        ;

    py::class_<MultiValueSeq, shared_ptr<MultiValueSeq>, AbstractMultiSeq>(m, "MultiValueSeq")
        .def_property_readonly("empty", &MultiValueSeq::empty)
        .def("resize", &MultiValueSeq::resize)
        .def("clear", &MultiValueSeq::clear)
        .def("at",
             (MultiValueSeq::Element& (MultiValueSeq::*)(int, int)) &MultiValueSeq::at,
             py::return_value_policy::reference_internal)
        .def("append", &MultiValueSeq::append)
        .def("pop_back", &MultiValueSeq::pop_back)
        .def("pop_front", &MultiValueSeq::pop_front, py::arg("n") = 1)
        .def("copySeqProperties", &MultiValueSeq::copySeqProperties)
        .def("clampFrameIndex", &MultiValueSeq::clampFrameIndex)
        .def("getClampFrameIndex", &MultiValueSeq::clampFrameIndex)
        .def("frame", (MultiValueSeq::Frame (MultiValueSeq::*)(int)) &MultiValueSeq::frame)
        .def("part", (MultiValueSeq::Part (MultiValueSeq::*)(int)) &MultiValueSeq::part)
        .def("loadPlainFormat",
             [](MultiValueSeq& self, const std::string& filename){
                 return self.loadPlainFormat(filename); })
        .def("saveAsPlainFormat",
             [](MultiValueSeq& self, const std::string& filename){
                 return self.saveAsPlainFormat(filename); })
        
        // deprecated
        .def("isEmpty", &MultiValueSeq::empty)
        .def("getFrame", (MultiValueSeq::Frame (MultiValueSeq::*)(int)) &MultiValueSeq::frame)
        .def("getPart", (MultiValueSeq::Part (MultiValueSeq::*)(int)) &MultiValueSeq::part)
        ;

    py::class_<ReferencedObjectSeq, shared_ptr<ReferencedObjectSeq>, AbstractSeq>(m, "ReferencedObjectSeq")
        .def_property_readonly("clear", &ReferencedObjectSeq::clear)
        .def_property_readonly("empty", &ReferencedObjectSeq::empty)
        //.def("at", (&ReferencedObjectSeq::element_type (ReferencedObjectSeq:*)(int)) &ReferencedObjectSeq::at)
        .def("__getitem__", [](ReferencedObjectSeq& self, int i){ return self[i]; })
        .def("__setitem__", [](ReferencedObjectSeq& self, int i, ReferencedPtr obj){ self[i] = obj; })
        .def_property(
            "front",
            [](ReferencedObjectSeq& self){ return self.empty() ? nullptr : self.front(); },
            [](ReferencedObjectSeq& self, ReferencedPtr obj){ if(!self.empty()) self.front() = obj; })
        .def_property(
            "back",
            [](ReferencedObjectSeq& self){ return self.empty() ? nullptr : self.back(); },
            [](ReferencedObjectSeq& self, ReferencedPtr obj){ if(!self.empty()) self.back() = obj; })
        ;
}

}
