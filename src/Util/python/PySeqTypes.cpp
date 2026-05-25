#include "../MultiValueSeq.h"
#include "../ReferencedObjectSeq.h"
#include "../ValueTree.h"
#include "../YAMLWriter.h"
#include "PyUtil.h"
#include <nanobind/stl/shared_ptr.h>

using namespace std;
using namespace cnoid;
namespace nb = nanobind;

namespace cnoid {

void exportPySeqTypes(nb::module_& m)
{
    nb::class_<AbstractSeq>(m, "AbstractSeq")
        .def("cloneSeq", &AbstractSeq::cloneSeq)
        .def("copySeqProperties", &AbstractSeq::copySeqProperties)
        .def_prop_ro("seqType", &AbstractSeq::seqType)
        .def("getSeqType", &AbstractSeq::seqType)
        .def_prop_rw("frameRate", &AbstractSeq::getFrameRate, &AbstractSeq::setFrameRate)
        .def("getFrameRate", &AbstractSeq::getFrameRate)
        .def("setFrameRate", &AbstractSeq::setFrameRate)
        .def_prop_rw("timeStep", &AbstractSeq::getTimeStep, &AbstractSeq::setTimeStep)
        .def("getTimeStep", &AbstractSeq::getTimeStep)
        .def("setTimeStep", &AbstractSeq::setTimeStep)
        .def("timeOfFrame", &AbstractSeq::getTimeOfFrame)
        .def("getTimeOfFrame", &AbstractSeq::getTimeOfFrame)
        .def("frameOfTime", &AbstractSeq::getFrameOfTime)
        .def("getFrameOfTime", &AbstractSeq::getFrameOfTime)
        .def_prop_ro("offsetTimeFrame", &AbstractSeq::getOffsetTimeFrame)
        .def("getOffsetTimeFrame", &AbstractSeq::getOffsetTimeFrame)
        .def_prop_ro("offsetTime", &AbstractSeq::getOffsetTime)
        .def("getOffsetTime", &AbstractSeq::getOffsetTime)
        .def_prop_rw("numFrames", &AbstractSeq::getNumFrames, [](AbstractSeq& self, int n){ self.setNumFrames(n); })
        .def("getNumFrames", &AbstractSeq::getNumFrames)
        .def("setNumFrames", &AbstractSeq::setNumFrames, nb::arg("n"), nb::arg("fillNewElements") = false)
        .def_prop_rw("timeLength", &AbstractSeq::getTimeLength, [](AbstractSeq& self, double length){ self.setTimeLength(length); })
        .def("getTimeLength", &AbstractSeq::getTimeLength)
        .def("setTimeLength", &AbstractSeq::setTimeLength, nb::arg("length"), nb::arg("clearNewElements") = false)
        .def_prop_rw("seqContentName", &AbstractSeq::seqContentName, &AbstractSeq::setSeqContentName)
        .def("setSeqContentName", &AbstractSeq::setSeqContentName)
        .def("readSeq", [](AbstractSeq& self, const Mapping* archive){ return self.readSeq(archive); })
        .def("writeSeq", &AbstractSeq::writeSeq)
        .def_prop_ro("seqMessage", &AbstractSeq::seqMessage)
        .def_prop_ro("defaultFrameRate", &AbstractSeq::defaultFrameRate)
        ;

    nb::class_<AbstractMultiSeq, AbstractSeq>(m, "AbstractMultiSeq")
        .def("copySeqProperties", &AbstractMultiSeq::copySeqProperties)
        .def("setDimension",
             &AbstractMultiSeq::setDimension,
             nb::arg("numFrames"), nb::arg("numParts"), nb::arg("clearNewElements") = false)
        .def_prop_rw(
            "numParts",
            &AbstractMultiSeq::getNumParts,
            [](AbstractMultiSeq& self, int numParts){ self.setNumParts(numParts); })
        .def("setNumParts", &AbstractMultiSeq::setNumParts, nb::arg("n"), nb::arg("fillNewElements") = false)
        .def("getNumParts", &AbstractMultiSeq::getNumParts)
        .def("partIndex", &AbstractMultiSeq::partIndex)
        .def("partLabel", &AbstractMultiSeq::partLabel)
        ;

    typedef Deque2D<double, std::allocator<double>> Deque2DDouble;

    nb::class_<Deque2DDouble::Row>(m, "Deque2DDouble_Row")
        .def_prop_ro("size", &Deque2DDouble::Row::size)
        .def("at", &Deque2DDouble::Row::at, nb::rv_policy::reference_internal)
        .def("__getitem__", [](Deque2DDouble::Row& self, int i){ return self[i]; })
        .def("__setitem__", [](Deque2DDouble::Row& self, int i, double x){ self[i] = x; })
        .def("__len__", &Deque2DDouble::Row::size)
        ;

    nb::class_<Deque2DDouble::Column>(m, "Deque2DDouble_Column")
        .def_prop_ro("size", &Deque2DDouble::Column::size)
        .def("at", &Deque2DDouble::Column::at, nb::rv_policy::reference_internal)
        .def("__getitem__", [](Deque2DDouble::Column& self, int i){ return self[i]; })
        .def("__setitem__", [](Deque2DDouble::Column& self, int i, double x){ self[i] = x; })
        .def("__len__", &Deque2DDouble::Column::size)
        ;

    nb::class_<MultiValueSeq, AbstractMultiSeq>(m, "MultiValueSeq")
        .def(nb::init<>())
        .def_prop_ro("empty", &MultiValueSeq::empty)
        .def("resize", &MultiValueSeq::resize)
        .def("clear", &MultiValueSeq::clear)
        .def("at",
             (MultiValueSeq::Element& (MultiValueSeq::*)(size_t, size_t)) &MultiValueSeq::at,
             nb::rv_policy::reference_internal)
        .def("append", &MultiValueSeq::append)
        .def("pop_back", &MultiValueSeq::pop_back)
        .def("pop_front", &MultiValueSeq::pop_front, nb::arg("n") = 1)
        .def("copySeqProperties", &MultiValueSeq::copySeqProperties)
        .def("clampFrameIndex", [](MultiValueSeq& self, int index){ return self.clampFrameIndex(index); })
        .def("frame", (MultiValueSeq::Frame (MultiValueSeq::*)(int)) &MultiValueSeq::frame)
        .def("part", (MultiValueSeq::Part (MultiValueSeq::*)(int)) &MultiValueSeq::part)
        .def("loadPlainFormat",
             [](MultiValueSeq& self, const std::string& filename){
                 return self.loadPlainFormat(filename); })
        .def("saveAsPlainFormat",
             [](MultiValueSeq& self, const std::string& filename){
                 return self.saveAsPlainFormat(filename); })
        ;

    nb::class_<ReferencedObjectSeq, AbstractSeq>(m, "ReferencedObjectSeq")
        .def(nb::init<>())
        .def_prop_ro("clear", &ReferencedObjectSeq::clear)
        .def_prop_ro("empty", &ReferencedObjectSeq::empty)
        .def("__getitem__", [](ReferencedObjectSeq& self, int i){ return self[i]; })
        .def("__setitem__", [](ReferencedObjectSeq& self, int i, Referenced* obj){ self[i] = obj; })
        .def_prop_rw(
            "front",
            [](ReferencedObjectSeq& self){ return self.empty() ? nullptr : self.front().get(); },
            [](ReferencedObjectSeq& self, Referenced* obj){ if(!self.empty()) self.front() = obj; })
        .def_prop_rw(
            "back",
            [](ReferencedObjectSeq& self){ return self.empty() ? nullptr : self.back().get(); },
            [](ReferencedObjectSeq& self, Referenced* obj){ if(!self.empty()) self.back() = obj; })
        ;
}

}
