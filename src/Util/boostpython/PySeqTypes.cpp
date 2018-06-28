/*!
 * @author Shin'ichiro Nakaoka
*/

#include "PyUtil.h"
#include "../MultiValueSeq.h"
#include "../Vector3Seq.h"
#include "../ValueTree.h"
#include "../YAMLWriter.h"

using namespace boost::python;
using namespace cnoid;

// for MSVC++2015 Update3
CNOID_PYTHON_DEFINE_GET_POINTER(AbstractSeq)
CNOID_PYTHON_DEFINE_GET_POINTER(AbstractMultiSeq)
CNOID_PYTHON_DEFINE_GET_POINTER(MultiValueSeq)

namespace {

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(AbstractSeq_setNumFrames, setNumFrames, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(AbstractSeq_setTimeLength, setTimeLength, 1, 2)

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(AbstractMultiSeq_setDimension, setDimension, 3, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(AbstractMultiSeq_setNumParts, setNumParts, 1, 2)

}

namespace cnoid {

void exportPySeqTypes()
{
    class_<AbstractSeq, boost::noncopyable>("AbstractSeq", no_init)
        .def("cloneSeq", &AbstractSeq::cloneSeq)
        .def("copySeqProperties", &AbstractSeq::copySeqProperties)
        .def("seqType", &AbstractSeq::seqType, return_value_policy<copy_const_reference>())
        .def("getSeqType", &AbstractSeq::seqType, return_value_policy<copy_const_reference>())
        .def("getFrameRate", &AbstractSeq::getFrameRate)
        .def("setFrameRate", &AbstractSeq::setFrameRate)
        .def("getTimeStep", &AbstractSeq::getTimeStep)
        .def("setTimeStep", &AbstractSeq::setTimeStep)
        .def("getTimeOfFrame", &AbstractSeq::getTimeOfFrame)
        .def("getOffsetTimeFrame", &AbstractSeq::getOffsetTimeFrame)
        .def("getOffsetTime", &AbstractSeq::getOffsetTime)
        .def("getNumFrames", &AbstractSeq::getNumFrames)
        .def("setNumFrames", &AbstractSeq::setNumFrames, AbstractSeq_setNumFrames())
        .def("setTimeLength", &AbstractSeq::setTimeLength, AbstractSeq_setTimeLength())
        .def("getTimeLength", &AbstractSeq::getTimeLength)
        .def("seqContentName", &AbstractSeq::seqContentName, return_value_policy<copy_const_reference>())
        .def("getSeqContentName", &AbstractSeq::seqContentName, return_value_policy<copy_const_reference>())
        .def("setSeqContentName", &AbstractSeq::setSeqContentName)
        .def("readSeq", &AbstractSeq::readSeq)
        .def("writeSeq", &AbstractSeq::writeSeq)
        .def("seqMessage", &AbstractSeq::seqMessage, return_value_policy<copy_const_reference>())
        .def("getSeqMessage", &AbstractSeq::seqMessage, return_value_policy<copy_const_reference>())
        .def("defaultFrameRate", &AbstractSeq::defaultFrameRate)
        .def("getDefaultFrameRate", &AbstractSeq::defaultFrameRate);

    register_ptr_to_python<AbstractSeqPtr>();    

    class_<AbstractMultiSeq, bases<AbstractSeq>, boost::noncopyable>("AbstractMultiSeq", no_init)
        .def("copySeqProperties", &AbstractMultiSeq::copySeqProperties)
        .def("setDimension", &AbstractMultiSeq::setDimension, AbstractMultiSeq_setDimension())
        .def("setNumParts", &AbstractMultiSeq::setNumParts, AbstractMultiSeq_setNumParts())
        .def("getNumParts", &AbstractMultiSeq::getNumParts)
        .def("partIndex", &AbstractMultiSeq::partIndex)
        .def("getPartIndex", &AbstractMultiSeq::partIndex)
        .def("partLabel", &AbstractMultiSeq::partLabel, return_value_policy<copy_const_reference>())
        .def("getPartLabel", &AbstractMultiSeq::partLabel, return_value_policy<copy_const_reference>());

    register_ptr_to_python<AbstractMultiSeqPtr>();
    implicitly_convertible<AbstractMultiSeqPtr, AbstractSeqPtr>();

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
    
    class_< MultiValueSeq, bases<AbstractMultiSeq> >("MultiValueSeq")

        .def("empty", &MultiValueSeq::empty)
        .def("isEmpty", &MultiValueSeq::empty)
        .def("resize", &MultiValueSeq::resize)
        .def("resizeColumn", &MultiValueSeq::resizeColumn)
        .def("rowSize", &MultiValueSeq::rowSize)
        .def("getRowSize", &MultiValueSeq::rowSize)
        .def("resizeRow", &MultiValueSeq::resizeRow)
        .def("colSize", &MultiValueSeq::colSize)
        .def("getColSize", &MultiValueSeq::colSize)
        .def("clear", &MultiValueSeq::clear)
        .def("at", MultiValueSeq_at_const, return_value_policy<copy_const_reference>())
        //.def("at", MultiValueSeq_at, return_value_policy<reference_existing_object>()) // This is impossible
        .def("row", MultiValueSeq_row)
        .def("getRow", MultiValueSeq_row)
        .def("row", MultiValueSeq_row_const)
        .def("getRow", MultiValueSeq_row_const)
        .def("column", MultiValueSeq_column)
        .def("getColumn", MultiValueSeq_column)
        .def("column", MultiValueSeq_column_const)
        .def("getColumn", MultiValueSeq_column_const)
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
        .def("getClampFrameIndex", &MultiValueSeq::clampFrameIndex)
        .def("frame", MultiValueSeq_frame)
        .def("getFrame", MultiValueSeq_frame)
        .def("frame", MultiValueSeq_frame_const)
        .def("getFrame", MultiValueSeq_frame_const)
        .def("part", MultiValueSeq_part)
        .def("getPart", MultiValueSeq_part)
        .def("part", MultiValueSeq_part_const)
        .def("getPart", MultiValueSeq_part_const)

        .def("loadPlainFormat", &MultiValueSeq::loadPlainFormat)
        .def("saveAsPlainFormat", &MultiValueSeq::saveAsPlainFormat);

    register_ptr_to_python<MultiValueSeqPtr>();
    implicitly_convertible<MultiValueSeqPtr, AbstractMultiSeqPtr>();
}

}
