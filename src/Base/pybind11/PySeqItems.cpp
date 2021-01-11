#include "PyItemList.h"
#include "../MultiValueSeqItem.h"
#include "../MultiSE3SeqItem.h"
#include "../MultiSE3MatrixSeqItem.h"
#include "../Vector3SeqItem.h"
#include "../ReferencedObjectSeqItem.h"
#include <cnoid/PyUtil>

using namespace std;
using namespace cnoid;
namespace py = pybind11;

namespace cnoid {

void exportPySeqItems(py::module m)
{
    py::class_<AbstractSeqItem, AbstractSeqItemPtr, Item> abstractSeqItemClass(m, "AbstractSeqItem");
    abstractSeqItemClass
        .def_property_readonly("abstractSeq", &AbstractSeqItem::abstractSeq)

        // deprecated
        .def("getAbstractSeq", &AbstractSeqItem::abstractSeq)
        ;

    PyItemList<AbstractSeqItem>(m, "AbstractSeqItemList", abstractSeqItemClass);

    py::class_<Vector3SeqItem, Vector3SeqItemPtr, AbstractSeqItem>(m, "Vector3SeqItem")
        .def(py::init<>())
        .def_property_readonly("seq", &Vector3SeqItem::seq)

        // deprecated
        .def("getSeq", &Vector3SeqItem::seq)
        ;

    PyItemList<Vector3SeqItem>(m, "Vector3SeqItemList");

    // multi seq items
    py::class_<AbstractMultiSeqItem, AbstractMultiSeqItemPtr, AbstractSeqItem>
        abstractMultiSeqItemClass(m, "AbstractMultiSeqItem");
    abstractMultiSeqItemClass
        .def_property_readonly("abstractMultiSeq", &AbstractMultiSeqItem::abstractMultiSeq)

        // deprecated
        .def("getAbstractMultiSeq", &AbstractMultiSeqItem::abstractMultiSeq)
        ;

    //PyItemList<AbstractMultiSeqItem>("AbstractMultiSeqItemList", abstractMultiSeqItemClass);
    
    py::class_<MultiValueSeqItem, MultiValueSeqItemPtr, AbstractMultiSeqItem>(m, "MultiValueSeqItem")
        .def(py::init<>())
        .def_property_readonly("seq", &MultiValueSeqItem::seq)
        
        // deprecated
        .def("getSeq", &MultiValueSeqItem::seq)
        ;
    
    PyItemList<MultiValueSeqItem>(m, "MultiValueSeqItemList");

    py::class_<MultiSE3MatrixSeqItem, MultiSE3MatrixSeqItemPtr, AbstractMultiSeqItem>(m, "MultiSE3MatrixSeqItem")
        .def(py::init<>())
        .def_property_readonly("seq", &MultiSE3MatrixSeqItem::seq)

        // deprecated
        .def("getSeq", &MultiSE3MatrixSeqItem::seq)
        ;

    PyItemList<MultiSE3MatrixSeqItem>(m, "MultiSE3MatrixSeqItemList");

    py::class_<MultiSE3SeqItem, MultiSE3SeqItemPtr, AbstractMultiSeqItem> (m, "MultiSE3SeqItem")
        .def(py::init<>())
        .def_property_readonly("seq", &MultiSE3SeqItem::seq)

        // deprecated
        .def("getSeq", &MultiSE3SeqItem::seq)
        ;
    
    PyItemList<MultiSE3SeqItem>(m, "MultiSE3SeqItemList");

    py::class_<ReferencedObjectSeqItem, ReferencedObjectSeqItemPtr, AbstractSeqItem>(m, "ReferencedObjectSeqItem")
        .def(py::init<>())
        .def_property_readonly("seq", &ReferencedObjectSeqItem::seq)
        .def("resetSeq", &ReferencedObjectSeqItem::resetSeq)
        ;

    PyItemList<ReferencedObjectSeqItem>(m, "ReferencedObjectSeqItemList");
}

}

    

