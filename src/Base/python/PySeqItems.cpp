#include "PyItemList.h"
#include "../MultiValueSeqItem.h"
#include "../MultiSE3SeqItem.h"
#include "../MultiSE3MatrixSeqItem.h"
#include "../Vector3SeqItem.h"
#include "../ReferencedObjectSeqItem.h"
#include <cnoid/PyUtil>
#include <nanobind/stl/shared_ptr.h>

using namespace std;
using namespace cnoid;
namespace nb = nanobind;

namespace cnoid {

void exportPySeqItems(nb::module_ m)
{
    nb::class_<AbstractSeqItem, Item> abstractSeqItemClass(m, "AbstractSeqItem");
    abstractSeqItemClass
        .def_prop_ro("abstractSeq", &AbstractSeqItem::abstractSeq)
        ;

    PyItemList<AbstractSeqItem>(m, "AbstractSeqItemList", abstractSeqItemClass);

    nb::class_<Vector3SeqItem, AbstractSeqItem>(m, "Vector3SeqItem")
        .def(nb::init<>())
        .def_prop_ro("seq", &Vector3SeqItem::seq)
        ;

    PyItemList<Vector3SeqItem>(m, "Vector3SeqItemList");

    // multi seq items
    nb::class_<AbstractMultiSeqItem, AbstractSeqItem>
        abstractMultiSeqItemClass(m, "AbstractMultiSeqItem");
    abstractMultiSeqItemClass
        .def_prop_ro("abstractMultiSeq", &AbstractMultiSeqItem::abstractMultiSeq)
        ;

    nb::class_<MultiValueSeqItem, AbstractMultiSeqItem>(m, "MultiValueSeqItem")
        .def(nb::init<>())
        .def_prop_ro("seq", &MultiValueSeqItem::seq)
        ;

    PyItemList<MultiValueSeqItem>(m, "MultiValueSeqItemList");

    nb::class_<MultiSE3MatrixSeqItem, AbstractMultiSeqItem>(m, "MultiSE3MatrixSeqItem")
        .def(nb::init<>())
        .def_prop_ro("seq", &MultiSE3MatrixSeqItem::seq)
        ;

    PyItemList<MultiSE3MatrixSeqItem>(m, "MultiSE3MatrixSeqItemList");

    nb::class_<MultiSE3SeqItem, AbstractMultiSeqItem> (m, "MultiSE3SeqItem")
        .def(nb::init<>())
        .def_prop_ro("seq", &MultiSE3SeqItem::seq)
        ;

    PyItemList<MultiSE3SeqItem>(m, "MultiSE3SeqItemList");

    nb::class_<ReferencedObjectSeqItem, AbstractSeqItem>(m, "ReferencedObjectSeqItem")
        .def(nb::init<>())
        .def_prop_ro("seq", [](ReferencedObjectSeqItem& self){ return self.seq(); })
        .def("resetSeq", &ReferencedObjectSeqItem::resetSeq)
        ;

    PyItemList<ReferencedObjectSeqItem>(m, "ReferencedObjectSeqItemList");
}

}
