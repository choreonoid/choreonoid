/*
 * @author Hisashi Ikari
 */

#include <boost/python.hpp>

// This header is an "EXPLICIT" item of dependence.
// And this items are located in the "cnoid/include" or same directory.
// Its meaning is exposed externally. For reason, we can will be selected here.

#include <cnoid/Referenced>
#include <cnoid/Item>
#include <cnoid/RootItem>
#include <cnoid/WorldItem>

#include "../../Base/python/PyBase.h"
#include "../PythonSimScriptItem.h"

using namespace boost::python;
using namespace cnoid;

namespace cnoid
{
    BOOST_PYTHON_MODULE(PythonSimScriptPlugin)
    {
        /*!
         * @brief Define the interface for PoseSeqItem.
         */
        class_ < PythonSimScriptItem, ref_ptr<PythonSimScriptItem>, bases<Item>, 
            boost::noncopyable >("PythonSimScriptItem", init<>())
            .def("__init__", boost::python::make_constructor(&createInstance<PythonSimScriptItem>));

    }

}; // end of namespace
