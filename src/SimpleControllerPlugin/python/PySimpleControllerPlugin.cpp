/*!
  @author Shin'ichiro Nakaoka
*/

#include "../SimpleControllerItem.h"
#include <cnoid/PyUtil>

using namespace boost::python;
using namespace cnoid;

BOOST_PYTHON_MODULE(SimpleControllerPlugin)
{
    class_< SimpleControllerItem, SimpleControllerItemPtr, bases<Item> >("SimpleControllerItem")
        .def("setControllerDllName", &SimpleControllerItem::setControllerDllName);
}
