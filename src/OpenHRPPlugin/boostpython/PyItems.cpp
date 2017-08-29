/*!
  @author Shin'ichiro Nakaoka
*/

#include "../OpenHRPInterpreterServiceItem.h"
#include <cnoid/PyBase>

using namespace boost::python;
using namespace cnoid;

BOOST_PYTHON_MODULE(OpenHRP31Plugin)
{
    class_< OpenHRPInterpreterServiceItem, OpenHRPInterpreterServiceItemPtr, bases<Item> >
        ("OpenHRPInterpreterServiceItem")
        .def("setRTCInstanceName", &OpenHRPInterpreterServiceItem::setRTCInstanceName);

    implicitly_convertible<OpenHRPInterpreterServiceItemPtr, ItemPtr>();
    PyItemList<OpenHRPInterpreterServiceItem>("OpenHRPInterpreterServiceItemList");
};
