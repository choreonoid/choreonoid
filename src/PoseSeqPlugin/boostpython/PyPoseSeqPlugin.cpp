/*!
  @author Shin'ichiro Nakaoka
*/

#include "../PoseSeqItem.h"
#include <cnoid/PyUtil>

using namespace boost::python;
using namespace cnoid;

BOOST_PYTHON_MODULE(PoseSeqPlugin)
{
    class_< PoseSeqItem, PoseSeqItemPtr, bases<Item> >("PoseSeqItem");
}
