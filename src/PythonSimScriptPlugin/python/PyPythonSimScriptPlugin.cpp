/*!
  @author Shin'ichiro Nakaoka
*/

#include "../PythonSimScriptItem.h"
#include <cnoid/PyUtil>

using namespace boost::python;
using namespace cnoid;

BOOST_PYTHON_MODULE(PythonSimScriptPlugin)
{
    class_< PythonSimScriptItem, PythonSimScriptItemPtr, bases<Item> >("PythonSimScriptItem");
};

