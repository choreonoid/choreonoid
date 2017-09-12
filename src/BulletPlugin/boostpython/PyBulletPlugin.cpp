/*!
 * @author Hervé Audren
 */

#include "../BulletSimulatorItem.h"
#include <cnoid/PyUtil>

using namespace boost::python;
using namespace cnoid;

BOOST_PYTHON_MODULE(BulletPlugin)
{
        /*!
         * @brief Provides following all item types.
         */
        class_ < BulletSimulatorItem,
                 BulletSimulatorItemPtr,
                 bases<SimulatorItem> >("BulletSimulatorItem");
}
