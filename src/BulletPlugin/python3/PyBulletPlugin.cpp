/*!
 * @author Hervé Audren
 */

#include "../BulletSimulatorItem.h"
#include <cnoid/Py3Util>

namespace py = pybind11;
using namespace cnoid;

PYBIND11_PLUGIN(BulletPlugin)
{
    py::module m("BulletPlugin", "BulletPlugin Python Module");
    /*!
     * @brief Provides following all item types.
     */
    py::class_ < BulletSimulatorItem, BulletSimulatorItemPtr, SimulatorItem >(m, "BulletSimulatorItem");

    return m.ptr();
}
