/*!
 * @author Hervé Audren
 */

#include "../BulletSimulatorItem.h"
#include <cnoid/PyBase>

using namespace cnoid;
namespace py = pybind11;

PYBIND11_MODULE(BulletPlugin, m)
{
    m.doc() = "Choreonoid BulletPlugin module";

    /*!
     * @brief Provides following all item types.
     */
    py::class_<BulletSimulatorItem, BulletSimulatorItemPtr, SimulatorItem>(m, "BulletSimulatorItem");
}
