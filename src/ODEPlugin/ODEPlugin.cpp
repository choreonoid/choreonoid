/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "ODESimulatorItem.h"
#include <cnoid/Plugin>
#ifdef GAZEBO_ODE
#include <gazebo/ode/ode.h>
#define ODESimulatorItem GazeboODESimulatorItem
#define PLUGIN_NAME "GAZEBO_ODE"
#else
#include <ode/ode.h>
#define PLUGIN_NAME "ODE"
#endif

using namespace std;
using namespace cnoid;

namespace {
    
class ODEPlugin : public Plugin
{
public:
    ODEPlugin() : Plugin(PLUGIN_NAME)
        { 
            require("Body");
        }
        
    virtual ~ODEPlugin()
        {

        }

    virtual bool initialize()
        {
            dInitODE2(0);
            dAllocateODEDataForThread(dAllocateMaskAll);

            ODESimulatorItem::initializeClass(this);
            
            return true;
        }
        
    virtual bool finalize()
        {
            // dCloseODE();   TODO ODESimulatorItemが全部deleteされた後に実行しないといけない。
            return true;
        }
};
}

CNOID_IMPLEMENT_PLUGIN_ENTRY(ODEPlugin);
