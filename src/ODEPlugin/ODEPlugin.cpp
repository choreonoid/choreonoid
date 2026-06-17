/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "ODESimulatorItem.h"
#include <cnoid/Plugin>
#include <ode/ode.h>

using namespace std;
using namespace cnoid;

namespace {
    
class ODEPlugin : public Plugin
{
public:
    ODEPlugin() : Plugin("ODE")
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
        /**
           \todo It is necessary to execute the following code
           after deleting all ODESimulatorItem.
        */
        // dCloseODE();   
        return true;
    }
};

}

CNOID_IMPLEMENT_PLUGIN_ENTRY(ODEPlugin);
