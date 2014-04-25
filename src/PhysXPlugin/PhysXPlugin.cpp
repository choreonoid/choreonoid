/*!
  @file
  @author Shizuko Hattori
*/

#include "PhysXSimulatorItem.h"
#include <cnoid/Plugin>

using namespace std;
using namespace cnoid;

namespace {
    
class PhysXPlugin : public Plugin
{
public:
    PhysXPlugin() : Plugin("PhysX")
        { 
            require("Body");
        }
        
    virtual ~PhysXPlugin()
        {

        }

    virtual bool initialize()
        {
            PhysXSimulatorItem::initialize(this);
            
            return true;
        }
        
    virtual bool finalize()
        {
            return true;
        }
};
}

CNOID_IMPLEMENT_PLUGIN_ENTRY(PhysXPlugin);
