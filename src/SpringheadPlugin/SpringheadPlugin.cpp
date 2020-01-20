/*!
  @file
  @author Yuichi Tazaki
*/

#include "SpringheadSimulatorItem.h"
#include <cnoid/Plugin>
#include <Springhead.h>
#define PLUGIN_NAME "Springhead"

using namespace std;
using namespace cnoid;

namespace {
    
class SpringheadPlugin : public Plugin
{
public:
    SpringheadPlugin() : Plugin(PLUGIN_NAME)
        { 
            require("Body");
        }
        
    virtual ~SpringheadPlugin()
        {

        }

    virtual bool initialize()
        {
            SpringheadSimulatorItem::initializeClass(this);
            
            return true;
        }
        
    virtual bool finalize()
        {
            return true;
        }
};
}

CNOID_IMPLEMENT_PLUGIN_ENTRY(SpringheadPlugin);
