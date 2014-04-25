/*!
  @file
  @author Shizuko Hattori
*/

#include "BulletSimulatorItem.h"
#include <cnoid/Plugin>

using namespace std;
using namespace cnoid;

namespace {
    
class BulletPlugin : public Plugin
{
public:
    BulletPlugin() : Plugin("Bullet")
        { 
            require("Body");
        }
        
    virtual ~BulletPlugin()
        {

        }

    virtual bool initialize()
        {
            BulletSimulatorItem::initialize(this);
            
            return true;
        }
        
    virtual bool finalize()
        {
            return true;
        }
};
}

CNOID_IMPLEMENT_PLUGIN_ENTRY(BulletPlugin);
