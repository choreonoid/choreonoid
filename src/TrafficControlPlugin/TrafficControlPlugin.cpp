/**
   @author Japan Atomic Energy Agency
*/

#include "TrafficControlPluginHeader.h"
#include "DynamicTrafficControlPluginHeader.h"

using namespace std;
using namespace cnoid;
using namespace boost;

class TrafficControlPlugin : public Plugin
{
public:
    
    TrafficControlPlugin() : Plugin("TrafficControl"){

    }

    virtual bool initialize(){
        bool ret = TrafficControlShare::instance()->initialize();

        if(ret==false) {
            return false;
        }

        TrafficControlSimulatorItem::initializeClass(this);
        DynamicTrafficControlSimulatorItem::initializeClass(this);

        return true;
    }

    virtual bool finalize(){
        return Plugin::finalize();
    }

private:

};

CNOID_IMPLEMENT_PLUGIN_ENTRY(TrafficControlPlugin)
