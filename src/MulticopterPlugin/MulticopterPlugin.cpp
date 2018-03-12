/**
   @author Japan Atomic Energy Agency
*/

#include "MulticopterPluginHeader.h"

using namespace std;
using namespace cnoid;
using namespace boost;
using namespace Multicopter;

class MulticopterPlugin : public Plugin
{
public:
    
    MulticopterPlugin() : Plugin("Multicopter"){
        require("Body");
    }

    virtual bool initialize(){
        
        bool ret;

        MulticopterSimulatorItem::initializeClass(this);
        
        ret = EventManager::instance()->initialize();
        if( ret == false){
            return ret;
        }

        ret = LinkManager::instance()->initialize();
        if( ret == false){
            return ret;
        }

        ret = SimulationManager::instance()->initialize(this);
        if( ret == false){
            return ret;
        }

        return true;
    }

    virtual bool finalize(){
        return Plugin::finalize();
    }

private:

};

CNOID_IMPLEMENT_PLUGIN_ENTRY(MulticopterPlugin);
