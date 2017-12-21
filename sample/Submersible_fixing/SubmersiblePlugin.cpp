/**
   @author Shin'ichiro Nakaoka
*/

//#include "SubmersibleSimulatorItem.h"
#include <cnoid/Plugin>
#include <cnoid/MenuManager>
//#include <cnoid/MessageView>
#include "ThrusterDevice.h"
#include "SubmersibleSimulatorItem2.h"

using namespace cnoid;

namespace cnoid {

class SubmersiblePlugin : public Plugin
{
public:
    
    SubmersiblePlugin() : Plugin("Submersible") {
        require("Body");
    }
    
    virtual bool initialize() {
//    	SubmersibleSimulatorItem::initializeClass(this);
//    	ThrusterDevice::initializeClass(this);
    	SubmersibleSimulatorItem2::initializeClass(this);
//    	Action* menuItem = menuManager().setPath("/File/New ...").addItem("Submersible");
    	//    	menuItem->sigTriggered().connect(bind(&SubmersiblePlugin::onSubmersibleTriggered, this));

        return true;
    }

//private:

//    void onSubmersibleTriggered()
//    {
//    	ItemList<SimulatorItem> simItems = simulatorItem
//    }
};

CNOID_IMPLEMENT_PLUGIN_ENTRY(SubmersiblePlugin);

}
