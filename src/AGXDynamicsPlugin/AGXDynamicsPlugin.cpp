#include "AGXSimulatorItem.h"
#include <agxSDK/Simulation.h>
#include <cnoid/Plugin>
#include <cnoid/MenuManager>
#include <cnoid/MessageView>
#include <cnoid/ItemTreeView>
#include <iostream>

using namespace std;
//using namespace cnoid;

namespace cnoid {
agx::AutoInit agxInit;
class AGXDynamicsPlugin : public Plugin{
public:

	AGXDynamicsPlugin() : Plugin("AGXDynamics"){ 
		require("Body");
	}
	virtual ~AGXDynamicsPlugin(){}
	virtual bool initialize(){
		AGXSimulatorItem::initializeClass(this);
		Action* menuItem = menuManager().setPath("/File").addItem("Save to agx file");
		menuItem->sigTriggered().connect(bind(&AGXDynamicsPlugin::onSaveSimulationToAGXFileTriggered, this));
		return true;
	}
	virtual bool finalize(){
		return true;
	}
private:
	void onSaveSimulationToAGXFileTriggered(){
		ItemList<SimulatorItem> simItems = ItemTreeView::mainInstance()->selectedItems<SimulatorItem>();
		for(size_t i=0; i < simItems.size(); ++i){
			SimulatorItem* simItem = simItems[i];
			AGXSimulatorItem* agxSimItem = dynamic_cast<AGXSimulatorItem*>(simItem);
			if(!agxSimItem) continue;
			if(agxSimItem->saveSimulationToAGXFile()){
				MessageView::instance()->putln("Save simulation to agx file.");
			}else{
				MessageView::instance()->putln("Failed to save simulation to agx file.");
			}
		}
	}
};

CNOID_IMPLEMENT_PLUGIN_ENTRY(AGXDynamicsPlugin);
}
