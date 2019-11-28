#include "AGXSimulatorItem.h"
#include <cnoid/Plugin>
#include <cnoid/MenuManager>
#include <cnoid/MessageView>
#include <cnoid/RootItem>
#include <cnoid/ItemList>
#include <agxSDK/Simulation.h>
#include <iostream>

using namespace std;

namespace cnoid {

class AGXDynamicsPlugin : public Plugin
{
public:
    agx::AutoInit agxInit;
    AGXDynamicsPlugin() : Plugin("AGXDynamics"){ require("Body"); }
    virtual ~AGXDynamicsPlugin(){}
    virtual bool initialize(){
        AGXSimulatorItem::initializeClass(this);
        Action* menuItem = menuManager().setPath("/File").addItem("Save to agx file");
        menuItem->sigTriggered().connect(bind(&AGXDynamicsPlugin::onSaveSimulationToAGXFileTriggered, this));
        //Action* menuItem2 = MenuManager().setPath("/File").addItem("Initialize agx simulation");
        //menuItem2->sigTriggered().connect(bind(&AGXDynamicsPlugin::onInitializeAGXSimulationTriggered, this));
        return true;
    }
    virtual bool finalize(){
        return true;
    }
private:
    void onSaveSimulationToAGXFileTriggered(){
        auto simItems = RootItem::instance()->selectedItems<SimulatorItem>();
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
    void onInitializeAGXSimulationTriggered(){
        auto simItems = RootItem::instance()->selectedItems<SimulatorItem>();
        for(size_t i=0; i < simItems.size(); ++i){
            SimulatorItem* simItem = simItems[i];
            AGXSimulatorItem* agxSimItem = dynamic_cast<AGXSimulatorItem*>(simItem);
            if(!agxSimItem) continue;
            //if(agxSimItem->initializeSimulation()){
            //    MessageView::instance()->putln("Save simulation to agx file.");
            //}else{
            //    MessageView::instance()->putln("Failed to save simulation to agx file.");
            //}
        }
    }
};

CNOID_IMPLEMENT_PLUGIN_ENTRY(AGXDynamicsPlugin)
}
