/**
   @author Shin'ichiro Nakaoka
*/

#include "SimulationBar.h"
#include "SimulatorItem.h"
#include "WorldItem.h"
#include <cnoid/TimeBar>
#include <cnoid/RootItem>
#include <cnoid/ItemTreeView>
#include <cnoid/MessageView>
#include <boost/bind.hpp>
#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;


SimulationBar* SimulationBar::initialize(ExtensionManager* ext)
{
    return instance();
}


SimulationBar* SimulationBar::instance()
{
    static SimulationBar* instance = new SimulationBar();
    return instance;
}


SimulationBar::SimulationBar()
    : ToolBar(N_("SimulationBar"))
{
    addButton(QIcon(":/Body/icons/store-world-initial.png"),
              _("Store body positions to the initial world state"))->
        sigClicked().connect(bind(&SimulationBar::onStoreInitialClicked, this));
    
    addButton(QIcon(":/Body/icons/restore-world-initial.png"),
              _("Restore body positions from the initial world state"))->
        sigClicked().connect(bind(&SimulationBar::onRestoreInitialClicked, this));

    typedef boost::function<void(SimulatorItem* simulator)> Callback;

    addButton(QIcon(":/Body/icons/start-simulation.png"), _("Start simulation from the beginning"))->
        sigClicked().connect(bind(&SimulationBar::forEachSimulator, this,
                                  Callback(bind(&SimulationBar::startSimulation, this, _1, true))));

    addButton(QIcon(":/Body/icons/restart-simulation.png"),
              _("Start simulation from the current state"))->
        sigClicked().connect(bind(&SimulationBar::forEachSimulator, this,
                                  Callback(bind(&SimulationBar::startSimulation, this, _1, false))));
    
    addButton(QIcon(":/Body/icons/stop-simulation.png"), _("Stop simulation"))->
        sigClicked().connect(bind(&SimulationBar::onStopSimulationClicked, this));
}


SimulationBar::~SimulationBar()
{

}


static void forEachTargetBodyItem(function<void(BodyItem*)> callback)
{
    ItemTreeView* itemTreeView = ItemTreeView::instance();

    ItemList<BodyItem> bodyItems;
    bodyItems.extractChildItems(RootItem::instance());
    
    for(int i=0; i < bodyItems.size(); ++i){
        BodyItem* bodyItem = bodyItems.get(i);
        bool isTarget = itemTreeView->isItemSelected(bodyItem);
        if(!isTarget){
            WorldItem* worldItem = bodyItem->findOwnerItem<WorldItem>();
            if(worldItem && itemTreeView->isItemSelected(worldItem)){
                isTarget = true;
            }
        }
        if(isTarget){
            callback(bodyItem);
        }
    }
}


static void storeInitialBodyState(BodyItem* bodyItem)
{
    bodyItem->storeInitialState();
    MessageView::instance()->putln(
        format(_("Current state of %1% has been set to the initial state.")) % bodyItem->name());
}


void SimulationBar::onStoreInitialClicked()
{
    forEachTargetBodyItem(storeInitialBodyState);
}


void SimulationBar::onRestoreInitialClicked()
{
    forEachTargetBodyItem(function<void(BodyItem*)>(bind(&BodyItem::restoreInitialState, _1)));
}


void SimulationBar::forEachSimulator(function<void(SimulatorItem* simulator)> callback)
{
    MessageView* mv = MessageView::instance();
    /*
      ItemList<SimulatorItem> simulators =
      ItemTreeView::mainInstance()->selectedItems<SimulatorItem>();
    */
    ItemList<SimulatorItem> simulators =
        ItemTreeView::mainInstance()->selectedItems<SimulatorItem>();

    if(simulators.empty()){
        simulators.extractChildItems(RootItem::instance());
        if(simulators.empty()){
            mv->notify(_("There is no simulator item."));
        } else  if(simulators.size() > 1){
            simulators.clear();
            mv->notify(_("Please select a simulator item to simulate."));
        } else {
            ItemTreeView::mainInstance()->selectItem(simulators.front());
        }
    }

    typedef map<WorldItem*, SimulatorItem*> WorldToSimulatorMap;
    WorldToSimulatorMap worldToSimulator;

    for(int i=0; i < simulators.size(); ++i){
        SimulatorItem* simulator = simulators.get(i);
        WorldItem* world = simulator->findOwnerItem<WorldItem>();
        if(world){
            WorldToSimulatorMap::iterator p = worldToSimulator.find(world);
            if(p == worldToSimulator.end()){
                worldToSimulator[world] = simulator;
            } else {
                p->second = 0; // skip if multiple simulators are selected
            }
        }
    }

    for(int i=0; i < simulators.size(); ++i){
        SimulatorItem* simulator = simulators.get(i);
        WorldItem* world = simulator->findOwnerItem<WorldItem>();
        if(!world){
            mv->notify(format(_("%1% cannot be processed because it is not related with a world."))
                       % simulator->name());
        } else {
            WorldToSimulatorMap::iterator p = worldToSimulator.find(world);
            if(p != worldToSimulator.end()){
                if(!p->second){
                    mv->notify(format(_("%1% cannot be processed because another simulator"
                                        "in the same world is also selected."))
                               % simulator->name());
                } else {
                    callback(simulator);
                }
            }
        }
    }
}


void SimulationBar::startSimulation(SimulatorItem* simulator, bool doReset)
{
    if(simulator->isRunning()){
        //simulator->selectMotionItems();
        TimeBar::instance()->startPlaybackFromFillLevel();

    } else {
        sigSimulationAboutToStart_(simulator);
        simulator->startSimulation(doReset);
    }
}


void SimulationBar::onStopSimulationClicked()
{
    forEachSimulator(bind(&SimulationBar::stopSimulation, this, _1));

    TimeBar* timeBar = TimeBar::instance();
    if(timeBar->isDoingPlayback()){
        timeBar->stopPlayback();
    }
}


void SimulationBar::stopSimulation(SimulatorItem* simulator)
{
    simulator->stopSimulation();
}
