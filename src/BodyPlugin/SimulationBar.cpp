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
#include <cnoid/OptionManager>
#include <cnoid/LazyCaller>
#include <fmt/format.h>
#include <functional>
#include "gettext.h"

using namespace std;
using namespace std::placeholders;
using namespace cnoid;
using fmt::format;

static SimulationBar* instance_ = 0;
    

static void onSigOptionsParsed(boost::program_options::variables_map& v)
{
    if(v.count("start-simulation")){
        callLater(std::bind(static_cast<void(SimulationBar::*)(bool)>(&SimulationBar::startSimulation), instance_, true));
    }
}


void SimulationBar::initialize(ExtensionManager* ext)
{
    if(!instance_){
        instance_ = new SimulationBar();
        ext->addToolBar(instance_);
        
        ext->optionManager()
            .addOption("start-simulation", "start simulation automatically")
            .sigOptionsParsed().connect(onSigOptionsParsed);
    }
}


SimulationBar* SimulationBar::instance()
{
    return instance_;
}


SimulationBar::SimulationBar()
    : ToolBar(N_("SimulationBar"))
{
    setVisibleByDefault(true);    
    
    addButton(QIcon(":/Body/icons/store-world-initial.png"),
              _("Store body positions to the initial world state"))->
        sigClicked().connect(std::bind(&SimulationBar::onStoreInitialClicked, this));
    
    addButton(QIcon(":/Body/icons/restore-world-initial.png"),
              _("Restore body positions from the initial world state"))->
        sigClicked().connect(std::bind(&SimulationBar::onRestoreInitialClicked, this));

    addButton(QIcon(":/Body/icons/start-simulation.png"), _("Start simulation from the beginning"))->
        sigClicked().connect(
            std::bind(static_cast<void(SimulationBar::*)(bool)>(&SimulationBar::startSimulation), this, true));

    addButton(QIcon(":/Body/icons/restart-simulation.png"),
              _("Start simulation from the current state"))->
        sigClicked().connect(
            std::bind(static_cast<void(SimulationBar::*)(bool)>(&SimulationBar::startSimulation), this, false));
    
    pauseToggle = addToggleButton(QIcon(":/Body/icons/pause-simulation.png"), _("Pause simulation"));
    pauseToggle->sigClicked().connect(std::bind(&SimulationBar::onPauseSimulationClicked, this));
    pauseToggle->setChecked(false);

    addButton(QIcon(":/Body/icons/stop-simulation.png"), _("Stop simulation"))->
        sigClicked().connect(std::bind(&SimulationBar::onStopSimulationClicked, this));

}


SimulationBar::~SimulationBar()
{

}


static void forEachTargetBodyItem(std::function<void(BodyItem*)> callback)
{
    ItemTreeView* itemTreeView = ItemTreeView::instance();

    ItemList<BodyItem> bodyItems;
    bodyItems.extractChildItems(RootItem::instance());
    
    for(size_t i=0; i < bodyItems.size(); ++i){
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
        format(_("Current state of {} has been set to the initial state."), bodyItem->name()));
}


void SimulationBar::onStoreInitialClicked()
{
    forEachTargetBodyItem(storeInitialBodyState);
}


void SimulationBar::onRestoreInitialClicked()
{
    forEachTargetBodyItem(std::function<void(BodyItem*)>(std::bind(&BodyItem::restoreInitialState, _1, true)));
}


void SimulationBar::forEachSimulator(std::function<void(SimulatorItem* simulator)> callback, bool doSelect)
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
            if(doSelect){
                ItemTreeView::instance()->selectItem(simulators.front());
            }
        }
    }

    typedef map<WorldItem*, SimulatorItem*> WorldToSimulatorMap;
    WorldToSimulatorMap worldToSimulator;

    for(size_t i=0; i < simulators.size(); ++i){
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

    for(size_t i=0; i < simulators.size(); ++i){
        SimulatorItem* simulator = simulators.get(i);
        WorldItem* world = simulator->findOwnerItem<WorldItem>();
        if(!world){
            mv->notify(format(_("{} cannot be processed because it is not related with a world."),
                              simulator->name()));
        } else {
            WorldToSimulatorMap::iterator p = worldToSimulator.find(world);
            if(p != worldToSimulator.end()){
                if(!p->second){
                    mv->notify(format(_("{} cannot be processed because another simulator"
                                        "in the same world is also selected."),
                                      simulator->name()));
                } else {
                    callback(simulator);
                }
            }
        }
    }
}


void SimulationBar::startSimulation(bool doRest)
{
    forEachSimulator(
        std::bind(
            static_cast<void(SimulationBar::*)(SimulatorItem*,bool)>(&SimulationBar::startSimulation),
            this, _1, doRest), true);
}


void SimulationBar::startSimulation(SimulatorItem* simulator, bool doReset)
{
    if(simulator->isRunning()){
    	if(pauseToggle->isChecked() && !doReset){
            simulator->restartSimulation();
            pauseToggle->setChecked(false);
    	}
        //simulator->selectMotionItems();
        TimeBar::instance()->startPlaybackFromFillLevel();
        
    } else {
        sigSimulationAboutToStart_(simulator);
        simulator->startSimulation(doReset);
        pauseToggle->setChecked(false);
    }
}


void SimulationBar::onStopSimulationClicked()
{
    forEachSimulator(std::bind(&SimulationBar::stopSimulation, this, _1));

    TimeBar* timeBar = TimeBar::instance();
    if(timeBar->isDoingPlayback()){
        timeBar->stopPlayback();
    }
    pauseToggle->setChecked(false);
}


void SimulationBar::stopSimulation(SimulatorItem* simulator)
{
    simulator->stopSimulation();
}


void SimulationBar::onPauseSimulationClicked()
{
    forEachSimulator(std::bind(&SimulationBar::pauseSimulation, this, _1));
}


void SimulationBar::pauseSimulation(SimulatorItem* simulator)
{
    if(pauseToggle->isChecked()){
        if(simulator->isRunning())
            simulator->pauseSimulation();
        TimeBar* timeBar = TimeBar::instance();
        if(timeBar->isDoingPlayback()){
            timeBar->stopPlayback();
        }
    } else {
        if(simulator->isRunning())
            simulator->restartSimulation();
        TimeBar::instance()->startPlaybackFromFillLevel();
    }
}
