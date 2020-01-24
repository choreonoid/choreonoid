/**
   @author Shin'ichiro Nakaoka
*/

#include "SimulationBar.h"
#include "SimulatorItem.h"
#include "WorldItem.h"
#include <cnoid/TimeBar>
#include <cnoid/RootItem>
#include <cnoid/MessageView>
#include <cnoid/OptionManager>
#include <cnoid/LazyCaller>
#include <fmt/format.h>
#include <functional>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

static SimulationBar* instance_ = 0;
    

static void onSigOptionsParsed(boost::program_options::variables_map& v)
{
    if(v.count("start-simulation")){
        callLater([](){ instance_->startSimulation(true); });
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
        sigClicked().connect([&](){ onStoreInitialClicked(); });
    
    addButton(QIcon(":/Body/icons/restore-world-initial.png"),
              _("Restore body positions from the initial world state"))->
        sigClicked().connect([&](){ onRestoreInitialClicked(); });

    addButton(QIcon(":/Body/icons/start-simulation.png"), _("Start simulation from the beginning"))->
        sigClicked().connect([&](){ startSimulation(true); });

    addButton(QIcon(":/Body/icons/restart-simulation.png"),
              _("Start simulation from the current state"))->
        sigClicked().connect([&](){ startSimulation(false); });
    
    pauseToggle = addToggleButton(QIcon(":/Body/icons/pause-simulation.png"), _("Pause simulation"));
    pauseToggle->sigClicked().connect([&](){ onPauseSimulationClicked(); });
    pauseToggle->setChecked(false);

    addButton(QIcon(":/Body/icons/stop-simulation.png"), _("Stop simulation"))->
        sigClicked().connect([&](){ onStopSimulationClicked(); });

}


SimulationBar::~SimulationBar()
{

}


static void forEachTargetBodyItem(std::function<void(BodyItem*)> callback)
{
    for(auto& bodyItem : RootItem::instance()->descendantItems<BodyItem>()){
        bool isTarget = bodyItem->isSelected();
        if(!isTarget){
            WorldItem* worldItem = bodyItem->findOwnerItem<WorldItem>();
            if(worldItem && worldItem->isSelected()){
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
    forEachTargetBodyItem(
        [](BodyItem* bodyItem){ bodyItem->restoreInitialState(true); });
}


void SimulationBar::forEachSimulator(std::function<void(SimulatorItem* simulator)> callback, bool doSelect)
{
    auto mv = MessageView::instance();

    auto simulators =  RootItem::instance()->selectedItems<SimulatorItem>();
    if(simulators.empty()){
        if(auto simulator = RootItem::instance()->findItem<SimulatorItem>()){
            simulator->setSelected(doSelect);
            simulators.push_back(simulator);
        } else {
            mv->notify(_("There is no simulator item."));
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
                p->second = nullptr; // skip if multiple simulators are selected
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


void SimulationBar::startSimulation(bool doReset)
{
    forEachSimulator(
        [=](SimulatorItem* simulator){ startSimulation(simulator, doReset); },
        true);
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
    forEachSimulator(
        [&](SimulatorItem* simulator){ stopSimulation(simulator); });

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
    forEachSimulator(
        [&](SimulatorItem* simulator){ pauseSimulation(simulator); });
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
