/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_PLUGIN_SIMULATION_BAR_H
#define CNOID_BODY_PLUGIN_SIMULATION_BAR_H

#include <cnoid/ToolBar>
#include <cnoid/Signal>
#include <functional>
#include "exportdecl.h"

namespace cnoid {

class SimulatorItem;

class CNOID_EXPORT SimulationBar : public ToolBar
{
public:
    static void initialize(ExtensionManager* ext);
    static SimulationBar* instance();

    SignalProxy<void(SimulatorItem*)> sigSimulationAboutToStart() {
        return sigSimulationAboutToStart_;
    }
            
    void startSimulation(bool doRest = true);

    virtual ~SimulationBar();

private:
    SimulationBar();

    void onStoreInitialClicked();
    void onRestoreInitialClicked();
    void forEachSimulator(std::function<void(SimulatorItem* simulator)> callback, bool doSelect = false);
    void startSimulation(SimulatorItem* simulator, bool doReset);
    void onStopSimulationClicked();
    void onPauseSimulationClicked();
    void pauseSimulation(SimulatorItem* simulator);
    ToolButton* pauseToggle;

    Signal<void(SimulatorItem*)> sigSimulationAboutToStart_;
};

}

#endif
