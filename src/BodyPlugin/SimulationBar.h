/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODYPLUGIN_SIMULATION_BAR_H_INCLUDED
#define CNOID_BODYPLUGIN_SIMULATION_BAR_H_INCLUDED

#include <cnoid/ToolBar>
#include <cnoid/SignalProxy>
#include <boost/function.hpp>
#include "exportdecl.h"

namespace cnoid {

class SimulatorItem;

class CNOID_EXPORT SimulationBar : public ToolBar
{
public:
    static SimulationBar* initialize(ExtensionManager* ext);
    static SimulationBar* instance();

    SignalProxy< boost::signal<void(SimulatorItem*)> > sigSimulationAboutToStart() {
        return sigSimulationAboutToStart_;
    }
            
    void startSimulation(SimulatorItem* simulator, bool doRest);
    void stopSimulation(SimulatorItem* simulator);

    virtual ~SimulationBar();

private:
    SimulationBar();

    void onStoreInitialClicked();
    void onRestoreInitialClicked();
    void forEachSimulator(boost::function<void(SimulatorItem* simulator)> callback);
    void onStopSimulationClicked();

    boost::signal<void(SimulatorItem*)> sigSimulationAboutToStart_;
};
}

#endif
