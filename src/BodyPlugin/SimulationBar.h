/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_PLUGIN_SIMULATION_BAR_H
#define CNOID_BODY_PLUGIN_SIMULATION_BAR_H

#include <cnoid/ToolBar>
#include <cnoid/Signal>
#include <cnoid/MenuManager>
#include <QMouseEvent>
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

protected:
    virtual bool eventFilter(QObject* obj, QEvent* event) override;
    virtual bool storeState(Archive& archive) override;
    virtual bool restoreState(const Archive& archive) override;

private:
    SimulationBar();

    void onStoreInitialClicked();
    void onRestoreInitialClicked();
    void forEachSimulator(std::function<void(SimulatorItem* simulator)> callback, bool doSelect = false);
    void startSimulation(SimulatorItem* simulator, bool doReset);
    void onStopButtonRightClicked(QMouseEvent* event);
    void onStopSimulationClicked();
    void onPauseSimulationClicked();
    void pauseSimulation(SimulatorItem* simulator);

    ToolButton* pauseToggle;
    ToolButton* stopButton;
    Signal<void(SimulatorItem*)> sigSimulationAboutToStart_;
    MenuManager menuManager;
    bool isStopConfirmationEnabled;
};

}

#endif
