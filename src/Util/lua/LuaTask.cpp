/*!
  @author Shin'ichiro Nakaoka
 */

#include "../Task.h"
#include "../AbstractTaskSequencer.h"
#include "LuaUtil.h"

using namespace std;
using namespace std::placeholders;
using namespace cnoid;

namespace {

class TaskWrap : public Task
{
public:
    sol::function onMenuRequest_;
    sol::function onActivated_;
    sol::function onDeactivated_;
    sol::function storeState_;
    sol::function restoreState_;
    
    TaskWrap() {
        /*
        onMenuRequest_ = bind(&Task::onMenuRequest, this, _1);
        onMenuRequest_ = [&](TaskMenu& menu){ onMenuRequest(menu); };
        onActivated_ = bind(&Task::onActivated, this, _1);
        onDeactivated_ = bind(&Task::onDeactivated, this, _1);
        storeState_ = bind(&Task::storeState, this, _1, _2);
        restoreState_ = bind(&Task::restoreState, this, _1, _2);
        */
    };
    //TaskWrap(const std::string& name, const std::string& caption) : Task(name, caption) { };
    //TaskWrap(const Task& org, bool doDeepCopy = true) : Task(org, doDeepCopy) { };

    virtual void onMenuRequest(TaskMenu& menu) override {
        onMenuRequest_(menu); // menu must be passed as a reference
    }
    /*
    virtual void onMenuRequest(TaskMenu& menu) override {
        if(onMenuRequest_){
            onMenuRequest_(menu);  // menu must be passed as a reference
        } else {
            Task::onMenuRequest(menu);
        }
    }
    void set_onMenuRequest(sol::function func){
        onMenuRequest_override = func;
    }
    sol::function get_onMenuRequest(){
        if(onMenuRequest_override){
            return onMenuRequest_override;
        } else {
            return [this](TaskMenu& menu){ this->onMenuRequest(menu); };
        }
    }
    */

    virtual void onActivated(AbstractTaskSequencer* sequencer){
        onActivated_(sequencer);
    }
    virtual void onDeactivated(AbstractTaskSequencer* sequencer){
        onDeactivated_(sequencer);
    }
};

typedef ref_ptr<TaskWrap> TaskWrapPtr;


}

namespace cnoid {

void exportLuaTaskTypes(sol::table& module)
{
    module.new_usertype<TaskProc>(
        "TaskProc",
        "new", sol::no_constructor,
        "currentPhaseIndex", &TaskProc::currentPhaseIndex,
        "isAutoMode", &TaskProc::isAutoMode,
        "breakSequence", &TaskProc::breakSequence,
        "setNextCommand", &TaskProc::setNextCommand,
        "setNextPhase", &TaskProc::setNextPhase,
        "setCommandLinkAutomatic", &TaskProc::setCommandLinkAutomatic,
        "executeCommand", &TaskProc::executeCommand,
        "wait", &TaskProc::wait,
        //"waitForCommandToFinish", TaskProc_waitForCommandToFinish1,
        //"waitForCommandToFinish", TaskProc_waitForCommandToFinish2,
        //"waitForCommandToFinish", TaskProc_waitForCommandToFinish3,
        "notifyCommandFinish", &TaskProc:: notifyCommandFinish
        //"notifyCommandFinish_true", TaskProc_notifyCommandFinishTrue,
        //"waitForSignal", TaskPorc_waitForSignal1,
        //"waitForSignal", TaskPorc_waitForSignal2,
        //"waitForBooleanSignal", TaskPorc_waitForBooleanSignal1,
        //"waitForBooleanSignal", TaskPorc_waitForBooleanSignal2
        );

    module.new_usertype<TaskToggleState>(
        "TaskToggleState",
        //sol::base_classes, sol::bases<Referenced>(),
        "isChecked", &TaskToggleState::isChecked,
        "setChecked", &TaskToggleState::setChecked,
        "sigToggled", &TaskToggleState::sigToggled
        );

    module.new_usertype<TaskCommand>(
        "TaskCommand",
        //sol::base_classes, sol::bases<Referenced>(),
        "new", sol::factories([](const char* caption) -> TaskCommandPtr { return new TaskCommand(caption); }),
        "caption", &TaskCommand::caption,
        //"setCaption", TaskCommand_setCaption,
        "description", &TaskCommand::description,
        //"setDescription", TaskCommand_setDescription,
        "function", &TaskCommand::function,
        //"setFunction", TaskCommand_setFunction,
        //"setDefault", TaskCommand_setDefault1,
        //"setDefault", TaskCommand_setDefault2,
        "isDefault", &TaskCommand::isDefault,
        //"setCheckable", TaskCommand_setCheckable1,
        //"setCheckable", TaskCommand_setCheckable2,
        //"setToggleState", TaskCommand_setToggleState,
        //"toggleState", TaskCommand_toggleState,
        //"setChecked", TaskCommand_setChecked,
        "isChecked", &TaskCommand::isChecked,
        "nextPhaseIndex", &TaskCommand::nextPhaseIndex,
        //"setPhaseLink", TaskCommand_setPhaseLink,
        //"setPhaseLinkStep", TaskCommand_setPhaseLinkStep,
        //"linkToNextPhase", TaskCommand_linkToNextPhase,
        "nextCommandIndex", &TaskCommand::nextCommandIndex,
        //"setCommandLink", TaskCommand_setCommandLink,
        //"setCommandLinkStep", TaskCommand_setCommandLinkStep,
        //"linkToNextCommand", TaskCommand_linkToNextCommand,
        "isCommandLinkAutomatic", &TaskCommand::isCommandLinkAutomatic,
        //"setCommandLinkAutomatic", TaskCommand_setCommandLinkAutomatic1,
        //"setCommandLinkAutomatic", TaskCommand_setCommandLinkAutomatic2,
        //"setLevel", TaskCommand_setLevel,
        "level", &TaskCommand::level
        );

    
    module.new_usertype<TaskPhase>(
        "TaskPhase",
        "new", sol::factories([](const char* caption) -> TaskPhasePtr { return new TaskPhase(caption); }),
        //init<const TaskPhase&>())
        //init<const TaskPhase&, bool>())
        //"clone", TaskPhase_clone1,
        //"clone", TaskPhase_clone2,
        "caption", &TaskPhase::caption,
        "setCaption", &TaskPhase::setCaption,
        "isSkipped", &TaskPhase::isSkipped,
        "setSkipped", &TaskPhase::setSkipped,
        //"setPreCommand", TaskPhase_setPreCommand,
        "setPreCommand", &TaskPhase::setPreCommand,
        "preCommand", &TaskPhase::preCommand,
        //"addCommand", TaskPhase_addCommand1,
        //"addCommand", TaskPhase_addCommand2,
        //"addToggleCommand", TaskPhase_addToggleCommand1,
        //"addToggleCommand", TaskPhase_addToggleCommand2,
        //"addCommandEx", python::raw_function(TaskPhase_addCommandEx, 2),
        "numCommands", &TaskPhase::numCommands,
        //"command", TaskPhase_command,
        "lastCommandIndex", &TaskPhase::lastCommandIndex,
        //"lastCommand", TaskPhase_lastCommand,
        "commandLevel", &TaskPhase::commandLevel,
        "maxCommandLevel", &TaskPhase::maxCommandLevel
        );

    module.new_usertype<TaskPhaseProxy>(
        "TaskPhaseProxy",
        "new", sol::no_constructor,
        "setCommandLevel", &TaskPhaseProxy::setCommandLevel,
        "commandLevel", &TaskPhaseProxy::commandLevel
        //"addCommand", TaskPhaseProxy_addCommand1,
        //"addCommand", TaskPhaseProxy_addCommand2,
        //"addToggleCommand", TaskPhaseProxy_addToggleCommand1,
        //"addToggleCommand", TaskPhaseProxy_addToggleCommand2
        );
    
    module.new_usertype<TaskMenu>(
        "TaskMenu",
        "new", sol::no_constructor
        //"addMenuItem", TaskMenu_addMenuItem1,
        //"addCheckMenuItem", TaskMenu_addCheckMenuItem1,
        //"addMenuSeparator", TaskMenu_addMenuSeparator
        );
    
    module.new_usertype<TaskWrap>(
        "Task",
        "new", sol::factories([]() -> TaskWrapPtr { return new TaskWrap(); }),
        "name", &Task::name, 
        "setName", &Task::setName,
        "caption", &Task::caption,
        "setCaption", &Task::setCaption,
        "numPhases", &Task::numPhases,
        //"phase", Task_phase,
        //"addPhase", Task_addPhase1,
        //"addPhase", Task_addPhase2,
        //"lastPhase", Task_lastPhase,
        //"setPreCommand", Task_setPreCommand,
        "setPreCommand", &Task::setPreCommand,
        //"addCommand", Task_addCommand1,
        //"addCommand", Task_addCommand2,
        //"addToggleCommand", Task_addToggleCommand1,
        //"addToggleCommand", Task_addToggleCommand2,
        //"addCommandEx", python::raw_function(Task_addCommandEx, 2),
        //"lastCommand", Task_lastCommand,
        "lastCommandIndex", &Task::lastCommandIndex,
        "funcToSetCommandLink", &Task::funcToSetCommandLink,
        "onMenuRequest", &Task::onMenuRequest,
        "onActivated", &Task::onActivated,
        "onDeactivated", &Task::onDeactivated,
        //"storeState", &Task::storeState,
        //"restoreState", &Task::restoreState,
        "commandLevel", &Task::commandLevel,
        "maxCommandLevel", &Task::maxCommandLevel
        );

    module.new_usertype<AbstractTaskSequencer>(
        "AbstractTaskSequencer",
        "new", sol::no_constructor,
        "activate", &AbstractTaskSequencer::activate,
        "isActive", &AbstractTaskSequencer::isActive,
        //"addTask", AbstractTaskSequencer_addTask,
        //"updateTask", AbstractTaskSequencer_updateTask,
        "removeTask", &AbstractTaskSequencer::removeTask,
        "sigTaskAdded", &AbstractTaskSequencer::sigTaskAdded,
        "sigTaskRemoved", &AbstractTaskSequencer::sigTaskRemoved,
        "clearTasks", &AbstractTaskSequencer::clearTasks,
        "numTasks", &AbstractTaskSequencer::numTasks,
        //"task", AbstractTaskSequencer_task,
        "currentTaskIndex", &AbstractTaskSequencer::currentTaskIndex,
        "setCurrentTask", &AbstractTaskSequencer::setCurrentTask,
        "sigCurrentTaskChanged", &AbstractTaskSequencer::sigCurrentTaskChanged,
        "currentPhaseIndex", &AbstractTaskSequencer::currentPhaseIndex,
        "setCurrentPhase", &AbstractTaskSequencer::setCurrentPhase,
        "sigCurrentPhaseChanged", &AbstractTaskSequencer::sigCurrentPhaseChanged,
        "currentCommandIndex", &AbstractTaskSequencer::currentCommandIndex,
        "executeCommand", &AbstractTaskSequencer::executeCommand,
        "sigCurrentCommandChanged", &AbstractTaskSequencer::sigCurrentCommandChanged,
        "isBusy", &AbstractTaskSequencer::isBusy,
        "sigBusyStateChanged", &AbstractTaskSequencer::sigBusyStateChanged,
        "isAutoMode", &AbstractTaskSequencer::isAutoMode,
        "setAutoMode", &AbstractTaskSequencer::setAutoMode,
        "sigAutoModeToggled", &AbstractTaskSequencer::sigAutoModeToggled
        );
}

}
