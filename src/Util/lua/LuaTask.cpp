/*!
  @author Shin'ichiro Nakaoka
 */

#include "../Task.h"
#include "../AbstractTaskSequencer.h"
#include "../ValueTree.h"
#include "LuaUtil.h"

using namespace std;
using namespace std::placeholders;
using namespace cnoid;

namespace {

class LuaTaskFunc
{
    sol::function func;
public:
    LuaTaskFunc(sol::function f) : func(f) { }
    void operator()(TaskProc* proc){
        try {
            func(proc);
        } catch(const sol::error& ex) {
            // put ex.what() by using func.lua_state() and the print function
        }
    }
};


class TaskWrap : public Task
{
public:
    sol::function onMenuRequest_;
    sol::function onActivated_;
    sol::function onDeactivated_;
    sol::function storeState_;
    sol::function restoreState_;
    
    TaskWrap(sol::this_state s) {
        /*
        sol::state_view lua(s);
        sol::table module = lua.create_table();
        module.set_function("f1", [&](TaskMenu& menu){ onMenuRequest(menu); });
        onMenuRequest_ = module["f1"];
        */

        /**
           onMenuRequest_ = bind(&TaskWrap::onMenuRequest, this, _1);
        */
        /*
        stackDump(s);
        sol::make_reference(s, sol::as_function([](TaskWrap* self, TaskMenu& menu) { self->onMenuRequest(menu); })).push();
        stackDump(s);
        onMenuRequest_ = sol::function(s);
        */

        onMenuRequest_ = makeLuaFunction(s, [](TaskWrap* self, TaskMenu& menu) { self->onMenuRequest(menu); });
        
        /*
        stackDump(s);
        sol::make_reference(s, bind(&TaskWrap::onActivated, this, _1)).push();
        stackDump(s);
        onActivated_ = sol::function(s);
        */
        onActivated_ = makeLuaFunction(s, [](TaskWrap* self, AbstractTaskSequencer* s) { self->onActivated(s); });

        /*
        stackDump(s);
        sol::make_reference(s, bind(&TaskWrap::onDeactivated, this, _1)).push();
        stackDump(s);
        onDeactivated_ = sol::function(s);
        */
        onDeactivated_ = makeLuaFunction(s, [](TaskWrap* self, AbstractTaskSequencer* s) { self->onDeactivated(s); });

        /*
        stackDump(s);
        sol::make_reference(s, bind(&TaskWrap::storeState, this, _1, _2)).push();
        stackDump(s);
        storeState_ = sol::function(s);
        */

        storeState_ = makeLuaFunction(s, [](TaskWrap* self, AbstractTaskSequencer* s, Mapping& a) { self->storeState(s, a); });

        /*
        stackDump(s);
        sol::make_reference(s, bind(&TaskWrap::restoreState, this, _1, _2)).push();
        stackDump(s);
        restoreState_ = sol::function(s);
        stackDump(s);
        */

        restoreState_ = makeLuaFunction(s, [](TaskWrap* self, AbstractTaskSequencer* s, Mapping& a) { self->restoreState(s, a); });
        
    }
    //TaskWrap(const std::string& name, const std::string& caption) : Task(name, caption) { };
    //TaskWrap(const Task& org, bool doDeepCopy = true) : Task(org, doDeepCopy) { };

    virtual void onMenuRequest(TaskMenu& menu) override {
        onMenuRequest_(std::ref(menu));
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
        onMenuRequest_ = func;
    }
    sol::function get_onMenuRequest(){
        if(onMenuRequest_){
            return onMenuRequest_;
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


bool waitForSignal(sol::object self, sol::object signalProxy, bool isBooleanSignal, double timeout, sol::this_state s)
{
    signalProxy.push();
    sol::stack::get_field(s, "connect");
    sol::function connect(s);

    sol::function notifyCommandFinish;
    if(isBooleanSignal){
        notifyCommandFinish = makeLuaFunction(s, [](TaskProc* self, bool isCompleted){ self->notifyCommandFinish(isCompleted); });
    } else {
        notifyCommandFinish = makeLuaFunction(s, [](TaskProc* self){ self->notifyCommandFinish(true); });
    }

    sol::object connection = connect(signalProxy, notifyCommandFinish);

    sol::function waitForCommandToFinish =
        makeLuaFunction(s, [](TaskProc* self, Connection c, double timeout){ return self->waitForCommandToFinish(c, timeout); });
    
    bool result = waitForCommandToFinish(self, connection, timeout);
    
    return result;
}

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
        "waitForCommandToFinish", sol::overload(
            [](TaskProc* self){ return self->waitForCommandToFinish(); },
            [](TaskProc* self, double t){ return self->waitForCommandToFinish(t); },
            [](TaskProc* self, Connection c, bool t){ return self->waitForCommandToFinish(c, t); }),
        "notifyCommandFinish", &TaskProc:: notifyCommandFinish,
        "waitForSignal", sol::overload(
            [](sol::object self, sol::object signalProxy, sol::this_state s){
                return waitForSignal(self, signalProxy, false, 0.0, s); },
            [](sol::object self, sol::object signalProxy, double timeout, sol::this_state s){
                return waitForSignal(self, signalProxy, false, timeout, s); }),
        "waitForBooleanSignal", sol::overload(
            [](sol::object self, sol::object signalProxy, sol::this_state s){
                return waitForSignal(self, signalProxy, true, 0.0, s); },
            [](sol::object self, sol::object signalProxy, double timeout, sol::this_state s){
                return waitForSignal(self, signalProxy, true, timeout, s); })
        );

    module.new_usertype<TaskToggleState>(
        "TaskToggleState",
        "isChecked", &TaskToggleState::isChecked,
        "setChecked", &TaskToggleState::setChecked,
        "sigToggled", &TaskToggleState::sigToggled
        );

    module.new_usertype<TaskCommand>(
        "TaskCommand",
        "new", sol::factories([](const char* caption) -> TaskCommandPtr { return new TaskCommand(caption); }),
        "caption", &TaskCommand::caption,
        "setCaption", [](TaskCommand* self, const char* caption) -> TaskCommandPtr { return self->setCaption(caption); },
        "description", &TaskCommand::description,
        "setDescription", [](TaskCommand* self, const char* description) -> TaskCommandPtr { return self->setDescription(description); },
        "function", &TaskCommand::function,
        "setFunction", [](TaskCommand* self, sol::function func) -> TaskCommandPtr { return self->setFunction(LuaTaskFunc(func)); },
        "setDefault", sol::overload(
            [](TaskCommand* self) -> TaskCommandPtr { return self->setDefault(); },
            [](TaskCommand* self, bool on) -> TaskCommandPtr { return self->setDefault(on); }),
        "isDefault", &TaskCommand::isDefault,
        "setCheckable", sol::overload(
            [](TaskCommand* self) -> TaskCommandPtr { return self->setCheckable(); },
            [](TaskCommand* self, bool on) -> TaskCommandPtr { return self->setCheckable(on); }),
        "setToggleState", [](TaskCommand* self, TaskToggleState* state) -> TaskCommandPtr { return self->setToggleState(state); },
        "toggleState", [](TaskCommand* self) -> TaskToggleStatePtr { return self->toggleState(); },
        "setChecked", [](TaskCommand* self, bool on) -> TaskCommandPtr { return self->setChecked(on); },
        "isChecked", &TaskCommand::isChecked,
        "nextPhaseIndex", &TaskCommand::nextPhaseIndex,
        "setPhaseLink", [](TaskCommand* self, int phaseIndex) -> TaskCommandPtr { return self->setPhaseLink(phaseIndex); },
        "setPhaseLinkStep", [](TaskCommand* self, int phaseIndexStep) -> TaskCommandPtr { return self->setPhaseLinkStep(phaseIndexStep); },
        "linkToNextPhase", [](TaskCommand* self) -> TaskCommandPtr { return self->linkToNextPhase(); },
        "nextCommandIndex", &TaskCommand::nextCommandIndex,
        "setCommandLink", [](TaskCommand* self, int commandIndex) -> TaskCommandPtr { return self->setCommandLink(commandIndex); },
        "setCommandLinkStep", [](TaskCommand* self, int commandIndexStep) -> TaskCommandPtr { return self->setCommandLinkStep(commandIndexStep); },
        "linkToNextCommand", [](TaskCommand* self) -> TaskCommandPtr { return self->linkToNextCommand(); },
        "isCommandLinkAutomatic", &TaskCommand::isCommandLinkAutomatic,
        "setCommandLinkAutomatic", sol::overload(
            [](TaskCommand* self) -> TaskCommandPtr { return self->setCommandLinkAutomatic(); },
            [](TaskCommand* self, bool on) -> TaskCommandPtr { return self->setCommandLinkAutomatic(on); }),
        "setLevel", [](TaskCommand* self, int level) -> TaskCommandPtr { return self->setLevel(level); },
        "level", &TaskCommand::level
        );

    module.new_usertype<TaskPhase>(
        "TaskPhase",
        "new", sol::factories(
            [](const char* caption) -> TaskPhasePtr { return new TaskPhase(caption); },
            [](TaskPhase* org) -> TaskPhasePtr { return new TaskPhase(*org); },
            [](TaskPhase* org, bool doDeepCopy) -> TaskPhasePtr { return new TaskPhase(*org, doDeepCopy); }),
        "clone", sol::overload(
            [](TaskPhase* self) -> TaskPhasePtr { return self->clone(); },
            [](TaskPhase* self, bool doDeepCopy) -> TaskPhasePtr { return self->clone(doDeepCopy); }),
        "caption", &TaskPhase::caption,
        "setCaption", &TaskPhase::setCaption,
        "isSkipped", &TaskPhase::isSkipped,
        "setSkipped", &TaskPhase::setSkipped,
        "setPreCommand", [](TaskPhase* self, sol::function func) { self->setPreCommand(LuaTaskFunc(func)); },
        "preCommand", &TaskPhase::preCommand,
        "addCommand", sol::overload(
            [](TaskPhase* self) -> TaskCommandPtr { return self->addCommand(); },
            [](TaskPhase* self, const char* caption) -> TaskCommandPtr { return self->addCommand(caption); }),
        "addToggleCommand", sol::overload(
            [](TaskPhase* self) -> TaskCommandPtr { return self->addToggleCommand(); },
            [](TaskPhase* self, const char* caption) -> TaskCommandPtr { return self->addToggleCommand(caption); }),
        "numCommands", &TaskPhase::numCommands,
        "command", [](TaskPhase* self, int index) -> TaskCommandPtr { return self->command(index); },
        "lastCommandIndex", &TaskPhase::lastCommandIndex,
        "lastCommand", [](TaskPhase* self) -> TaskCommandPtr { return self->lastCommand(); },
        "commandLevel", &TaskPhase::commandLevel,
        "maxCommandLevel", &TaskPhase::maxCommandLevel
        );

    module.new_usertype<TaskPhaseProxy>(
        "TaskPhaseProxy",
        "new", sol::no_constructor,
        "setCommandLevel", &TaskPhaseProxy::setCommandLevel,
        "commandLevel", &TaskPhaseProxy::commandLevel,
        "addCommand", sol::overload(
            [](TaskPhaseProxy* self) -> TaskCommandPtr { return self->addCommand(); },
            [](TaskPhaseProxy* self, const char* caption) -> TaskCommandPtr { return self->addCommand(caption); }),
        "addToggleCommand", sol::overload(
            [](TaskPhaseProxy* self) -> TaskCommandPtr { return self->addToggleCommand(); },
            [](TaskPhaseProxy* self, const char* caption) -> TaskCommandPtr { return self->addToggleCommand(caption); })
        );
    
    module.new_usertype<TaskMenu>(
        "TaskMenu",
        "new", sol::no_constructor,
        "addMenuItem", [](TaskMenu& self, const char* caption, sol::function func) { self.addMenuItem(caption, func); },
        "addCheckMenuItem", [](TaskMenu& self, const char* caption, bool isChecked, sol::function func) {
            self.addCheckMenuItem(caption, isChecked, func); },
        "addMenuSeparator", [](TaskMenu& self) { self.addMenuSeparator(); }
        );

    module.new_usertype<Task>(
        "TaskBase",
        "new", sol::no_constructor,
        "name", &Task::name, 
        "setName", &Task::setName,
        "caption", &Task::caption,
        "setCaption", &Task::setCaption,
        "numPhases", &Task::numPhases,
        "phase", [](Task* self, int index) -> TaskPhasePtr { return self->phase(index); },
        "addPhase", sol::overload(
            [](Task* self, TaskPhase* phase) -> TaskPhasePtr { return self->addPhase(phase); },
            [](Task* self, const char* caption) -> TaskPhasePtr { return self->addPhase(caption); }),
        "lastPhase", [](Task* self) -> TaskPhasePtr { return self->lastPhase(); },
        "setPreCommand", [](TaskWrap* self, sol::function func) { self->setPreCommand(LuaTaskFunc(func)); },
        "addCommand", sol::overload(
            [](Task* self) -> TaskCommandPtr { return self->addCommand(); },
            [](Task* self, const char* caption) -> TaskCommandPtr { return self->addCommand(caption); }),
        "addToggleCommand", sol::overload(
            [](Task* self) -> TaskCommandPtr { return self->addToggleCommand(); },
            [](Task* self, const char* caption) -> TaskCommandPtr { return self->addToggleCommand(caption); }),
        "lastCommand", [](Task* self) -> TaskCommandPtr { return self->lastCommand(); },
        "lastCommandIndex", &Task::lastCommandIndex,
        "funcToSetCommandLink", &Task::funcToSetCommandLink,
        "commandLevel", &Task::commandLevel,
        "maxCommandLevel", &Task::maxCommandLevel
        );
        
    
    module.new_usertype<TaskWrap>(
        "Task",
        sol::base_classes, sol::bases<Task>(),
        "new", sol::factories([](sol::this_state s) -> TaskWrapPtr { return new TaskWrap(s); }),
        "onMenuRequest", &TaskWrap::onMenuRequest_,
        "onActivated", &TaskWrap::onActivated_,
        "onDeactivated", &TaskWrap::onDeactivated_,
        "storeState", &TaskWrap::storeState_,
        "restoreState", &TaskWrap::restoreState_
        );

    module.new_usertype<AbstractTaskSequencer>(
        "AbstractTaskSequencer",
        "new", sol::no_constructor,
        "activate", sol::overload(
            [](AbstractTaskSequencer* self){ self->activate(true); },
            [](AbstractTaskSequencer* self, bool on){ self->activate(on); }),
        "isActive", &AbstractTaskSequencer::isActive,
        "addTask", &AbstractTaskSequencer::addTask,
        "updateTask", &AbstractTaskSequencer::updateTask,
        "removeTask", &AbstractTaskSequencer::removeTask,
        "sigTaskAdded", &AbstractTaskSequencer::sigTaskAdded,
        "sigTaskRemoved", &AbstractTaskSequencer::sigTaskRemoved,
        "clearTasks", &AbstractTaskSequencer::clearTasks,
        "numTasks", &AbstractTaskSequencer::numTasks,
        "task", [](AbstractTaskSequencer* self, int index) -> TaskPtr { return self->task(index); },
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
