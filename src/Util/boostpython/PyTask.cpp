/*!
  @author Shizuko Hattori
  @author Shin'ichiro Nakaoka
 */

#include "../Task.h"
#include "../AbstractTaskSequencer.h"
#include "../ValueTree.h"
#include "PyUtil.h"
#include <cnoid/PythonUtil>
#include <boost/python/raw_function.hpp>
#include <set>
#include <map>

using namespace std;
using namespace boost::python;
namespace python = boost::python;
using namespace cnoid;

// for MSVC++2015 Update3
CNOID_PYTHON_DEFINE_GET_POINTER(TaskProc)
CNOID_PYTHON_DEFINE_GET_POINTER(TaskMenu)
CNOID_PYTHON_DEFINE_GET_POINTER(AbstractTaskSequencer)
        
namespace {

void TaskProc_notifyCommandFinishTrue(TaskProc& self){
    self.notifyCommandFinish(true);
}

bool TaskPorc_waitForSignal1(python::object self, python::object signalProxy)
{
    python::object notifyCommandFinish = self.attr("notifyCommandFinish_true");
    python::object connection = signalProxy.attr("connect")(notifyCommandFinish);
    return python::extract<bool>(self.attr("waitForCommandToFinish")(connection, 0.0));
}

bool TaskPorc_waitForSignal2(python::object self, python::object signalProxy, double timeout)
{
    python::object notifyCommandFinish = self.attr("notifyCommandFinish_true");
    python::object connection = signalProxy.attr("connect")(notifyCommandFinish);
    return python::extract<bool>(self.attr("waitForCommandToFinish")(connection, timeout));
}

bool TaskPorc_waitForBooleanSignal1(python::object self, python::object signalProxy)
{
    python::object notifyCommandFinish = self.attr("notifyCommandFinish");
    python::object connection = signalProxy.attr("connect")(notifyCommandFinish);
    return python::extract<bool>(self.attr("waitForCommandToFinish")(connection, 0.0));
}

bool TaskPorc_waitForBooleanSignal2(python::object self, python::object signalProxy, double timeout)
{
    python::object notifyCommandFinish = self.attr("notifyCommandFinish");
    python::object connection = signalProxy.attr("connect")(notifyCommandFinish);
    return python::extract<bool>(self.attr("waitForCommandToFinish")(connection, timeout));
}

bool TaskProc_waitForCommandToFinish1(TaskProc& self)
{
    bool ret;
    Py_BEGIN_ALLOW_THREADS
    ret = self.waitForCommandToFinish();
    Py_END_ALLOW_THREADS
    return ret;
}

bool TaskProc_waitForCommandToFinish2(TaskProc& self, double timeout)
{
    bool ret;
    Py_BEGIN_ALLOW_THREADS
    ret = self.waitForCommandToFinish(timeout);
    Py_END_ALLOW_THREADS
    return ret;
}

bool TaskProc_waitForCommandToFinish3(TaskProc& self, Connection connectionToDisconnect, double timeout)
{
    bool ret;
    Py_BEGIN_ALLOW_THREADS
    ret = self.waitForCommandToFinish(connectionToDisconnect, timeout);
    Py_END_ALLOW_THREADS
    return ret;
}

/**
   \todo Currently boost::python::object is used for storing the callback function object,
   but this generates a circular reference between the task object and the function object
   because callback functions are ususally instance functions of the task object and the reference
   to the task object (self) is contained in the function objcets. In this case, the task object
   is never released even if the task is removed from the task sequencer and there is no
   varibale that refers to the task in Python. Using the weakref module may solve this problem.
*/
struct PyTaskFunc
{
    python::object func;
    PyTaskFunc(python::object f) : func(f) {
        if(!PyFunction_Check(f.ptr()) && !PyMethod_Check(f.ptr())){
            PyErr_SetString(PyExc_TypeError, "Task command must be a function type object");
            python::throw_error_already_set();
        }
    }
    void operator()(TaskProc* proc) {
        PyGILock lock;
        try {
            int numArgs = python::extract<int>(func.attr("func_code").attr("co_argcount"));
            if(numArgs == 0){
                func();
            } else {
                func(boost::ref(proc));
            }
        } catch(python::error_already_set const& ex) {
            handlePythonException();
        }
    }
};

struct PyMenuItemFunc
{
    python::object func;
    PyMenuItemFunc(python::object f) : func(f) { }
    void operator()() {
        PyGILock lock;
        try {
            func();
        } catch(python::error_already_set const& ex) {
            handlePythonException();
        }
    }
};

struct PyCheckMenuItemFunc
{
    python::object func;
    PyCheckMenuItemFunc(python::object f) : func(f) { }
    void operator()(bool on) {
        PyGILock lock;
        try {
            func(on);
        } catch(python::error_already_set const& ex) {
            handlePythonException();
        }
    }
};
    

TaskCommandPtr TaskCommand_setCaption(TaskCommand& self, const std::string& caption){
    return self.setCaption(caption);
}

TaskCommandPtr TaskCommand_setDescription(TaskCommand& self, const std::string& description){
    return self.setDescription(description);
}

TaskCommandPtr TaskCommand_setFunction(TaskCommand& self, python::object func){
    return self.setFunction(PyTaskFunc(func));
}

TaskCommandPtr TaskCommand_setDefault1(TaskCommand& self) {
    return self.setDefault();
}

TaskCommandPtr TaskCommand_setDefault2(TaskCommand& self, bool on) {
    return self.setDefault(on);
}

TaskCommandPtr TaskCommand_setCheckable1(TaskCommand& self){
    return self.setCheckable();
}

TaskCommandPtr TaskCommand_setCheckable2(TaskCommand& self, bool on) {
    return self.setCheckable(on);
}

TaskCommandPtr TaskCommand_setToggleState(TaskCommand& self, TaskToggleState* state){
    return self.setToggleState(state);
}

TaskToggleStatePtr TaskCommand_toggleState(TaskCommand& self){
    return self.toggleState();
}

TaskCommandPtr TaskCommand_setChecked(TaskCommand& self, bool on){
    return self.setChecked(on);
}

TaskCommandPtr TaskCommand_setPhaseLink(TaskCommand& self, int phaseIndex) {
    return self.setPhaseLink(phaseIndex);
}

TaskCommandPtr TaskCommand_setPhaseLinkStep(TaskCommand& self, int phaseIndexStep) {
    return self.setPhaseLinkStep(phaseIndexStep);
}

TaskCommandPtr TaskCommand_linkToNextPhase(TaskCommand& self) {
    return self.linkToNextPhase();
}

TaskCommandPtr TaskCommand_setCommandLink(TaskCommand& self, int commandIndex){
    return self.setCommandLink(commandIndex);
}

TaskCommandPtr TaskCommand_setCommandLinkStep(TaskCommand& self, int commandIndexStep){
    return self.setCommandLinkStep(commandIndexStep);
}

TaskCommandPtr TaskCommand_linkToNextCommand(TaskCommand& self) {
    return self.linkToNextCommand();
}

TaskCommandPtr TaskCommand_setCommandLinkAutomatic1(TaskCommand& self) {
    return self.setCommandLinkAutomatic();
}

TaskCommandPtr TaskCommand_setCommandLinkAutomatic2(TaskCommand& self, bool on) {
    return self.setCommandLinkAutomatic(on);
}

TaskCommandPtr TaskCommand_setLevel(TaskCommand& self, int level){
    return self.setLevel(level);
}

TaskPhasePtr TaskPhase_clone1(TaskPhase& self){
    return self.clone();
}

TaskPhasePtr  TaskPhase_clone2(TaskPhase& self, bool doDeepCopy ){
    return self.clone(doDeepCopy);
}

void TaskPhase_setPreCommand(TaskPhase& self, python::object func){
    return self.setPreCommand(PyTaskFunc(func));
}

TaskCommandPtr TaskPhase_addCommand1(TaskPhase& self) {
    return self.addCommand();
}

TaskCommandPtr TaskPhase_addCommand2(TaskPhase& self, const std::string& caption) {
    return self.addCommand(caption);
}

TaskCommandPtr TaskPhase_addToggleCommand1(TaskPhase& self) {
    return self.addToggleCommand();
}

TaskCommandPtr TaskPhase_addToggleCommand2(TaskPhase& self, const std::string& caption) {
    return self.addToggleCommand(caption);
}

TaskCommandPtr TaskPhase_addCommandExMain(TaskPhase* self, const std::string& caption, python::dict kw) {

    TaskCommandPtr command = self->addCommand(caption);

    python::list args = kw.items();
    const int n = python::len(args);
    for(int i=0; i < n; ++i){
        python::object arg(args[i]);
        std::string key = python::extract<std::string>(arg[0]);
        python::object value(arg[1]);
        if(key == "default" && PyBool_Check(value.ptr())){
            if(python::extract<bool>(value)){
                command->setDefault();
            }
        } else if(key == "function"){
            //TaskCommand_setFunction(*command, value);
        }
    }
    return command;
}

TaskCommandPtr TaskPhase_addCommandEx(python::tuple args_, python::dict kw) {
    TaskPhasePtr self = python::extract<TaskPhasePtr>(args_[0]);
    const std::string caption = python::extract<std::string>(args_[1]);
    return TaskPhase_addCommandExMain(self, caption, kw);
}    


TaskCommandPtr TaskPhase_command(TaskPhase& self, int index) {
    return self.command(index);
}

TaskCommandPtr TaskPhase_lastCommand(TaskPhase& self) {
    return self.lastCommand();
}


TaskCommandPtr TaskPhaseProxy_addCommand1(TaskPhaseProxy& self) {
    return self.addCommand();
}

TaskCommandPtr TaskPhaseProxy_addCommand2(TaskPhaseProxy& self, const std::string& caption) {
    return self.addCommand(caption);
}

TaskCommandPtr TaskPhaseProxy_addToggleCommand1(TaskPhaseProxy& self) {
    return self.addToggleCommand();
}

TaskCommandPtr TaskPhaseProxy_addToggleCommand2(TaskPhaseProxy& self, const std::string& caption) {
    return self.addToggleCommand(caption);
}

void TaskMenu_addMenuItem1(TaskMenu& self, const std::string& caption, python::object func){
    self.addMenuItem(caption, PyMenuItemFunc(func));
}

void TaskMenu_addCheckMenuItem1(TaskMenu& self, const std::string& caption, bool isChecked, python::object func){
    self.addCheckMenuItem(caption, isChecked, PyCheckMenuItemFunc(func));
}

void TaskMenu_addMenuSeparator(TaskMenu& self){
    self.addMenuSeparator();
}

class TaskWrap : public Task, public python::wrapper<Task>
{
public :
    TaskWrap() { };

    TaskWrap(const std::string& name, const std::string& caption) : Task(name, caption) { };
    
    TaskWrap(const Task& org, bool doDeepCopy = true) : Task(org, doDeepCopy) { };

    virtual void onMenuRequest(TaskMenu& menu){
        bool called = false;
        {
            PyGILock lock;
            try {
                if(python::override onMenuRequest = this->get_override("onMenuRequest")){
                    called = true;
                    onMenuRequest(boost::ref(menu));
                } 
            } catch(python::error_already_set const& ex) {
                cnoid::handlePythonException();
            }
        }
        if(!called){
            Task::onMenuRequest(menu);
        }
    }

    void default_onMenuRequest(TaskMenu& menu) {
        return this->Task::onMenuRequest(menu);
    }

    virtual void onActivated(AbstractTaskSequencer* sequencer){
        bool isOverridden = false;
        {
            PyGILock lock;
            try {
                if(python::override func = this->get_override("onActivated")){
                    isOverridden = true;
                    func(boost::ref(sequencer));
                }
            } catch(python::error_already_set const& ex) {
                cnoid::handlePythonException();
            }
        }
        if(!isOverridden){
            Task::onActivated(sequencer);
        }
    }

    void default_onActivated(AbstractTaskSequencer* sequencer){
        return this->Task::onActivated(sequencer);
    }

    virtual void onDeactivated(AbstractTaskSequencer* sequencer){
        bool isOverridden = false;
        {
            PyGILock lock;
            try {
                if(python::override func = this->get_override("onDeactivated")){
                    isOverridden = true;
                    func(boost::ref(sequencer));
                }
            } catch(python::error_already_set const& ex) {
                cnoid::handlePythonException();
            }
        }
        if(!isOverridden){
            Task::onDeactivated(sequencer);
        }
    }

    void default_onDeactivated(AbstractTaskSequencer* sequencer){
        return this->Task::onDeactivated(sequencer);
    }

    virtual void storeState(AbstractTaskSequencer* sequencer, Mapping& archive){
        bool isOverridden = false;
        {
            PyGILock lock;
            try {
                if(python::override storeStateFunc = this->get_override("storeState")){
                    isOverridden = true;
                    MappingPtr a = &archive;
                    storeStateFunc(boost::ref(sequencer), a);
                }
            } catch(python::error_already_set const& ex) {
                cnoid::handlePythonException();
            }
        }
        if(!isOverridden){
            Task::storeState(sequencer, archive);
        }
    }

    void default_storeState(AbstractTaskSequencer* sequencer, Mapping& archive){
        return this->Task::storeState(sequencer, archive);
    }

    virtual void restoreState(AbstractTaskSequencer* sequencer, const Mapping& archive){
        bool isOverridden = false;
        {
            PyGILock lock;
            try {
                if(python::override restoreState = this->get_override("restoreState")){
                    isOverridden = true;
                    MappingPtr a = const_cast<Mapping*>(&archive);
                    restoreState(boost::ref(sequencer), a);
                }
            } catch(python::error_already_set const& ex) {
                cnoid::handlePythonException();
            }
        }
        if(!isOverridden){
            Task::restoreState(sequencer, archive);
        }
    }

    void default_restoreState(AbstractTaskSequencer* sequencer, const Mapping& archive){
        return this->Task::restoreState(sequencer, archive);
    }
};

typedef ref_ptr<TaskWrap> TaskWrapPtr;


TaskPhasePtr Task_phase(Task& self, int index){
    return self.phase(index);
}

TaskPhasePtr Task_addPhase1(Task& self, TaskPhase* phase){
    return self.addPhase(phase);
}

TaskPhasePtr Task_addPhase2(Task& self, const std::string& caption){
    return self.addPhase(caption);
}

TaskPhasePtr Task_lastPhase(Task& self){
    return self.lastPhase();
}

void Task_setPreCommand(Task& self, python::object func){
    return self.setPreCommand(PyTaskFunc(func));
}

TaskCommandPtr Task_addCommand1(Task& self){
    return self.addCommand();
}

TaskCommandPtr Task_addCommand2(Task& self, const std::string& caption){
    return self.addCommand(caption);
}

TaskCommandPtr Task_addToggleCommand1(Task& self){
    return self.addToggleCommand();
}

TaskCommandPtr Task_addToggleCommand2(Task& self, const std::string& caption){
    return self.addToggleCommand(caption);
}

TaskCommandPtr Task_lastCommand(Task& self){
    return self.lastCommand();
}

TaskCommandPtr Task_addCommandEx(python::tuple args_, python::dict kw){
    TaskWrapPtr self = python::extract<TaskWrapPtr>(args_[0]);
    const std::string caption = python::extract<std::string>(args_[1]);
    return TaskPhase_addCommandExMain(self->lastPhase(), caption, kw);
}


typedef std::set<AbstractTaskSequencer*> TaskSequencerSet;
TaskSequencerSet taskSequencers;

typedef std::map<TaskPtr, object> PyTaskMap;
PyTaskMap pyTasks;

void onTaskRemoved(Task* task)
{
    PyTaskMap::iterator p  = pyTasks.find(task);
    if(p != pyTasks.end()){
        PyGILock lock;
        pyTasks.erase(p);
    }
}


TaskPtr registerTask(AbstractTaskSequencer* sequencer, object& pyTask)
{
    PyGILock lock;
    TaskPtr task = extract<TaskPtr>(pyTask);
    if(task){
        if(taskSequencers.find(sequencer) == taskSequencers.end()){
            sequencer->sigTaskRemoved().connect(onTaskRemoved);
            taskSequencers.insert(sequencer);
        }
        pyTasks[task] = pyTask;
        return task;
    }
    return TaskPtr();
}
    

void AbstractTaskSequencer_addTask(AbstractTaskSequencer& self, object pyTask)
{
    if(TaskPtr task = registerTask(&self, pyTask)){
        self.addTask(task);
    }
}


bool AbstractTaskSequencer_updateTask(AbstractTaskSequencer& self, object pyTask)
{
    if(TaskPtr task = registerTask(&self, pyTask)){
        return self.updateTask(task);
    }
    return false;
}


TaskPtr AbstractTaskSequencer_task(AbstractTaskSequencer& self, int index)
{
    return self.task(index);
}

}

namespace cnoid {

void exportPyTaskTypes()
{
    class_<TaskProc, TaskProc*, boost::noncopyable>("TaskProc", no_init)
        .def("currentPhaseIndex", &TaskProc::currentPhaseIndex)
        .def("isAutoMode", &TaskProc::isAutoMode)
        .def("breakSequence", &TaskProc::breakSequence)
        .def("setNextCommand", &TaskProc::setNextCommand)
        .def("setNextPhase", &TaskProc::setNextPhase)
        .def("setCommandLinkAutomatic", &TaskProc::setCommandLinkAutomatic)
        .def("executeCommand", &TaskProc::executeCommand)
        .def("wait", &TaskProc::wait)
        .def("waitForCommandToFinish", TaskProc_waitForCommandToFinish1)
        .def("waitForCommandToFinish", TaskProc_waitForCommandToFinish2)
        .def("waitForCommandToFinish", TaskProc_waitForCommandToFinish3)
        .def("notifyCommandFinish", &TaskProc:: notifyCommandFinish)
        .def("notifyCommandFinish_true", TaskProc_notifyCommandFinishTrue)
        .def("waitForSignal", TaskPorc_waitForSignal1)
        .def("waitForSignal", TaskPorc_waitForSignal2)
        .def("waitForBooleanSignal", TaskPorc_waitForBooleanSignal1)
        .def("waitForBooleanSignal", TaskPorc_waitForBooleanSignal2)
        ;

    class_<TaskFunc>("TaskFunc")
        .def("__call__", &TaskFunc::operator())
        ;

    class_<TaskToggleState, TaskToggleStatePtr, bases<Referenced>, boost::noncopyable >("TaskToggleState")
        .def("isChecked", &TaskToggleState::isChecked)
        .def("setChecked", &TaskToggleState::setChecked)
        .def("sigToggled", &TaskToggleState::sigToggled)
        ;

    implicitly_convertible<TaskToggleStatePtr, ReferencedPtr>();

    class_<TaskCommand, TaskCommandPtr, bases<Referenced> >("TaskCommand", init<const std::string&>())
        .def("caption", &TaskCommand::caption, return_value_policy<copy_const_reference>())
        .def("setCaption", TaskCommand_setCaption)
        .def("description", &TaskCommand::description, return_value_policy<copy_const_reference>())
        .def("setDescription", TaskCommand_setDescription)
        .def("function", &TaskCommand::function)
        .def("setFunction", TaskCommand_setFunction)
        .def("setDefault", TaskCommand_setDefault1)
        .def("setDefault", TaskCommand_setDefault2)
        .def("isDefault", &TaskCommand::isDefault)
        .def("setCheckable", TaskCommand_setCheckable1)
        .def("setCheckable", TaskCommand_setCheckable2)
        .def("setToggleState", TaskCommand_setToggleState)
        .def("toggleState", TaskCommand_toggleState)
        .def("setChecked", TaskCommand_setChecked)
        .def("isChecked", &TaskCommand::isChecked)
        .def("nextPhaseIndex", &TaskCommand::nextPhaseIndex)
        .def("setPhaseLink", TaskCommand_setPhaseLink)
        .def("setPhaseLinkStep", TaskCommand_setPhaseLinkStep)
        .def("linkToNextPhase", TaskCommand_linkToNextPhase)
        .def("nextCommandIndex", &TaskCommand::nextCommandIndex)
        .def("setCommandLink", TaskCommand_setCommandLink)
        .def("setCommandLinkStep", TaskCommand_setCommandLinkStep)
        .def("linkToNextCommand", TaskCommand_linkToNextCommand)
        .def("isCommandLinkAutomatic", &TaskCommand::isCommandLinkAutomatic)
        .def("setCommandLinkAutomatic", TaskCommand_setCommandLinkAutomatic1)
        .def("setCommandLinkAutomatic", TaskCommand_setCommandLinkAutomatic2)
        .def("setLevel", TaskCommand_setLevel)
        .def("level", &TaskCommand::level)
        ;
    
    implicitly_convertible<TaskCommandPtr, ReferencedPtr>();
    
    class_<TaskPhase, TaskPhasePtr, bases<Referenced>, boost::noncopyable >
        ("TaskPhase", init<const std::string&>())
        .def(init<const TaskPhase&>())
        .def(init<const TaskPhase&, bool>())
        .def("clone", TaskPhase_clone1)
        .def("clone", TaskPhase_clone2)
        .def("caption", &TaskPhase::caption, return_value_policy<copy_const_reference>())
        .def("setCaption", &TaskPhase::setCaption)
        .def("isSkipped", &TaskPhase::isSkipped)
        .def("setSkipped", &TaskPhase::setSkipped)
        .def("setPreCommand", TaskPhase_setPreCommand)
        .def("setPreCommand", &TaskPhase::setPreCommand)
        .def("preCommand", &TaskPhase::preCommand)
        .def("addCommand", TaskPhase_addCommand1)
        .def("addCommand", TaskPhase_addCommand2)
        .def("addToggleCommand", TaskPhase_addToggleCommand1)
        .def("addToggleCommand", TaskPhase_addToggleCommand2)
        .def("addCommandEx", python::raw_function(TaskPhase_addCommandEx, 2))
        .def("numCommands", &TaskPhase::numCommands)
        .def("command", TaskPhase_command)
        .def("lastCommandIndex", &TaskPhase::lastCommandIndex)
        .def("lastCommand", TaskPhase_lastCommand)
        .def("commandLevel", &TaskPhase::commandLevel)
        .def("maxCommandLevel", &TaskPhase::maxCommandLevel)
        ;

    implicitly_convertible<TaskPhasePtr, ReferencedPtr>();

    class_<TaskPhaseProxy, TaskPhaseProxyPtr, bases<Referenced>, boost::noncopyable >("TaskPhaseProxy", no_init)
        .def("setCommandLevel", &TaskPhaseProxy::setCommandLevel)
        .def("commandLevel", &TaskPhaseProxy::commandLevel)
        .def("addCommand", TaskPhaseProxy_addCommand1)
        .def("addCommand", TaskPhaseProxy_addCommand2)
        .def("addToggleCommand", TaskPhaseProxy_addToggleCommand1)
        .def("addToggleCommand", TaskPhaseProxy_addToggleCommand2)
        ;
    
    implicitly_convertible<TaskPhaseProxyPtr, ReferencedPtr>();
    
    class_<TaskMenu, boost::noncopyable >("TaskMenu", no_init)
        .def("addMenuItem", TaskMenu_addMenuItem1)
        .def("addCheckMenuItem", TaskMenu_addCheckMenuItem1)
        .def("addMenuSeparator", TaskMenu_addMenuSeparator)
        ;
    
    class_<TaskWrap, TaskWrapPtr, bases<Referenced>, boost::noncopyable >("Task", init<>())
        .def(init<const std::string&, const std::string&>())
        .def(init<const Task&, bool>())
        .def(init<const Task&>())
        .def("name", &Task::name, return_value_policy<copy_const_reference>())
        .def("setName", &Task::setName)
        .def("caption", &Task::caption, return_value_policy<copy_const_reference>())
        .def("setCaption", &Task::setCaption)
        .def("numPhases", &Task::numPhases)
        .def("phase", Task_phase)
        .def("addPhase", Task_addPhase1)
        .def("addPhase", Task_addPhase2)
        .def("lastPhase", Task_lastPhase)
        .def("setPreCommand", Task_setPreCommand)
        .def("setPreCommand", &Task::setPreCommand)
        .def("addCommand", Task_addCommand1)
        .def("addCommand", Task_addCommand2)
        .def("addToggleCommand", Task_addToggleCommand1)
        .def("addToggleCommand", Task_addToggleCommand2)
        .def("addCommandEx", python::raw_function(Task_addCommandEx, 2))
        .def("lastCommand", Task_lastCommand)
        .def("lastCommandIndex", &Task::lastCommandIndex)
        .def("funcToSetCommandLink", &Task::funcToSetCommandLink)
        .def("onMenuRequest", &Task::onMenuRequest, &TaskWrap::default_onMenuRequest)
        .def("onActivated", &Task::onActivated, &TaskWrap::default_onActivated)
        .def("onDeactivated", &Task::onDeactivated, &TaskWrap::default_onDeactivated)
        .def("storeState", &Task::storeState, &TaskWrap::default_storeState)
        .def("restoreState", &Task::restoreState, &TaskWrap::default_restoreState)
        .def("commandLevel", &Task::commandLevel)
        .def("maxCommandLevel", &Task::maxCommandLevel)
        ;

    implicitly_convertible<TaskPtr, ReferencedPtr>();
    implicitly_convertible<TaskWrapPtr, TaskPtr>();

    class_<AbstractTaskSequencer, AbstractTaskSequencer*, boost::noncopyable>("AbstractTaskSequencer", no_init)
        .def("activate", &AbstractTaskSequencer::activate)
        .def("isActive", &AbstractTaskSequencer::isActive)
        .def("addTask", AbstractTaskSequencer_addTask)
        .def("updateTask", AbstractTaskSequencer_updateTask)
        .def("removeTask", &AbstractTaskSequencer::removeTask)
        .def("sigTaskAdded", &AbstractTaskSequencer::sigTaskAdded)
        .def("sigTaskRemoved", &AbstractTaskSequencer::sigTaskRemoved)
        .def("clearTasks", &AbstractTaskSequencer::clearTasks)
        .def("numTasks", &AbstractTaskSequencer::numTasks)
        .def("task", AbstractTaskSequencer_task)
        .def("currentTaskIndex", &AbstractTaskSequencer::currentTaskIndex)
        .def("setCurrentTask", &AbstractTaskSequencer::setCurrentTask)
        .def("sigCurrentTaskChanged", &AbstractTaskSequencer::sigCurrentTaskChanged)
        .def("currentPhaseIndex", &AbstractTaskSequencer::currentPhaseIndex)
        .def("setCurrentPhase", &AbstractTaskSequencer::setCurrentPhase)
        .def("sigCurrentPhaseChanged", &AbstractTaskSequencer::sigCurrentPhaseChanged)
        .def("currentCommandIndex", &AbstractTaskSequencer::currentCommandIndex)
        .def("executeCommand", &AbstractTaskSequencer::executeCommand)
        .def("sigCurrentCommandChanged", &AbstractTaskSequencer::sigCurrentCommandChanged)
        .def("isBusy", &AbstractTaskSequencer::isBusy)
        .def("sigBusyStateChanged", &AbstractTaskSequencer::sigBusyStateChanged)
        .def("isAutoMode", &AbstractTaskSequencer::isAutoMode)
        .def("setAutoMode", &AbstractTaskSequencer::setAutoMode)
        .def("sigAutoModeToggled", &AbstractTaskSequencer::sigAutoModeToggled)
        ;
}

}
