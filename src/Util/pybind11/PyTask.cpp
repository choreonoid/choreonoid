/*!
  @author Shizuko Hattori
  @author Shin'ichiro Nakaoka
 */

#include "PyReferenced.h"
#include "PySignal.h"
#include "../Task.h"
#include "../AbstractTaskSequencer.h"
#include "../ValueTree.h"
#include <pybind11/stl.h>
#include <set>
#include <map>

using namespace std;
using namespace cnoid;
namespace py = pybind11;

namespace {

typedef py::call_guard<py::gil_scoped_release> release_gil;

/**
   \note The header "pybind11/functional.h" must not included in this file to use a custom
   Python-function wrapper "PyTaskFunc" and a custom Python wrapper of "TaskFunc" defined in
   this file instead of the automatic conversion between std::function and pybind11::function.

   \note A callback function defined in Python is stored as a pybind11::function object.
   If the function is an instance function of a Task instance, a circular reference the task object
   and the function object is generated because a referenced-counted reference to the task object
   is contained in the function object. In this case, the task object is never released even if the
   task is removed from the task sequencer and there is no varibale that refers to the task in Python.
   Using the weakref module may solve this problem.
*/
struct PyTaskFunc
{
    py::function func;

    PyTaskFunc(py::function f) : func(f) { }

    void operator()(TaskProc* proc)
    {
        py::gil_scoped_acquire lock;
        try {
            int numArgs = func.attr("__code__").attr("co_argcount").cast<int>();
            if(py::hasattr(func, "__self__")){
                bool isBoundMethod = !func.attr("__self__").is_none();
                if(isBoundMethod){
                    --numArgs; // for the first 'self' argument
                }
            }
            if(numArgs == 0){
                func();
            } else {
                func(proc);
            }
        } catch (const py::error_already_set& ex){ 
            py::print(ex.what());
        }
    }
};

struct PyMenuItemFunc
{
    py::object func;

    PyMenuItemFunc(py::object f) : func(f) { }

    void operator()()
    {
        py::gil_scoped_acquire lock;
        try {
            func();
        } catch(py::error_already_set& ex) {
            py::print(ex.what());
        }
    }
};

struct PyCheckMenuItemFunc
{
    py::object func;
    PyCheckMenuItemFunc(py::object f) : func(f) { }
    void operator()(bool on) {
        py::gil_scoped_acquire lock;
        try {
            func(on);
        } catch(py::error_already_set& ex) {
            py::print(ex.what());
        }
    }
};


class PyTask : public Task
{
public:
    /* Inherit the constructors */
    using Task::Task;

    void onMenuRequest(TaskMenu& menu) override{
        py::gil_scoped_acquire lock;
        if(py::function overload = py::get_overload(this, "onMenuRequest")){
            try {
                overload(py::cast(menu, py::return_value_policy::reference));
            } catch(py::error_already_set& ex) {
                py::print(ex.what());
            }
        } else {
            Task::onMenuRequest(menu);
        }
    }

    void onActivated(AbstractTaskSequencer* sequencer) override
    {
        py::gil_scoped_acquire lock;
        if(py::function overload = py::get_overload(this, "onActivated")){
            try {
                overload(sequencer);
            } catch(py::error_already_set& ex){
                py::print(ex.what());
            }
        } else {
            Task::onActivated(sequencer);
        }
    }

    void onDeactivated(AbstractTaskSequencer* sequencer) override
    {
        py::gil_scoped_acquire lock;
        if(py::function overload = py::get_overload(this, "onDeactivated")){
            try {
                overload(sequencer);
            } catch(py::error_already_set& ex){
                py::print(ex.what());
            }
        } else {
            Task::onDeactivated(sequencer);
        }
    }

    void storeState(AbstractTaskSequencer* sequencer, Mapping& archive) override
    {
        py::gil_scoped_acquire gil;
        if(py::function overload = py::get_overload(this, "storeState")){
            try {
                overload(sequencer, &archive);
            } catch(py::error_already_set& ex){
                py::print(ex.what());
            }
        } else {
            Task::storeState(sequencer, archive);
        }
    }
    
    void restoreState(AbstractTaskSequencer* sequencer, const Mapping& archive) override
    {
        py::gil_scoped_acquire gil;
        if(py::function overload = py::get_overload(this, "restoreState")){
            try {
                overload(sequencer, &archive);
            } catch(py::error_already_set& ex){
                py::print(ex.what());
            }
        } else {
            Task::restoreState(sequencer, archive);
        }
    }
};


typedef std::set<AbstractTaskSequencer*> TaskSequencerSet;
TaskSequencerSet taskSequencers;

typedef std::map<TaskPtr, py::object> PyTaskMap;
PyTaskMap pyTasks;

void onTaskRemoved(Task* task)
{
    PyTaskMap::iterator p  = pyTasks.find(task);
    if(p != pyTasks.end()){
        py::gil_scoped_acquire lock;
        pyTasks.erase(p);
    }
}


TaskPtr registerTask(AbstractTaskSequencer* sequencer, py::object& pyTask)
{
    py::gil_scoped_acquire lock;
    TaskPtr task = pyTask.cast<TaskPtr>();
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
    
} // namespace 

namespace cnoid {

void exportPyTaskTypes(py::module& m)
{
    py::class_<TaskProc>(m, "TaskProc")
        .def("currentPhaseIndex", &TaskProc::currentPhaseIndex)
        .def("isAutoMode", &TaskProc::isAutoMode)
        .def("breakSequence", &TaskProc::breakSequence)
        .def("setNextCommand", &TaskProc::setNextCommand)
        .def("setNextPhase", &TaskProc::setNextPhase)
        .def("setCommandLinkAutomatic", &TaskProc::setCommandLinkAutomatic)
        .def("executeCommand", &TaskProc::executeCommand)
        .def("wait", &TaskProc::wait)
        .def("waitForCommandToFinish", (bool(TaskProc::*)(double)) &TaskProc::waitForCommandToFinish, release_gil())
        .def("waitForCommandToFinish", [](TaskProc& self){ return self.waitForCommandToFinish(); }, release_gil())
        .def("waitForCommandToFinish",
             (bool(TaskProc::*)(Connection, double)) &TaskProc::waitForCommandToFinish, release_gil())
        .def("notifyCommandFinish", &TaskProc:: notifyCommandFinish)
        .def("notifyCommandFinish", [](TaskProc& self){ self.notifyCommandFinish(); })
        .def("waitForSignal", &TaskProc::waitForSignal, release_gil())
        .def("waitForSignal", [](TaskProc& self, SignalProxy<void()> signalProxy){
                return self.waitForSignal(signalProxy); }, release_gil())
        .def("waitForBooleanSignal", &TaskProc::waitForBooleanSignal, release_gil())
        .def("waitForBooleanSignal", [](TaskProc& self, SignalProxy<void(bool)> signalProxy){
                return self.waitForBooleanSignal(signalProxy); }, release_gil())
        ;

    py::class_<TaskFunc>(m, "TaskFunc")
        .def("__call__", &TaskFunc::operator())
        ;

    py::class_<TaskToggleState, TaskToggleStatePtr, Referenced>(m, "TaskToggleState")
        .def("isChecked", &TaskToggleState::isChecked)
        .def("setChecked", &TaskToggleState::setChecked)
        .def("sigToggled", &TaskToggleState::sigToggled)
        ;

    py::class_<TaskCommand, TaskCommandPtr, Referenced>(m, "TaskCommand")
        .def(py::init<>())
        .def(py::init<const std::string&>())
        .def("caption", &TaskCommand::caption)
        .def("setCaption", &TaskCommand::setCaption)
        .def("description", &TaskCommand::description)
        .def("setDescription", &TaskCommand::setDescription)
        .def("function", &TaskCommand::function)
        .def("setFunction", &TaskCommand::setFunction)
        .def("setFunction", [](TaskCommand& self, py::function func){ return self.setFunction(PyTaskFunc(func)); })
        .def("setDefault", &TaskCommand::setDefault)
        .def("setDefault", [](TaskCommand& self){ return self.setDefault(); })
        .def("isDefault", &TaskCommand::isDefault)
        .def("setCheckable", &TaskCommand::setCheckable)
        .def("setCheckable", [](TaskCommand& self){ return self.setCheckable(); })
        .def("setToggleState", &TaskCommand::setToggleState)
        .def("toggleState",  &TaskCommand::toggleState)
        .def("setChecked",  &TaskCommand::setChecked)
        .def("isChecked", &TaskCommand::isChecked)
        .def("nextPhaseIndex", &TaskCommand::nextPhaseIndex)
        .def("setPhaseLink", &TaskCommand::setPhaseLink)
        .def("setPhaseLinkStep", &TaskCommand::setPhaseLinkStep)
        .def("linkToNextPhase", &TaskCommand::linkToNextPhase)
        .def("nextCommandIndex", &TaskCommand::nextCommandIndex)
        .def("setCommandLink", &TaskCommand::setCommandLink)
        .def("setCommandLinkStep", &TaskCommand::setCommandLinkStep)
        .def("linkToNextCommand", &TaskCommand::linkToNextCommand)
        .def("isCommandLinkAutomatic", &TaskCommand::isCommandLinkAutomatic)
        .def("setCommandLinkAutomatic", &TaskCommand::setCommandLinkAutomatic)
        .def("setCommandLinkAutomatic", [](TaskCommand &self){ self.setCommandLinkAutomatic(); })
        .def("setLevel", &TaskCommand::setLevel)
        .def("level", &TaskCommand::level)
        ;

    py::class_<TaskPhase, TaskPhasePtr, Referenced>(m, "TaskPhase")
        .def(py::init<const std::string&>())
        .def(py::init<const TaskPhase&>())
        .def(py::init<const TaskPhase&, bool>())
        .def("clone", &TaskPhase::clone)
        .def("clone", [](TaskPhase& self){ return self.clone(); })
        .def("caption", &TaskPhase::caption)
        .def("setCaption", &TaskPhase::setCaption)
        .def("isSkipped", &TaskPhase::isSkipped)
        .def("setSkipped", &TaskPhase::setSkipped)
        .def("setPreCommand", &TaskPhase::setPreCommand)
        .def("setPreCommand", [](TaskPhase& self, py::function func){ return self.setPreCommand(PyTaskFunc(func)); })
        .def("preCommand", &TaskPhase::preCommand)
        .def("addCommand", (TaskCommand*(TaskPhase::*)()) &TaskPhase::addCommand)
        .def("addCommand", (TaskCommand*(TaskPhase::*)(const string&)) &TaskPhase::addCommand)
        .def("addToggleCommand", (TaskCommand*(TaskPhase::*)()) &TaskPhase::addToggleCommand)
        .def("addToggleCommand", (TaskCommand*(TaskPhase::*)(const string&)) &TaskPhase::addToggleCommand)
        .def("numCommands", &TaskPhase::numCommands)
        .def("command", &TaskPhase::command)
        .def("lastCommandIndex", &TaskPhase::lastCommandIndex)
        .def("lastCommand", &TaskPhase::lastCommand)
        .def("commandLevel", &TaskPhase::commandLevel)
        .def("maxCommandLevel", &TaskPhase::maxCommandLevel)
        ;

    py::class_<TaskPhaseProxy, TaskPhaseProxyPtr, Referenced >(m, "TaskPhaseProxy")
        .def(py::init<TaskPhase*>())
        .def("setCommandLevel", &TaskPhaseProxy::setCommandLevel)
        .def("commandLevel", &TaskPhaseProxy::commandLevel)
        .def("addCommand", (TaskCommand*(TaskPhaseProxy::*)()) &TaskPhaseProxy::addCommand)
        .def("addCommand", (TaskCommand*(TaskPhaseProxy::*)(const string&)) &TaskPhaseProxy::addCommand)
        .def("addToggleCommand", (TaskCommand*(TaskPhaseProxy::*)()) &TaskPhaseProxy::addToggleCommand)
        .def("addToggleCommand", (TaskCommand*(TaskPhaseProxy::*)(const string&)) &TaskPhaseProxy::addToggleCommand)
        ;

    py::class_<TaskMenu>(m, "TaskMenu")
        .def("addMenuItem", [](TaskMenu& self, const std::string& caption, py::object func){
                self.addMenuItem(caption, PyMenuItemFunc(func)); })
        .def("addCheckMenuItem", [](TaskMenu& self, const std::string& caption, bool isChecked, py::object func){
                self.addCheckMenuItem(caption, isChecked, PyCheckMenuItemFunc(func)); })
        .def("addMenuSeparator", &TaskMenu::addMenuSeparator)
        ;

    py::class_<Task, TaskPtr, PyTask, Referenced>(m, "Task")
        .def(py::init<>())
        .def(py::init<const string&, const string&>())
        .def(py::init<const Task&, bool>())
        .def("name", &Task::name)
        .def("setName", &Task::setName)
        .def("caption", &Task::caption)
        .def("setCaption", &Task::setCaption)
        .def("numPhases", &Task::numPhases)
        .def("phase", &Task::phase)
        .def("addPhase", (TaskPhase*(Task::*)(TaskPhase*)) &Task::addPhase)
        .def("addPhase", (TaskPhase*(Task::*)(const string&)) &Task::addPhase)
        .def("lastPhase", &Task::lastPhase)
        .def("setPreCommand", &Task::setPreCommand)
        .def("setPreCommand", [](Task& self, py::object func){ return self.setPreCommand(PyTaskFunc(func)); })
        .def("addCommand", (TaskCommand*(Task::*)()) &Task::addCommand)
        .def("addCommand", (TaskCommand*(Task::*)(const string&)) &Task::addCommand)
        .def("addToggleCommand", (TaskCommand*(Task::*)()) &Task::addToggleCommand)
        .def("addToggleCommand", (TaskCommand*(Task::*)(const string&)) &Task::addToggleCommand)
        .def("lastCommand", &Task::lastCommand)
        .def("lastCommandIndex", &Task::lastCommandIndex)
        .def("commandLevel", &Task::commandLevel)
        .def("maxCommandLevel", &Task::maxCommandLevel)
        .def("funcToSetCommandLink", &Task::funcToSetCommandLink)
        .def("onActivated", &Task::onActivated)
        .def("onDeactivated", &Task::onDeactivated)
        .def("storeState", &Task::storeState)
        .def("restoreState", &Task::restoreState)
        .def("onMenuRequest", &Task::onMenuRequest)
        ;

    PySignal<void(Task*)>(m, "TaskSignal");
    
    py::class_<AbstractTaskSequencer>(m, "AbstractTaskSequencer")
        .def("activate", &AbstractTaskSequencer::activate)
        .def("activate", [](AbstractTaskSequencer& self){ self.activate(); })
        .def("isActive", &AbstractTaskSequencer::isActive)

        .def("addTask", [](AbstractTaskSequencer* self, py::object pyTask){
                if(TaskPtr task = registerTask(self, pyTask)){
                    self->addTask(task);
                }
            })
        
        .def("updateTask", [](AbstractTaskSequencer* self, py::object pyTask){
                if(TaskPtr task = registerTask(self, pyTask)){
                    return self->updateTask(task);
                }
                return false;
            })
        
        .def("removeTask", &AbstractTaskSequencer::removeTask)
        .def("clearTasks", &AbstractTaskSequencer::clearTasks)
        .def("sigTaskAdded", &AbstractTaskSequencer::sigTaskAdded)
        .def("sigTaskRemoved", &AbstractTaskSequencer::sigTaskRemoved)
        .def("numTasks", &AbstractTaskSequencer::numTasks)
        .def("task",  &AbstractTaskSequencer::task)
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

} // namespace cnoid
