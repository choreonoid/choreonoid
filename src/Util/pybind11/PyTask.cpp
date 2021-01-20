/*!
  @author Shizuko Hattori
  @author Shin'ichiro Nakaoka
 */

#include "PyUtil.h"
#include "../Task.h"
#include "../AbstractTaskSequencer.h"
#include "../ValueTree.h"
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
        .def_property_readonly("currentPhaseIndex", &TaskProc::currentPhaseIndex)
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

        // deprecated
        .def("getCurrentPhaseIndex", &TaskProc::currentPhaseIndex)
        ;

    py::class_<TaskFunc>(m, "TaskFunc")
        .def("__call__", &TaskFunc::operator())
        ;

    py::class_<TaskToggleState, TaskToggleStatePtr, Referenced>(m, "TaskToggleState")
        .def("isChecked", &TaskToggleState::isChecked)
        .def("setChecked", &TaskToggleState::setChecked)
        .def_property_readonly("sigToggled", &TaskToggleState::sigToggled)

        // deprecated
        .def("getSigToggled", &TaskToggleState::sigToggled)
        ;

    py::class_<TaskCommand, TaskCommandPtr, Referenced>(m, "TaskCommand")
        .def(py::init<>())
        .def(py::init<const std::string&>())
        .def_property("caption", &TaskCommand::caption, &TaskCommand::setCaption)
        .def("setCaption", &TaskCommand::setCaption)
        .def_property("description", &TaskCommand::description, &TaskCommand::setDescription)
        .def("setDescription", &TaskCommand::setDescription)
        .def_property("function", &TaskCommand::function, &TaskCommand::setFunction)
        .def("setFunction", &TaskCommand::setFunction)
        .def("setFunction", [](TaskCommand& self, py::function func){ return self.setFunction(PyTaskFunc(func)); })
        .def("setDefault", &TaskCommand::setDefault, py::arg("on") = true)
        .def("isDefault", &TaskCommand::isDefault)
        .def("setCheckable", &TaskCommand::setCheckable, py::arg("on") = true)
        .def_property("toggleState",  &TaskCommand::toggleState, &TaskCommand::setToggleState)
        .def("setToggleState", &TaskCommand::setToggleState)
        .def("setChecked",  &TaskCommand::setChecked)
        .def("isChecked", &TaskCommand::isChecked)
        .def_property_readonly("nextPhaseIndex", &TaskCommand::nextPhaseIndex)
        .def("setPhaseLink", &TaskCommand::setPhaseLink)
        .def("setPhaseLinkStep", &TaskCommand::setPhaseLinkStep)
        .def("linkToNextPhase", &TaskCommand::linkToNextPhase)
        .def_property_readonly("nextCommandIndex", &TaskCommand::nextCommandIndex)
        .def("setCommandLink", &TaskCommand::setCommandLink)
        .def("setCommandLinkStep", &TaskCommand::setCommandLinkStep)
        .def("linkToNextCommand", &TaskCommand::linkToNextCommand)
        .def("isCommandLinkAutomatic", &TaskCommand::isCommandLinkAutomatic)
        .def("setCommandLinkAutomatic", &TaskCommand::setCommandLinkAutomatic, py::arg("on") = true)
        .def_property("level", &TaskCommand::level, &TaskCommand::setLevel)
        .def("setLevel", &TaskCommand::setLevel)
        .def("linkToNextTask", &TaskCommand::linkToNextTask)

        // deprecated
        .def("getCaption", &TaskCommand::caption)
        .def("getDescription", &TaskCommand::description)
        .def("getFunction", &TaskCommand::function)
        .def("getToggleState",  &TaskCommand::toggleState)
        .def("getNextPhaseIndex", &TaskCommand::nextPhaseIndex)
        .def("getNextCommandIndex", &TaskCommand::nextCommandIndex)
        .def("getLevel", &TaskCommand::level)
        ;

    py::class_<TaskPhase, TaskPhasePtr, Referenced>(m, "TaskPhase")
        .def(py::init<const std::string&>())
        .def(py::init<const TaskPhase&>())
        .def(py::init<const TaskPhase&, bool>())
        .def("clone", &TaskPhase::clone, py::arg("doDeepCopy") = true)
        .def_property("caption", &TaskPhase::caption, &TaskPhase::setCaption)
        .def("setCaption", &TaskPhase::setCaption)
        .def("isSkipped", &TaskPhase::isSkipped)
        .def("setSkipped", &TaskPhase::setSkipped)
        .def_property("preCommand", &TaskPhase::preCommand, &TaskPhase::setPreCommand)
        .def("setPreCommand", &TaskPhase::setPreCommand)
        .def("setPreCommand", [](TaskPhase& self, py::function func){ return self.setPreCommand(PyTaskFunc(func)); })
        .def("addCommand", (TaskCommand*(TaskPhase::*)()) &TaskPhase::addCommand)
        .def("addCommand", (TaskCommand*(TaskPhase::*)(const string&)) &TaskPhase::addCommand)
        .def("addToggleCommand", (TaskCommand*(TaskPhase::*)()) &TaskPhase::addToggleCommand)
        .def("addToggleCommand", (TaskCommand*(TaskPhase::*)(const string&)) &TaskPhase::addToggleCommand)
        .def_property_readonly("numCommands", &TaskPhase::numCommands)
        .def("command", &TaskPhase::command)
        .def_property_readonly("lastCommandIndex", &TaskPhase::lastCommandIndex)
        .def("lastCommand", &TaskPhase::lastCommand)
        .def_property_readonly("commandLevel", &TaskPhase::commandLevel)
        .def_property_readonly("maxCommandLevel", &TaskPhase::maxCommandLevel)

        // deprecated
        .def("getCaption", &TaskPhase::caption)
        .def("getPreCommand", &TaskPhase::preCommand)
        .def("getNumCommands", &TaskPhase::numCommands)
        .def("getCommand", &TaskPhase::command)
        .def("getLastCommandIndex", &TaskPhase::lastCommandIndex)
        .def("getCommandLevel", &TaskPhase::commandLevel)
        .def("getMaxCommandLevel", &TaskPhase::maxCommandLevel)
        ;

    py::class_<TaskPhaseProxy, TaskPhaseProxyPtr, Referenced>(m, "TaskPhaseProxy")
        .def(py::init<TaskPhase*>())
        .def_property("commandLevel", &TaskPhaseProxy::commandLevel, &TaskPhaseProxy::setCommandLevel)
        .def("getCommandLevel", &TaskPhaseProxy::commandLevel)
        .def("setCommandLevel", &TaskPhaseProxy::setCommandLevel)
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
        .def_property("name", &Task::name, &Task::setName)
        .def("setName", &Task::setName)
        .def_property("caption", &Task::caption, &Task::setCaption)
        .def("setCaption", &Task::setCaption)
        .def_property_readonly("numPhases", &Task::numPhases)
        .def("phase", &Task::phase)
        .def("addPhase", (TaskPhase*(Task::*)(TaskPhase*)) &Task::addPhase)
        .def("addPhase", (TaskPhase*(Task::*)(const string&)) &Task::addPhase)
        .def_property_readonly("lastPhase", &Task::lastPhase)
        .def("setPreCommand", &Task::setPreCommand)
        .def("setPreCommand", [](Task& self, py::object func){ return self.setPreCommand(PyTaskFunc(func)); })
        .def("addCommand", (TaskCommand*(Task::*)()) &Task::addCommand)
        .def("addCommand", (TaskCommand*(Task::*)(const string&)) &Task::addCommand)
        .def("addToggleCommand", (TaskCommand*(Task::*)()) &Task::addToggleCommand)
        .def("addToggleCommand", (TaskCommand*(Task::*)(const string&)) &Task::addToggleCommand)
        .def_property_readonly("lastCommand", &Task::lastCommand)
        .def_property_readonly("lastCommandIndex", &Task::lastCommandIndex)
        .def_property_readonly("commandLevel", &Task::commandLevel)
        .def_property_readonly("maxCommandLevel", &Task::maxCommandLevel)
        .def_property_readonly("funcToSetCommandLink", &Task::funcToSetCommandLink)
        .def("onActivated", &Task::onActivated)
        .def("onDeactivated", &Task::onDeactivated)
        .def("storeState", &Task::storeState)
        .def("restoreState", &Task::restoreState)
        .def("onMenuRequest", &Task::onMenuRequest)

        // deprecated
        .def("getName", &Task::name)
        .def("getCaption", &Task::caption)
        .def("getNumPhases", &Task::numPhases)
        .def("getPhase", &Task::phase)
        .def("getLastPhase", &Task::lastPhase)
        .def("getLastCommand", &Task::lastCommand)
        .def("getLastCommandIndex", &Task::lastCommandIndex)
        .def("getCommandLevel", &Task::commandLevel)
        .def("getMaxCommandLevel", &Task::maxCommandLevel)
        .def("getFuncToSetCommandLink", &Task::funcToSetCommandLink)
        ;

    PySignal<void(Task*)>(m, "TaskSignal");
    
    py::class_<AbstractTaskSequencer>(m, "AbstractTaskSequencer")
        .def("activate", &AbstractTaskSequencer::activate, py::arg("on") = true)
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
        .def_property_readonly("sigTaskAdded", &AbstractTaskSequencer::sigTaskAdded)
        .def_property_readonly("sigTaskRemoved", &AbstractTaskSequencer::sigTaskRemoved)
        .def_property_readonly("numTasks", &AbstractTaskSequencer::numTasks)
        .def("task",  &AbstractTaskSequencer::task)
        .def_property_readonly("currentTaskIndex", &AbstractTaskSequencer::currentTaskIndex)
        .def("setCurrentTask", &AbstractTaskSequencer::setCurrentTask)
        .def_property_readonly("sigCurrentTaskChanged", &AbstractTaskSequencer::sigCurrentTaskChanged)
        .def_property_readonly("currentPhaseIndex", &AbstractTaskSequencer::currentPhaseIndex)
        .def("setCurrentPhase", &AbstractTaskSequencer::setCurrentPhase)
        .def_property_readonly("sigCurrentPhaseChanged", &AbstractTaskSequencer::sigCurrentPhaseChanged)
        .def_property_readonly("currentCommandIndex", &AbstractTaskSequencer::currentCommandIndex)
        .def("executeCommand", &AbstractTaskSequencer::executeCommand)
        .def_property_readonly("sigCurrentCommandChanged", &AbstractTaskSequencer::sigCurrentCommandChanged)
        .def("isBusy", &AbstractTaskSequencer::isBusy)
        .def_property_readonly("sigBusyStateChanged", &AbstractTaskSequencer::sigBusyStateChanged)
        .def("isAutoMode", &AbstractTaskSequencer::isAutoMode)
        .def("setAutoMode", &AbstractTaskSequencer::setAutoMode)
        .def("sigAutoModeToggled", &AbstractTaskSequencer::sigAutoModeToggled)
        .def("serializeTasks", &AbstractTaskSequencer::serializeTasks)

        // deprecated
        .def("getSigTaskAdded", &AbstractTaskSequencer::sigTaskAdded)
        .def("getSigTaskRemoved", &AbstractTaskSequencer::sigTaskRemoved)
        .def("getNumTasks", &AbstractTaskSequencer::numTasks)
        .def("getTask",  &AbstractTaskSequencer::task)
        .def("getCurrentTaskIndex", &AbstractTaskSequencer::currentTaskIndex)
        .def("getSigCurrentTaskChanged", &AbstractTaskSequencer::sigCurrentTaskChanged)
        .def("getCurrentPhaseIndex", &AbstractTaskSequencer::currentPhaseIndex)
        .def("getSigCurrentPhaseChanged", &AbstractTaskSequencer::sigCurrentPhaseChanged)
        .def("getCurrentCommandIndex", &AbstractTaskSequencer::currentCommandIndex)
        .def("getSigBusyStateChanged", &AbstractTaskSequencer::sigBusyStateChanged)
        .def("getSigCurrentCommandChanged", &AbstractTaskSequencer::sigCurrentCommandChanged)
        ;
}

} // namespace cnoid
