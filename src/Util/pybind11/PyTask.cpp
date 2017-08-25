/*!
  @author Shizuko Hattori
  @author Shin'ichiro Nakaoka
 */

#include "PyUtil.h"
#include "../Task.h"
#include "../AbstractTaskSequencer.h"
#include "../ValueTree.h"
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include <set>
#include <map>

using namespace std;
using namespace cnoid;
namespace py = pybind11;

namespace {

struct PyTaskFunc
{
    py::function func;
    PyTaskFunc(py::function f) : func(f) {
    }

    void operator()(TaskProc* proc) {
        py::gil_scoped_acquire lock;
        try {
            int numArgs = func.attr("__code__").attr("co_argcount").cast<int>();
            if(numArgs == 0){
                func();
            } else {
                func(proc);
            }
        }catch(py::error_already_set const& ex){
            py::print(ex.what());
        }
    }
};

struct PyMenuItemFunc
{
    py::object func;
    PyMenuItemFunc(py::object f) : func(f) { }
    void operator()() {
        py::gil_scoped_acquire lock;
        try {
            func();
        } catch(py::error_already_set const& ex) {
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
        } catch(py::error_already_set const& ex) {
            py::print(ex.what());
        }
    }
};


TaskCommandPtr TaskPhase_addCommandExMain(TaskPhase* self, const std::string& caption, py::dict kw) {

    TaskCommandPtr command = self->addCommand(caption);

    for (auto item : kw){
        std::string key = item.first.cast<std::string>();
        py::handle value = item.second;

        if(key == "default" && PyBool_Check(value.ptr())){
            if(value.cast<bool>()){
                command->setDefault();
            }
        } else if(key == "function"){
            //;
        }
    }
    return command;
}


TaskCommandPtr TaskPhase_addCommandEx(py::tuple args_, py::dict kw) {
    TaskPhasePtr self = args_[0].cast<TaskPhasePtr>();
    const std::string caption = args_[1].cast<std::string>();
    return TaskPhase_addCommandExMain(self, caption, kw);
}    

class PyTask : public Task
{
public:
    /* Inherit the constructors */
    using Task::Task;

    void onMenuRequest(TaskMenu& menu) override{
        py::gil_scoped_acquire lock;
        py::function overload = py::get_overload(static_cast<const Task *>(this), "onMenuRequest");
        if (overload){
            try {
                overload(py::cast(menu, py::return_value_policy::reference));
            } catch(py::error_already_set const& ex) {
                py::print(ex.what());
            }
        }else{
            Task::onMenuRequest(menu);
        }
    }

    void onActivated(AbstractTaskSequencer* sequencer)  override{
        py::gil_scoped_acquire lock;
        py::function overload = py::get_overload(static_cast<const Task *>(this), "onActivated");
        if (overload){
            try {
                overload(sequencer);
            } catch(py::error_already_set const& ex) {
                py::print(ex.what());
            }
        }else{
            Task::onActivated(sequencer);
        }
    }

    void onDeactivated(AbstractTaskSequencer* sequencer)  override{
        py::gil_scoped_acquire lock;
        py::function overload = py::get_overload(static_cast<const Task *>(this), "onDeactivated");
        if (overload){
            try {
                overload(sequencer);
            } catch(py::error_already_set const& ex) {
                py::print(ex.what());
            }
        }else{
            Task::onDeactivated(sequencer);
        }
    }

    void storeState(AbstractTaskSequencer* sequencer, Mapping& archive)  override {
        py::gil_scoped_acquire lock;
        py::function overload = py::get_overload(static_cast<const Task *>(this), "storeState");
        if (overload){
            try {
                MappingPtr a = &archive;
                overload(sequencer, a);
            } catch(py::error_already_set const& ex) {
                py::print(ex.what());
            }
        }else{
            Task::storeState(sequencer, archive);
        }
    }
    
    void restoreState(AbstractTaskSequencer* sequencer, const Mapping& archive)  override {
        py::gil_scoped_acquire lock;
        py::function overload = py::get_overload(static_cast<const Task *>(this), "restoreState");
        if (overload){
            try {
                MappingPtr a = const_cast<Mapping*>(&archive);
                overload(sequencer, a);
            } catch(py::error_already_set const& ex) {
                py::print(ex.what());
            }
        }else{
            Task::restoreState(sequencer, archive);
        }
    }

};
typedef ref_ptr<PyTask> PyTaskPtr;


TaskCommandPtr Task_addCommandEx(py::tuple args_, py::dict kw){
    PyTaskPtr self = args_[0].cast<PyTaskPtr>();
    const std::string caption = args_[1].cast<std::string>();
    return TaskPhase_addCommandExMain(self->lastPhase(), caption, kw);
}


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
        .def("waitForCommandToFinish", (bool(TaskProc::*)(double)) &TaskProc::waitForCommandToFinish, py::release_gil())
        .def("waitForCommandToFinish", [](TaskProc& self){ return self.waitForCommandToFinish(); }, py::release_gil())
        .def("waitForCommandToFinish", (bool(TaskProc::*)(Connection, double)) &TaskProc::waitForCommandToFinish, py::release_gil())
        .def("notifyCommandFinish", &TaskProc:: notifyCommandFinish)
        .def("notifyCommandFinish", [](TaskProc& self){ self.notifyCommandFinish(); })
        .def("waitForSignal", &TaskProc::waitForSignal<void()>, py::release_gil())
        .def("waitForSignal", [](TaskProc& self, SignalProxy<void()> signalProxy){
                return self.waitForSignal(signalProxy); }, py::release_gil())
        .def("waitForBooleanSignal", &TaskProc::waitForBooleanSignal<void(bool)>, py::release_gil())
        .def("waitForBooleanSignal", [](TaskProc& self, SignalProxy<void(bool)> signalProxy){
                return self.waitForBooleanSignal(signalProxy); }, py::release_gil())
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
        .def(py::init<const std::string&>())
        .def("caption", &TaskCommand::caption, py::return_value_policy::reference)
        .def("setCaption", [](TaskCommand& self, const std::string& caption){
            return TaskCommandPtr(self.setCaption(caption));
        })
        .def("description", &TaskCommand::description, py::return_value_policy::reference)
        .def("setDescription", [](TaskCommand& self, const std::string& description){
            return TaskCommandPtr(self.setDescription(description));
        })
        .def("function", &TaskCommand::function)
        .def("setFunction", [](TaskCommand& self, py::object func){
            return TaskCommandPtr(self.setFunction(PyTaskFunc(func)));
        })
        .def("setDefault", [](TaskCommand& self){
            return TaskCommandPtr(self.setDefault());
        })
        .def("setDefault", [](TaskCommand& self, bool on){
             return TaskCommandPtr(self.setDefault(on));
        })
        .def("isDefault", &TaskCommand::isDefault)
        .def("setCheckable", [](TaskCommand& self, bool on){
            return TaskCommandPtr(self.setCheckable());
        }, py::arg("on")=true)
        .def("setToggleState", [](TaskCommand& self,  TaskToggleState* state){
            return TaskCommandPtr(self.setToggleState(state));
        })
        .def("toggleState",  [](TaskCommand& self){
            return TaskToggleStatePtr(self.toggleState());
        })
        .def("setChecked",  [](TaskCommand& self, bool on){
            return TaskCommandPtr(self.setChecked(on));
        })
        .def("isChecked", &TaskCommand::isChecked)
        .def("nextPhaseIndex", &TaskCommand::nextPhaseIndex)
        .def("setPhaseLink", [](TaskCommand& self, int phaseIndex) {
            return TaskCommandPtr(self.setPhaseLink(phaseIndex));
        })
        .def("setPhaseLinkStep", [](TaskCommand& self, int phaseIndexStep) {
            return TaskCommandPtr(self.setPhaseLinkStep(phaseIndexStep));
        })
        .def("linkToNextPhase", [](TaskCommand& self) {
            return TaskCommandPtr(self.linkToNextPhase());
        })
        .def("nextCommandIndex", &TaskCommand::nextCommandIndex)
        .def("setCommandLink", [](TaskCommand& self, int commandIndex){
            return TaskCommandPtr(self.setCommandLink(commandIndex));
        })
        .def("setCommandLinkStep", [](TaskCommand& self, int commandIndexStep){
            return TaskCommandPtr(self.setCommandLinkStep(commandIndexStep));
        })
        .def("linkToNextCommand", [](TaskCommand& self){
            return TaskCommandPtr(self.linkToNextCommand());
        })
        .def("isCommandLinkAutomatic", &TaskCommand::isCommandLinkAutomatic)
        .def("setCommandLinkAutomatic", [](TaskCommand& self) {
            return TaskCommandPtr(self.setCommandLinkAutomatic());
        })
        .def("setCommandLinkAutomatic", [](TaskCommand& self, bool on){
            return TaskCommandPtr(self.setCommandLinkAutomatic(on));
        })
        .def("setLevel", [](TaskCommand& self, int level){
            return TaskCommandPtr(self.setLevel(level));
        })
        .def("level", &TaskCommand::level)
        ;

    py::class_<TaskPhase, TaskPhasePtr, Referenced>(m, "TaskPhase")
        .def(py::init<const std::string&>())
        .def(py::init<const TaskPhase&>())
        .def(py::init<const TaskPhase&, bool>())
        .def("clone", [](TaskPhase& self){
            return TaskPhasePtr(self.clone());
        })
        .def("clone", [](TaskPhase& self, bool doDeepCopy){
            return TaskPhasePtr(self.clone(doDeepCopy));
        })
        .def("caption", &TaskPhase::caption, py::return_value_policy::reference)
        .def("setCaption", &TaskPhase::setCaption)
        .def("isSkipped", &TaskPhase::isSkipped)
        .def("setSkipped", &TaskPhase::setSkipped)
        .def("setPreCommand", [](TaskPhase& self, py::function func){
            return self.setPreCommand(PyTaskFunc(func));
        })
        .def("setPreCommand", &TaskPhase::setPreCommand)
        .def("preCommand", &TaskPhase::preCommand)
        .def("addCommand", [](TaskPhase& self){
            return TaskCommandPtr(self.addCommand());
        })
        .def("addCommand", [](TaskPhase& self, const std::string& caption) {
            return TaskCommandPtr(self.addCommand(caption));
        })
        .def("addToggleCommand", [](TaskPhase& self) {
            return TaskCommandPtr(self.addToggleCommand());
        })
        .def("addToggleCommand", [](TaskPhase& self, const std::string& caption){
            return TaskCommandPtr(self.addToggleCommand(caption));
        })
        .def("addCommandEx", &TaskPhase_addCommandEx)
        .def("numCommands", &TaskPhase::numCommands)
        .def("command", [](TaskPhase& self, int index) {
            return TaskCommandPtr(self.command(index));
        })
        .def("lastCommandIndex", &TaskPhase::lastCommandIndex)
        .def("lastCommand", [](TaskPhase& self) {
            return TaskCommandPtr(self.lastCommand());
        })
        .def("commandLevel", &TaskPhase::commandLevel)
        .def("maxCommandLevel", &TaskPhase::maxCommandLevel)
        ;

    py::class_<TaskPhaseProxy, TaskPhaseProxyPtr, Referenced >(m, "TaskPhaseProxy")
        .def("setCommandLevel", &TaskPhaseProxy::setCommandLevel)
        .def("commandLevel", &TaskPhaseProxy::commandLevel)
        .def("addCommand", [](TaskPhaseProxy& self) {
            return TaskCommandPtr(self.addCommand());
        })
        .def("addCommand", [](TaskPhaseProxy& self, const std::string& caption) {
            return TaskCommandPtr(self.addCommand(caption));
        })
        .def("addToggleCommand", [](TaskPhaseProxy& self) {
            return TaskCommandPtr(self.addToggleCommand());
        })
        .def("addToggleCommand", [](TaskPhaseProxy& self, const std::string& caption) {
            return TaskCommandPtr(self.addToggleCommand(caption));
        })
        ;

    py::class_<TaskMenu>(m, "TaskMenu")
        .def("addMenuItem", [](TaskMenu& self, const std::string& caption, py::object func) {
            self.addMenuItem(caption, PyMenuItemFunc(func));
        })
        .def("addCheckMenuItem", [](TaskMenu& self, const std::string& caption,
                bool isChecked, py::object func){
            self.addCheckMenuItem(caption, isChecked, PyCheckMenuItemFunc(func));
        })
        .def("addMenuSeparator", [](TaskMenu& self) {
            self.addMenuSeparator();
        })
        ;

    py::class_<Task, TaskPtr, PyTask>(m, "Task")
        .def(py::init<>())
        .def(py::init<const std::string&, const std::string&>())
        .def(py::init<const Task&, bool>(), py::arg("org"), py::arg("doDeepCopy")=true)
        .def("name", &Task::name, py::return_value_policy::reference)
        .def("setName", &Task::setName)
        .def("caption", &Task::caption, py::return_value_policy::reference)
        .def("setCaption", &Task::setCaption)
        .def("numPhases", &Task::numPhases)
        .def("phase", [](Task& self, int index){
            return TaskPhasePtr(self.phase(index));
        })
        .def("addPhase", [](Task& self, TaskPhase* phase) {
            return TaskPhasePtr(self.addPhase(phase));
        })
        .def("addPhase",  [](Task& self, const std::string& caption) {
            return TaskPhasePtr(self.addPhase(caption));
        })
        .def("lastPhase", [](Task& self) {
            return TaskPhasePtr(self.lastPhase());
        })
        .def("setPreCommand", [](Task& self, py::object func) {
            return self.setPreCommand(PyTaskFunc(func));
        })
        .def("setPreCommand", &Task::setPreCommand)
        .def("addCommand", [](Task& self) {
            return TaskCommandPtr(self.addCommand());
        })
        .def("addCommand", [](Task& self, const std::string& caption){
            return TaskCommandPtr(self.addCommand(caption));
        })
        .def("addToggleCommand", [](Task& self) {
            return TaskCommandPtr(self.addToggleCommand());
        })
        .def("addToggleCommand", [](Task& self, const std::string& caption) {
            return TaskCommandPtr(self.addToggleCommand(caption));
        })
        .def("addCommandEx", &Task_addCommandEx)
        .def("lastCommand", [](Task& self) {
            return TaskCommandPtr(self.lastCommand());
        })
        .def("lastCommandIndex", &Task::lastCommandIndex)
        .def("funcToSetCommandLink", &Task::funcToSetCommandLink)
        .def("onMenuRequest", &Task::onMenuRequest)
        .def("onActivated", &Task::onActivated)
        .def("onDeactivated", &Task::onDeactivated)
        .def("storeState", &Task::storeState)
        .def("restoreState", &Task::restoreState)
        .def("commandLevel", &Task::commandLevel)
        .def("maxCommandLevel", &Task::maxCommandLevel)
        ;

    py::class_<AbstractTaskSequencer>(m, "AbstractTaskSequencer")
        .def("activate", &AbstractTaskSequencer::activate)
        .def("isActive", &AbstractTaskSequencer::isActive)
        .def("addTask", [](AbstractTaskSequencer& self, py::object pyTask) {
            if(TaskPtr task = registerTask(&self, pyTask)){
                self.addTask(task);
            }
        })
        .def("updateTask", [](AbstractTaskSequencer& self, py::object pyTask) {
            if(TaskPtr task = registerTask(&self, pyTask)){
                self.updateTask(task);
            }
            return false;
        })
        .def("removeTask", &AbstractTaskSequencer::removeTask)
        .def("sigTaskAdded", &AbstractTaskSequencer::sigTaskAdded)
        .def("sigTaskRemoved", &AbstractTaskSequencer::sigTaskRemoved)
        .def("clearTasks", &AbstractTaskSequencer::clearTasks)
        .def("numTasks", &AbstractTaskSequencer::numTasks)
        .def("task",  [](AbstractTaskSequencer& self, int index) {
            return TaskPtr(self.task(index));
        })
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
