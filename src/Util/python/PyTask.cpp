#include "PyUtil.h"
#include "PySignal.h"
#include "../Task.h"
#include "../AbstractTaskSequencer.h"
#include "../ValueTree.h"
#include <nanobind/trampoline.h>
#include <set>
#include <map>

// PySignal.h is included for the PySignal<void(Task*)> registration below. It
// also defines a std::function<...> type_caster, which means TaskFunc (=
// std::function<void(TaskProc*)>) could be converted automatically. We avoid
// relying on that: callbacks are taken as nb::callable and wrapped in the custom
// PyTaskFunc below, which (a) avoids the reference cycle between a Task and a
// callback bound to it and (b) swallows Python-side exceptions so that a C++
// signal/sequencer emission is not aborted. We therefore do not bind any
// setFunction overload that takes a std::function directly.

using namespace std;
using namespace cnoid;
namespace nb = nanobind;

// TaskFunc is std::function<void(TaskProc*)>. PySignal.h installs a generic
// std::function type_caster, but we want to bind TaskFunc as an opaque class
// (so that TaskCommand::function() can return a callable wrapper and expose
// __call__). NB_MAKE_OPAQUE overrides the caster for this specific signature
// with the standard class caster, leaving other std::function types untouched.
NB_MAKE_OPAQUE(std::function<void(cnoid::TaskProc*)>)

namespace {

using release_gil = nb::call_guard<nb::gil_scoped_release>;

struct PyTaskFunc
{
    nb::object func;

    PyTaskFunc(nb::object f) : func(std::move(f)) { }

    void operator()(TaskProc* proc)
    {
        nb::gil_scoped_acquire lock;
        try {
            int numArgs = nb::cast<int>(func.attr("__code__").attr("co_argcount"));
            if(nb::hasattr(func, "__self__")){
                if(!func.attr("__self__").is_none()){
                    --numArgs; // for the first 'self' argument
                }
            }
            if(numArgs == 0){
                func();
            } else {
                func(proc);
            }
        } catch(nb::python_error& e){
            e.discard_as_unraisable("cnoid.TaskFunc");
        }
    }
};

struct PyMenuItemFunc
{
    nb::object func;
    PyMenuItemFunc(nb::object f) : func(std::move(f)) { }
    void operator()() {
        nb::gil_scoped_acquire lock;
        try {
            func();
        } catch(nb::python_error& e){
            e.discard_as_unraisable("cnoid.TaskMenu item");
        }
    }
};

struct PyCheckMenuItemFunc
{
    nb::object func;
    PyCheckMenuItemFunc(nb::object f) : func(std::move(f)) { }
    void operator()(bool on) {
        nb::gil_scoped_acquire lock;
        try {
            func(on);
        } catch(nb::python_error& e){
            e.discard_as_unraisable("cnoid.TaskMenu check item");
        }
    }
};


class PyTask : public Task
{
public:
    NB_TRAMPOLINE(Task, 5);

    void onMenuRequest(TaskMenu& menu) override {
        nb::detail::ticket nb_ticket(nb_trampoline, "onMenuRequest", false);
        if(nb_ticket.key.is_valid()){
            try {
                nb_trampoline.base().attr(nb_ticket.key)(nb::cast(menu, nb::rv_policy::reference));
            } catch(nb::python_error& e){
                e.discard_as_unraisable("cnoid.Task.onMenuRequest");
            }
        } else {
            Task::onMenuRequest(menu);
        }
    }

    void onActivated(AbstractTaskSequencer* sequencer) override {
        nb::detail::ticket nb_ticket(nb_trampoline, "onActivated", false);
        if(nb_ticket.key.is_valid()){
            try {
                nb_trampoline.base().attr(nb_ticket.key)(sequencer);
            } catch(nb::python_error& e){
                e.discard_as_unraisable("cnoid.Task.onActivated");
            }
        } else {
            Task::onActivated(sequencer);
        }
    }

    void onDeactivated(AbstractTaskSequencer* sequencer) override {
        nb::detail::ticket nb_ticket(nb_trampoline, "onDeactivated", false);
        if(nb_ticket.key.is_valid()){
            try {
                nb_trampoline.base().attr(nb_ticket.key)(sequencer);
            } catch(nb::python_error& e){
                e.discard_as_unraisable("cnoid.Task.onDeactivated");
            }
        } else {
            Task::onDeactivated(sequencer);
        }
    }

    void storeState(AbstractTaskSequencer* sequencer, Mapping& archive) override {
        nb::detail::ticket nb_ticket(nb_trampoline, "storeState", false);
        if(nb_ticket.key.is_valid()){
            try {
                nb_trampoline.base().attr(nb_ticket.key)(sequencer, &archive);
            } catch(nb::python_error& e){
                e.discard_as_unraisable("cnoid.Task.storeState");
            }
        } else {
            Task::storeState(sequencer, archive);
        }
    }

    void restoreState(AbstractTaskSequencer* sequencer, const Mapping& archive) override {
        nb::detail::ticket nb_ticket(nb_trampoline, "restoreState", false);
        if(nb_ticket.key.is_valid()){
            try {
                nb_trampoline.base().attr(nb_ticket.key)(sequencer, &archive);
            } catch(nb::python_error& e){
                e.discard_as_unraisable("cnoid.Task.restoreState");
            }
        } else {
            Task::restoreState(sequencer, archive);
        }
    }
};


typedef std::set<AbstractTaskSequencer*> TaskSequencerSet;
TaskSequencerSet taskSequencers;

typedef std::map<TaskPtr, nb::object> PyTaskMap;
PyTaskMap pyTasks;

void onTaskRemoved(Task* task)
{
    auto p = pyTasks.find(task);
    if(p != pyTasks.end()){
        nb::gil_scoped_acquire lock;
        pyTasks.erase(p);
    }
}


TaskPtr registerTask(AbstractTaskSequencer* sequencer, nb::object& pyTask)
{
    nb::gil_scoped_acquire lock;
    TaskPtr task;
    if(!nb::try_cast<TaskPtr>(pyTask, task) || !task){
        return TaskPtr();
    }
    if(taskSequencers.find(sequencer) == taskSequencers.end()){
        sequencer->sigTaskRemoved().connect(onTaskRemoved);
        taskSequencers.insert(sequencer);
    }
    pyTasks[task] = pyTask;
    return task;
}

} // namespace

namespace cnoid {

void exportPyTaskTypes(nb::module_& m)
{
    nb::class_<TaskProc>(m, "TaskProc")
        .def_prop_ro("currentPhaseIndex", &TaskProc::currentPhaseIndex)
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
        .def("notifyCommandFinish", &TaskProc::notifyCommandFinish)
        .def("notifyCommandFinish", [](TaskProc& self){ self.notifyCommandFinish(); })
        .def("waitForSignal", &TaskProc::waitForSignal, release_gil())
        .def("waitForSignal", [](TaskProc& self, SignalProxy<void()> signalProxy){
                return self.waitForSignal(signalProxy); }, release_gil())
        .def("waitForBooleanSignal", &TaskProc::waitForBooleanSignal, release_gil())
        .def("waitForBooleanSignal", [](TaskProc& self, SignalProxy<void(bool)> signalProxy){
                return self.waitForBooleanSignal(signalProxy); }, release_gil())
        ;

    nb::class_<TaskFunc>(m, "TaskFunc")
        .def("__call__", &TaskFunc::operator())
        ;

    nb::class_<TaskToggleState, Referenced>(m, "TaskToggleState")
        .def("isChecked", &TaskToggleState::isChecked)
        .def("setChecked", &TaskToggleState::setChecked)
        .def_prop_ro("sigToggled", &TaskToggleState::sigToggled)
        ;

    nb::class_<TaskCommand, Referenced>(m, "TaskCommand")
        .def(nb::init<>())
        .def(nb::init<const std::string&>())
        .def_prop_rw("caption", &TaskCommand::caption, &TaskCommand::setCaption)
        .def("setCaption", &TaskCommand::setCaption)
        .def_prop_rw("description", &TaskCommand::description, &TaskCommand::setDescription)
        .def("setDescription", &TaskCommand::setDescription)
        .def_prop_ro("function", &TaskCommand::function)
        .def("setFunction", [](TaskCommand& self, nb::callable func){
                return self.setFunction(PyTaskFunc(nb::object(func))); })
        .def("setDefault", &TaskCommand::setDefault, nb::arg("on") = true)
        .def("isDefault", &TaskCommand::isDefault)
        .def("setCheckable", &TaskCommand::setCheckable, nb::arg("on") = true)
        .def_prop_rw("toggleState",  &TaskCommand::toggleState, &TaskCommand::setToggleState)
        .def("setToggleState", &TaskCommand::setToggleState)
        .def("setChecked",  &TaskCommand::setChecked)
        .def("isChecked", &TaskCommand::isChecked)
        .def_prop_ro("nextPhaseIndex", &TaskCommand::nextPhaseIndex)
        .def("setPhaseLink", &TaskCommand::setPhaseLink)
        .def("setPhaseLinkStep", &TaskCommand::setPhaseLinkStep)
        .def("linkToNextPhase", &TaskCommand::linkToNextPhase)
        .def_prop_ro("nextCommandIndex", &TaskCommand::nextCommandIndex)
        .def("setCommandLink", &TaskCommand::setCommandLink)
        .def("setCommandLinkStep", &TaskCommand::setCommandLinkStep)
        .def("linkToNextCommand", &TaskCommand::linkToNextCommand)
        .def("isCommandLinkAutomatic", &TaskCommand::isCommandLinkAutomatic)
        .def("setCommandLinkAutomatic", &TaskCommand::setCommandLinkAutomatic, nb::arg("on") = true)
        .def_prop_rw("level", &TaskCommand::level, &TaskCommand::setLevel)
        .def("setLevel", &TaskCommand::setLevel)
        .def("linkToNextTask", &TaskCommand::linkToNextTask)
        ;

    nb::class_<TaskPhase, Referenced>(m, "TaskPhase")
        .def(nb::init<const std::string&>())
        .def(nb::init<const TaskPhase&>())
        .def(nb::init<const TaskPhase&, bool>())
        .def("clone", &TaskPhase::clone, nb::arg("doDeepCopy") = true)
        .def_prop_rw("caption", &TaskPhase::caption, &TaskPhase::setCaption)
        .def("setCaption", &TaskPhase::setCaption)
        .def("isSkipped", &TaskPhase::isSkipped)
        .def("setSkipped", &TaskPhase::setSkipped)
        .def_prop_rw("preCommand", &TaskPhase::preCommand, &TaskPhase::setPreCommand)
        .def("setPreCommand", [](TaskPhase& self, nb::callable func){ return self.setPreCommand(PyTaskFunc(nb::object(func))); })
        .def("addCommand", (TaskCommand*(TaskPhase::*)()) &TaskPhase::addCommand)
        .def("addCommand", (TaskCommand*(TaskPhase::*)(const string&)) &TaskPhase::addCommand)
        .def("addToggleCommand", (TaskCommand*(TaskPhase::*)()) &TaskPhase::addToggleCommand)
        .def("addToggleCommand", (TaskCommand*(TaskPhase::*)(const string&)) &TaskPhase::addToggleCommand)
        .def_prop_ro("numCommands", &TaskPhase::numCommands)
        .def("command", &TaskPhase::command)
        .def_prop_ro("lastCommandIndex", &TaskPhase::lastCommandIndex)
        .def("lastCommand", &TaskPhase::lastCommand)
        .def_prop_ro("commandLevel", &TaskPhase::commandLevel)
        .def_prop_ro("maxCommandLevel", &TaskPhase::maxCommandLevel)
        ;

    nb::class_<TaskPhaseProxy, Referenced>(m, "TaskPhaseProxy")
        .def(nb::init<TaskPhase*>())
        .def_prop_rw("commandLevel", &TaskPhaseProxy::commandLevel, &TaskPhaseProxy::setCommandLevel)
        .def("getCommandLevel", &TaskPhaseProxy::commandLevel)
        .def("setCommandLevel", &TaskPhaseProxy::setCommandLevel)
        .def("addCommand", (TaskCommand*(TaskPhaseProxy::*)()) &TaskPhaseProxy::addCommand)
        .def("addCommand", (TaskCommand*(TaskPhaseProxy::*)(const string&)) &TaskPhaseProxy::addCommand)
        .def("addToggleCommand", (TaskCommand*(TaskPhaseProxy::*)()) &TaskPhaseProxy::addToggleCommand)
        .def("addToggleCommand", (TaskCommand*(TaskPhaseProxy::*)(const string&)) &TaskPhaseProxy::addToggleCommand)
        ;

    nb::class_<TaskMenu>(m, "TaskMenu")
        .def("addMenuItem", [](TaskMenu& self, const std::string& caption, nb::object func){
                self.addMenuItem(caption, PyMenuItemFunc(func)); })
        .def("addCheckMenuItem", [](TaskMenu& self, const std::string& caption, bool isChecked, nb::object func){
                self.addCheckMenuItem(caption, isChecked, PyCheckMenuItemFunc(func)); })
        .def("addMenuSeparator", &TaskMenu::addMenuSeparator)
        ;

    nb::class_<Task, Referenced, PyTask>(m, "Task")
        .def(nb::init<>())
        .def(nb::init<const string&, const string&>())
        .def(nb::init<const Task&, bool>())
        .def_prop_rw("name", &Task::name, &Task::setName)
        .def("setName", &Task::setName)
        .def_prop_rw("caption", &Task::caption, &Task::setCaption)
        .def("setCaption", &Task::setCaption)
        .def_prop_ro("numPhases", &Task::numPhases)
        .def("phase", &Task::phase)
        .def("addPhase", (TaskPhase*(Task::*)(TaskPhase*)) &Task::addPhase)
        .def("addPhase", (TaskPhase*(Task::*)(const string&)) &Task::addPhase)
        .def_prop_ro("lastPhase", &Task::lastPhase)
        .def("setPreCommand", [](Task& self, nb::callable func){ return self.setPreCommand(PyTaskFunc(nb::object(func))); })
        .def("addCommand", (TaskCommand*(Task::*)()) &Task::addCommand)
        .def("addCommand", (TaskCommand*(Task::*)(const string&)) &Task::addCommand)
        .def("addToggleCommand", (TaskCommand*(Task::*)()) &Task::addToggleCommand)
        .def("addToggleCommand", (TaskCommand*(Task::*)(const string&)) &Task::addToggleCommand)
        .def_prop_ro("lastCommand", &Task::lastCommand)
        .def_prop_ro("lastCommandIndex", &Task::lastCommandIndex)
        .def_prop_ro("commandLevel", &Task::commandLevel)
        .def_prop_ro("maxCommandLevel", &Task::maxCommandLevel)
        .def_prop_ro("funcToSetCommandLink", &Task::funcToSetCommandLink)
        .def("onActivated", &Task::onActivated)
        .def("onDeactivated", &Task::onDeactivated)
        .def("storeState", &Task::storeState)
        .def("restoreState", &Task::restoreState)
        .def("onMenuRequest", &Task::onMenuRequest)
        ;

    PySignal<void(Task*)>(m, "TaskSignal");

    nb::class_<AbstractTaskSequencer>(m, "AbstractTaskSequencer")
        .def("activate", &AbstractTaskSequencer::activate, nb::arg("on") = true)
        .def("isActive", &AbstractTaskSequencer::isActive)

        .def("addTask", [](AbstractTaskSequencer* self, nb::object pyTask){
                if(TaskPtr task = registerTask(self, pyTask)){
                    self->addTask(task);
                }
            })

        .def("updateTask", [](AbstractTaskSequencer* self, nb::object pyTask){
                if(TaskPtr task = registerTask(self, pyTask)){
                    return self->updateTask(task);
                }
                return false;
            })

        .def("removeTask", &AbstractTaskSequencer::removeTask)
        .def("clearTasks", &AbstractTaskSequencer::clearTasks)
        .def_prop_ro("sigTaskAdded", &AbstractTaskSequencer::sigTaskAdded)
        .def_prop_ro("sigTaskRemoved", &AbstractTaskSequencer::sigTaskRemoved)
        .def_prop_ro("numTasks", &AbstractTaskSequencer::numTasks)
        .def("getTask",  &AbstractTaskSequencer::task)
        .def_prop_ro("currentTaskIndex", &AbstractTaskSequencer::currentTaskIndex)
        .def("setCurrentTask", &AbstractTaskSequencer::setCurrentTask)
        .def_prop_ro("sigCurrentTaskChanged", &AbstractTaskSequencer::sigCurrentTaskChanged)
        .def_prop_ro("currentPhaseIndex", &AbstractTaskSequencer::currentPhaseIndex)
        .def("setCurrentPhase", &AbstractTaskSequencer::setCurrentPhase)
        .def_prop_ro("sigCurrentPhaseChanged", &AbstractTaskSequencer::sigCurrentPhaseChanged)
        .def_prop_ro("currentCommandIndex", &AbstractTaskSequencer::currentCommandIndex)
        .def("executeCommand", &AbstractTaskSequencer::executeCommand)
        .def_prop_ro("sigCurrentCommandChanged", &AbstractTaskSequencer::sigCurrentCommandChanged)
        .def("isBusy", &AbstractTaskSequencer::isBusy)
        .def_prop_ro("sigBusyStateChanged", &AbstractTaskSequencer::sigBusyStateChanged)
        .def("isAutoMode", &AbstractTaskSequencer::isAutoMode)
        .def("setAutoMode", &AbstractTaskSequencer::setAutoMode)
        .def("sigAutoModeToggled", &AbstractTaskSequencer::sigAutoModeToggled)
        .def("serializeTasks", &AbstractTaskSequencer::serializeTasks)
        ;
}

} // namespace cnoid
