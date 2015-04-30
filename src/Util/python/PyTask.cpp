/*!
  @author Shizuko Hattori
  @author Shin'ichiro Nakaoka
 */

#include <iostream>
#include "../Task.h"
#include "../AbstractTaskSequencer.h"
#include "../ValueTree.h"
#include "PyUtil.h"
#include <cnoid/PythonUtil>
#include <boost/python/raw_function.hpp>

using namespace boost::python;
namespace python = boost::python;
using namespace cnoid;

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
    

TaskCommandPtr TaskCommand_setFunction(TaskCommand& self, python::object func){
    return self.setFunction(PyTaskFunc(func));
}

TaskCommandPtr TaskCommand_setDefault1(TaskCommand& self) {
    return self.setDefault();
}

TaskCommandPtr TaskCommand_setDefault2(TaskCommand& self, bool on) {
    return self.setDefault(on);
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

TaskPhasePtr TaskPhase_clone1(TaskPhase& self){
    return self.clone();
}

TaskPhasePtr  TaskPhase_clone2(TaskPhase& self, bool doDeepCopy ){
    return self.clone(doDeepCopy);
}

void TaskPhase_setPreCommand(TaskPhase& self, python::object func){
    return self.setPreCommand(PyTaskFunc(func));
}

TaskCommandPtr TaskPhase_addCommand(TaskPhase& self, const std::string& caption) {
    return self.addCommand(caption);
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
    TaskWrap(){};
    TaskWrap(const std::string& name, const std::string& caption) : Task(name, caption) {};
    TaskWrap(const Task& org, bool doDeepCopy = true) : Task(org, doDeepCopy) {};

    void onMenuRequest(TaskMenu& menu){
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

    bool storeState(AbstractTaskSequencer* sequencer, Mapping& archive){
        bool called = false;
        bool result = false;
        {
            PyGILock lock;
            try {
                if(python::override storeStateFunc = this->get_override("storeState")){
                    called = true;
                    MappingPtr a = &archive;
                    result = storeStateFunc(boost::ref(sequencer), a);
                }
            } catch(python::error_already_set const& ex) {
                cnoid::handlePythonException();
            }
        }
        if(!called){
            result = Task::storeState(sequencer, archive);
        }
        return result;
    }

    bool restoreState(AbstractTaskSequencer* sequencer, const Mapping& archive){
        bool called = false;
        bool result = false;
        {
            PyGILock lock;
            try {
                if(python::override restoreState = this->get_override("restoreState")){
                    called = true;
                    MappingPtr a = const_cast<Mapping*>(&archive);
                    result = restoreState(boost::ref(sequencer), a);
                }
            } catch(python::error_already_set const& ex) {
                cnoid::handlePythonException();
            }
        }
        if(!called){
            result = Task::restoreState(sequencer, archive);
        }
        return result;
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

TaskCommandPtr Task_addCommand(Task& self, const std::string& caption){
    return self.addCommand(caption);
}

TaskCommandPtr Task_lastCommand(Task& self){
    return self.lastCommand();
}

TaskCommandPtr Task_addCommandEx(python::tuple args_, python::dict kw){
    TaskWrapPtr self = python::extract<TaskWrapPtr>(args_[0]);
    const std::string caption = python::extract<std::string>(args_[1]);
    return TaskPhase_addCommandExMain(self->lastPhase(), caption, kw);
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

    class_<TaskCommand, TaskCommandPtr, bases<Referenced> >
        ("TaskCommand", init<const std::string&>())
        .def("caption", &TaskCommand::caption, return_value_policy<copy_const_reference>())
        .def("function", &TaskCommand::function)
        .def("setFunction", TaskCommand_setFunction)
        .def("setDefault", TaskCommand_setDefault1)
        .def("setDefault", TaskCommand_setDefault2)
        .def("isDefault", &TaskCommand::isDefault)
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
        .def("addCommand", TaskPhase_addCommand)
        .def("addCommandEx", python::raw_function(TaskPhase_addCommandEx, 2))
        .def("numCommands", &TaskPhase::numCommands)
        .def("command", TaskPhase_command)
        .def("lastCommandIndex", &TaskPhase::lastCommandIndex)
        .def("lastCommand", TaskPhase_lastCommand)
        ;
    
    implicitly_convertible<TaskPhasePtr, ReferencedPtr>();
    
    class_<TaskMenu, boost::noncopyable >("TaskMenu", no_init)
        .def("addMenuItem", TaskMenu_addMenuItem1)
        .def("addCheckMenuItem", TaskMenu_addCheckMenuItem1)
        .def("addMenuSeparator", TaskMenu_addMenuSeparator)
        ;
    
    class_<TaskWrap, TaskWrapPtr, bases<Referenced>, boost::noncopyable >("Task")
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
        .def("addCommand", Task_addCommand)
        .def("addCommandEx", python::raw_function(Task_addCommandEx, 2))
        .def("lastCommand", Task_lastCommand)
        .def("lastCommandIndex", &Task::lastCommandIndex)
        .def("funcToSetCommandLink", &Task::funcToSetCommandLink)
        .def("onMenuRequest", &Task::onMenuRequest)
        ;
    
    implicitly_convertible<TaskWrapPtr, ReferencedPtr>();

    class_<AbstractTaskSequencer, AbstractTaskSequencer*, boost::noncopyable>("AbstractTaskSequencer", no_init)
        .def("addTask", &AbstractTaskSequencer::addTask)
        .def("updateTask", &AbstractTaskSequencer::updateTask)
        .def("numTasks", &AbstractTaskSequencer::numTasks)
        .def("task", AbstractTaskSequencer_task)
        .def("currentTaskIndex", &AbstractTaskSequencer::currentTaskIndex)
        .def("setCurrentTask", &AbstractTaskSequencer::setCurrentTask)
        .def("sigCurrentTaskChanged", &AbstractTaskSequencer::sigCurrentTaskChanged)
        .def("currentPhaseIndex", &AbstractTaskSequencer::currentPhaseIndex)
        .def("setCurrentPhase", &AbstractTaskSequencer::setCurrentPhase)
        .def("sigCurrentPhaseChanged", &AbstractTaskSequencer::sigCurrentPhaseChanged)
        .def("currentCommandIndex", &AbstractTaskSequencer::currentCommandIndex)
        .def("sigCurrentCommandChanged", &AbstractTaskSequencer::sigCurrentCommandChanged)
        .def("isBusy", &AbstractTaskSequencer::isBusy)
        .def("sigBusyStateChanged", &AbstractTaskSequencer::sigBusyStateChanged)
        .def("isAutoMode", &AbstractTaskSequencer::isAutoMode)
        .def("setAutoMode", &AbstractTaskSequencer::setAutoMode)
        .def("sigAutoModeToggled", &AbstractTaskSequencer::sigAutoModeToggled)
        ;
}

}
