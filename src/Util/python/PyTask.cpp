/*!
  @author Shizuko Hattori
  @author Shin'ichiro Nakaoka
 */

#include "../Task.h"
#include "../ValueTree.h"
#include "PyUtil.h"
#include <cnoid/PythonUtil>

using namespace boost;
using namespace boost::python;
using namespace cnoid;

namespace {

void TaskProc_notifyCommandFinishTrue(TaskProc& self){
    self.notifyCommandFinish(true);
}

bool TaskPorc_waitForSignal1(boost::python::object self, boost::python::object signalProxy)
{
    boost::python::object notifyCommandFinish = self.attr("notifyCommandFinish_true");
    boost::python::object connection = signalProxy.attr("connect")(notifyCommandFinish);
    self.attr("waitForCommandToFinish")(connection, 0.0);
}

bool TaskPorc_waitForSignal2(boost::python::object self, boost::python::object signalProxy, double timeout)
{
    boost::python::object notifyCommandFinish = self.attr("notifyCommandFinish_true");
    boost::python::object connection = signalProxy.attr("connect")(notifyCommandFinish);
    self.attr("waitForCommandToFinish")(connection, timeout);
}

bool TaskPorc_waitForBooleanSignal1(boost::python::object self, boost::python::object signalProxy)
{
    boost::python::object notifyCommandFinish = self.attr("notifyCommandFinish");
    boost::python::object connection = signalProxy.attr("connect")(notifyCommandFinish);
    self.attr("waitForCommandToFinish")(connection, 0.0);
}

bool TaskPorc_waitForBooleanSignal2(boost::python::object self, boost::python::object signalProxy, double timeout)
{
    boost::python::object notifyCommandFinish = self.attr("notifyCommandFinish");
    boost::python::object connection = signalProxy.attr("connect")(notifyCommandFinish);
    self.attr("waitForCommandToFinish")(connection, timeout);
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
    boost::python::object obj;
    boost::python::object func;
    PyTaskFunc(boost::python::object o, boost::python::object f) : obj(o), func(f) { }
    void operator()(TaskProc* proc) {
        PyGILock lock;
        try {
            func(obj, boost::ref(proc));
        } catch(boost::python::error_already_set const& ex) {
            handlePythonException();
        }
    }
    void operator()(bool on) {
        PyGILock lock;
        try {
            func(obj, on);
        } catch(boost::python::error_already_set const& ex) {
            handlePythonException();
        }
    }
    void operator()() {
        PyGILock lock;
        try {
            func(obj);
        } catch(boost::python::error_already_set const& ex) {
            handlePythonException();
        }
    }
};

TaskCommand* TaskCommand_setFunction(TaskCommand& self, boost::python::object func, boost::python::object obj){
    return self.setFunction(PyTaskFunc(obj, func)); }

TaskCommand* TaskCommand_setDefault1(TaskCommand& self) {
    return self.setDefault(); }

TaskCommand* TaskCommand_setDefault2(TaskCommand& self, bool on) {
    return self.setDefault(on); }

TaskCommand* TaskCommand_setCommandLinkAutomatic1(TaskCommand& self) {
    return self.setCommandLinkAutomatic(); }

TaskCommand* TaskCommand_setCommandLinkAutomatic2(TaskCommand& self, bool on) {
    return self.setCommandLinkAutomatic(on); }

TaskPhase* TaskPhase_clone1(TaskPhase& self){
    return self.clone(); }

TaskPhase*  TaskPhase_clone2(TaskPhase& self, bool doDeepCopy ){
    return self.clone(doDeepCopy); }

void TaskPhase_setPreCommand(TaskPhase& self, boost::python::object func, boost::python::object obj){
    return self.setPreCommand(PyTaskFunc(obj, func)); }

void TaskMenu_addMenuItem1(TaskMenu& self, const std::string& caption,
        boost::python::object func, boost::python::object obj){
    self.addMenuItem(caption, PyTaskFunc(obj, func)); }

void TaskMenu_addCheckMenuItem1(TaskMenu& self,const std::string& caption,
        bool isChecked, boost::python::object func, boost::python::object obj){
    self.addCheckMenuItem(caption, isChecked, PyTaskFunc(obj, func)); }

void TaskMenu_addMenuSeparator(TaskMenu& self){
    self.addMenuSeparator(); }

class TaskWrap : public Task, public wrapper<Task>
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
                if(override onMenuRequest = this->get_override("onMenuRequest")){
                    called = true;
                    onMenuRequest(boost::ref(menu));
                } 
            } catch(boost::python::error_already_set const& ex) {
                cnoid::handlePythonException();
            }
        }
        if(!called){
            Task::onMenuRequest(menu);
        }
    }

    bool storeState(TaskProc* proc, Mapping& archive){
        bool called = false;
        bool result = false;
        {
            PyGILock lock;
            try {
                if(override storeState = this->get_override("storeState")){
                    called = true;
                    result = storeState(proc, boost::ref(archive));
                }
            } catch(boost::python::error_already_set const& ex) {
                cnoid::handlePythonException();
            }
        }
        if(!called){
            result = Task::storeState(proc, archive);
        }
        return result;
    }

    bool restoreState(TaskProc* proc, const Mapping& archive){
        bool called = false;
        bool result = false;
        {
            PyGILock lock;
            try {
                if(override restoreState = this->get_override("restoreState")){
                    called = true;
                    result = restoreState(proc, boost::ref(archive));
                }
            } catch(boost::python::error_already_set const& ex) {
                cnoid::handlePythonException();
            }
        }
        if(!called){
            result = Task::restoreState(proc, archive);
        }
        return result;
    }
    
};

typedef ref_ptr<TaskWrap> TaskWrapPtr;

TaskPhase* (Task::*addPhase1)(TaskPhase*) = &Task::addPhase;
TaskPhase* (Task::*addPhase2)(const std::string&) = &Task::addPhase;

void Task_setPreCommand(Task& self, boost::python::object func, boost::python::object obj){
    return self.setPreCommand(PyTaskFunc(obj, func)); }

}

namespace cnoid {

void exportPyTaskTypes()
{
    class_<TaskProc, TaskProc*, boost::noncopyable>("TaskProc", no_init)
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
        .def("setFunction", TaskCommand_setFunction, return_value_policy<reference_existing_object>())
        .def("setDefault", TaskCommand_setDefault1, return_value_policy<reference_existing_object>())
        .def("setDefault", TaskCommand_setDefault2, return_value_policy<reference_existing_object>())
        .def("isDefault", &TaskCommand::isDefault)
        .def("nextPhaseIndex", &TaskCommand::nextPhaseIndex)
        .def("setPhaseLink", &TaskCommand::setPhaseLink, return_value_policy<reference_existing_object>())
        .def("setPhaseLinkStep", &TaskCommand::setPhaseLinkStep, return_value_policy<reference_existing_object>())
        .def("linkToNextPhase", &TaskCommand::linkToNextPhase, return_value_policy<reference_existing_object>())
        .def("nextCommandIndex", &TaskCommand::nextCommandIndex)
        .def("setCommandLink", &TaskCommand::setCommandLink, return_value_policy<reference_existing_object>())
        .def("setCommandLinkStep", &TaskCommand::setCommandLinkStep, return_value_policy<reference_existing_object>())
        .def("linkToNextCommand", &TaskCommand::linkToNextCommand, return_value_policy<reference_existing_object>())
        .def("isCommandLinkAutomatic", &TaskCommand::isCommandLinkAutomatic)
        .def("setCommandLinkAutomatic", TaskCommand_setCommandLinkAutomatic1, return_value_policy<reference_existing_object>())
        .def("setCommandLinkAutomatic", TaskCommand_setCommandLinkAutomatic2, return_value_policy<reference_existing_object>())
        ;
    
    implicitly_convertible<TaskCommandPtr, ReferencedPtr>();
    
    class_<TaskPhase, TaskPhasePtr, bases<Referenced>, boost::noncopyable >
        ("TaskPhase", init<const std::string&>())
        .def(init<const TaskPhase&>())
        .def(init<const TaskPhase&, bool>())
        .def("clone", TaskPhase_clone1, return_value_policy<reference_existing_object>())
        .def("clone", TaskPhase_clone2, return_value_policy<reference_existing_object>())
        .def("caption", &TaskPhase::caption, return_value_policy<copy_const_reference>())
        .def("setCaption", &TaskPhase::setCaption)
        .def("isSkipped", &TaskPhase::isSkipped)
        .def("setSkipped", &TaskPhase::setSkipped)
        .def("setPreCommand", TaskPhase_setPreCommand)
        .def("setPreCommand", &TaskPhase::setPreCommand)
        .def("preCommand", &TaskPhase::preCommand)
        .def("addCommand", &TaskPhase::addCommand, return_value_policy<reference_existing_object>())
        .def("numCommands", &TaskPhase::numCommands)
        .def("command", &TaskPhase::command, return_value_policy<reference_existing_object>())
        .def("lastCommandIndex", &TaskPhase::lastCommandIndex)
        .def("lastCommand", &TaskPhase::lastCommand, return_value_policy<reference_existing_object>())
        ;
    
    implicitly_convertible<TaskPhasePtr, ReferencedPtr>();
    
    class_<TaskMenu, boost::noncopyable >("TaskMenu", no_init)
        .def("addMenuItem", TaskMenu_addMenuItem1)
        .def("addCheckMenuItem", TaskMenu_addCheckMenuItem1)
        .def("addMenuSeparator", TaskMenu_addMenuSeparator)
        ;
    
    class_<TaskWrap, TaskWrapPtr, boost::noncopyable >("Task")
        .def(init<const std::string&, const std::string&>())
        .def(init<const Task&, bool>())
        .def(init<const Task&>())
        .def("name", &Task::name, return_value_policy<copy_const_reference>())
        .def("setName", &Task::setName)
        .def("caption", &Task::caption, return_value_policy<copy_const_reference>())
        .def("setCaption", &Task::setCaption)
        .def("numPhases", &Task::numPhases)
        .def("phase", &Task::phase, return_value_policy<reference_existing_object>())
        .def("addPhase", addPhase1, return_value_policy<reference_existing_object>())
        .def("addPhase", addPhase2, return_value_policy<reference_existing_object>())
        .def("lastPhase", &Task::lastPhase, return_value_policy<reference_existing_object>())
        .def("setPreCommand", Task_setPreCommand)
        .def("setPreCommand", &Task::setPreCommand)
        .def("addCommand", &Task::addCommand, return_value_policy<reference_existing_object>())
        .def("lastCommand", &Task::lastCommand, return_value_policy<reference_existing_object>())
        .def("funcToSetCommandLink", &Task::funcToSetCommandLink)
        .def("sigUpdated", &Task::sigUpdated)
        .def("notifyUpdate", &Task::notifyUpdate)
        .def("onMenuRequest", &Task::onMenuRequest)
        ;
    
    implicitly_convertible<TaskWrapPtr, TaskPtr>();
    implicitly_convertible<TaskPtr, ReferencedPtr>();
    
    //python::def("execPythonCode", cnoid::execPythonCode);
}

}
